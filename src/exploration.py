#!/usr/bin/python2.7

"""exploration.py: According to the map, select points and evaluate them."""
import itertools
import random
import sys
import traceback

import matplotlib.pyplot as plt

import numpy

from sympy import Polygon, Point, Symbol, Segment

import rospy
import message_filters
from std_msgs.msg import Int8
from sensor_msgs.msg import NavSatFix
from message_filters import ApproximateTimeSynchronizer, Subscriber
from heron_sonde.msg import Sonde
#from simulated_sensor.msg import ChloData
from gps_nav.srv import *
#from simulated_sensor.msg import Measurement
from maxima_sampling.msg import NavStatus, SampleCandidate
from sensor_msgs.msg import Range

from gps_tools import *
import world

def locate_min(a):
    smallest = min(a)
    return smallest, [index for index, element in enumerate(a) 
                              if smallest == element]

def polygon_area(x,y):
    area = 0.5*numpy.abs(numpy.dot(x,numpy.roll(y,1))-numpy.dot(y,numpy.roll(x,1)))
    return area

class Evaluation(object):
    def __init__(self, weights, scale_factors):
        self.weights = weights
        self.scale_factors = scale_factors
        self.grow_weight = True
        
    def evaluate(self, current_pose, locations, world, time):
        """
        Args:
            current_pose: GPS pose of the robot.
            locations: pairs of integer, representing the cell index.
            world: world model.
        """
        # TODO(alberto) general evaluation class that allows to plugin different
        # criteria.
        locations, variances = world.predict(locations)
        evaluation = [numpy.inf] * len(locations)
        current_pose_utm = convert_gps_to_utm(current_pose[0], current_pose[1])
        #print current_pose_utm
        distances = [numpy.inf] * len(locations)
        for i, l in enumerate(locations):
            # Distance. Poses and locations in meters.
            distances[i] = numpy.linalg.norm(numpy.array(l)-numpy.array([current_pose_utm.x, current_pose_utm.y]))

        max_d = max(distances)
        max_v = max(variances)

        if self.grow_weight:
            self.weights[1] = time / (time + 700.0) # TODO parameter.
            self.weights[0] = 1 - self.weights[1]
            print "WEIGHTS = "
            print self.weights

        #print variances
        for i, l in enumerate(locations):
            # Variance of the location.
            d = distances[i]
            v = variances[i]
            
            # Evaluation function.
            evaluation[i] = (d / max_d) * self.weights [0] + (1 - v / max_v) * self.weights[1]
        smallest, best_indices = locate_min(evaluation) 
        #print evaluation
        return best_indices#evaluation.index(min(evaluation))

class Exploration(object):
    def __init__(self, latitude, longitude, width, height, spacing, orientation, weights, scale_factors, time_threshold=1000.0, replan_rate=1.0, plan_window=(3,3)):
        # TODO(alberto) general subscribing to topics.

        self.world = world.World(latitude, longitude, width, height, spacing, orientation)
        
        self.pub = rospy.Publisher('maxima_sampling_status', NavStatus, queue_size=1)
        self.sample_pub = rospy.Publisher('/sample_candidates', SampleCandidate, queue_size=1)
        self.path_pub = rospy.Publisher('explorer_path',ExplorerPath, queue_size=1)
        self.waypoint_goto_result_sub = rospy.Subscriber('waypoint_goto_result', Int8, self.goto_waypoint_result_callback)
        #self.data_sub = rospy.Subscriber('/heron2/sonde', Sonde, self.data_callback)
        #self.data_sub = rospy.Subscriber('chlorophyll_reading', Measurement, self.data_callback)
        subscribers = [Subscriber('navsat/fix', NavSatFix), Subscriber('atu120at/sonar', Range)]
        self.time_synchronizer = ApproximateTimeSynchronizer(subscribers, 10, 1.0)
        self.time_synchronizer.registerCallback(self.data_callback)

        #self.gps_sub = message_filters.Subscriber('navsat/fix', NavSatFix)
        #self.data_sub = message_filters.Subscriber('sonde', Sonde)
        #self.data_sub = message_filters.Subscriber('chlorophyll_reading',ChloData)
        #self.ts = message_filters.ApproximateTimeSynchronizer([self.gps_sub, self.data_sub], 10)
        #self.ts.registerCallback(data_callback)

            
        # TODO(alberto) parameters for evaluatio.
        self.evaluation = Evaluation(weights, scale_factors)
        self.curr_time=0 # not general, TODO from actual time.
        self.state = 0 # 0: IDLE, 1: GO_TO.
        self.current_gps = None
        self.time_last_exploration_plan_computed = None
        self.candidate_list = []
        #print "Before request in Init"
        #self.request_gps()
        #print "after request in Init"
        self.got_first_data_when_stopped = False
        
        self.time_threshold = time_threshold # Time threshold checked to cancel the last exploration waypoint.
        
        self.replan_rate = replan_rate
        
        self.plan_window = plan_window # (width, height).

    def request_gps(self):
        gps_msg = rospy.wait_for_message('navsat/fix', NavSatFix)
        self.current_gps = (gps_msg.latitude, gps_msg.longitude)

    def data_callback(self, gps, measurement):
        #print "In data_callback of chlorophyll_reading"
        if measurement.header.stamp.secs != 0:
            self.curr_time = measurement.header.stamp.secs
            if (self.state > 0) or (not self.got_first_data_when_stopped):
                #self.request_gps()
                #self.last_data = data
                #self.world.add_measurement(data.header.stamp.secs, self.current_gps, data.clorophyll_u)
                gps_location = [gps.latitude, gps.longitude]
                self.world.add_measurement(measurement.header.stamp.secs, gps_location, measurement.data)
                sample_candidate_msg = SampleCandidate()
                sample_candidate_msg.header.stamp = rospy.get_rostime()
                sample_candidate_msg.latitude = gps_location[0]#self.current_gps[0]
                sample_candidate_msg.longitude = gps_location[1]#self.current_gps[1]
                sample_candidate_msg.data = measurement.range
                #sample_candidate_msg.data = self.last_data.clorophyll_u
                self.sample_pub.publish(sample_candidate_msg)
                if self.state != 0:
                    self.got_first_data_when_stopped = False
                else:
                    self.got_first_data_when_stopped = True
    
    def goto_waypoint_result_callback(self, msg):
        if msg.data == 1:
            self.state = 0
        else:
            # TODO(alberto) recovery mechanism.
            rospy.logerr("goto failed.")
            self.state = 0
    
    def pick_locations_to_evaluate(self, method="fixed"):
        """Returns a list of cells where the robot can go.
        Args:
            method: {fixed, contour}
        Return:
            List of cells.
        """
        current_cell = self.world.cell_corresponding_to_gps(self.current_gps[0], self.current_gps[1])
        current_pose_utm = convert_gps_to_utm(self.current_gps[0], self.current_gps[1])
        if method == "fixed":
            x_values = range(max(0, current_cell[0] - self.plan_window[0] / 2), min(self.world.width-1, current_cell[0] + self.plan_window[0] / 2)+1)
            y_values = range(max(0, current_cell[1] - self.plan_window[1] / 2), min(self.world.height-1, current_cell[1] + self.plan_window[1] / 2)+1)
            #print x_values
            #print y_values
            c1=[[min(x_values)], y_values]
            c2=[[max(x_values)], y_values]
            c3=[x_values, [min(y_values)]]
            c4=[x_values, [max(y_values)]]
            l1=list(itertools.product(*c1))
            l2=list(itertools.product(*c2))
            l3=list(itertools.product(*c3))
            l4=list(itertools.product(*c4))
            candidate_list = list(set(l1+l2+l3+l4))
            self.candidate_list = list(set(self.candidate_list+candidate_list))
        elif method == "contour":
            # Create mesh TODO move to constructor.

            # Predict for the whole world.
            locations, variances = self.world.predict(self.world.cell_locations)
            L = numpy.array(locations)
            X_utm = L[:,0].reshape(self.world.width, self.world.height)
            Y_utm = L[:,1].reshape(self.world.width, self.world.height)
            V = variances.reshape(self.world.width, self.world.height)

            # Find contours.
            C = plt.contour(X_utm, Y_utm, V, colors='black', linewidth=.5)
            contour_with_robot = None
            
            # Find frontier.
            for c in C.collections:
                #if c.get_paths()[0].contains_point((current_pose_utm.x, current_pose_utm.y)):
                #p = Polygon(*c.get_paths()[0].to_polygons()[0])
                p = c.get_paths()[0].to_polygons()[0]
                p = numpy.array(p)
                #print "p"
                #print p
                #print "contour_with_robot"
                #print contour_with_robot
                if contour_with_robot is not None:
                    if polygon_area(p[:,0], p[:,1]) > polygon_area(contour_with_robot[:,0], contour_with_robot[:,1]):
                        contour_with_robot = p
                else:
                    contour_with_robot = p
            # Find locations on the frontier.
            self.candidate_list = []
            if contour_with_robot is not None:
                #contour_with_robot = Polygon(*contour_with_robot)
                for i in xrange(len(contour_with_robot)-1):
                #for s in contour_with_robot.sides:
                    s = Segment(contour_with_robot[i], contour_with_robot[i+1])
                    step_size = s.length / self.world.spacing
                    step_size = 1.0 / step_size.evalf()
                    t = Symbol('t', real=True)
                    #print s
                    #print contour_with_robot
                    point_t = s.arbitrary_point()

                    for step in numpy.arange(0.0, 1.0000001, step_size):
                        p = Point(point_t.x.subs(t, step), point_t.y.subs(t, step))
                        cell = self.world.cell_corresponding_to_gps(p.x.evalf(), p.y.evalf(), utm=True)
                        if cell not in self.candidate_list:
                            self.candidate_list.append(cell)
        print "current cell", current_cell
        print "current gps", self.current_gps
        
        #print "Candidate list", self.candidate_list
        return self.candidate_list
        #candidate_locations = [x_values, y_values]
        #return list(itertools.product(*candidate_locations))
    
    def spin(self):    
        r = rospy.Rate(self.replan_rate)
        prev_best_candidate = []
        print "SPINNING"
        while not rospy.is_shutdown():
        
            if self.state == 0:
                # Update the current pose.
                print "Before request GPS"
                self.request_gps()
                
                msg = NavStatus()
                msg.t = rospy.get_rostime()
                msg.curr_lat = self.current_gps[0] 
                msg.curr_lon = self.current_gps[1]

                # TODO choice of strategy.
                try:
                #if True:
                    rospy.loginfo("first try")
                    # Update the model.
                    #SANDEEP
                    if not self.world.update():
                        continue
                    #self.world.update()
                    print "WORLD UPDATE"

                    print "SELECT LOCATIONS"
                    best_candidate = self.world.findMaxVariance()
                    # According to the world, select a pose.
                    msg.status = 0

                except:
                    traceback.print_exc(file=sys.stdout)
                    rospy.loginfo("third try")
                    best_candidate_index_x = random.randint(0, self.world.width-1)
                    best_candidate_index_y = random.randint(0, self.world.height-1)
                    best_candidate = self.world.gps_corresponding_to_cell(best_candidate_index_x,best_candidate_index_y)
                    msg.status = 1
            
                msg.dest_lat = best_candidate.latitude
                msg.dest_lon = best_candidate.longitude
                print "GOTO", best_candidate
                # Go to.
                rospy.loginfo("Call goto service: lat {} lon {}".format(best_candidate.latitude, 
                  best_candidate.longitude))
                print "goto SERVICE"
                rospy.wait_for_service('goto')
                print "goto SERVICE available"
                gotoWaypoint = rospy.ServiceProxy('goto', Goto)
                resp = gotoWaypoint(best_candidate.latitude, best_candidate.longitude) # lat, lon.
                if resp:
                    result = True
                else:
                    rospy.logerr("goto service failed.")
                    result = False
                if result:
                    self.state = 1
                self.time_last_exploration_plan_computed = rospy.get_rostime()
            elif self.state == 1:
                current_time = rospy.get_rostime()
                if (current_time.secs - self.time_last_exploration_plan_computed.secs) > self.time_threshold:
                    # TODO(alberto) ensure that controller stops.
                    self.state = 0
            r.sleep()    

        
def main():
    rospy.init_node('maxima_sampling', anonymous=False)  #initializing the node
    #latitude, longitude = (13.19131, -59.64222)
    #width, height, spacing, orientation = 90.0, 100.0, 2.0, 0.0
    #latitude, longitude = (33.44481, -118.48498)
    #width, height, spacing, orientation = 72.2376, 35.6616, 2.0, -0.6084887621843595
    #latitude, longitude = (37.2919, -107.84686)
    #width, height, spacing, orientation = 50.557, 50, 2.0, -0.6084887621843595
    #latitude, longitude = 37.23831,-107.90936
    #width, height, spacing, orientation = 130, 150, 5.0, (numpy.pi/7)

    #Adding changes for new simulated data
    #latitude, longitude = 37.23771, -107.90995 #37.2377, -107.90998 #37.23791, -107.90921
    #width, height, spacing, orientation = 80, 90, 5.0, 0.0 # 85, 95, 5.0, 0.0
    #BBDS082018
    latitude, longitude = 13.19135, -59.64208
    width, height, spacing, orientation = 90, 90, 3.0, 0
    # positive angle: counterclockwise
    #This was used with Nighthorse_inlet_turbid3.mat
    #width, height, spacing, orientation = 45, 70, 5.0, (numpy.pi/4)

    #Sandeep: Variance should have higher priority, hence higher weights. 
    #Because we are using (1-variance) in the cost function for evaluation.
    weights = [0.3, 0.7]
    scale_factors = [10.5 * numpy.sqrt(2), 1.0]
    exploration = Exploration(latitude, longitude, width, height, spacing, orientation, weights, scale_factors)
    exploration.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: 
        pass
