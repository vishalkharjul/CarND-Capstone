#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import tf

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        # List of positions that correspond to the line to stop in front of for a given intersection
        self.stop_line_positions = self.config['stop_line_positions']
        self.stop_line_wps = []

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()

        carla_run = True if rospy.get_param('~carla_run', 0) == 1 else False
        self.TL_DISTANCE_THRESHOLD = rospy.get_param('~tl_distance_thresh', 20) 
        self.light_classifier = TLClassifier(carla_run)
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.last_wp_ind = None

        rospy.spin()

    def pose_cb(self, poseStamped):
        self.pose = poseStamped

    def waypoints_cb(self, lane):
        if self.waypoints == None or len(self.stop_line_wps) < len(self.stop_line_positions):
            self.waypoints = lane.waypoints
            #Use bit of DP, so that we don't have to calculate the 
            #way points for stop lights
            n = len(lane.waypoints)
            for i in range(len(self.stop_line_positions)):
                stop_x = self.stop_line_positions[i][0]
                stop_y = self.stop_line_positions[i][1]
                min_distance = float("inf")
                min_wp=-1
                for i in range(n):
                    waypoint = self.waypoints[i]
                    waypoint_x = waypoint.pose.pose.position.x
                    waypoint_y = waypoint.pose.pose.position.y
                    dist = math.sqrt((stop_x -waypoint_x)**2  + (stop_y -waypoint_y)**2)
                    if dist < min_distance:
                        min_distance = dist
                        min_wp = i
                self.stop_line_wps.append(min_wp)


    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        if self.waypoints == None:
            return -1

        posex = pose.position.x
        posey = pose.position.y
        min_distance = float("inf")
        min_dist_loc=-1
        start_index=0
        num_pts = len(self.waypoints)
        num_inds = num_pts
        if self.last_wp_ind != None:
            start_index = self.last_wp_ind
            num_inds=20 # We are sampling at 50hz. Looking 20 ahead should be enough         
        for i in range(num_inds):
            ind = (start_index+i)%num_pts
            waypoint = self.waypoints[ind]
            waypoint_x = waypoint.pose.pose.position.x
            waypoint_y = waypoint.pose.pose.position.y
            # dist between car and waypoint
            dist = math.sqrt((posex -waypoint_x)**2  + (posey -waypoint_y)**2)
            
            # check if this distance is minimum distance calculated so far
            if dist < min_distance:
                min_distance = dist
                min_dist_loc = ind 

        return min_dist_loc

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

         #dummy implementation for simulator for now
        #return light.state


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.pose == None:
            return -1, TrafficLight.UNKNOWN

        light = None
        
        car_position = self.get_closest_waypoint(self.pose.pose)
        near_wp = self.waypoints[car_position]
        waypoint_x = near_wp.pose.pose.position.x
        waypoint_y = near_wp.pose.pose.position.y

        # Get Car;s orientation Convert Quaternions to Euler
        # We want to check only if the stop line is ahead of the car
        orient = near_wp.pose.pose.orientation
        q = [orient.x, orient.y, orient.z, orient.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(q)

        #find the closest visible traffic light (if one exists)
        min_distance = float("inf")
        tl_stop_wp=-1
        for i in range(len(self.stop_line_positions)):
            stop_x = self.stop_line_positions[i][0]
            stop_y = self.stop_line_positions[i][1]
            # Convert waypoint to car's reference
            wp_x = stop_x - waypoint_x
            wp_y = stop_y - waypoint_y
            car_coord_x = wp_x*math.cos(yaw) + wp_y*math.sin(yaw)

            dist = math.sqrt((stop_x -waypoint_x)**2  + (stop_y -waypoint_y)**2)
            if car_coord_x > 0 and dist < self.TL_DISTANCE_THRESHOLD and dist < min_distance:
                min_distance = dist
                light=self.lights[i]
                tl_stop_wp = self.stop_line_wps[i]



        if light:
            state = self.get_light_state(light)            
            return tl_stop_wp, state
        
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
