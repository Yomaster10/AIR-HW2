#!/usr/bin/env python
'''
In order to run the assignment, open 3 terminals, source ROS Melodic and the workspace in each, and run the following commands:

Terminal 1:
$ roslaunch MRS_236609 turtlebot3_closed_room_with_2_spheres.launch

Terminal 2:
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/AIR_ws/src/MRS_236609/maps/closed_room.yaml

Terminal 3:
$ rosrun MRS_236609 assignment2.py --centers_list 2.70626 -1.70573 -3.12399 -2.98513
'''

## Original imports
import rospy
import numpy as np
import argparse
from geometry_msgs.msg import PoseWithCovarianceStamped
import dynamic_reconfigure.client
##

## Our imports
import time, random, actionlib
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
##
        
INITIAL_THRESHOLD = 2.0
QUARTER_THRESHOLD = 0.25
PADDING = 1.0

class Map:
    def __init__(self):
        self.map = None
        self.shape = None
        self.origin = None
        self.resolution = None
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.init_map)
        rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.update)
        while self.map is None:
            continue

    def init_map(self, msg):
        print('Initialising map...')
        self.shape = msg.info.height, msg.info.width
        self.map = np.array(msg.data).reshape(self.shape)
        self.origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.resolution = msg.info.resolution

    def update(self, msg):
        shape = msg.height, msg.width
        new_data = np.array(msg.data).reshape(shape)
        self.map[msg.y:msg.y + shape[0], msg.x: msg.x + shape[1]] = new_data

    def show(self):
        if not self.map is None:
            plt.imshow(self.map)
            plt.show()

    def position_to_map(self, pos):
        return (pos - self.origin) // self.resolution

class TurtleBot:
    def __init__(self):
        self.start_time = time.time()
        self.map = Map()

        # Navigation action client
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Navigation publisher (for random stepping)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
        # Pose subscriber
        pose_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=5)
        self.curr_loc = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y]
        print("Initial Position: {}".format(self.curr_loc))
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback=self.update_loc)

    def update_loc(self, msg):
        """ This function serves as the callback for the AMCL pose subscriber, getting the current XY position of the robot """
        data = msg.pose.pose
        self.curr_loc = [data.position.x, data.position.y]

    def add_goal(self, x, y, yaw=1.0):
        """ Creates a new goal with the MoveBaseGoal constructor """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = yaw
        self.action_client.send_goal(goal)

    def move(self, point, threshold):
        """ Moves the robot to the desired location using MoveBase """
        self.action_client.wait_for_server()
        self.add_goal(point[0], point[1])

        pose_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=5)
        self.curr_loc = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y]

        curr_pos = self.curr_loc
        while np.linalg.norm(np.array([point[0] - curr_pos[0], point[1] - curr_pos[1]])) >= threshold:
            self.action_client.wait_for_result(rospy.Duration(0.5))
            curr_pos = self.curr_loc
        return self.action_client.get_result()

    def quarter_turn(self, object_center, radius_est):
        pose_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=5)
        self.curr_loc = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y]

        object_vec = np.subtract(object_center, self.curr_loc)
        r = radius_est + PADDING
        theta = np.arctan2(object_vec[1], object_vec[0])
        
        new_x = r*np.cos(theta+np.pi/4)
        new_y = r*np.sin(theta+np.pi/4)
        new_point = np.add([new_x, new_y], object_center)
        return new_point

    def step(self, object_center):
        radius_estimate = None
        while radius_estimate is None:
            self.move(object_center, threshold=INITIAL_THRESHOLD)
            object_vec = np.subtract(object_center, self.curr_loc)
            norm = np.linalg.norm(object_vec, 2)
            unit_vec = object_vec / norm

            X = np.linspace(0,norm,50)
            for x in X:
                temp = x*unit_vec
                idx_x, idx_y = self.map.position_to_map(temp)
                if self.map.map[int(idx_x)][int(idx_y)] > 90:
                    radius_estimate = norm - x
                    break
         
        print("\tRadius estimate for current object: {:0.2f}".format(radius_estimate))
        for _ in range(4):
            new_point = self.quarter_turn(object_center, radius_estimate)
            self.move(new_point, threshold=QUARTER_THRESHOLD)
        return self.action_client.get_result()

    @staticmethod
    def random_step(self):
        print("\tMoving randomly...")
        vel_msg = Twist()
        vel_msg.linear.x = random.uniform(-0.1, 0.1)
        vel_msg.angular.z = random.uniform(-2, 2)
        self.pub.publish(vel_msg)

    def run(self, obj_cen, time_limit):
        """ Main function for the inspection planning, tells the robot to keep looking for objects until time runs out """
        
        print("\nBeginning Run...")
        while time.time() - self.start_time < time_limit:
            # If there are no known goal objects, make random steps until the robot finds one
            while len(obj_cen) == 0:
                self.random_step()
                # update the obj_cen list
                args = CLI.parse_args()
                flat_centers = args.centers_list
                for i in range(len(flat_centers) // 2):
                    obj_cen.append([flat_centers[2 * i], flat_centers[2 * i + 1]])

            # If there are known goal objects, inspect them all
            for i, obj in enumerate(obj_cen):
                if time.time() - self.start_time > time_limit:
                    return
                self.step(obj)
                print("\tObject {} at {} has been mapped".format(i, obj))
                #self.map.show()
            print("\nAll objects have been mapped!")
            print("Total Runtime: {}".format(time.time()-self.start_time))
        return

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    # init
    rospy.init_node('assignment2')

    CLI = argparse.ArgumentParser()
    CLI.add_argument(
        "--centers_list",
        nargs="*",
        type=float,
        default=[],
    )
    CLI.add_argument(
        "--time",
        type=float,
        default=120.0,
    )
    args = CLI.parse_args()
    flat_centers = args.centers_list
    Time = args.time
 
    centers = []
    for i in range(len(flat_centers) // 2):
        centers.append([flat_centers[2*i], flat_centers[2*i+1]])

    # update cost maps
    gcm_client = dynamic_reconfigure.client.Client("/move_base/global_costmap/inflation_layer")
    gcm_client.update_configuration({"inflation_radius": 0.1}) # changed to 0.2, otherwise move_base gets stuck
    lcm_client = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation_layer")
    lcm_client.update_configuration({"inflation_radius": 0.1}) # changed to 0.2, otherwise move_base gets stuck

    tb3 = TurtleBot()
    tb3.run(centers, Time)

    # show the mapped map
    tb3.map.show()


"""
        pose_dist = np.subtract(object_center, self.curr_loc)
        #new_point = object_center + pose_dist
        self.add_goal(new_point[0], new_point[1])#, text="opposite")
        #start_time = time.time()

        last_point = new_point
        #while np.linalg.norm(np.array([new_point[0] - pose.x, new_point[1] - pose.y])) > 1.3:
        while np.linalg.norm(np.subtract(new_point, self.curr_loc)) > 1.3:
            pose_dist = new_point - self.curr_loc #np.array([new_point[0] - pose.x, new_point[1] - pose.y])
            total_dist = np.linalg.norm(pose_dist)
            pose_sign = np.sign(pose_dist)

            if total_dist > 1.5:
                last_point = (new_point[0] + 0.5 * pose_sign[0], new_point[1] - 0.5 * pose_sign[1])
            #else:
            #    return self.action_client.get_result()

            #wait = self.action_client.wait_for_result(rospy.Duration(0.5))
            #if server_unavailable(start_time):
            #    exit(1)

        #self.add_goal(last_point[0], last_point[1], text="opposite")
        #if not self.move(last_point, threshold=0.25, start_time=start_time):
        #    exit(1)
        self.move(last_point, threshold=0.25)

        #print(new_point)
            #self.add_goal(object_center[0], object_center[1])
            #t = self.move(object_center, threshold=1.1)#, start_time=time.time())
            #print(t)
            #if not self.move(object_center, threshold=1.1, start_time=time.time()):
            #    exit(1)

            # circle the object
            #pose_dist = object_center - np.array([pose.x, pose.y])

"""


"""

def server_unavailable(start_time):
    if time.time() - start_time > 100:
        rospy.logerr("Action server is unavailable")
        rospy.signal_shutdown("Action server is unavailable")
        return True
    return False

    def move(self, point, threshold, start_time):
        global pose
        while np.linalg.norm(np.array([point[0] - pose.x, point[1] - pose.y])) > threshold:
            wait = self.action_client.wait_for_result(rospy.Duration(0.5))
            if server_unavailable(start_time):
                return False
        return True
"""
    