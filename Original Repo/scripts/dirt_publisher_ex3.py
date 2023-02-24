#!/usr/bin/env python
import rospy
import math
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import yaml
import os


class DirtPublisher:

    def __init__(self, num_of_agents, radius, dirt_pieces=None):
        if dirt_pieces is None:
            dirt_pieces = []
        self.dirt_pub = rospy.Publisher('dirt', String, queue_size=10, latch=True)
        self.dirt_pieces = dirt_pieces
        self.odom_subsribers = []
        self.num_of_agents = num_of_agents
        # initialize the array with zeros (numpy zeros)
        self.collected_per_agent = np.zeros(num_of_agents)

        for i in range(0, self.num_of_agents):
            self.odom_subsribers.append(rospy.Subscriber('/tb3_%d/odom' % i, Odometry, self.update_dirt_status))

        self.radius = radius

    def run(self):
        # Main while loop
        while not rospy.is_shutdown():

            print('\ncurrent status: agent_0 collected %d and agent_1 collected %d remaining dirt:' % (
                self.collected_per_agent[0], self.collected_per_agent[1]))
            print(self.dirt_pieces)
            self.publish_objects()
            if len(self.dirt_pieces) == 0:
                print('all dirt collected. exiting')
                exit(0)
            self.publish_objects()
            rospy.sleep(3)

    def publish_objects(self):

        self.dirt_pub.publish(''.join([str(x) for x in self.dirt_pieces]))

    def update_dirt_status(self, msg):

        if self.num_of_agents == 1:
            # odom
            agent_id = 0
            pose_x = msg.pose.pose.position.x
            pose_y = msg.pose.pose.position.y

        else:
            parsed_id = (msg.header.frame_id.replace('tb3_', '')).replace('/odom', '')
            agent_id = int(parsed_id)
            pose_x = msg.pose.pose.position.x
            pose_y = msg.pose.pose.position.y

        current_dirt_pieces = []
        for cur_dirt in self.dirt_pieces:
            dirt_x = float(cur_dirt[0])
            dirt_y = float(cur_dirt[1])
            # euclidean distance
            dist = math.sqrt(math.pow((dirt_x - pose_x), 2) + math.pow((dirt_y - pose_y), 2))
            # print('dist is %f and radius is %f'%(dist,self.radius))

            if dist < self.radius:

                print('\ndirt piece (%f %f) collected by agent %d' % (dirt_x, dirt_y, agent_id))
                # add to the agent's list of collected items
                self.collected_per_agent[agent_id] += 1
            else:  # not collected
                current_dirt_pieces.append([dirt_x, dirt_y])

        self.dirt_pieces = current_dirt_pieces


if __name__ == '__main__':
    rospy.init_node('dirt_publisher')
    path_to_yaml = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/config/dirt_config.yaml"
    if not os.path.exists(path_to_yaml):
        dirt_dict = {'pieces': [(2.4, 0.3), (0.5, 0.2), (0, 0.1)], 'number_agents': 2, 'radius': 1.0}
        with open(path_to_yaml, 'w') as f:
            yaml.dump(dirt_dict, f)
            data = dirt_dict
    else:
        with open(path_to_yaml, 'r') as f:
            data = yaml.load(f)
    num_of_agents = data['number_agents']
    dirt_pieces = data['pieces']  # '[0.3:0.4],[0.5:0.2],[0:0.1]'
    radius = data['radius']
    try:
        dirt_pub = DirtPublisher(num_of_agents, radius, dirt_pieces)
        dirt_pub.run()
    except rospy.ROSInterruptException:
        pass
