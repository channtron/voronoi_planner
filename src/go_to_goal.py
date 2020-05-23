#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
import tf.transformations
from planner.srv import GoTo

class Robot:

    def __init__(self):
        # Creates a node with name 'robot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('robot_controller', anonymous=True)

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/odom'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/odom',
                                                Odometry, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

        self.goal_srv = rospy.Service('goto', GoTo, self.goto_callback)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        
        # mellodic
        self.pose.x = round(data.pose.pose.position.x, 4)
        self.pose.y = round(data.pose.pose.position.y, 4)
        # kinetic
        # self.pose.x = round(data.pose.pose.position.x-7.5, 4)
        # self.pose.y = round(data.pose.pose.position.y-4.5, 4)
	    # print('x: ' + str(self.pose.x) + ' y: ' + str(self.pose.y))
        
        orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        (_,_,yaw) = tf.transformations.euler_from_quaternion(orientation)
        
        self.pose.theta = yaw

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=0.5):
        
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=5):
        
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def goto_callback(self,req):

        goal_pose = Pose()
        goal_pose.x = req.x
        goal_pose.y = req.y

        print('Going to goal: ' + str(goal_pose.x) + ',' + str(goal_pose.y))    
         
        self.move2goal(goal_pose, req.dist_tolerance)

        return True

    def move2goal(self, goal_pose, distance_tolerance):
        """Moves the robot to the goal."""
        
        vel_msg = Twist()
        # Desacoplo del control en velocidad angular del control en velocidad lineal

        while abs(self.steering_angle(goal_pose) - self.pose.theta) >= 0.2:
            # Linear velocity in the x-axis.
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        x = Robot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
