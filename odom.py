#!/usr/bin/env python

"""
This node receives the motors telemetry from the espeleorobo and calculates the odometry based on the wheels velocity

It subscribes to the wheels velocities in ros_eposmcd/motor1 to motor6, published by ros_eposmcd
It publishes the odometry to odom topic

It can calculate the odometry using differential or skidsteering kinematic models, 
just change the flag skid_steer to 1 if you want skid steer or to 0 if you want differential

The parameters used both for robot and skidsteer come from Eduardo Cota master thesis
"""

from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState


class odometry:
    def __init__(self):

        # Kinematic model
        self.skid_steer = 0

        # robot parameters, from cota thesis
        self.wheel_diameter = 0.30
        self.wheel_radius = self.wheel_diameter / 2
        self.robot_width = 0.43*2
        self.robot_width_internal = 0.35
        self.robot_width_external = 0.48

        # Skidsteer parameters, from cota thesis
        self.alpha_skid = 0.9838227539528335
        self.ycir_skid = 0.3045030420948333 

        # Robot Pose
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Motor velocities
        self.motor_velocity1 = 0
        self.motor_velocity2 = 0
        self.motor_velocity3 = 0
        self.motor_velocity4 = 0
        self.motor_velocity5 = 0
        self.motor_velocity6 = 0

        self.reduction = 111

        self.ros_init()

    def ros_init(self):

        # Initialize node
        rospy.init_node('odometry_publisher', anonymous=True)

        # Times used to integrate velocity to pose
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        # Create subscribers that receives the wheels velocities
        self.subscriber_motor1 = rospy.Subscriber("/device1/get_joint_state", JointState, self.motor1_callback)
        self.subscriber_motor2 = rospy.Subscriber("/device2/get_joint_state", JointState, self.motor2_callback)
        self.subscriber_motor3 = rospy.Subscriber("/device3/get_joint_state", JointState, self.motor3_callback)
        self.subscriber_motor4 = rospy.Subscriber("/device4/get_joint_state", JointState, self.motor4_callback)
        self.subscriber_motor5 = rospy.Subscriber("/device5/get_joint_state", JointState, self.motor5_callback)
        self.subscriber_motor6 = rospy.Subscriber("/device6/get_joint_state", JointState, self.motor6_callback)

        # odom publisher
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

        # Tf broadcaster
        self.odom_broadcaster = tf.TransformBroadcaster()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


    # Motor velocity callbacks
    def motor1_callback(self, message):
        self.motor_velocity1 = message.velocity[0]

    def motor2_callback(self, message):
        self.motor_velocity2 = message.velocity[0]

    def motor3_callback(self, message):
        self.motor_velocity3 = message.velocity[0]

    def motor4_callback(self, message):
        self.motor_velocity4 = message.velocity[0]

    def motor5_callback(self, message):
        self.motor_velocity5 = message.velocity[0]

    def motor6_callback(self, message):
        self.motor_velocity6 = message.velocity[0]

        # This is the last motor to have its telemetry sent, so, we use it to call the odometry calculation
        self.odometry_calculation()

    def odometry_calculation(self):

        self.current_time = rospy.Time.now()

        # velocities of each side of the robot, the average of the wheels velocities in RPM
        velocity_right_rpm = (self.motor_velocity1 + self.motor_velocity2 + self.motor_velocity3)/(self.reduction * 3)
        velocity_left_rpm = (self.motor_velocity4 + self.motor_velocity5 + self.motor_velocity6)/(self.reduction * 3)

        # RPM to rad/s, and multiplying by the wheel_radius
        # Changed RPM to rad/s constant value from  0.10471675688 to 0.10471975511965977 -> (2*pi)/60
        velocity_right = (0.10471975511965977) * velocity_right_rpm * self.wheel_radius
        velocity_left = (0.10471975511965977) * velocity_left_rpm * self.wheel_radius
        
        if self.skid_steer:
            ### Skid-Steering model

            # Linear velocity
            v_robot = (velocity_right + velocity_left)*(self.alpha_skid/2)

            # Angular velocity
            w_robot = (velocity_right - velocity_left)*(self.alpha_skid/(2*self.ycir_skid))

            # Velocity in the XY plane
            x_robot = v_robot * cos(self.th)
            y_robot = v_robot * sin(self.th)

            # Calculating odometry
            dt = (self.current_time - self.last_time).to_sec()
            delta_x = (x_robot * cos(self.th) - y_robot * sin(self.th)) * dt
            delta_y = (x_robot * sin(self.th) + y_robot * cos(self.th)) * dt
            delta_th = w_robot * dt

            # Integrating pose
            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

        else:

            """

            # Linear velocity
            v_robot = (velocity_right + velocity_left) / 2

            # Angular velocity
            w_robot = (velocity_right - velocity_left) / self.robot_width

            ### Differential model

            # Velocity in the XY plane
            x_robot = v_robot * cos(self.th)
            y_robot = v_robot * sin(self.th)


            # Calculating odometry
            dt = (self.current_time - self.last_time).to_sec()
            delta_x = (x_robot * cos(self.th) - y_robot * sin(self.th)) * dt
            delta_y = (x_robot * sin(self.th) + y_robot * cos(self.th)) * dt
            delta_th = w_robot * dt

            # Integrating pose
            self.x += delta_x
            self.y += delta_y
            self.th += delta_th
            """


            # Odometry by geometry

            """
            dt = (self.current_time - self.last_time).to_sec()

            d_center = (velocity_right + velocity_left)*dt / 2
            delta_th = (velocity_right - velocity_left)*dt / self.robot_width
            r_center = d_center/delta_th

            self.x += r_center * (-sin(self.th) + sin(delta_th)*cos(self.th) + sin(self.th)*cos(delta_th))
            self.y += r_center * (cos(self.th) - cos(delta_th)*cos(self.th) + sin(self.th)*sin(delta_th))
            self.th += delta_th

            """


            # Odometry by dead reckoning

            
            dt = (self.current_time - self.last_time).to_sec()
            L = self.robot_width_external/2
            v_robot = (self.wheel_radius / 2) * (velocity_right + velocity_left)
            w_robot = (self.wheel_radius / 2) * (velocity_right / L - velocity_left / L )
            
            if (velocity_right == velocity_left):
                self.th = self.th
                self.x += v_robot * dt * cos(self.th)
                self.y += v_robot * dt * sin(self.th)
            elif (velocity_right == -velocity_left):
                self.th += w_robot * dt
                self.x = self.x
                self.y = self.y
            else:
                r_center = v_robot / w_robot
                delta_th = w_robot * dt
                self.x += r_center * (sin(self.th + delta_th) - sin(self.th))
                self.y += r_center * (cos(self.th + delta_th) - cos(self.th))
                self.th += delta_th
            
            



        # Since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # First, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(x_robot, y_robot, 0), Vector3(0, 0, w_robot))

        # Calculating the covariance based on the angular velocity
        # if the robot is rotating, the covariance is higher than if its going straight
        if w_robot>abs(0.2):
            covariance_cons = 0.3
        else:
            covariance_cons = 0.05

        odom.pose.covariance[0] = covariance_cons
        odom.pose.covariance[7] = covariance_cons
        odom.pose.covariance[35] = covariance_cons

        odom.pose.covariance[1] = covariance_cons
        odom.pose.covariance[6] = covariance_cons

        odom.pose.covariance[31] = covariance_cons
        odom.pose.covariance[11] = covariance_cons

        odom.pose.covariance[30] = covariance_cons
        odom.pose.covariance[5] = covariance_cons

        odom.pose.covariance[14] = 0.1
        odom.pose.covariance[21] = 0.1
        odom.pose.covariance[28] = 0.1

        odom.twist.covariance[0] = covariance_cons
        odom.twist.covariance[7] = covariance_cons
        odom.twist.covariance[35] = covariance_cons

        odom.twist.covariance[1] = covariance_cons
        odom.twist.covariance[6] = covariance_cons

        odom.twist.covariance[31] = covariance_cons
        odom.twist.covariance[11] = covariance_cons

        odom.twist.covariance[30] = covariance_cons
        odom.twist.covariance[5] = covariance_cons

        odom.twist.covariance[14] = 0.1
        odom.twist.covariance[21] = 0.1
        odom.twist.covariance[28] = 0.1



        # publish the message
        self.odom_pub.publish(odom)

        self.last_time = self.current_time

if __name__ == '__main__':
    odometry_obj = odometry()
