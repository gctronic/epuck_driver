#!/usr/bin/env python


import rospy
import numpy as np
from cv_bridge.core import CvBridge
from epuck.ePuck2 import ePuck2
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8MultiArray
from visualization_msgs.msg import Marker
import math
import tf
import time


## Camera parameters
IMAGE_FORMAT = 'RGB_565'
CAMERA_ZOOM = 8

## e-puck2 dimensions
# Wheel Radio (cm)
WHEEL_DIAMETER = 4
# Separation between wheels (cm)
WHEEL_SEPARATION = 5.3

# Distance between wheels in meters (axis length); it's the same value as "WHEEL_SEPARATION" but expressed in meters.
WHEEL_DISTANCE = 0.053
# Wheel circumference (meters).
WHEEL_CIRCUMFERENCE = ((WHEEL_DIAMETER*math.pi)/100.0)
# Distance for each motor step (meters); a complete turn is 1000 steps.
MOT_STEP_DIST = (WHEEL_CIRCUMFERENCE/1000.0)    # 0.000125 meters per step (m/steps)
ROBOT_RADIUS = 0.035 # meters.

STANDARD_GRAVITY = 9.80665
GYRO_RAW2DPS = (250.0/32768.0) # 250DPS (degrees per second) scale for int16 raw value
GYRO_RAW2RAD = GYRO_RAW2DPS * math.pi / 180.0 

# available sensors
sensors = ['imu', 'proximity', 'motor_position', 'light',
           'floor', 'camera', 'selector', 'motor_speed', 'microphone', 'distance_sensor']


class EPuck2Driver(object):
    """

    :param epuck2_name:
    :param epuck2_address:
    """

    def __init__(self, epuck2_name, epuck2_address, init_xpos, init_ypos, init_theta):
        self._bridge = ePuck2(epuck2_address, False)
        self._name = epuck2_name

        self.enabled_sensors = {s: None for s in sensors}

        self.prox_publisher = []
        self.prox_msg = []

        self.theta = init_theta
        self.x_pos = init_xpos
        self.y_pos = init_ypos
        self.leftStepsPrev = 0
        self.rightStepsPrev = 0
        self.leftStepsDiff = 0
        self.rightStepsDiff = 0
        self.deltaSteps = 0
        self.deltaTheta = 0
        self.startTime = time.time()
        self.endTime = time.time()
        self.br = tf.TransformBroadcaster()

    def greeting(self):
        """
        Hello by robot.
        """
        self._bridge.set_body_led(1)
        self._bridge.set_front_led(1)
        rospy.sleep(0.5)
        self._bridge.set_body_led(0)
        self._bridge.set_front_led(0)

    def disconnect(self):
        """
        Close bluetooth connection
        """
        self._bridge.close()

    def setup_sensors(self):
        """
        Enable epuck2 sensors based on the parameters.
        By default, all sensors are false.

        """
        # get parameters to enable sensors
        for sensor in sensors:
            self.enabled_sensors[sensor] = rospy.get_param('~' + sensor, False)
            #print rospy.get_param('~' + sensor, False)

        # Only enabled sensors
        enable = [s for s, en in self.enabled_sensors.items() if en]

        # Enable the right sensors
        self._bridge.enable(*enable)

        if self.enabled_sensors['camera']:
            self._bridge.set_camera_parameters(IMAGE_FORMAT, 40, 40, CAMERA_ZOOM)

        #if self.enabled_sensors['proximity']:
            #self._bridge.calibrate_proximity_sensors()

    def run(self):
        # Connect to the ePuck2
        self._bridge.connect()

        # Setup the necessary sensors.
        self.setup_sensors()

        # Disconnect when rospy is going to down
        rospy.on_shutdown(self.disconnect)

        self.greeting()

        self._bridge.step()

        # Subscribe to Command Velocity Topic
        rospy.Subscriber("mobile_base/cmd_vel", Twist, self.handler_velocity)

        # Subscribe to RGB topic
        rospy.Subscriber("mobile_base/rgb_leds", UInt8MultiArray, self.handler_rgb)

        # Subscribe to LEDs topic
        rospy.Subscriber("mobile_base/cmd_led", UInt8MultiArray, self.handler_leds)

        # Sensor Publishers
        # rospy.Publisher("/%s/mobile_base/" % self._name, )

        if self.enabled_sensors['camera']:
            self.image_publisher = rospy.Publisher("camera", Image, queue_size=1)

        if self.enabled_sensors['proximity']:
            for i in range(0,8):
                self.prox_publisher.append(rospy.Publisher("proximity"+str(i), Range, queue_size=1))
                self.prox_msg.append(Range())
                self.prox_msg[i].radiation_type = Range.INFRARED
                self.prox_msg[i].header.frame_id =  self._name+"/base_prox" + str(i)
                self.prox_msg[i].field_of_view = 0.26 	# About 15 degrees...to be checked!
                self.prox_msg[i].min_range = 0.005	# 0.5 cm
                self.prox_msg[i].max_range = 0.05		# 5 cm
            self.laser_publisher = rospy.Publisher("scan", LaserScan, queue_size=1)

        if self.enabled_sensors['motor_position']:
            self.odom_publisher = rospy.Publisher('odom', Odometry, queue_size=1)

        if self.enabled_sensors['imu']:
            self.imu_publisher = rospy.Publisher('imu', Imu, queue_size=1)

        if self.enabled_sensors['selector']:
            self.selector_publisher = rospy.Publisher('selector', Marker, queue_size=1)

        if self.enabled_sensors['light']:
            self.light_publisher = rospy.Publisher('light', Marker, queue_size=1)

        if self.enabled_sensors['motor_speed']:
            self.motor_speed_publisher = rospy.Publisher('motor_speed', Marker, queue_size=1)

        if self.enabled_sensors['microphone']:
            self.microphone_publisher = rospy.Publisher('microphone', Marker, queue_size=1)

        if self.enabled_sensors['floor']:
            self.floor_publisher = rospy.Publisher('floor', Marker, queue_size=1)

        if self.enabled_sensors['distance_sensor']:
			self.distance_sensor_publisher = rospy.Publisher("dist_sens", Range, queue_size=1)
			self.distance_sensor_msg = Range()
			self.distance_sensor_msg.radiation_type = Range.INFRARED
			self.distance_sensor_msg.header.frame_id =  self._name+"/base_dist_sens"
			self.distance_sensor_msg.field_of_view = 0.43 	# About 25 degrees (+/- 12.5)
			self.distance_sensor_msg.min_range = 0.005	# 5 mm
			self.distance_sensor_msg.max_range = 2		# 2 m

        # Spin almost forever
        #rate = rospy.Rate(7)   # 7 Hz. If you experience "timeout" problems with multiple robots try to reduce this value.
        self.startTime = time.time()
        while not rospy.is_shutdown():
            self._bridge.step()
            self.update_sensors()
            #rate.sleep()	# Do not call "sleep" otherwise the bluetooth communication will hang.
                            # We communicate as fast as possible, this shouldn't be a problem...

    def update_sensors(self):
        # print "proximity:", self._bridge.get_proximity()
        # print "light:", self._bridge.get_light_sensor()
        # print "motor_position:", self._bridge.get_motor_position()
        # print "floor:", self._bridge.get_floor_sensors()
        # print "image:", self._bridge.get_image()

        ## If image from camera
        if self.enabled_sensors['camera']:
            # Get Image
            image = self._bridge.get_image()
            #print image
            if image is not None:
                nimage = np.asarray(image)
                image_msg = CvBridge().cv2_to_imgmsg(nimage, "rgb8")
                self.image_publisher.publish(image_msg)

        if self.enabled_sensors['proximity']:
            prox_sensors = self._bridge.get_proximity()
            for i in range(0,8):
                if prox_sensors[i] > 0:
                    self.prox_msg[i].range = 0.5/math.sqrt(prox_sensors[i])	# Transform the analog value to a distance value in meters (given from field tests).
                else:
                    self.prox_msg[i].range = self.prox_msg[i].max_range

                if self.prox_msg[i].range > self.prox_msg[i].max_range:
                    self.prox_msg[i].range = self.prox_msg[i].max_range
                if self.prox_msg[i].range < self.prox_msg[i].min_range:
                    self.prox_msg[i].range = self.prox_msg[i].min_range
                self.prox_msg[i].header.stamp = rospy.Time.now()
                self.prox_publisher[i].publish(self.prox_msg[i])


            # e-puck proximity positions (cm), x pointing forward, y pointing left
            #           P7(3.5, 1.0)   P0(3.5, -1.0)
            #       P6(2.5, 2.5)           P1(2.5, -2.5)
            #   P5(0.0, 3.0)                   P2(0.0, -3.0)
            #       P4(-3.5, 2.0)          P3(-3.5, -2.0)
            #
            # e-puck proximity orentations (degrees)
            #           P7(10)   P0(350)
            #       P6(40)           P1(320)
            #   P5(90)                   P2(270)
            #       P4(160)          P3(200)
            self.br.sendTransform((0.035, -0.010, 0.034), tf.transformations.quaternion_from_euler(0, 0, 6.11), rospy.Time.now(), self._name+"/base_prox0", self._name+"/base_link")
            self.br.sendTransform((0.025, -0.025, 0.034), tf.transformations.quaternion_from_euler(0, 0, 5.59), rospy.Time.now(), self._name+"/base_prox1", self._name+"/base_link")
            self.br.sendTransform((0.000, -0.030, 0.034), tf.transformations.quaternion_from_euler(0, 0, 4.71), rospy.Time.now(), self._name+"/base_prox2", self._name+"/base_link")
            self.br.sendTransform((-0.035, -0.020, 0.034), tf.transformations.quaternion_from_euler(0, 0, 3.49), rospy.Time.now(), self._name+"/base_prox3", self._name+"/base_link")
            self.br.sendTransform((-0.035, 0.020, 0.034), tf.transformations.quaternion_from_euler(0, 0, 2.8), rospy.Time.now(), self._name+"/base_prox4", self._name+"/base_link")
            self.br.sendTransform((0.000, 0.030, 0.034), tf.transformations.quaternion_from_euler(0, 0, 1.57), rospy.Time.now(), self._name+"/base_prox5", self._name+"/base_link")
            self.br.sendTransform((0.025, 0.025, 0.034), tf.transformations.quaternion_from_euler(0, 0, 0.70), rospy.Time.now(), self._name+"/base_prox6", self._name+"/base_link")
            self.br.sendTransform((0.035, 0.010, 0.034), tf.transformations.quaternion_from_euler(0, 0, 0.17), rospy.Time.now(), self._name+"/base_prox7", self._name+"/base_link")

            laser_msg = LaserScan()
            laser_msg.header.stamp = rospy.Time.now()
            laser_msg.header.frame_id = self._name+"/base_laser"
            laser_msg.angle_min = -math.pi/2.0
            laser_msg.angle_max = math.pi/2.0
            laser_msg.angle_increment = math.pi/18.0 # 10 degrees.
            laser_msg.range_min = 0.005 + ROBOT_RADIUS # 0.5 cm + ROBOT_RADIUS.
            laser_msg.range_max = 0.05 + ROBOT_RADIUS # 5 cm + ROBOT_RADIUS. 
            laser_msg.ranges = [None] * 19
            laser_msg.intensities = [None] * 19

            # We use the information from the 6 proximity sensors on the front side of the robot to get 19 laser scan points. The interpolation used is the following:
            # -90 degrees: P2
            # -80 degrees: 4/5*P2 + 1/5*P1
            # -70 degrees: 3/5*P2 + 2/5*P1
            # -60 degrees: 2/5*P2 + 3/5*P1
            # -50 degrees: 1/5*P2 + 4/5*P1
            # -40 degrees: P1
            # -30 degrees: 2/3*P1 + 1/3*P0
            # -20 degrees: 1/3*P1 + 2/3*P0
            # -10 degrees: P0
            # 0 degrees: 1/2*P0 + 1/2*P7
            # 10 degrees: P7
            # 20 degrees: 1/3*P6 + 2/3*P7
            # 30 degrees: 2/3*P6 + 1/3*P7
            # 40 degrees: P6
            # 50 degrees: 1/5*P5 + 4/5*P6
            # 60 degrees: 2/5*P5 + 3/5*P6
            # 70 degrees: 3/5*P5 + 2/5*P6
            # 80 degrees: 4/5*P5 + 1/5*P6
            # 90 degrees: P5
        
            tempProx = [None] * 19
            tempProx[0] = prox_sensors[2]
            tempProx[1] = prox_sensors[2]*4/5 + prox_sensors[1]*1/5
            tempProx[2] = prox_sensors[2]*3/5 + prox_sensors[1]*2/5
            tempProx[3] = prox_sensors[2]*2/5 + prox_sensors[1]*3/5
            tempProx[4] = prox_sensors[2]*1/5 + prox_sensors[1]*4/5
            tempProx[5] = prox_sensors[1]
            tempProx[6] = prox_sensors[1]*2/3 + prox_sensors[0]*1/3
            tempProx[7] = prox_sensors[1]*1/3 + prox_sensors[0]*2/3
            tempProx[8] = prox_sensors[0]
            tempProx[9] = (prox_sensors[0]+prox_sensors[7])>>1
            tempProx[10] = prox_sensors[7]
            tempProx[11] = prox_sensors[7]*2/3 + prox_sensors[6]*1/3
            tempProx[12] = prox_sensors[7]*1/3 + prox_sensors[6]*2/3
            tempProx[13] = prox_sensors[6];
            tempProx[14] = prox_sensors[6]*4/5 + prox_sensors[5]*1/5;
            tempProx[15] = prox_sensors[6]*3/5 + prox_sensors[5]*2/5;
            tempProx[16] = prox_sensors[6]*2/5 + prox_sensors[5]*3/5;
            tempProx[17] = prox_sensors[6]*1/5 + prox_sensors[5]*4/5;
            tempProx[18] = prox_sensors[5]

            for i in range(0,19):
                if(tempProx[i] > 0):
                    laser_msg.ranges[i] = (0.5/math.sqrt(tempProx[i])) + ROBOT_RADIUS # Transform the analog value to a distance value in meters (given from field tests).
                    laser_msg.intensities[i] = tempProx[i]
                else: # Sometimes the values could be negative due to the calibration, it means there is no obstacles.
                    laser_msg.ranges[i] = laser_msg.range_max
                    laser_msg.intensities[i] = 0
                if(laser_msg.ranges[i] > laser_msg.range_max):
                    laser_msg.ranges[i] = laser_msg.range_max
                if(laser_msg.ranges[i] < laser_msg.range_min):   
                    laser_msg.ranges[i] = laser_msg.range_min

            self.laser_publisher.publish(laser_msg)
            self.br.sendTransform((0.0, 0.0, 0.034), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), self._name+"/base_laser", self._name+"/base_link")

        if self.enabled_sensors['imu']:
            accel = self._bridge.get_accelerometer()
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = self._name+"/base_link"
            imu_msg.linear_acceleration.x = (accel[1]-2048.0)/800.0*9.81 # 1 g = about 800, then transforms in m/s^2.
            imu_msg.linear_acceleration.y = (accel[0]-2048.0)/800.0*9.81
            imu_msg.linear_acceleration.z = (accel[2]-2048.0)/800.0*9.81
            imu_msg.linear_acceleration_covariance = [0.01,0.0,0.0, 0.0,0.01,0.0, 0.0,0.0,0.01]
            #print "accel raw: " + str(accel[0]) + ", " + str(accel[1]) + ", " + str(accel[2])
            #print "accel (m/s2): " + str((accel[0]-2048.0)/800.0*9.81) + ", " + str((accel[1]-2048.0)/800.0*9.81) + ", " + str((accel[2]-2048.0)/800.0*9.81)
            gyro = self._bridge.get_gyro()
            imu_msg.angular_velocity.x = gyro[0]*GYRO_RAW2RAD # rad/s
            imu_msg.angular_velocity.y = gyro[1]*GYRO_RAW2RAD
            imu_msg.angular_velocity.z = gyro[2]*GYRO_RAW2RAD
            imu_msg.angular_velocity_covariance = [0.01,0.0,0.0, 0.0,0.01,0.0, 0.0,0.0,0.01]
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            imu_msg.orientation = Quaternion(*q)
            imu_msg.orientation_covariance = [0.01,0.0,0.0, 0.0,0.01,0.0, 0.0,0.0,0.01]
            self.imu_publisher.publish(imu_msg)

        if self.enabled_sensors['motor_position']:
            motor_pos = list(self._bridge.get_motor_position()) # Get a list since tuple elements returned by the function are immutable.
            # Convert to 16 bits signed integer.
            if(motor_pos[0] & 0x8000):
                motor_pos[0] = -0x10000 + motor_pos[0]
            if(motor_pos[1] & 0x8000):
                motor_pos[1] = -0x10000 + motor_pos[1]
            #print "motor_pos: " + str(motor_pos[0]) + ", " + str(motor_pos[1])

            self.leftStepsDiff = motor_pos[0]*MOT_STEP_DIST - self.leftStepsPrev    # Expressed in meters.
            self.rightStepsDiff = motor_pos[1]*MOT_STEP_DIST - self.rightStepsPrev  # Expressed in meters.
            #print "left, right steps diff: " + str(self.leftStepsDiff) + ", " + str(self.rightStepsDiff)

            self.deltaTheta = (self.rightStepsDiff - self.leftStepsDiff)/WHEEL_DISTANCE # Expressed in radiant.
            self.deltaSteps = (self.rightStepsDiff + self.leftStepsDiff)/2  # Expressed in meters.
            #print "delta theta, steps: " + str(self.deltaTheta) + ", " + str(self.deltaSteps)

            self.x_pos += self.deltaSteps*math.cos(self.theta + self.deltaTheta/2)  # Expressed in meters.
            self.y_pos += self.deltaSteps*math.sin(self.theta + self.deltaTheta/2)  # Expressed in meters.
            self.theta += self.deltaTheta   # Expressed in radiant.
            #print "x, y, theta: " + str(self.x_pos) + ", " + str(self.y_pos) + ", " + str(self.theta)

            self.leftStepsPrev = motor_pos[0]*MOT_STEP_DIST  # Expressed in meters.
            self.rightStepsPrev = motor_pos[1]*MOT_STEP_DIST    # Expressed in meters.

            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = self._name+"/base_link"
            odom_msg.pose.pose.position = Point(self.x_pos, self.y_pos, 0)
            q = tf.transformations.quaternion_from_euler(0, 0, self.theta)
            odom_msg.pose.pose.orientation = Quaternion(*q)
            self.endTime = time.time()
            odom_msg.twist.twist.linear.x = self.deltaSteps / (self.endTime-self.startTime) # self.deltaSteps is the linear distance covered in meters from the last update (delta distance);
                                                                                            # the time from the last update is measured in seconds thus to get m/s we multiply them.
            odom_msg.twist.twist.angular.z = self.deltaTheta / (self.endTime-self.startTime)    # self.deltaTheta is the angular distance covered in radiant from the last update (delta angle);
                                                                                                # the time from the last update is measured in seconds thus to get rad/s we multiply them.
            #print "time elapsed = " + str(self.endTime-self.startTime) + " seconds"
            self.startTime = self.endTime

            self.odom_publisher.publish(odom_msg)
            pos = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z)
            ori = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
            self.br.sendTransform(pos, ori, odom_msg.header.stamp, odom_msg.child_frame_id, odom_msg.header.frame_id)

        if self.enabled_sensors['light']:
            light_sensors = self._bridge.get_light_sensor()
            light_sensors_marker_msg = Marker()
            light_sensors_marker_msg.header.frame_id = self._name+"/base_link"
            light_sensors_marker_msg.header.stamp = rospy.Time.now()
            light_sensors_marker_msg.type = Marker.TEXT_VIEW_FACING
            light_sensors_marker_msg.pose.position.x = 0.15
            light_sensors_marker_msg.pose.position.y = 0
            light_sensors_marker_msg.pose.position.z = 0.15
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            light_sensors_marker_msg.pose.orientation = Quaternion(*q)
            light_sensors_marker_msg.scale.z = 0.01
            light_sensors_marker_msg.color.a = 1.0
            light_sensors_marker_msg.color.r = 1.0
            light_sensors_marker_msg.color.g = 1.0
            light_sensors_marker_msg.color.b = 1.0
            light_sensors_marker_msg.text = "light: " + str(light_sensors)
            self.light_publisher.publish(light_sensors_marker_msg)

        if self.enabled_sensors['floor']:
            floor_sensors = self._bridge.get_floor_sensors()
            floor_marker_msg = Marker()
            floor_marker_msg.header.frame_id = self._name+"/base_link"
            floor_marker_msg.header.stamp = rospy.Time.now()
            floor_marker_msg.type = Marker.TEXT_VIEW_FACING
            floor_marker_msg.pose.position.x = 0.15
            floor_marker_msg.pose.position.y = 0
            floor_marker_msg.pose.position.z = 0.13
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            floor_marker_msg.pose.orientation = Quaternion(*q)
            floor_marker_msg.scale.z = 0.01
            floor_marker_msg.color.a = 1.0
            floor_marker_msg.color.r = 1.0
            floor_marker_msg.color.g = 1.0
            floor_marker_msg.color.b = 1.0
            floor_marker_msg.text = "floor: " + str(floor_sensors)
            self.floor_publisher.publish(floor_marker_msg)

        if self.enabled_sensors['selector']:
            curr_sel = self._bridge.get_selector()
            selector_marker_msg = Marker()
            selector_marker_msg.header.frame_id = self._name+"/base_link"
            selector_marker_msg.header.stamp = rospy.Time.now()
            selector_marker_msg.type = Marker.TEXT_VIEW_FACING
            selector_marker_msg.pose.position.x = 0.15
            selector_marker_msg.pose.position.y = 0
            selector_marker_msg.pose.position.z = 0.11
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            selector_marker_msg.pose.orientation = Quaternion(*q)
            selector_marker_msg.scale.z = 0.01
            selector_marker_msg.color.a = 1.0
            selector_marker_msg.color.r = 1.0
            selector_marker_msg.color.g = 1.0
            selector_marker_msg.color.b = 1.0
            selector_marker_msg.text = "selector: " + str(curr_sel)
            self.selector_publisher.publish(selector_marker_msg)

        if self.enabled_sensors['motor_speed']:
            motor_speed = list(self._bridge.get_motor_speed()) # Get a list since tuple elements returned by the function are immutable.
            # Convert to 16 bits signed integer.
            if(motor_speed[0] & 0x8000):
                motor_speed[0] = -0x10000 + motor_speed[0]
            if(motor_speed[1] & 0x8000):
                motor_speed[1] = -0x10000 + motor_speed[1]
            motor_speed_marker_msg = Marker()
            motor_speed_marker_msg.header.frame_id = self._name+"/base_link"
            motor_speed_marker_msg.header.stamp = rospy.Time.now()
            motor_speed_marker_msg.type = Marker.TEXT_VIEW_FACING
            motor_speed_marker_msg.pose.position.x = 0.15
            motor_speed_marker_msg.pose.position.y = 0
            motor_speed_marker_msg.pose.position.z = 0.09
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            motor_speed_marker_msg.pose.orientation = Quaternion(*q)
            motor_speed_marker_msg.scale.z = 0.01
            motor_speed_marker_msg.color.a = 1.0
            motor_speed_marker_msg.color.r = 1.0
            motor_speed_marker_msg.color.g = 1.0
            motor_speed_marker_msg.color.b = 1.0
            motor_speed_marker_msg.text = "speed: " + str(motor_speed)
            self.motor_speed_publisher.publish(motor_speed_marker_msg)

        if self.enabled_sensors['microphone']:
            mic = self._bridge.get_microphone()
            microphone_marker_msg = Marker()
            microphone_marker_msg.header.frame_id = self._name+"/base_link"
            microphone_marker_msg.header.stamp = rospy.Time.now()
            microphone_marker_msg.type = Marker.TEXT_VIEW_FACING
            microphone_marker_msg.pose.position.x = 0.15
            microphone_marker_msg.pose.position.y = 0
            microphone_marker_msg.pose.position.z = 0.07
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            microphone_marker_msg.pose.orientation = Quaternion(*q)
            microphone_marker_msg.scale.z = 0.01
            microphone_marker_msg.color.a = 1.0
            microphone_marker_msg.color.r = 1.0
            microphone_marker_msg.color.g = 1.0
            microphone_marker_msg.color.b = 1.0
            microphone_marker_msg.text = "microphone: " + str(mic)
            self.microphone_publisher.publish(microphone_marker_msg)

        if self.enabled_sensors['distance_sensor']:
			self.distance_sensor_msg.range = self._bridge.get_distance_sensor()[0]/1000.0
			if self.distance_sensor_msg.range > self.distance_sensor_msg.max_range:
				self.distance_sensor_msg.range = self.distance_sensor_msg.max_range
			if self.distance_sensor_msg.range < self.distance_sensor_msg.min_range:
				self.distance_sensor_msg.range = self.distance_sensor_msg.min_range
			self.distance_sensor_msg.header.stamp = rospy.Time.now()
			self.distance_sensor_publisher.publish(self.distance_sensor_msg)
			self.br.sendTransform((0.035, 0, 0.034), tf.transformations.quaternion_from_euler(0, -0.21, 0), rospy.Time.now(), self._name+"/base_dist_sens", self._name+"/base_link")

    def handler_velocity(self, data):
        """
        Controls the velocity of each wheel based on linear and angular velocities.
        :param data:
        """
        linear = data.linear.x
        angular = data.angular.z

        # Kinematic model for differential robot.
        wl = (linear - (WHEEL_SEPARATION / 2.) * angular) / WHEEL_DIAMETER
        wr = (linear + (WHEEL_SEPARATION / 2.) * angular) / WHEEL_DIAMETER

        # At input 1000, angular velocity is 1 cycle / s or  2*pi/s.
        left_vel = wl * 1000.
        right_vel = wr * 1000.
        self._bridge.set_motors_speed(left_vel, right_vel)

    def handler_rgb(self, data):
        """
        Set the RGB LEDs color based on data received.
        :param data: r,g,b values for each LEDs in sequence
        """
        #print(data)
        self._bridge.set_rgb_leds(ord(data.data[0]),ord(data.data[1]),ord(data.data[2]), ord(data.data[3]),ord(data.data[4]),ord(data.data[5]), ord(data.data[6]),ord(data.data[7]),ord(data.data[8]), ord(data.data[9]),ord(data.data[10]),ord(data.data[11]))

    def handler_leds(self, data):
        """
        Controls the leds: led0, led2, led4, led6, body led, front led.
        :param data:
        """
        self._bridge.set_led(0, ord(data.data[0]))
        self._bridge.set_led(2, ord(data.data[1]))
        self._bridge.set_led(4, ord(data.data[2]))
        self._bridge.set_led(6, ord(data.data[3]))
        self._bridge.set_body_led(ord(data.data[4]))
        self._bridge.set_front_led(ord(data.data[5]))

def run():
    rospy.init_node("epuck2_drive", anonymous=True)

    epuck2_address = rospy.get_param("~epuck2_address")
    epuck2_name = rospy.get_param("~epuck2_name", "epuck2")
    init_xpos = rospy.get_param("~xpos", 0.0)
    init_ypos = rospy.get_param("~ypos", 0.0)
    init_theta = rospy.get_param("~theta", 0.0)
    #print "init x, y, th: " + str(init_xpos) + ", " + str(init_ypos) + ", " + str(init_theta)

    EPuck2Driver(epuck2_name, epuck2_address, init_xpos, init_ypos, init_theta).run()


if __name__ == "__main__":
    run()
