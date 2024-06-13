#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import numpy as np

class IMUtoPWMCalibrator:
    def __init__(self):
        rospy.init_node('imu_to_pwm_calibrator', anonymous=True)
        
        # Subscribers
        rospy.Subscriber('/rov/sensors/ahrs/imu/data', Imu, self.imu_callback)

        # Publishers
        self.pwm_pub = rospy.Publisher('/servo_1', Float64, queue_size=10)

        self.angle_list = []
        self.pwm_list = []
        self.calibrated = False
        self.current_pwm = -1.0
        self.step = 0.1  # Step size for PWM values
        self.sampling = True

        self.rate = rospy.Rate(10)  # 10hz

        # Calibration constants
        self.A = 0
        self.B = 0

        # Timer to switch PWM values periodically
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        rospy.spin()

    def timer_callback(self, event):
        if self.sampling:
            self.pwm_pub.publish(self.current_pwm)
            self.current_pwm += self.step
            if self.current_pwm > 1.0:
                self.sampling = False

    def imu_callback(self, msg):
        if self.sampling:
            # Extract the pitch angle from the IMU data
            orientation_q = msg.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (_, pitch, _) = euler_from_quaternion(orientation_list)

            self.angle_list.append(pitch)
            self.pwm_list.append(self.current_pwm - self.step)  # Use the previous PWM value

            if len(self.angle_list) >= (2 / self.step) and not self.calibrated:
                self.calibrate()

        if self.calibrated:
            # Extract the pitch angle from the IMU data
            orientation_q = msg.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (_, pitch, _) = euler_from_quaternion(orientation_list)
            self.publish_pwm(pitch)

    def calibrate(self):
        # Perform a linear fit to find the relation between angle and PWM
        self.A, self.B = np.polyfit(self.angle_list, self.pwm_list, 1)

        rospy.loginfo("Calibration complete: PWM = A * angle + B")
        rospy.loginfo("A: %f, B: %f" % (self.A, self.B))
        self.calibrated = True

    def publish_pwm(self, angle):
        pwm_value = self.A * angle + self.B
        self.pwm_pub.publish(pwm_value)

def euler_from_quaternion(quat):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    quat = [x, y, z, w]
    """
    x, y, z, w = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians

if __name__ == '__main__':
    try:
        calibrator = IMUtoPWMCalibrator()
    except rospy.ROSInterruptException:
        pass
