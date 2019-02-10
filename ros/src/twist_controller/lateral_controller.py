import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

class LateralController(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']
        
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 0.1,
                                            self.max_lat_accel, self.max_steer_angle)
        self.last_time = rospy.get_time()

        # PID controller
        kp = 0.5
        ki = 0.00001
        kd = 0.25
        mn = -self.max_steer_angle # Minimum throttle value
        mx = self.max_steer_angle # Maximum throttle value
        self.pid_controller = PID(kp, ki, kd, mn, mx)

    def reset(self):
        self.pid_controller.reset()

    def control(self, current_vel, linear_vel, angular_vel, cte):

        steering_yaw = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        steering_pid = self.pid_controller.step(cte, sample_time)
        rospy.logwarn("x track error: {0}".format(cte))
        rospy.logwarn("Yaw Steering: {0}".format(steering_yaw))
        rospy.logwarn("PID Steering : {0}".format(steering_pid))

        return steering_yaw + steering_pid
        

        

        
