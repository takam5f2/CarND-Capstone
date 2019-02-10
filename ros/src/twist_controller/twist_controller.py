import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
from lateral_controller import LateralController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']
        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']
        
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 0.1,
                                            self.max_lat_accel, self.max_steer_angle)

        self.last_time = rospy.get_time()

        # PID cntroller
        kp = 0.6
        ki = 0.001
        kd = 0.4
        mn = 0. # Minimum throttle value
        mx = 0.2 # Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # Low pass filter
        tau = 0.5 # 1/(2pi*tau) = cutoff frequency
        ts  = .02 # Sample time

        self.vel_lpf = LowPassFilter(tau, ts)

        self.lateral_controller = LateralController(wheel_base=self.wheel_base,
                                                    steer_ratio=self.steer_ratio,
                                                    max_lat_accel=self.max_lat_accel,
                                                    max_steer_angle=self.max_steer_angle)


    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel, cte):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            self.lateral_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)

        # rospy.logwarn("Angular vel: {0}".format(angular_vel))
        # rospy.logwarn("Target velocity : {0}".format(linear_vel))
        # rospy.logwarn("Target angular velocity: {0}\n".format(angular_vel))
        # rospy.logwarn("Current velocity: {0}".format(current_vel))
        # rospy.logwarn("Filtered velocity: {0}".format(self.vel_lpf.get()))

        # steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        steering = self.lateral_controller.control(current_vel, linear_vel, angular_vel, cte)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 400
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius
            
        return throttle, brake, steering
    
