#!/usr/bin/env python

from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    def __init__(self, wheel_base, wheel_radius, vehicle_mass, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.vehicle_mass = vehicle_mass
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

    def control(self, target_linear_vel, target_angular_vel, current_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        min_speed = min(current_vel, target_linear_vel)
        
        yaw_controller = YawController(
            self.wheel_base, self.steer_ratio, min_speed, self.max_lat_accel, self.max_steer_angle
        )
        steer = yaw_controller.get_steering(
            target_linear_vel, target_angular_vel, current_vel
        )
        if target_linear_vel > current_vel:
            # https://discussions.udacity.com/t/how-to-convert-between-throttle-output-and-acceleration/249092/4
            throttle = 0.1*target_linear_vel
            brake = 0
        else :
            # T = N* m = accler * mass of the car * wheel radius
            # https://github.com/khushn/CarND-Capstone/blob/master/controller_questions.md#brake-values-passed-to-publish-should-be-in-units-of-torque-nm-the-correct-values-for-brake-can-be-computed-using-the-desired-acceleration-weight-of-the-vehicle-and-wheel-radius
            brake = .224 * self.vehicle_mass * self.wheel_radius
            throttle = 0
        return throttle, brake, steer