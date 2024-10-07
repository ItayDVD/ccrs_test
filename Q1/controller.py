import glob
import os
import sys


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import math

class Controller():

    def get_controls(current_distance: float, current_velocity: float, target_distance = 1, brake_threshold = 10, max_velocity = math.inf):
        """
        Computes vehicle control commands (throttle, break inputs) for the ego vehicle based on the relative distance to the target vehicle.

        args:
            -current_distance (float): The current distance from the ego vehicle to the target stationary vehicle (meters).
            -current_velocity (float): The current velocity of the ego vehicle (meters/second).
            -target_distance (float, optional): The desired distance between the ego car and the preceding vehicle. The ego car will decelerate until it reaches and this distance from the stationary car.
            -break_threshold (float, optional): Minimum distance between the ego carand preceding vehicle to apply brakes.
            -max_velocity (float, optional): Maximum velocity for ego vehicle. If surpassed throttle will be set to 0.

        Returns:
            carla.VehicleControl as defined via the carla PythonAPI.
            see https://carla.readthedocs.io/en/latest/python_api/#carlavehiclecontrol

        Notes:
            It is a possibility to later add additional control features such as gain for throttling / braking.
        """
    
        if current_distance is None or current_velocity is None:
            raise ValueError("Distance and velocity must be set before computing controls.")

        distance_to_stop = current_distance - target_distance

        if distance_to_stop > brake_threshold:
            # Let it rip baby!
            throttle_input = 1.0
            brake_input = 0.0
        else:
            throttle_input = min(1.0, distance_to_stop / 100.0)
            brake_input = 1.0 - throttle_input
        
        if current_velocity > max_velocity:
            throttle_input = 0.0

        return carla.VehicleControl(throttle=throttle_input, steer=0.0, brake=brake_input, hand_brake=False, reverse=False, manual_gear_shift=False)
        


    