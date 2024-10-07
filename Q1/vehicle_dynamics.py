
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
import numpy as np

class Dynamics():

    """
    
    
    """

    def __init__(self, vehicle, timestep=0.05):
        self.vehice = vehicle
        self.acceleration = self.get_acceleration(self.vehicle)
        self.timestep = timestep

    def get_position(vehicle: carla.Vehicle) -> carla.Location:
        """
        
        """
        transform = vehicle.get_transform()
        return transform.location
    
    def get_velocity(vehicle: carla.Vehicle) -> carla.Vector3D:
        """
        
        """
        velocity = vehicle.get_velocity() #meters/sec
        return velocity
    
    def get_acceleration(vehicle: carla.Vehicle) -> carla.Vector3D:
        """
        
        """
        acceleration = vehicle.get_acceleration() #meter/sec^2
        return acceleration

    def get_jerk(vehicle: carla.Vehicle) -> carla.Vector3D:
        """
        
        """
        return

    def get_true_distance(ego_vehicle: carla.Vehicle, stationary_vehicle: carla.Vehicle, ego_dims: list, stationary_dims: list) -> float:
        """
        
        """
        ego_location = ego_vehicle.get_location()
        stationary_location = stationary_vehicle.get_location()
        distance_between_centers = ego_location.distance(stationary_location)

        return distance_between_centers - (ego_dims[0] / 2) - (stationary_dims[0] / 2)