
# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================

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


# ==============================================================================
# -- main  ---------------------------------------------------------------------
# ==============================================================================

from sync_mode import CarlaSyncMode
from controller import Controller
from vehicle_dynamics import Dynamics
from utility import *

import carla
import pygame
import csv
import numpy as np

from constants import TARGET_DISTANCE, BRAKE_THRESHOLD, MAX_VELOCITY


def main():
    
    pygame.init()
    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)

    world = client.load_world('/Game/Carla/Maps/Town02')

    actors_list = []
    saved_data = []

    # Set the maximum number of frames
    max_frames = 500  # Change this to your desired number of frames
    frame_count = 0

    try:
        # Spawn vehicles
        ego_vehicle_startpoint = carla.Transform(carla.Location(x=1.8, y= 10.0, z=0.3), 
                                                    carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))
        stationary_vehicle_startpoint = carla.Transform(carla.Location(x=1.8, y=110.0, z=0.3), 
                                            carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))

        
        ego_vehicle = spawn_vehicle(world, ego_vehicle_startpoint)
        print('created  %s' % ego_vehicle.type_id)
        
        stationary_vehicle = spawn_vehicle(world, stationary_vehicle_startpoint)
        print('created  %s' % stationary_vehicle.type_id)

        # Get vehicle dimensions
        ego_vehicle_dims = get_vehicle_dimensions(ego_vehicle)
        stationary_vehicle_dims = get_vehicle_dimensions(stationary_vehicle)

        # Time for the ego vehicle to hit the gas
        ego_vehicle.set_simulate_physics(True)
        ego_vehicle.set_autopilot(True)

        # Add sensors (RGB camera)
        camera = spawn_camera(world, ego_vehicle, ego_vehicle_dims, view_width=720, view_height=576, view_fov=60)
        print('created %s' % camera.type_id)

        # Append vehicle and sensors to actors list
        actors_list.extend([ego_vehicle, stationary_vehicle, camera])

        # Run the simulation in syncronize mode
        with CarlaSyncMode(world, camera, fps=20) as sync_mode:
            while True:
                    
                clock.tick()

                # Advance the simulation and wait for data
                data = sync_mode.tick(timeout=1.0)

                # Calculate distance between vehicles
                distance = Dynamics.get_true_distance(ego_vehicle, 
                                        stationary_vehicle, ego_vehicle_dims, 
                                        stationary_vehicle_dims)
                # Calculate control inputs
                velocity = ego_vehicle.get_velocity()
                acceleration = ego_vehicle.get_acceleration()
                velocity_magnitude = np.linalg.norm([velocity.x, velocity.y, velocity.z])
                control = Controller.get_controls(distance, velocity_magnitude, 
                                                target_distance=TARGET_DISTANCE, 
                                                brake_threshold=BRAKE_THRESHOLD, 
                                                max_velocity=MAX_VELOCITY)

                # Apply controls to ego vehicle
                if distance < 10.0:
                    ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                    print("Ego vehicle braking!")


                # Save images from camera
                for n, item in enumerate(data[1:]):
                     item.save_to_disk('_out/%01d_%08d' % (n, sync_mode.frame))

                # Save data
                saved_data.append({
                    'frame': sync_mode.frame,
                    'velocity_x': velocity.x,
                    'velocity_y': velocity.y,
                    'velocity_z': velocity.z,
                    'acceleration_x': acceleration.x,
                    'acceleration_y': acceleration.y,
                    'acceleration_z': acceleration.z,
                    'distance': distance
                })

                frame_count += 1
    finally:
        # Save collected data to a .csv file
        save_data_to_csv(saved_data)

        # Delete all actors
        for actor in actors_list:
            actor.destroy()

if __name__ == '__main__':
    main()
        

