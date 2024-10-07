
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
from bounding_box import BoundingBox
from utility import *

import carla
import pygame
import numpy as np

from constants import TARGET_DISTANCE, BRAKE_THRESHOLD, MAX_VELOCITY, MAX_FRAMES, DELTA_TIMESTEP

import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s - %(message)s'
)

# Set constants
max_frames = MAX_FRAMES
delta_timestep = DELTA_TIMESTEP
max_velocity = MAX_VELOCITY
brake_threshold = BRAKE_THRESHOLD
target_distance = TARGET_DISTANCE

def main():

    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)

    world = client.load_world('/Game/Carla/Maps/Town01')

    # Arrays to hold active actors and data saved from ego vehicle
    actors_list = []
    saved_data = []

    # Hold a measurement of previous acceleration for jerk calculation
    previous_acceleration = carla.Vector3D(0.0, 0.0, 0.0)

    # Initiate a frame counter
    frame_count = 0

    # Set spectator
    spectator = world.get_spectator()

    try:
        # Spawn vehicles. Chose a random road in Town01 that is more than 100 meters long
        ego_vehicle_startpoint = carla.Transform(carla.Location(x=-2.03, y= 9.56, z=0.3), 
                                            carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))
        stationary_vehicle_startpoint = carla.Transform(carla.Location(x=-2.03, y= 119.341, z=0.3), 
                                            carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))

        
        ego_vehicle = spawn_vehicle(world, ego_vehicle_startpoint)
        logging.info('created  %s' % ego_vehicle.type_id)
        
        stationary_vehicle = spawn_vehicle(world, stationary_vehicle_startpoint)
        logging.info('created  %s' % stationary_vehicle.type_id)

        # Get vehicle dimensions
        ego_vehicle_dims = get_vehicle_dimensions(ego_vehicle)
        stationary_vehicle_dims = get_vehicle_dimensions(stationary_vehicle)

        # Time for the ego vehicle to hit the gas
        ego_vehicle.set_simulate_physics(True)
        ego_vehicle.set_autopilot(True) #Autopilot can be set off.
        #Vehicle controls can be calculated manually via Controller (see controller.py)

        # Add sensors (RGB camera)
        camera, camera_bp = spawn_camera(world, ego_vehicle, ego_vehicle_dims, 
                                         view_width=720, view_height=576, view_fov=60)
        logging.info('created %s' % camera.type_id)

        # Append vehicle and sensors to actors list
        actors_list.extend([ego_vehicle, stationary_vehicle, camera])

        # Run the simulation in syncronize mode
        with CarlaSyncMode(world, camera, fps=20) as sync_mode:
            while True:

                # Advance the simulation and wait for data
                _, img = sync_mode.tick(timeout=1.0)

                # Adjust spectator viewpoint to behind ego vehicle
                spectator_transform = carla.Transform(ego_vehicle.get_transform().location + carla.Location(x=0.0, y=-3.0, z=2.0),
                                                      ego_vehicle.get_transform().rotation)
                spectator.set_transform(spectator_transform)

                # Calculate distance between vehicles
                distance = get_true_distance(ego_vehicle, stationary_vehicle, ego_vehicle_dims, 
                                            stationary_vehicle_dims)
                
                # Calculate control inputs and measurements
                velocity = ego_vehicle.get_velocity()
                acceleration = ego_vehicle.get_acceleration()
                jerk = get_jerk(acceleration, previous_acceleration, delta_timestep)
                previous_acceleration = acceleration
                velocity_magnitude = np.linalg.norm([velocity.x, velocity.y, velocity.z])


                # Draw npc vehicles bounding boxes and get their 2D projected coordinates
                bbox = BoundingBox(world, camera, camera_bp, img, ego_vehicle)
                bbox_verdicts, img_with_bbox = bbox.init_bbox()

                # Save images from camera (with the bounding box)
                img_bgr = cv2.cvtColor(img_with_bbox, cv2.COLOR_RGBA2BGR)
                cv2.imwrite(('_out/%01d.png' % frame_count), img_bgr)
                
                # Save data
                saved_data.append({
                    'frame': frame_count,
                    'velocity_x': velocity.x,
                    'velocity_y': velocity.y,
                    'velocity_z': velocity.z,
                    'acceleration_x': acceleration.x,
                    'acceleration_y': acceleration.y,
                    'acceleration_z': acceleration.z,
                    'jerk_x': jerk.x,
                    'jerk_y': jerk.y,
                    'jerk_z': jerk.z,
                    'distance': distance,
                    'bbox_x_min': bbox_verdicts[0],
                    'bbox_x_max': bbox_verdicts[1],
                    'bbox_y_min': bbox_verdicts[2],
                    'bbox_y_max': bbox_verdicts[3]
                })

                frame_count += 1

                # If vehicle came to a stop at target distance stop simulation
                if ((distance < target_distance and velocity_magnitude < 0.05) 
                    or (frame_count > max_frames)):
                    break

    finally:
        # Save collected data to a .csv file
        save_data_to_csv(saved_data)

        # Delete all actors
        for actor in actors_list:
            actor.destroy()
        
        logging.info("Simulation ended")

if __name__ == '__main__':
    main()
        

