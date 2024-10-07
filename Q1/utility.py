import carla
import random
import math
import cv2
import numpy as np
import pandas as pd

# ==============================================================================
# -- Spawn Actors ---------------------------------------------------------------
# ==============================================================================

def generate_spawn_points(world):
    """
    Itay: I didn't use this function, tried to find two relevant spawn points given a random map but
    ended up just choosing Town01 and two fixed spawn points on some relevant road. 
    """
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points available.")

    # Choose a random spawn point for the first vehicle
    ego_spawn_point = random.choice(spawn_points)

    # Calculate the second spawn point 100m away in the direction the vehicle will drive
    direction = ego_spawn_point.get_forward_vector()
    stationary_spawn_point_location = carla.Location(
        x=ego_spawn_point.location.x + direction.x * 100,
        y=ego_spawn_point.location.y + direction.y * 100,
        z=ego_spawn_point.location.z
    )
    stationary_spawn_point = carla.Transform(stationary_spawn_point_location, ego_spawn_point.rotation)

    return ego_spawn_point, stationary_spawn_point


def spawn_vehicle(world, transform, vehicle_blueprint=None):
    """
    Spawn a vehicle in the simulation world at a specified transform.

    Parameters:
        world (carla.World): The CARLA simulation world object.
        transform (carla.Transform): The transform specifying the vehicle's initial position and orientation.
        vehicle_blueprint (str, optional): The blueprint name of the vehicle to spawn. If None, a random vehicle is selected.
    Returns:
        carla.Vehicle: The spawned vehicle actor.
    """
    bp_lib = world.get_blueprint_library()

    if not vehicle_blueprint:
        vehicle = world.spawn_actor(random.choice(bp_lib.filter('vehicle.*')), transform)
    else:
        vehicle = world.spawn_actor(bp_lib.find(vehicle_blueprint), transform)

    return vehicle


def spawn_camera(world, vehicle, vehicle_dims, view_width=1920, view_height=1080, view_fov=60):
   """
    Spawn an RGB camera attached to vehicle.

    Parameters:
        world (carla.World): The CARLA simulation world object.
        vehicle (carla.Vehicle): The vehicle to which the camera will be attached.
        vehicle_dims (list): Dimensions of the vehicle in the order [length, width, height].
        view_width (int, optional): The width of the camera image in pixels. Defaults to 1920.
        view_height (int, optional): The height of the camera image in pixels. Defaults to 1080.
        view_fov (int, optional): The field of view of the camera in degrees. Defaults to 60.
    Returns:
        tuple: A tuple containing:
            - camera (carla.Actor): The spawned camera actor.
            - cam_rgb (carla.Blueprint): The blueprint of the RGB camera.
    """
   
    cam_rgb = world.get_blueprint_library().find('sensor.camera.rgb')
    cam_rgb.set_attribute('image_size_x', str(view_width))
    cam_rgb.set_attribute('image_size_y', str(view_height))
    cam_rgb.set_attribute('fov', str(view_fov))

    camera_offsets = [x/2 for x in vehicle_dims]

    camera = world.spawn_actor(
        cam_rgb,
        carla.Transform(carla.Location(x=camera_offsets[0], y=camera_offsets[1], z=camera_offsets[2]), carla.Rotation(pitch=0, roll=0)),
        attach_to=vehicle)
    #
    return camera, cam_rgb

# ==============================================================================
# -- Vehicle Dynamics ----------------------------------------------------------
# ==============================================================================

def get_vehicle_dimensions(vehicle):
    """
    Calculate the dimensions of a vehicle.

    This function retrieves the bounding box of the specified vehicle and calculates its
    length, width, and height based on the bounding box extents.

    Parameters:
        - vehicle (carla.Vehicle): The vehicle object for which to calculate dimensions.
    Returns:
        - list: A list containing the dimensions of the vehicle in the order [length, width, height].
    """
    bounding_box = vehicle.bounding_box

    length = 2 * bounding_box.extent.x 
    width = 2 * bounding_box.extent.y
    height = 2 * bounding_box.extent.z

    return [length, width, height]

def get_true_distance(ego_vehicle, stationary_vehicle, ego_vehicle_dims, stationary_vehicle_dims):
    """
    Calculate the distance between the front of rear vehicle to the back of front vehicle (collision distance).

    Parameters:
        - ego_vehicle (carla.Vehicle): The ego vehicle object.
        - stationary_vehicle (carla.Vehicle): The stationary vehicle object.
        - ego_vehicle_dims (list): Dimensions of the ego vehicle in the order [length, width, height].
        - stationary_vehicle_dims (list): Dimensions of the stationary vehicle in the order [length, width, height].

    Returns:
        - float: The adjusted distance between the two vehicle centers, accounting for their dimensions.
    """
    ego_location = ego_vehicle.get_location()
    stationary_location = stationary_vehicle.get_location()
    distance_between_centers = ego_location.distance(stationary_location)

    return distance_between_centers - (ego_vehicle_dims[0] / 2) - (stationary_vehicle_dims[0] / 2)
    
    
def get_jerk(current_acceleration, previous_acceleration, delta_seconds=0.05):
    """
    Calculates jerk of a vehicle.
    Itay: I'm 100% positive that there is a better way to do this, but I mechanics are not the subject
    of the test :)
    """
    if delta_seconds <= 0:
        raise ValueError("Timestep interval must be positive")
    
    jerk_x = (current_acceleration.x - previous_acceleration.x) / delta_seconds
    jerk_y = (current_acceleration.y - previous_acceleration.y) / delta_seconds
    jerk_z = (current_acceleration.z - previous_acceleration.z) / delta_seconds
    
    return carla.Vector3D(jerk_x, jerk_y, jerk_z)




# ==============================================================================
# -- misc ----------------------------------------------------------------------
# ==============================================================================

def save_data_to_csv(data):
    df = pd.DataFrame(data)
    
    df.to_csv('vehicle_data.csv', index=False) 

    return
