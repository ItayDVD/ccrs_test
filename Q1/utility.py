import carla
import random
import math
import pandas as pd

def generate_spawn_points(world):
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
    bp_lib = world.get_blueprint_library()

    if not vehicle_blueprint:
        vehicle = world.spawn_actor(random.choice(bp_lib.filter('vehicle.*')), transform)
    else:
        vehicle = world.spawn_actor(bp_lib.find(vehicle_blueprint), transform)

    return vehicle

def spawn_camera(world, vehicle, vehicle_dims, view_width=1920, view_height=1080, view_fov=60):
    cam_rgb = world.get_blueprint_library().find('sensor.camera.rgb')
    cam_rgb.set_attribute('image_size_x', str(view_width))
    cam_rgb.set_attribute('image_size_y', str(view_height))
    cam_rgb.set_attribute('fov', str(view_fov))

    camera_offsets = [x/2 for x in vehicle_dims]
    
    camera = world.spawn_actor(
        cam_rgb,
        carla.Transform(carla.Location(x=camera_offsets[0], y=camera_offsets[1], z=camera_offsets[2]), carla.Rotation(pitch=0, yaw=0, roll=0)),
        attach_to=vehicle)
    #
    return camera

def get_vehicle_dimensions(vehicle):
    bounding_box = vehicle.bounding_box

    length = 2 * bounding_box.extent.x 
    width = 2 * bounding_box.extent.y
    height = 2 * bounding_box.extent.z

    return [length, width, height]
    

# def save_data_to_csv(data):
#     with open('vehicle_data.csv', mode='w', newline='') as csvfile:
#         fieldnames = ['frame', 'velocity_x', 'velocity_y', 'velocity_z', 'acceleration_x', 'acceleration_y', 'acceleration_z', 'distance']
#         writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
#         writer.writeheader()
#         for entry in data:
#             writer.writerow(entry)
#     return

def save_data_to_csv(data):
    df = pd.DataFrame(data)
    
    df.to_csv('vehicle_data.csv', index=False) 

    return


def get_controls(distance, velocity, brake_threshold = 10, max_velocity = math.inf):
    """
    Computes vehicle control commands (throttle, break inputs) for the ego vehicle based on the relative distance to the target vehicle.

    args:
        -distance): The current distance from the ego vehicle to the target stationary vehicle (meters).
        -velocity (float): The current velocity of the ego vehicle (meters/second).
        -target_distance (float, optional): The desired distance between the ego car and the preceding vehicle. The ego car will decelerate until it reaches and this distance from the stationary car.
        -break_threshold (float, optional): Minimum distance between the ego carand preceding vehicle to apply brakes.
        -max_velocity (float, optional): Maximum velocity for ego vehicle. If surpassed throttle will be set to 0.

    Returns:
        carla.VehicleControl as defined via the carla PythonAPI.
        see https://carla.readthedocs.io/en/latest/python_api/#carlavehiclecontrol

    Notes:
        It is a possibility to later add additional control features such as gain for throttling / braking.
    """

    if distance or velocity is None:
        raise ValueError("Distance and velocity must be set before computing controls.")

    if distance > brake_threshold:
        # Let it rip baby!
        throttle_input = 1.0
        brake_input = 0.0
    else:
        throttle_input=0.0
        brake_input=1.0

    if velocity > max_velocity:
        throttle_input = 0.0

    return carla.VehicleControl(throttle=throttle_input, steer=0.0, brake=brake_input, hand_brake=False, reverse=False, manual_gear_shift=False)