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
    """
    bp_lib = world.get_blueprint_library()

    if not vehicle_blueprint:
        vehicle = world.spawn_actor(random.choice(bp_lib.filter('vehicle.*')), transform)
    else:
        vehicle = world.spawn_actor(bp_lib.find(vehicle_blueprint), transform)

    return vehicle

def spawn_camera(world, vehicle, vehicle_dims, view_width=1920, view_height=1080, view_fov=60):
    """
    """
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
    return camera, cam_rgb

# ==============================================================================
# -- Vehicle Dynamics ----------------------------------------------------------
# ==============================================================================

def get_vehicle_dimensions(vehicle):
    """
    """
    bounding_box = vehicle.bounding_box

    length = 2 * bounding_box.extent.x 
    width = 2 * bounding_box.extent.y
    height = 2 * bounding_box.extent.z

    return [length, width, height]

def get_true_distance(ego_vehicle, stationary_vehicle, ego_vehicle_dims, stationary_vehicle_dims):
    """
    
    """
    ego_location = ego_vehicle.get_location()
    stationary_location = stationary_vehicle.get_location()
    distance_between_centers = ego_location.distance(stationary_location)

    return distance_between_centers - (ego_vehicle_dims[0] / 2) - (stationary_vehicle_dims[0] / 2)
    
    
def get_jerk(current_acceleration, previous_acceleration, delta_seconds=0.05):
    """
    """
    if delta_seconds <= 0:
        raise ValueError("Timestep interval must be positive")
    
    jerk_x = (current_acceleration.x - previous_acceleration.x) / delta_seconds
    jerk_y = (current_acceleration.y - previous_acceleration.y) / delta_seconds
    jerk_z = (current_acceleration.z - previous_acceleration.z) / delta_seconds
    
    return carla.Vector3D(jerk_x, jerk_y, jerk_z)


# ==============================================================================
# -- Bounding Boxes and Projections  -------------------------------------------
# ==============================================================================
"""
This section follows https://carla.readthedocs.io/en/latest/tuto_G_bounding_boxes/
"""

def build_projection_matrix(w, h, fov, is_behind_camera=False):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)

    if is_behind_camera:
        K[0, 0] = K[1, 1] = -focal
    else:
        K[0, 0] = K[1, 1] = focal

    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K

def get_image_point(loc, K, w2c):
    """
    """
    # Calculate 2D projection of 3D coordinate

    # Format the input coordinate (loc is a carla.Position object)
    point = np.array([loc.x, loc.y, loc.z, 1])
    # transform to camera coordinates
    point_camera = np.dot(w2c, point)

    # New we must change from UE4's coordinate system to an "standard"
    # (x, y ,z) -> (y, -z, x)
    # and we remove the fourth componebonent also
    point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

    # now project 3D->2D using the camera matrix
    point_img = np.dot(K, point_camera)
    # normalize
    point_img[0] /= point_img[2]
    point_img[1] /= point_img[2]

    return point_img[0:2]


def get_projection_matrix(camera, camera_bp):
    """
    """
    # Get the world to camera matrix
    world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

    # Get the attributes from the camera
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()
    fov = camera_bp.get_attribute("fov").as_float()

    # Calculate the camera projection matrix to project from 3D -> 2D
    K = build_projection_matrix(image_w, image_h, fov)
    
    return K

def init_bbox(image, camera, camera_bp, world, vehicle):
    """
    """
    img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

    # Get matrices
    world_2_camera = np.array(camera.get_transform().get_inverse_matrix())
    K = get_projection_matrix(camera, camera_bp)

    # Define bbox verdicts. If no verdicts are found during the run returns 0 for all
    x_min = 0.0
    x_max = 0.0
    y_min = 0.0
    y_max = 0.0
    
    for npc in world.get_actors().filter('*vehicle*'):

        # Filter out the ego vehicle
        if npc.id != vehicle.id:

            bb = npc.bounding_box
            dist = npc.get_transform().location.distance(vehicle.get_transform().location)

            # Filter for the vehicles within 120m
            if dist < 120:

            # Calculate the dot product between the forward vector
            # of the vehicle and the vector between the vehicle
            # and the other vehicle. We threshold this dot product
            # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
                forward_vec = vehicle.get_transform().get_forward_vector()
                ray = npc.get_transform().location - vehicle.get_transform().location

                if forward_vec.dot(ray) > 0:
                    p1 = get_image_point(bb.location, K, world_2_camera)
                    verts = [v for v in bb.get_world_vertices(npc.get_transform())]
                    x_max = -10000
                    x_min = 10000
                    y_max = -10000
                    y_min = 10000

                    for vert in verts:
                        p = get_image_point(vert, K, world_2_camera)
                        # Find the rightmost vertex
                        if p[0] > x_max:
                            x_max = p[0]
                        # Find the leftmost vertex
                        if p[0] < x_min:
                            x_min = p[0]
                        # Find the highest vertex
                        if p[1] > y_max:
                            y_max = p[1]
                        # Find the lowest  vertex
                        if p[1] < y_min:
                            y_min = p[1]

                    cv2.line(img, (int(x_min),int(y_min)), (int(x_max),int(y_min)), (0,0,255, 255), 1)
                    cv2.line(img, (int(x_min),int(y_max)), (int(x_max),int(y_max)), (0,0,255, 255), 1)
                    cv2.line(img, (int(x_min),int(y_min)), (int(x_min),int(y_max)), (0,0,255, 255), 1)
                    cv2.line(img, (int(x_max),int(y_min)), (int(x_max),int(y_max)), (0,0,255, 255), 1)

    cv2.imshow('Bounding Box',img)
    cv2.waitKey(1)
    cv2.destroyAllWindows()

    return [x_min, x_max, y_min, y_max]


# ==============================================================================
# -- misc ----------------------------------------------------------------------
# ==============================================================================

def save_data_to_csv(data):
    df = pd.DataFrame(data)
    
    df.to_csv('vehicle_data.csv', index=False) 

    return
