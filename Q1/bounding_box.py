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
import cv2

try:
    import queue
except ImportError:
    import Queue as queue


class BoundingBox(object):

    """
    Class for handling projections and bounding boxes
    This section is taken from: This section follows https://carla.readthedocs.io/en/latest/tuto_G_bounding_boxes/
    """

    def __init__(self, world, camera, bp, image, vehicle):
        self.world = world
        self.camera = camera
        self.bp = bp #camera blueprint
        self.image = image
        self.vehicle = vehicle #ego vehicle

    
    def get_projection_matrix(self, is_behind_camera=False):

        w = self.bp.get_attribute("image_size_x").as_int() #width
        h = self.bp.get_attribute("image_size_y").as_int() #height
        fov = self.bp.get_attribute("fov").as_float()

        focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
        K = np.identity(3)

        if is_behind_camera:
            K[0, 0] = K[1, 1] = -focal
        else:
            K[0, 0] = K[1, 1] = focal

        K[0, 2] = w / 2.0
        K[1, 2] = h / 2.0
        
        return K
    


    def init_bbox(self):
        """
        Initializes and draws bounding boxes for detected vehicles in the camera's view.

        The function processes the camera image and identifies vehicles within a certain distance.
        For each detected vehicle, it calculates the bounding box and draws it on the image.

        Returns:
            tuple: A tuple containing:
                - list: Coordinates of the bounding box (x_min, x_max, y_min, y_max).
                - numpy.ndarray: The image with bounding boxes drawn.
        """
        img = np.reshape(np.copy(self.image.raw_data), (self.image.height, self.image.width, 4))

        # Get matrices
        world_2_camera = np.array(self.camera.get_transform().get_inverse_matrix())
        K = self.get_projection_matrix()

        # Define bbox verdicts. If no verdicts are found during the run returns 0 for all
        x_min = 0.0
        x_max = 0.0
        y_min = 0.0
        y_max = 0.0
        
        for npc in self.world.get_actors().filter('*vehicle*'):

            # Filter out the ego vehicle
            if npc.id != self.vehicle.id:

                bb = npc.bounding_box
                dist = npc.get_transform().location.distance(self.vehicle.get_transform().location)

                # Filter for the vehicles within 120m
                if dist < 120:

                # Calculate the dot product between the forward vector
                # of the vehicle and the vector between the vehicle
                # and the other vehicle. We threshold this dot product
                # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
                    forward_vec = self.vehicle.get_transform().get_forward_vector()
                    ray = npc.get_transform().location - self.vehicle.get_transform().location

                    if forward_vec.dot(ray) > 0:
                        p1 = self.get_image_point(bb.location, K, world_2_camera)
                        verts = [v for v in bb.get_world_vertices(npc.get_transform())]
                        x_max = -10000
                        x_min = 10000
                        y_max = -10000
                        y_min = 10000

                        for vert in verts:
                            p = self.get_image_point(vert, K, world_2_camera)
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

                        # Draw bounding box
                        cv2.line(img, (int(x_min),int(y_min)), (int(x_max),int(y_min)), (0,0,255, 255), 1)
                        cv2.line(img, (int(x_min),int(y_max)), (int(x_max),int(y_max)), (0,0,255, 255), 1)
                        cv2.line(img, (int(x_min),int(y_min)), (int(x_min),int(y_max)), (0,0,255, 255), 1)
                        cv2.line(img, (int(x_max),int(y_min)), (int(x_max),int(y_max)), (0,0,255, 255), 1)

        cv2.imshow('Bounding Box',img)
        cv2.waitKey(1)
        cv2.destroyAllWindows()

        return [x_min, x_max, y_min, y_max], img


    @staticmethod
    def get_image_point(loc, K, w2c):
        """
        Calculate the 2D projection of a 3D coordinate.

        This method transforms a 3D point in world coordinates into 2D image coordinates using
        the camera's intrinsic matrix and the world-to-camera transformation matrix.

        Parameters:
            - loc (carla.Position): The 3D position to project, represented as a CARLA position object.
            - K (numpy.ndarray): The camera intrinsic matrix, which contains focal lengths and optical center.
            - w2c (numpy.ndarray): The world-to-camera transformation matrix.

        Returns:
            - numpy.ndarray: A 2D array containing the (x, y) coordinates of the projected point on the image.
        """

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