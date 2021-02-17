from rigidbody import pr, rotation, SE3
from triangulation import triangulate
import numpy as np
import cv2


class MVTriangulation:
    def __init__(self, num_view, cam_params):
        self.num_view = num_view
        self.M = cam_params['K']
        self.rvecs = cam_params['r']
        self.tvecs = cam_params['t']

    def triangulate_midpoint(self, pts2d):
        orientations = map(rotation.exp, self.rvecs)
        poses = [SE3(r, p) for r, p in zip(orientations, self.tvecs)]
        pts3d = triangulate(pts2d, poses, algorithm='midpoint')
        return pts3d