#!/usr/bin/env python

# Built-in imports
import math

# General module imports
import numpy as np

# Own imports
import baxter_essentials.transformation as transf


class BaxterCameraToolToFace:
    """
    Tranform input dictionary from face_detect algorithm output, to specific 
    relative cartesian points in a (Xi, Yi, Zi) format.
    :param face_detection_dict: input python dictionary with the face_detect
        algorithm output.
        example:
            {
                'detected_face': True,
                'img_height': 480L,
                'img_width': 640L,
                'faces': array([[174, 192, 217, 217]])
            }
    """

    def __init__(self, face_detection_dict, z_offset):
        # Ratios for X/Z and Y/Z in meters (experimental factor from camera)
        self.ratio_planar_x_z = 0.8
        self.ratio_planar_y_z = 0.9

        self.z_offset = z_offset

        self.face_detection_dict = face_detection_dict
        self.analyze_if_face_detected()

    def analyze_if_face_detected(self):
        if self.face_detection_dict["detected_face"] == True:
            self.map_point()
        else:
            return False

    def map_point(self):
        self.get_face_detection_dict_info()
        self.get_img_centroid()
        self.get_face_centroid()
        self.calculate_x_ref_in_meters()
        self.calculate_y_ref_in_meters()
        self.calculate_scale_factor_x_meters_per_pix()
        self.calculate_scale_factor_y_meters_per_pix()
        self.find_tm_from_tool_to_face()

    def get_face_detection_dict_info(self):
        self.faces = self.face_detection_dict["faces"]
        self.img_height = self.face_detection_dict["img_height"]
        self.img_width = self.face_detection_dict["img_width"]

    def get_img_centroid(self):
        self.img_centroid_x = int(self.img_width / 2)
        self.img_centroid_y = int(self.img_height / 2)
        print("img_centroid (x, y): ", self.img_centroid_x, self.img_centroid_y)

    def get_face_centroid(self):
        x, y, w, h = self.faces[0]
        print("face_point (x, y, w, h): ", x, y, w, h)
        self.face_centroid_x = float(x + w / 2)
        self.face_centroid_y = float(y + h / 2)
        print("face_centroid (x_f, y_f): ",
              self.face_centroid_x, self.face_centroid_y)

    def calculate_x_ref_in_meters(self):
        self.x_ref_in_meters = self.ratio_planar_x_z * self.z_offset

    def calculate_y_ref_in_meters(self):
        self.y_ref_in_meters = self.ratio_planar_y_z * self.z_offset

    def calculate_scale_factor_x_meters_per_pix(self):
        self.scale_factor_x_meters_per_pix = self.x_ref_in_meters / self.img_width
        print("self.scale_factor_x_meters_per_pix: ",
              self.scale_factor_x_meters_per_pix)

    def calculate_scale_factor_y_meters_per_pix(self):
        self.scale_factor_y_meters_per_pix = self.y_ref_in_meters / self.img_height
        print("self.scale_factor_y_meters_per_pix: ",
              self.scale_factor_y_meters_per_pix)

    def find_tm_from_tool_to_face(self):
        # Find TM_tool_cv
        x_distance = - float(self.scale_factor_x_meters_per_pix) * \
            float(self.img_centroid_x)
        y_distance = float(self.scale_factor_y_meters_per_pix) * \
            float(self.img_centroid_y)
        self.tm_from_tool_to_cv = transf.Transformation(
            0,
            0,
            -math.pi/2,
            [
                x_distance,
                y_distance,
                self.z_offset
            ]
        ).TM

        # Find TM_cv_face
        x_distance = float(self.scale_factor_x_meters_per_pix) * \
            float(self.face_centroid_x)
        y_distance = float(self.scale_factor_y_meters_per_pix) * \
            float(self.face_centroid_y)
        self.tm_from_cv_to_face = transf.Transformation(
            0,
            0,
            0,
            [
                x_distance,
                y_distance,
                0
            ]
        ).TM

        self.tm_from_tool_to_face = np.dot(
            self.tm_from_tool_to_cv, self.tm_from_cv_to_face)

        print("tm_from_tool_to_face: ", self.tm_from_tool_to_face)

    def get_tm_from_tool_to_face(self):
        if self.face_detection_dict["detected_face"] == True:
            return self.tm_from_tool_to_face
        else:
            return False
