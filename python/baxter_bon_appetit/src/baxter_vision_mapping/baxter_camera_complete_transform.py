#!/usr/bin/env python

# Built-in imports
import math

# General module imports
import numpy as np

# Own imports
import baxter_essentials.baxter_class as bc
import baxter_camera_transform_tool_to_face as b_camera


class BaxterCameraCompleteTransform:
    """
    Tranform input dictionary from face_detect algorithm output, to specific 
    transformation matrix from w0 to face.
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
        self.bcam = b_camera.BaxterCameraToolToFace(
            face_detection_dict, z_offset)

    def get_tm_from_w0_to_face(self):
        baxter_class = bc.BaxterClass()
        tm_w0_tool = baxter_class.TM_left_limb_camera
        tm_tool_face = self.bcam.get_tm_from_tool_to_face()
        tm_w0_face = np.dot(tm_w0_tool, tm_tool_face)

        print(tm_w0_face)

        return tm_w0_face
