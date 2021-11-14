# :robot: SETUP BAXTER BON APPETIT :robot:

To properly install and configure baxter-bon-appetit functionalities, you must follow these steps:

## 1. Setup Baxter Workstation
The [Workstation setup](https://sdk.rethinkrobotics.com/wiki/Workstation_Setup) is a guide to install all the Baxter software dependencies and development configurations that are required for working with this robot.<br>

## 2. Cloning baxter-bon-appetit repository
To clone baxter-bon-appetit repository, you can follow these commands:
```bash
cd /tmp
git clone https://github.com/san99tiago/baxter-bon-appetit.git
```

## 3. Adding baxter-bon-appetit source codes to ros_ws
To include these scripts and programs in the ROS workspace directory (ros_ws), you can execute the following commands:
```bash
cp -r /tmp/baxter-bon-appetit/python/baxter_bon_appetit /home/<user>/ros_ws/src
```
* Important remark: change \<user> to the user of your machine.

## 4. Compile ROS workspace
To compile ROS workspace (including the recently added baxter-bon-appetit functionalities), you can run:
```bash
cd /home/<user>/ros_ws
catkin_make
```
* Important remark: change \<user> to the user of your machine.

## 5. Install face-detection algorithm module
To install face-detection-haar-cascade library, please follow the steps shown in [FDHC README](https://github.com/san99tiago/baxter-bon-appetit/tree/main/python/computer_vision/face_detect_haar_cascade).


## 6. Install cvxpy python dependency
As Ubuntu 14.04 is an "old" Linux distribution, you can follow a series of special steps to install this module:<br>
* [cvxpy installation guide](https://ajfriendcvxpy.readthedocs.io/en/latest/install/)
* [cvxpy repository](https://github.com/cvxpy/cvxpy)

## 7. Troubleshoot additional dependencies
It is possible that some of the previous have additional python dependencies. It is crucial to have them installed in case of any dependency error. For example:
```bash
pip install <python-dependency>
```

---

# :scroll: REPOSITORY OVERVIEW :scroll:

Baxter Bon Appetit repository has multiple important files. In this tree-wise diagram, the most important directories are going to be explained and shown as a general overview of the project.<br>

```bash
.
├── LICENSE
├── README.md  # Principal README.md
├── assets
├── guide_to_replicate_in_other_robots.md  # Guide to use functionalities in other robots
├── matlab  # Scripts for mathematical models and analysis (not necessary for implementation)
│   ├── BaxterClass.m
│   ├── BaxterFunctions.m
│   ├── BaxterJacobian.m
│   ├── Baxter_Jacobian_7dof.m
│   ├── Baxter_Simulation.m
│   ├── Baxter_Workspace.fig
│   ├── Baxter_Workspace.m
│   ├── Baxter_Workspace_DT.fig
│   ├── DirectCinematic.m
│   ├── Exe_Sim_Baxter.m
│   ├── FPK.m
│   ├── IPK.m
│   ├── IPK_7dof.m
│   ├── OrientationAngles.m
│   ├── Puntos_Simulacion.m
│   ├── README.md
│   ├── RotationMatrix.m
│   ├── RotationMatrixExpand.m
│   ├── TransformationMatrix.m
│   ├── Trayectoria_5_orden.m
│   ├── assets
│   ├── baxter_calibration_measurements.m
│   └── baxter_jacobian_calculation.m
├── python  # Main functionalities for the development of the feeding-robot-application
│   ├── baxter_bon_appetit  # Python scripts to run Baxter Bon Apptit
│   │   ├── CMakeLists.txt  # Dependencies for low-level C libraries
│   │   ├── README.md
│   │   ├── assets
│   │   ├── cfg  # Only usefull for impedance-control tests 
│   │   │   └── JointSpringsBonAppetit.cfg
│   │   ├── launch  # Enables the execution of multiple ROS nodes
│   │   │   ├── fake_mpc_impedance_control_nodes.launch
│   │   │   ├── fake_mpc_position_control_nodes.launch
│   │   │   ├── fake_ol_position_control_nodes.launch
│   │   │   ├── mpc_impedance_control_nodes.launch
│   │   │   ├── mpc_position_control_nodes.launch
│   │   │   └── ol_position_control_nodes copy.launch
│   │   ├── package.xml  # Manage dependencies and important project's information
│   │   ├── samples  # Scripts to validate specific Baxter Bon Appetit functionalities
│   │   │   ├── sample_cartesian_increments.py
│   │   │   ├── sample_control_loop.py
│   │   │   └── sample_get_baxter_camera_point.py
│   │   ├── scripts  # Most important functionalities for Baxter Bon Appetit
│   │   │   ├── face_detection_limb_cam.py
│   │   │   ├── gui_assets
│   │   │   ├── map_workspace.py
│   │   │   ├── node_fsm.py
│   │   │   ├── node_go_to_home.py
│   │   │   ├── node_gui.py  # *** ENTRY LEVEL TO MAIN GUI OF THE PROJECT ***
│   │   │   ├── node_impedance_control.py
│   │   │   ├── node_joint_position_control.py
│   │   │   ├── node_mpc_control_trajectory.py
│   │   │   ├── node_open_loop_control_trajectory.py
│   │   │   ├── node_pick_up_food.py
│   │   │   ├── node_publish_face_coordinates.py
│   │   │   ├── node_publish_fake_face_coordinates.py
│   │   │   ├── node_save_data.py
│   │   │   ├── recordings  # Allows Baxter to replicate recorded movements
│   │   │   │   ├── blessing.csv
│   │   │   │   └── pick_up_food.csv
│   │   │   └── set_cartesian_position_ipk.py
│   │   ├── setup.py  # Enables the project packaging with its dependencies
│   │   └── src  # Mandatory scripts that build up Baxter complex solutions
│   │       ├── __init__.py
│   │       ├── baxter_bon_appetit.egg-info
│   │       ├── baxter_control  # MPC control algorithms
│   │       │   ├── __init__.py
│   │       │   ├── cartesian_increment.py
│   │       │   └── mpc_controller.py
│   │       ├── baxter_essentials  # Main robotic transforms for mathematical models
│   │       │   ├── __init__.py
│   │       │   ├── baxter_class.py
│   │       │   ├── baxter_fpk.py
│   │       │   ├── baxter_ipk.py
│   │       │   ├── baxter_jacobian.py
│   │       │   ├── denavit_hartenberg.py
│   │       │   └── transformation.py
│   │       └── baxter_vision_mapping  # Enables the transformations from the camera to the user
│   │           ├── __init__.py
│   │           ├── baxter_camera_complete_transform.py
│   │           └── baxter_camera_transform_tool_to_face.py
│   └── computer_vision  # Python scripts for face detection algorithms
│       ├── assets
│       ├── face_detect_face_recognition  # Alternative for face detection algorithm
│       │   ├── LICENSE
│       │   ├── README.md
│       │   ├── requirements.txt
│       │   ├── samples
│       │   │   ├── __init__.py
│       │   │   ├── get_assets_folder.py
│       │   │   ├── sample_depencies.py
│       │   │   ├── sample_image_basics.py
│       │   │   ├── sample_image_time.py
│       │   │   ├── sample_video_correct_frames.py
│       │   │   ├── sample_video_device_camera.py
│       │   │   ├── sample_video_general_purpose.py
│       │   │   └── sample_video_resources.py
│       │   ├── setup.py
│       │   ├── src
│       │   │   ├── __init__.py
│       │   │   ├── face_detect_fr.py
│       │   │   └── fdfr.egg-info
│       │   └── tests
│       │       ├── get_assets_folder.py
│       │       ├── test_open_image_fr.py
│       │       └── test_process_image_fr.py
│       └── face_detect_haar_cascade  # Main face detection algorithm (recommended)
│           ├── LICENSE
│           ├── README.md
│           ├── samples
│           │   ├── __init__.py
│           │   ├── get_assets_folder.py
│           │   ├── sample_depencies.py
│           │   ├── sample_image_basics.py
│           │   ├── sample_image_time.py
│           │   ├── sample_process_multiple_photos.py
│           │   ├── sample_video_correct_frames.py
│           │   ├── sample_video_device_camera.py
│           │   ├── sample_video_general_purpose.py
│           │   └── sample_video_resources.py
│           ├── setup.py
│           ├── src
│           │   ├── __init__.py
│           │   ├── face_detect_hc.py
│           │   ├── fdhc.egg-info
│           │   └── haarcascade_frontalface_default.xml
│           └── tests
│               ├── get_assets_folder.py
│               ├── test_open_image_hc.py
│               └── test_process_image_hc.py
└── setup_baxter_bon_appetit.md
```
