# GUIDE TO REPLICATE BAXTER BON APPETIT IN OTHER ROBOTS <img src="assets/imgs/gif_robotboy_flying.gif">

This is a step-by-step guide on how to migrate the source code from [Baxter Bon Appetit](https://github.com/san99tiago/baxter-bon-appetit) to similar robotic platforms. <br>

## Minimum requirements :construction_worker:

To be able to implement Baxter Bon Appetit algorithms in other robots, it is mandatory to understand that your robotic platform must have the following:

- The manipulator (or robot) must have at least 3 [degrees of freedom](<https://en.wikipedia.org/wiki/Degrees_of_freedom_(mechanics)>), enabling it to move freely in space.
- The system must be able to measure the current relative angles between their joints at real time.
- There must be a camera that is located at a known location and it can be mathematically expressed from a reference frame.
- The robot's processor should be similar or better than a [3rd Gen Intel Core i7-3770 Processor](https://ark.intel.com/content/www/us/en/ark/products/65719/intel-core-i73770-processor-8m-cache-up-to-3-90-ghz.html)
- You must be curious and know the basics of: [Robotics](https://en.wikipedia.org/wiki/Robotics), [ROS](https://www.ros.org), [Python](https://www.python.org), [Linux distributions](https://en.wikipedia.org/wiki/Linux_distribution) and [software development](https://en.wikipedia.org/wiki/Software_development)

## Step 1: Understand Baxter Bon Appetit repository :baby:

To be able to migrate Baxter Bon Appetit algorithms, you should first understand the underlying functionalities of Baxter Bon Appetit. To do so, please check out the following documentation:

- [Baxter Bon Appetit Overview](https://github.com/san99tiago/baxter-bon-appetit/blob/main/README.md)
- [Baxter Bon Appetit Setup and Documentation](https://github.com/san99tiago/baxter-bon-appetit/blob/main/setup_baxter_bon_appetit.md)

It is important to navigate throughout the directories and the files of this project. Remember to be curious and feel free to question the low-level functionalities (even to open issues if necessary).<br>

## Step 2: Let's get started :wrench:

The mathematical "building blocks" of the robot's algorithms are inside `baxter_essentials` directory. The first step to extrapolate this to your solution is to change `baxter` naming to your robot's name. As follows:

```bash
.
├── baxter_essentials  # Rename to "<your_robot>_essentials"
│   ├── __init__.py
│   ├── baxter_class.py  # Rename to "<your_robot>_class.py"
│   ├── baxter_fpk.py  # Rename to "<your_robot>_fpk.py"
│   ├── baxter_ipk.py  # Rename to "<your_robot>_ipk.py"
│   ├── baxter_jacobian.py  # Rename to "<your_robot>_jacobian.py"
│   ├── denavit_hartenberg.py
│   └── transformation.py
```

After changing the naming convention, please edit the following files:

### Change `<your_robot>_fpk.py`...

This file corresponds to the Forward Pose Kinematics calculation based on the current joint angles of the robot. It is important to modify the internal methods of this file to guarantee that the calculation is done for the structure of your robot. We encourage you to read about Denavit Hartenbergs parameters to achieve this mathematical expression. One useful reading is the amazing robotics book written by John J. Craig [Introduction to Robotics](https://www.amazon.com/Introduction-Robotics-Mechanics-Control-3rd/dp/0201543613).

<br>

Another important tip to modify this expression, is to use the Python's docstring that we developed for Baxter Bon Appetit into each one of the classes, such as these one:

```python
class BaxterFPK:
    """
    Calculate Baxter's Forward Pose Kinematics for each limb and with the
    desired degrees of freedom for the total joints.

    :param baxter_distances: list of baxter_distances from BaxterClass.
    :param baxter_transformation_matrices: list of
        baxter_transformation_matrices from BaxterClass.
    :param joint_values: list of joint-values to calculate fpk.
        example: [value_limb_s0, value_limb_s1, value_limb_left_e0,
                value_limb_left_e1, value_limb_left_w0, value_limb_left_w1,
                value_limb_left_w2]
    :param limb: arm to calculate fpk.
        example: "left" or "right".
    :param ndof: number of degrees of freedom for fpk.
        example: 6 or 7.
    """
```

### Change `<your_robot>_ipk.py`...

This file corresponds to the Inverse Pose Kinematics calculation based on the current transformation matrix of the end effector of the robot. It is important to modify the internal methods of this file to guarantee that the calculation is done for the structure of your robot. We encourage you to read about how an IPK is done based on the disposition and known measurements of the robot to achieve this mathematical expression. One useful reading is the amazing robotics book written by John J. Craig [Introduction to Robotics](https://www.amazon.com/Introduction-Robotics-Mechanics-Control-3rd/dp/0201543613).

<br>

Another important tip to modify this expression, is to use the Python's docstring that we developed for Baxter Bon Appetit into each one of the classes, such as these one:

```python
class BaxterIPK:
    """
    Calculate Baxter's Inverse Pose Kinematics for each limb and with the
    desired degrees of freedom for the total joints.

    :param TM_w0_tool: Transformation Matrix from W0
        (origin of the workspace), to Baxter's Tool (end of the arm).
    :param baxter_distances: list of baxter_distances from BaxterClass.
    :param baxter_transformation_matrices: list of
        baxter_transformation_matrices from BaxterClass.
    :param limb: arm to calculate fpk.
        example: "left" or "right".
    :param elbow_disposition: Elbow disposition for the mathematical
        ipk solution.
        example: "up", "down".
    """
```

### Change `<your_robot>_jacobian.py`...

This file corresponds to a class that is designed for calculating the current Jacobian matrix depending on the current joint angles measured from the robot. Remember to derive this expression with the mathematical theory of how a Jacobian is created based on the derivatives of the Cartesian variables with respect to the relative joint angles. One useful reading is the amazing robotics book written by John J. Craig [Introduction to Robotics](https://www.amazon.com/Introduction-Robotics-Mechanics-Control-3rd/dp/0201543613).

<br>

Another important tip to modify this expression, is to use the Python's docstring that we developed for Baxter Bon Appetit into each one of the classes, such as these one:

```python
class BaxterJacobian:
    """
    Calculate Baxter's Jacobian from w0 to tool.
        Remark: the expression used was calculated based on the article
        "Baxter Humanoid Robot Kinematics" from Ohio University.

    :param baxter_distances: list of baxter_distances from BaxterClass.
    :param joint_values: list of joint-values.
        example: [value_limb_s0, value_limb_s1, value_limb_left_e0,
                value_limb_left_e1, value_limb_left_w0, value_limb_left_w1,
                value_limb_left_w2]
    :param limb: arm to calculate jacobian.
        example: "left" or "right".
    """
```

### Change `<your_robot>_class.py`...

This is one of the most important files of Baxter Bon Appetit project. This file is the key to centralize the usage of your robot's methods and intrinsic variables that can affect any other algorithm, such as measurements, transformation matrices, constants and more.

<br>

For this class, it is crucial to have a deeper knowledge on the characteristics of your robotic platform, because the will vary significantly between robots. The magic of this file is that it encapsulates the main methods of the models of the robots and, based on its parameters, it can be tweaked experimentally.

<br>

Another important aspect of this main `<your_robot>_class.py`, is that it is the main "entry-point" for you robots centralized algorithms that are going to be implemented in the computer vision and control structures.

<br>

Another important tip to modify this expression, is to use the Python's docstring that we developed for Baxter Bon Appetit into each one of the classes, such as these one:

```python
class BaxterClass():
    """
    Baxter class for defining general-purpose distances and methods for Baxter
    Robot that are commonly implemented in other scripts/classes.
    """
```

## Step 3: Remember to modify the naming convention :electric_plug:

Now that the essential algorithms are developed, it is important to keep in mind that the changes of the names of the files can affect the packaging of the python module. To correctly modify the general package, you must change the following:

### Change the imports

Please go through the Python scripts and change the [imports](https://docs.python.org/3/reference/import.html) of the main package. To do so, one example would be to do the following:

```python
# Change this....
import baxter_essentials.baxter_class as bc

# Into this...
import <your_robot>_essentials.<your_robot>_class as <your_robot_first_letter>c
```

Notice that the imports in Python, can be absolute or relative. In this case, it is assumed that you have [ROS](https://www.ros.org) installed, and, therefore, you will have the package locally installed after running a `catkin_make` (explained in [Setup Baxter Bon Appetit](https://github.com/san99tiago/baxter-bon-appetit/blob/main/setup_baxter_bon_appetit.md)).

<br>

The previous change may alter multiple files, but feel free to use your IDK of your development environment to modify them with shortcuts. For example, in [VSCode](https://code.visualstudio.com), you can press `Ctrl + Shift + F` to search the pattern and replace it in multiple locations.

### Change the `setup.py`

The `setup.py` located inside `<your_robot>_bon_appetit` file, is the one that creates and packaged the Python resulting module to run this application. It is mandatory to change the three main folder directories names, into the ones of your robots to create the project with the right naming convention. One example could be as follows:

```python
# Change this...
d = generate_distutils_setup()
d['packages'] = ['baxter_essentials', 'baxter_vision_mapping', 'baxter_control']
d['package_dir'] = {'': 'src'}

# Into this...
d = generate_distutils_setup()
d['packages'] = ['<your_robot>_essentials', '<your_robot>_vision_mapping', '<your_robot>_control']
d['package_dir'] = {'': 'src'}

```

### Change `CMakeList.txt`

The `CMakeList.txt` is the one that creates the project and "links" all low-level requirements. It also configures important dependencies and path resolutions. To change the name of the resulting project, change the following:

```bash
# Change this...
project(baxter_bon_appetit)

# Into this...
project(<you_project>_bon_appetit)
```

### Change `package.xml`

This XML file is the one in charge of the metadata of the project. It handles extra information and dependencies that the source code has. With this file, the final Python packaging is achieved. To change this file, modify the XML with your personal information and important source dependencies. For example:

```xml
<!-- Change this... -->
<package>
    <name>baxter_bon_appetit</name>
    <version>0.0.1</version>
    <description>
        Programs for Baxter as a Feeding Robot Assistant.
    </description>
</package>

<!-- Into this... -->
<package>
    <name>"YOUR_ROBOT"_bon_appetit</name>
    <version>0.0.1</version>
    <description>
        Programs for "YOUR_ROBOT" as a Feeding Robot Assistant.
    </description>
</package>
```

## Step 4: So Far So Good :video_game:

At this moment, you should have the essentials working and compiling perfectly. It's now time to explore the real robust and amazing Baxter Bon Appetit functionalities, so that you can modify them based on your robot's and your application's needs.

<br>

We encourage you to dive deeper into the `scripts` directory, because it has the main application functionalities that are supported based on the already explained files. The structure and a simple explanation of their goal is as follows:

```bash
.
├── scripts  # Directory with the real application scripts
│   ├── face_detection_limb_cam.py  # Example to validate face-detection algorithm
│   ├── gui_assets  # Assets to show in the Graphical User Interface
│   ├── map_workspace.py  # Useful script to apply a FPK and get the robot's workspace
│   ├── node_fsm.py  # ROS node that orchestrates each state of the Finite State Machine
│   ├── node_go_to_home.py  # ROS node that moves robot's arms to their home position
│   ├── node_gui.py  # *** ENTRYPOINT OF THE APPLICATION *** (deploys the GUI)
│   ├── node_impedance_control.py  # Example to validate impedance-control structure
│   ├── node_joint_position_control.py  # ROS node that sends the joint-control commands to the robot
│   ├── node_mpc_control_trajectory.py  # ROS node that implements a MPC strategy for the trajectory
│   ├── node_open_loop_control_trajectory.py  # ROS node that implements a IPK-PID strategy for the trajectory
│   ├── node_pick_up_food.py  # ROS node that has the purpose of acquiring the food with the left arm
│   ├── node_publish_face_coordinates.py  # ROS node for face detection algorithms and publishes mouth location
│   ├── node_publish_fake_face_coordinates.py  # ROS node that publishes a fake mouth location (test purposes)
│   ├── node_save_data.py  # ROS node that saves the data of the variables of the robot at runtime
│   ├── recordings  # Recordings in CSV format that are replicated by the robot
│   │   └── pick_up_food.csv  # CSV file to execute the "scooping food" action
│   └── set_cartesian_position_ipk.py  # Example to validate the IPK functionalities of the robot
```

## Step 5: Final Tweaks :trumpet:

Congratulations, you are almost done after executing the previous steps. The final important part of migrating this application into another robotic platform, is to understand which nodes you need for your individual tests and create the necessary files to run them.

<br>

The way in which this project orchestrates the ROS nodes is with [ROS-launch](<http://library.isr.ist.utl.pt/docs/roswiki/roslaunch(2f)Tutorials(2f)Roslaunch(20)tips(20)for(20)larger(20)projects.html>). This functionality allows us to run nodes in an easy way, without the hard terminal-work that is required for running them individually. An example of a possible modified launch for your robot could be:

```xml
<!-- Change this... -->
<launch>
    <!-- Arguments -->
    <arg name="N" default="1" doc="Prediction Horizon for MPC control algorithm" />
    <arg name="M" default="1" doc="Control Horizon for MPC control algorithm" />
    <!-- Run the necessary nodes for Baxter fake MPC execution -->
    <node name="go_to_home" pkg="baxter_bon_appetit" type="node_go_to_home.py" respawn="true" output="screen" />
    <node name="publish_face_coordinates" pkg="baxter_bon_appetit" type="node_publish_face_coordinates.py" respawn="true" output="screen" />
    <node name="mpc_control" pkg="baxter_bon_appetit" type="node_mpc_control_trajectory.py" args="$(arg N) $(arg M)" respawn="true" output="screen" />
    <node name="joint_position_control" pkg="baxter_bon_appetit" type="node_joint_position_control.py" respawn="true" output="screen" />
</launch>

<!-- Into this... -->
<launch>
    <!-- Arguments -->
    <arg name="N" default="1" doc="Prediction Horizon for MPC control algorithm" />
    <arg name="M" default="1" doc="Control Horizon for MPC control algorithm" />
    <!-- Run the necessary nodes for "YOUR_ROBOT" fake MPC execution -->
    <node name="go_to_home" pkg="YOUR_ROBOT_bon_appetit" type="node_go_to_home.py" respawn="true" output="screen" />
    <node name="publish_face_coordinates" pkg="YOUR_ROBOT_bon_appetit" type="node_publish_face_coordinates.py" respawn="true" output="screen" />
    <node name="mpc_control" pkg="YOUR_ROBOT_bon_appetit" type="node_mpc_control_trajectory.py" args="$(arg N) $(arg M)" respawn="true" output="screen" />
    <node name="joint_position_control" pkg="YOUR_ROBOT_bon_appetit" type="node_joint_position_control.py" respawn="true" output="screen" />
</launch>
```

<br>

Now you are free to have fun and play with "Bon Appetit" functionalities in your robot.

<br>

Thank you for implementing this project, it was developed by:

- [Santiago Garcia Arango](https://github.com/san99tiago)
- [Elkin Javier Guerra Galeano](https://github.com/Elkinmt19)

<br>

> Feel free to contact us!
> tesisbaxter@gmail.com

<br>

<img src="assets/imgs/santi_elkin_in_the_university.png" width=60%><img src="assets/imgs/gif_baxter_blessing.gif" width=36.3%>
