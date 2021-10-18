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
