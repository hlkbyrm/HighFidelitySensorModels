# ROS Based AGV Simulation with High Fidelity IMU and Wheel Encoder Models
This repo is based on a ROS Book Chapter where the wheel encoder (differential drive) and IMU sensor plugins are modified and developed for modelling sensor uncertainties more accurately in Gazebo. 
The original study is conducted with an industrial type AGV Gazebo model developed in Ford OTOSAN, Turkey.
In this repo, the aforementioned plugins are integrated with a simple Turtlebot3 Gazebo model for better explanation. For the theoretical background and use cases, please refer to the paper.


## High Fidelity Gazebo Sensor Plugins

Further explanation to be added

* libnoisy_imu_sensor.so

* libnoisy_diff_drive.so

## Installation & Build
### ROS Version
The project was developed on Ubuntu 18.04 LTS with [ROS Melodic](http://wiki.ros.org/melodic), [Gazebo 9.0](http://gazebosim.org/) and [catkin](http://wiki.ros.org/catkin) installed. However it should also work with Ubuntu 16.04 and ROS Kinetic.


### Building the Workspace
Copy the package contents to ``catkin_ws/src`` folder


Use ``catkin`` to build the packages from source. From ``catkin_ws``, run:

``catkin_make; source devel/setup.bash``

to build the workspace packages and source your workspace.

Also define the ``GAZEBO_MODEL_PATH`` to your .bashrc 

``export GAZEBO_MODEL_PATH=~/catkin_ws/build/turtlebot3_description_poisson``

### Package Contents
The repo contains the modified version of the Turtlebot3 and an empty gazebo world. 

Keep in mind that this package does not depend on official Turtlebot3 package provided by [ROBOTIS](https://github.com/ROBOTIS-GIT/turtlebot3).
Turtlebot3 is selected as an demonstration platform for the developed sensor plugins and may not reflect all the details of official Turtlebot3 package.

Please check the original [Turtlebot3 Repo](https://github.com/ROBOTIS-GIT/turtlebot3) for the official description file. Here is the short summary of what has been modified in the turtlebot3_description package.

``turtlebot_description`` package is renamed as ``turtlebot3_highfidelityWheelEncoderIMU`` in order to break the link of pre-installed turtlebot3 packages (via ``sudo-apt get install ros-melodic-turtebot3*``)

* ``turtlebot3_burger.gazebo.xacro``
    * The two noisy plugins are added for differential drive and IMU sensors
        * ``libgazebo_ros_diff_drive.so`` replaced with ``libnoisy_diff_drive.so``
        * ``libgazebo_ros_imu.so`` replaced with ``libnoisy_imu_sensor.so``
* ``turtlebot3_burger.urdf.xacro``
    * All file paths are replaced with the new package name.
* ``scripts`` and ``plugins`` folders are added where
    * ``scripts`` folder contains Poisson Based Noise ROS node and a config file
    * ``plugins`` folder contains Gazebo plugins for Poisson Based Noise interaction.
    
### Running the Scripts

In order to spawn the Turtlebot3 in Gazebo, run:

``roslaunch turtlebot3_highfidelityWheelEncoderIMU turtlebot_gazebo.launch``

This should initialize the Turtlebot3 in an empty Gazebo world with noisy plugins activated.
Then run ``sensor_uncertainty_generator`` ROS node script to initialize the poisson based noise 

``rosrun turtlebot3_highfidelityWheelEncoderIMU sensor_uncertainty_generator.py``

Characteristic of the sensor noise can be adjusted by parameters given in the ``sensor_uncertainty_config.json`` file under ``scripts`` folder. Parameters and their effects are given below.

* ``time_interval:`` Time interval in seconds used for poisson distribution

* ``imu_num_of_error_occurences:`` During time_interval, the number of error occurrences used by poisson distribution for the imu plugin

* ``diff_drive_num_of_error_occurences:`` During time_interval, the number of error occurrences used by poisson distribution for the differential drive plugin

* ``rate:`` Loop rate (Hz)

* ``imu:`` Activate/Deactivate noise in the imu plugin

* ``diff_drive:`` Activate/Deactivate noise in the differential drive plugin

* ``imu_noise_duration:``How long (in sec) the high noise is applied in the imu plugin

* ``diff_drive_noise_duration:``How long (in sec) the high noise is applied in the differential drive plugin

    * Standard deviation values of the low noise level in the IMU’s orientation

* ``imu_orientation_x_low_noise``
* ``imu_orientation_y_low_noise``
* ``imu_orientation_z_low_noise``
* ``imu_orientation_w_low_noise``

    * Standard deviation values of the low noise level in the IMU’s angular velocity
    
* ``imu_angular_velocity_x_low_noise``
* ``imu_angular_velocity_y_low_noise``
* ``imu_angular_velocity_z_low_noise``

    * Standard deviation values of the low noise level in the IMU’s linear acceleration 
    
* ``imu_linear_acceleration_x_low_noise``
* ``imu_linear_acceleration_y_low_noise``
* ``imu_linear_acceleration_z_low_noise``

    * Standard deviation values of the high noise level in the IMU’s orientation
    
* ``imu_orientation_x_high_noise``
* ``imu_orientation_y_high_noise``
* ``imu_orientation_z_high_noise``
* ``imu_orientation_w_high_noise``

    * Standard deviation values of the high noise level in the IMU’s angular velocity
    
* ``imu_angular_velocity_x_high_noise``
* ``imu_angular_velocity_y_high_noise``
* ``imu_angular_velocity_z_high_noise``

    * Standard deviation values of the high noise level in the IMU’s linear acceleration
    
* ``imu_linear_acceleration_x_high_noise``
* ``imu_linear_acceleration_y_high_noise``
* ``imu_linear_acceleration_z_high_noise``

    * Standard deviation values of the low noise level in the distance taken by left and right wheels
    
* ``diff_drive_left_wheel_low_noise``
* ``diff_drive_right_wheel_low_noise``

    * Standard deviation values of the high noise level in the distance taken by left and right wheels

* ``diff_drive_left_wheel_high_noise``
* ``diff_drive_right_wheel_high_noise``

    
### How to cite
You can cite our paper:

Ozdemir, E., Gokoglu, H., Yilmaz, M.K., Dumandag, U., Savci, I.H., Bayram, H. (2023). High Fidelity IMU and Wheel Encoder Models for ROS Based AGV Simulations. In: Koubaa, A. (eds) Robot Operating System (ROS). Studies in Computational Intelligence, vol 1051. Springer, Cham. https://doi.org/10.1007/978-3-031-09062-2_7
