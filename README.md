# VOXL1 Waypoint Following - Offboard Mode

Homework 01 for the course _ENAE788M: Hands-on Autonomous Aerial Robots (Spring 2023)_.


## Plots

### Trajectory v/s Time Plot

<p align="center">
  <img src="https://user-images.githubusercontent.com/40534801/190576008-6ef15b41-68dc-401e-ae69-06a554e4d8a9.png" width="600" height="400">
</p

### 3D Trajectory Plot

<p align="center">
  <img src="https://user-images.githubusercontent.com/40534801/190575327-45e55da9-9828-4041-b77c-a54f6546acfa.jpeg" width="600" height="300">
</p>


## Developer Documentation

### Dependencies 
* PX4 jmavsim
* QGroundControl
* roscpp
* os
* pandas
* bagpy
* matplotlib

### Running the Codes

1. To run the waypoint tracking demo:
    * Start the PX4 ```jmavsim``` simulator:
        ```
        cd /PX4-Autopilot
        make px4_sitl_default jmavsim
        ```
    * Launch QGroundControl by double clicking the ```QGroundControl.AppImage``` file.
    * Open another terminal and run the following to launch the offboard node and save the bag data:
        ```
        cd ~/catkin_ws
        catkin_make
        roslaunch offboard hw1.launch savedata:=true
        ```

2. To read the generated bag file and plot the corresponding commanded and actual trajectory of the drone:
    ```
    cd /src
    python3 enae788m_plot.py
    ```

## Package Structure