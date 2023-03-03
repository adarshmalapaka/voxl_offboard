# VOXL1 Waypoint Following - Offboard Mode

Homework 01 for the course _ENAE788M: Hands-on Autonomous Aerial Robots (Spring 2023)_.

Using https://docs.px4.io/master/en/ros/mavros_offboard.html as reference, the code is modified to demonstrate the ModalAI VOXL drone beginning at (x: 0, y: 0, z: 10) and then follow the repeating pattern:
- Move forward 10 virtual meters
- Move up (heave) 15 virtual meters
- Move left 5 virtual meters
- Return to (0,0,10)

### Simulation Demo
<p align="center">
  <img src="https://user-images.githubusercontent.com/40534801/220497867-db602e25-23a4-436b-aed1-218f8a1b2543.gif" width="600" height="400">
</p>
<!-- <p align = "center">
Simulation Demo
</p> -->

### Actual Drone Demo 
<p align="center">
<a href="https://www.youtube.com/watch?v=OxwKLBLmSRM" target="_blank">
 <img src="https://user-images.githubusercontent.com/40534801/222610290-23971b0c-a4b2-468f-8402-08df58c8e68b.png" alt="Watch the Simulation" width="450" height="300" border="10" />
</a>
</p>
<!-- <p align = "center">
Demo on an actual ModalAI VOXL drone.
</p> -->


## Plots

### Trajectory v/s Time Plot

<p align="center">
  <img src="https://user-images.githubusercontent.com/40534801/220487968-b638adb0-17e4-4ceb-b03d-d665add99305.png" width="600" height="600">
</p>

### 3D Trajectory Plot

<p align="center">
  <img src="https://user-images.githubusercontent.com/40534801/220487934-8dcc68c7-337e-4780-ad9b-77483228bbeb.png" width="300" height="300">
</p>


## Developer Documentation

### Dependencies 
* [PX4 jmavsim](http://docs.px4.io/main/en/simulation/jmavsim.html)
* [QGroundControl](http://qgroundcontrol.com/)
* [ROS Noetic (roscpp)](http://wiki.ros.org/noetic)
* [MAVROS](https://github.com/mavlink/mavros)
* [os](https://docs.python.org/3/library/os.html)
* [pandas](https://pandas.pydata.org/)
* [bagpy](https://github.com/jmscslgroup/bagpy)
* [matplotlib](https://matplotlib.org/)

### Running the Codes

1. Open a terminal and clone the repository in the ```src``` directory of your ROS catkin workspace into a folder/package called ```offboard```.
  ```
  source /opt/ros/noetic/setup.bash
  cd ~/catkin_ws/src
  git clone https://github.com/adarshmalapaka/voxl_offboard.git offboard
  cd ..
  catkin_make 
  ```
2. To run the waypoint tracking demo:
    * Start the PX4 ```jmavsim``` simulator:
        ```
        cd /PX4-Autopilot
        make px4_sitl_default jmavsim
        ```
    * Launch QGroundControl by double clicking the ```QGroundControl.AppImage``` file.
    * Perform a take-off of the drone and wait until it reaches a height of 10m. 
    * Open another terminal and run the following to launch the offboard node and save the bag data:
        ```
        source /opt/ros/noetic/setup.bash
        roslaunch offboard hw1.launch savedata:=true
        ```

        Note: After executing the above roslaunch command, change the mode in QGroundControl from either Hold/Position to Offboard.
3. To read the generated bag file and plot the corresponding commanded and actual trajectory of the drone:
    ```
    cd ~/catkin_ws/src/offboard/src
    python3 enae788m_plot.py
    ```

## Package Structure
```
├─ CMakeLists.txt
├─ README.md
├─ bag
│  ├─ enae788m_hw1_VOXL1
│  │  ├─ mavros-local_position-pose.csv
│  │  └─ mavros-setpoint_raw-local.csv
│  └─ enae788m_hw1_VOXL1.bag                # Saved rosbag for /mavros/local_position/pose & /mavros/setpoint_raw/local
├─ launch
│  ├─ hw1.launch                            # Launch file to run enae788m_hw1.cpp and rosbag record 
│  ├─ record_data.launch
│  └─ total_launch.launch
├─ package.xml
└─ src
   ├─ enae788m_hw1.cpp                      # Code to perform waypoint tracking as given in HW1 
   ├─ enae788m_plot.py                      # Code to visualize recorded ROS topics using rosbag
   └─ offboard_example.cpp
