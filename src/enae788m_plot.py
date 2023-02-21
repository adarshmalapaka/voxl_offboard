#!/usr/bin/env python3

import os
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt

os.chdir('bag')     # print(os.getcwd())
b = bagreader('enae788m_hw1_VOXL1.bag')

actual_pose_msg = b.message_by_topic(topic='/mavros/local_position/pose')
desired_pose_msg = b.message_by_topic(topic='/mavros/setpoint_raw/local')

desired_pose_data = pd.read_csv(desired_pose_msg)
actual_pose_data = pd.read_csv(actual_pose_msg)

desired_pose_data['Time'] = desired_pose_data['Time'] - desired_pose_data['Time'][0]
actual_pose_data['Time'] = actual_pose_data['Time'] - actual_pose_data['Time'][0]

# 3D Plot
plt.figure(1, figsize=(6,6))
ax = plt.axes(projection='3d')
ax.plot3D(desired_pose_data['position.x'], desired_pose_data['position.y'], desired_pose_data['position.z'], label='Commanded')
ax.plot3D(actual_pose_data['pose.position.x'], actual_pose_data['pose.position.y'], actual_pose_data['pose.position.z'], label='Actual')
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.set_title('VOXL1 Flight 3D Trajectory',fontsize='x-large',fontweight='bold')
ax.legend(loc='upper right')

# XYZ Position w.r.t. time
fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(10, 15))
plt.subplots_adjust(bottom=0.1, top=0.9)
fig.suptitle('VOXL1 Flight Temporal Trajectory',fontsize='x-large',fontweight='bold')
axes[0].plot(desired_pose_data['Time'], desired_pose_data['position.x'], label='Commanded')
axes[0].plot(actual_pose_data['Time'], actual_pose_data['pose.position.x'], label='Actual')
axes[0].set_xlabel('Time (sec)')
axes[0].set_ylabel('X Position (m)')
# axes[0].set_title('X Position')
axes[0].legend()
axes[0].grid()

axes[1].plot(desired_pose_data['Time'], desired_pose_data['position.y'], label='Commanded')
axes[1].plot(actual_pose_data['Time'], actual_pose_data['pose.position.y'], label='Actual')
axes[1].set_xlabel('Time (sec)')
axes[1].set_ylabel('Y Position (m)')
# axes[1].set_title('Y Position')
axes[1].legend()
axes[1].grid()

axes[2].plot(desired_pose_data['Time'], desired_pose_data['position.z'], label='Commanded')
axes[2].plot(actual_pose_data['Time'], actual_pose_data['pose.position.z'], label='Actual')
axes[2].set_xlabel('Time (sec)')
axes[2].set_ylabel('Z Position (m)')
# axes[2].set_title('Z Position')
axes[2].legend()
axes[2].grid()


plt.show()