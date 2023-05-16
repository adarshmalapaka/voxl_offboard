/**
 * @file enae788m_hw1.cpp
 * @brief HW1 for the ENAE788M course at UMD to manuever the drone to follow a repeating pattern of waypoints using MAVROS.
          Based on the Offboard control example node (offb_node.cpp)
 * @author Adarsh Malapaka (UID: 118119625)
 * @cite https://docs.px4.io/v1.12/en/ros/mavros_offboard.html
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>

mavros_msgs::State current_state;
mavros_msgs::PositionTarget pose_vel;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

double feet2meters(double feet){
    return 0.3048*feet;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_hw5");
    ros::NodeHandle nh;

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher local_pos_pub_mavros = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 5);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
   //  while(ros::ok() && !current_state.connected){
   //     ros::spinOnce();
   //     rate.sleep();
   //  }

    mavros_msgs::PositionTarget pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0.6;
    pose.type_mask = pose.IGNORE_VX | pose.IGNORE_VY | pose.IGNORE_VZ | pose.IGNORE_AFZ | pose.IGNORE_AFY | pose.IGNORE_AFX;
    pose.coordinate_frame = 1;
    pose.yaw = 3.141592/2;

    // send a few setpoints before starting
    // for(int i = 200; ros::ok() && i > 0; --i){
    //    local_pos_pub.publish(pose);
    //    ros::spinOnce();
    //    rate.sleep();
    // }

    //pose_vel.coordinate_frame = pose_vel.FRAME_LOCAL_NED;
    //pose_vel.type_mask =  pose_vel.IGNORE_AFX | pose_vel.IGNORE_AFY | pose_vel.IGNORE_AFZ | pose_vel.FORCE | pose_vel.IGNORE_YAW | pose_vel.IGNORE_PX | pose_vel.IGNORE_PY | pose_vel.IGNORE_PZ;


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time init_time;

    // int count = 1;
    
    // std::vector<std::vector<float>> waypoints{{0, 0, 3.141592/2},
    //                                    {0, 14.0, 3.141592/2}, 
    //                                    {2.0, 14.0, 0.0}, 
    //                                    {2.0, 0, -3.141592/2}, 
    //                                    {4.0, 0, 0.0},
    //                                    {4.0, 14.0, 3.141592/2},
    //                                    {6.0, 14.0, 0.0},
    //                                    {6.0, 0, -3.141592/2},
    //                                    {8.0, 0, 0.0},
    //                                    {8.0, 14.0, 3.141592/2}};

    std::vector<std::vector<float>> waypoints{{0, 0, 3.141592/2},
                                       {0, 14.0, 3.141592/2}, 
                                       {4.0, 14.0, 0.0},
                                       {4.0, 0.0, -3.141592/2},
                                       {8.0, 0, 0.0},
                                       {8.0, 14.0, 3.141592/2}};

   //  if( current_state.mode != "OFFBOARD"){
   //    if( set_mode_client.call(offb_set_mode) &&
   //       offb_set_mode.response.mode_sent){
   //       ROS_INFO("Offboard enabled");
   //    }
   // } else {
   //       if( !current_state.armed){
   //          if( arming_client.call(arm_cmd) &&
   //                arm_cmd.response.success){
   //                ROS_INFO("Vehicle armed");
   //          }
   //       }
   //    }
    int count = 0;
    while(ros::ok()){ // && current_state.mode == "OFFBOARD"){
      //   if( current_state.mode != "OFFBOARD"){
      //       if( set_mode_client.call(offb_set_mode) &&
      //           offb_set_mode.response.mode_sent){
      //           ROS_INFO("Offboard enabled");
      //       }
      //       // last_request = ros::Time::now();
      //   } else {
      //       if( !current_state.armed){
      //           if( arming_client.call(arm_cmd) &&
      //               arm_cmd.response.success){
      //               ROS_INFO("Vehicle armed");
      //           }
      //          //  last_request = ros::Time::now();
      //       }
      //   }
        ROS_INFO_STREAM("Drone Mode: " << current_state.mode);

        // if (count<300){
        //     if (count == 1) {
        //         ROS_INFO_STREAM("(0, 0, pi/2)");
        //     }
        //     pose.position.x = waypoints[0][0];
        //     pose.position.y = waypoints[0][1];
        //     pose.position.z = 0.6;
        //     pose.yaw = waypoints[0][2];
        // }
        // else if (count<600){
        //     if (count == 300) {
        //         ROS_INFO_STREAM("{0, 14.0, pi/2}"); 
        //     }
        //     pose.position.x = waypoints[1][0];
        //     pose.position.y = waypoints[1][1];
        //     pose.position.z = 0.6;
        //     pose.yaw = waypoints[1][2];
        // }
        // else if (count<900){
        //     if (count == 600) {
        //         ROS_INFO_STREAM("{2.0, 14.0, 0.0}"); 
        //     }
        //     pose.position.x = waypoints[2][0];
        //     pose.position.y = waypoints[2][1];
        //     pose.position.z = 0.6;
        //     pose.yaw = waypoints[2][2];
        // }
        // else if (count<1200){
        //     if (count == 900) {
        //         ROS_INFO_STREAM("{2.0, 0, -pi/2}"); 
        //     }
        //     pose.position.x = waypoints[3][0];
        //     pose.position.y = waypoints[3][1];
        //     pose.position.z = 0.6;
        //     pose.yaw = waypoints[3][2];
        // // Include publishing pose during delay to prevent drone from going to position mode  
        // }
        // else if (count<1500){
        //     if (count == 1200) {
        //         ROS_INFO_STREAM("{4.0, 0, 0.0}"); 
        //     }
        //     pose.position.x = waypoints[4][0];
        //     pose.position.y = waypoints[4][1];
        //     pose.position.z = 0.6;
        //     pose.yaw = waypoints[4][2];
        //     // Include publishing pose during delay to prevent drone from going to position mode  
        // }
        // else if (count<1800){
        //     if (count == 1500) {
        //         ROS_INFO_STREAM("{4.0, 14.0, pi/2}"); 
        //     }
        //     pose.position.x = waypoints[5][0];
        //     pose.position.y = waypoints[5][1];
        //     pose.position.z = 0.6;
        //     pose.yaw = waypoints[5][2];
        //     // Include publishing pose during delay to prevent drone from going to position mode  
        // }
        // else if (count<2100){
        //     if (count == 1800) {
        //         ROS_INFO_STREAM("{6.0, 14.0, 0.0}"); 
        //     }
        //     pose.position.x = waypoints[6][0];
        //     pose.position.y = waypoints[6][1];
        //     pose.position.z = 0.6;
        //     pose.yaw = waypoints[6][2];
        //     // Include publishing pose during delay to prevent drone from going to position mode  
        // }
        // else if (count<2400){
        //     if (count == 2100) {
        //         ROS_INFO_STREAM("{6.0, 0, -pi/2}"); 
        //     }
        //     pose.position.x = waypoints[7][0];
        //     pose.position.y = waypoints[7][1];
        //     pose.position.z = 0.6;
        //     pose.yaw = waypoints[7][2];
        //     // Include publishing pose during delay to prevent drone from going to position mode  
        // }
        // else if (count<2700){
        //     if (count == 2400) {
        //         ROS_INFO_STREAM("{8.0, 0, 0.0}"); 
        //     }
        //     pose.position.x = waypoints[8][0];
        //     pose.position.y = waypoints[8][1];
        //     pose.position.z = 0.6;
        //     pose.yaw = waypoints[8][2];
        //     // Include publishing pose during delay to prevent drone from going to position mode  
        // }
        // else if (count<3000){
        //     if (count == 2700) {
        //         ROS_INFO_STREAM("{8.0, 14.0, pi/2}"); 
        //     }
        //     pose.position.x = waypoints[9][0];
        //     pose.position.y = waypoints[9][1];
        //     pose.position.z = 0.6;
        //     pose.yaw = waypoints[9][2];
        //     // Include publishing pose during delay to prevent drone from going to position mode  
        // }
        // else{
        //     count=0;
        // }

        for (auto point : waypoints) {
            while (count < 300) {
                if (count == 0) {
                    ROS_INFO_STREAM("{" << point[0] << ", " << point[1] << ", " << point[2] << "}");
                }
                pose.position.x = feet2meters(point[0]);
                pose.position.y = feet2meters(point[1]);
                pose.position.z = 0.6;
                pose.yaw = point[2];
                local_pos_pub_mavros.publish(pose);

                count++;
                ros::spinOnce();
                rate.sleep();
            }
            count = 0;
        }
        // if (count<300){
        //     if (count == 1) {
        //         ROS_INFO_STREAM("(0, 0, pi/2)");
        //     }
        //     pose.position.x = waypoints[0][0];
        //     pose.position.y = waypoints[0][1];
        //     pose.position.z = 0.6;
        //     pose.yaw = waypoints[0][2];
        // }
        // else if (count<600){
        //     if (count == 300) {
        //         ROS_INFO_STREAM("{0, 14.0, pi/2}"); 
        //     }
        //     pose.position.x = waypoints[1][0];
        //     pose.position.y = waypoints[1][1];
        //     pose.position.z = 0.6;
        //     pose.yaw = waypoints[1][2];
        // }
        // else if (count<900){
        //     if (count == 600) {
        //         ROS_INFO_STREAM("{4.0, 14.0, 0.0}"); 
        //     }
        //     pose.position.x = waypoints[2][0];
        //     pose.position.y = waypoints[2][1];
        //     pose.position.z = 0.6;
        //     pose.yaw = waypoints[2][2];
        // }
        // else if (count<1200){
        //     if (count == 900) {
        //         ROS_INFO_STREAM("{4.0, 0.0, -pi/2}"); 
        //     }
        //     pose.position.x = waypoints[3][0];
        //     pose.position.y = waypoints[3][1];
        //     pose.position.z = 0.6;
        //     pose.yaw = waypoints[3][2];
        // // Include publishing pose during delay to prevent drone from going to position mode  
        // }
        // else if (count<1500){
        //     if (count == 1200) {
        //         ROS_INFO_STREAM("{8.0, 0, 0.0}"); 
        //     }
        //     pose.position.x = waypoints[4][0];
        //     pose.position.y = waypoints[4][1];
        //     pose.position.z = 0.6;
        //     pose.yaw = waypoints[4][2];
        //     // Include publishing pose during delay to prevent drone from going to position mode  
        // }
        // else if (count<1800){
        //     if (count == 1500) {
        //         ROS_INFO_STREAM("{8.0, 14.0, pi/2}"); 
        //     }
        //     pose.position.x = waypoints[5][0];
        //     pose.position.y = waypoints[5][1];
        //     pose.position.z = 0.6;
        //     pose.yaw = waypoints[5][2];
        //     // Include publishing pose during delay to prevent drone from going to position mode  
        // }
        // else{
        //     count=0;
        // }


        // if ((count+1)%300 == 0){
        //     std::cout<<count<<std::endl;
        //     ROS_INFO_STREAM("Waiting for 1 second");
        //     init_time = ros::Time::now();
        //     std::cout<<"count"<<std::endl;
        //     while(ros::Time::now() - init_time < ros::Duration(0.5)) {
        //        local_pos_pub_mavros.publish(pose);
        //     }
        // } else {
        // local_pos_pub_mavros.publish(pose);
        // }

        // pose.position.x = feet2meters(waypoints[count][0]);
        // pose.position.y = feet2meters(waypoints[count][1]);
        // pose.position.z = 0.6;

        // pose.yaw = waypoints[count][2];
        
        // local_pos_pub_mavros.publish(pose);

        // count++;
        // if (count == waypoints.size()){
        //     ROS_INFO_STREAM("Exiting");
        //     break;
        // }

        // ros::spinOnce();
        // rate.sleep();
   }
    return 0;
}
