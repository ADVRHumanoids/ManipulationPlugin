#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <ADVR_ROS/advr_segment_control.h>

#include <trajectory_utils/segment.h>
#include <trajectory_utils/Cartesian.h>


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "simple_service_caller");
    ros::NodeHandle n;
    
    ////////////////////////////
    // in order to receive the pose of the obj to manipulate (e.g. handle)
    /////////////////////////////
    
    // blocking call: wait for a pose on topic handle_pose
    geometry_msgs::PoseStamped::ConstPtr p = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("handle_pose");
    
    // once we continue here the p will be filled by the publisher
    std::cout << p->pose.position.x << std::endl;
    
    
    ////////////////////////////
    // how to send a trajectory for the end effector as a segment
    ////////////////////////////
    
    
    // define in the init of your plugin and put the client in the shared data struct
    ros::ServiceClient client = n.serviceClient<ADVR_ROS::advr_segment_control>("segment_control");

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    start_frame.pose.position.x = 0.8;
    start_frame.pose.position.y = 0.2;
    start_frame.pose.position.z = 1.5;
    
    start_frame.pose.orientation.x = 0;
    start_frame.pose.orientation.y = 0;
    start_frame.pose.orientation.z = 0;
    start_frame.pose.orientation.w = 1;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = "l_handj";
    start.frame = start_frame;
    
    
    
    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    end_frame.pose.position.x = 1.0;
    end_frame.pose.position.y = 0.2;
    end_frame.pose.position.z = 1.0;
    
    end_frame.pose.orientation.x = 0;
    end_frame.pose.orientation.y = 0;
    end_frame.pose.orientation.z = 0;
    end_frame.pose.orientation.w = 1;
    
    trajectory_utils::Cartesian end;
    end.distal_frame = "l_handj";
    end.frame = end_frame;


    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 5.0;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = end;            // end pose 
    
    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);
    
    // prapere the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "base_link";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    client.call(srv);
    
    // in the run of your state spince once
    while(ros::ok()) {
        ros::spinOnce();
        sleep(1);
    }
    
    return 0;
}