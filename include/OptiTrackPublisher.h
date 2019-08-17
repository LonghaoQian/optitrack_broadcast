#pragma once
#include "ros/ros.h"
#include <optitrack_broadcast/Mocap.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <OptiTrackFeedBackRigidBody.h>
#include <iostream>
using namespace Eigen;
class OptiTrackPublisher{
    public: 
        OptiTrackPublisher(const char* TopicName,
                           ros::NodeHandle& n, 
                           unsigned int buffersize, 
                           const char* MessageType);
        ~OptiTrackPublisher();
        void PublishData(rigidbody_state& StateInput);
    private:
        ros::Publisher publisher_; 
        int messagetype_;
        optitrack_broadcast::Mocap MessageMocap_;
        nav_msgs::Odometry  MessageOdometry_;   
        geometry_msgs::Twist  MessageTwist_;
        double position[3];
        double velocity[3];
        double angular_velocity[3];
        double quaternion[4];
};


