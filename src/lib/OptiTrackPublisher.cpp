#include "OptiTrackPublisher.h"

OptiTrackPublisher::OptiTrackPublisher(const char* TopicName,
                                       ros::NodeHandle& n,
                                       unsigned int buffersize, 
                                       const char* MessageType)
{
    messagetype_ = 0;
    if (strcmp(MessageType, "Mocap") == 0) {
        messagetype_ = 0;
    }
    if (strcmp(MessageType, "Odometry") == 0) {
        messagetype_ = 1;
    }
    if (strcmp(MessageType, "Twist") == 0) {
        messagetype_ = 2;
    }
    switch (messagetype_) {
        case 0: {
            publisher_ = n.advertise<optitrack_broadcast::Mocap>(TopicName, buffersize);
            break;
        }
        case 1: {
            publisher_ = n.advertise<nav_msgs::Odometry>(TopicName, buffersize);
            break;
        }
        case 2: {
            publisher_ = n.advertise<geometry_msgs::Twist>(TopicName, buffersize);
            break;
        }
        default: {
            publisher_ = n.advertise<optitrack_broadcast::Mocap>(TopicName, buffersize);
            break;
        }
    }
}
OptiTrackPublisher::~OptiTrackPublisher()
{

}
void OptiTrackPublisher::PublishData(rigidbody_state& StateInput)
{
    for ( int i = 0; i < 3; i++) {

        position[i] = StateInput.Position(i);
        velocity[i] = StateInput.V_I(i);
        angular_velocity[i] = StateInput.Omega_BI(i);
    }
    for( int i = 0; i < 4; i++) {
        quaternion[i] = StateInput.quaternion(i);
    }    
    
    switch (messagetype_) {
        case 0: {
                for ( int i = 0; i < 3; i++) {
                    MessageMocap_.position[i] = StateInput.Position(i);
                    MessageMocap_.velocity[i] = StateInput.V_I(i);
                    MessageMocap_.angular_velocity[i] = StateInput.Omega_BI(i);
                }
                for( int i = 0; i < 4; i++) {
                    MessageMocap_.quaternion[i] = StateInput.quaternion(i);
                }
            MessageMocap_.header.stamp = ros::Time::now();// use time now as time stamp
            publisher_.publish(MessageMocap_);
            break;
        }
        case 1: {
            geometry_msgs::Pose& pose = MessageOdometry_.pose.pose;
            geometry_msgs::Twist& twist = MessageOdometry_.twist.twist;
            for (int i = 0; i < 4; ++i)
                *(&pose.orientation.x + (i + 3) % 4) = StateInput.quaternion[i];
            Eigen::Map<Eigen::Vector3d>(&pose.position.x) = StateInput.Position;
            Eigen::Map<Eigen::Vector3d>(&twist.linear.x) = StateInput.V_I;
            Eigen::Map<Eigen::Vector3d>(&twist.angular.x) = StateInput.Omega_BI;

            MessageOdometry_.header.stamp = ros::Time::now();
            publisher_.publish(MessageOdometry_);
            break;
        }
        case 2: {
            /* TO DO */
            publisher_.publish(MessageTwist_);
            break;
        }
        default: {
                for ( int i = 0; i < 3; i++) {
                    MessageMocap_.position[i] = StateInput.Position(i);
                    MessageMocap_.velocity[i] = StateInput.V_I(i);
                    MessageMocap_.angular_velocity[i] = StateInput.Omega_BI(i);
                }
                for( int i = 0; i < 4; i++) {
                    MessageMocap_.quaternion[i] = StateInput.quaternion(i);
                }
            MessageMocap_.header.stamp = ros::Time::now();
            publisher_.publish(MessageMocap_);
            break;
        }
    }
}
