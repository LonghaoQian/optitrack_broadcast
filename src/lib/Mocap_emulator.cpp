#include "Mocap_emulator.h"

Mocap_emulator::Mocap_emulator(const char* PubTopicName,
                               const char* SubTopicName,
                               ros::NodeHandle& n,
                               unsigned int buffersize)
{
    pubmocap_ = n.advertise<optitrack_broadcast::Mocap>(PubTopicName, buffersize);
    subgazebo_ = n.subscribe(SubTopicName, buffersize, &Mocap_emulator::SubscribeFromGazebo,this);
}
Mocap_emulator::~Mocap_emulator()
{

}
void Mocap_emulator::PublishData()
{
    MessageMocap_.position[0] = Drone_state_.pose.pose.position.x;
    MessageMocap_.position[1] = Drone_state_.pose.pose.position.y;
    MessageMocap_.position[2] = Drone_state_.pose.pose.position.z;
    MessageMocap_.velocity[0] = Drone_state_.twist.twist.linear.x;
    MessageMocap_.velocity[1] = Drone_state_.twist.twist.linear.y;
    MessageMocap_.velocity[2] = Drone_state_.twist.twist.linear.z;

    MessageMocap_.angular_velocity[0] = Drone_state_.twist.twist.angular.x;
    MessageMocap_.angular_velocity[1] = Drone_state_.twist.twist.angular.y;
    MessageMocap_.angular_velocity[2] = Drone_state_.twist.twist.angular.z;
    MessageMocap_.quaternion[0] = Drone_state_.pose.pose.orientation.w;
    MessageMocap_.quaternion[1] = Drone_state_.pose.pose.orientation.x;
    MessageMocap_.quaternion[2] = Drone_state_.pose.pose.orientation.y;
    MessageMocap_.quaternion[3] = Drone_state_.pose.pose.orientation.z;
    MessageMocap_.header = Drone_state_.header;
    pubmocap_.publish(MessageMocap_);
}
void Mocap_emulator::SubscribeFromGazebo(const nav_msgs::Odometry& msg)
{
    // waiting for plugin message
    Drone_state_ = msg;
    // publish the plugin message
    PublishData();
}
