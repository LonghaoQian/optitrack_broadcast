#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <optitrack_broadcast/Mocap.h>
#include <OptiTrackFeedBackRigidBody.h>
#include <vector>

optitrack_broadcast::Mocap UAV_motion;
optitrack_broadcast::Mocap Payload_motion;

rigidbody_state UAVstate;
rigidbody_state Payloadstate;
struct SubTopic
{
    char str[100];
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "broadcast_position_velocity");
    ros::NodeHandle nh("~");

    // publisher
    ros::Publisher UAV_motion_pub = nh.advertise<optitrack_broadcast::Mocap>("/mocap/UAV", 1000);
    ros::Publisher Payload_motion_pub = nh.advertise<optitrack_broadcast::Mocap>("/mocap/Payload", 1000);
    // 频率
    ros::Rate rate(60.0);
    // topic list based on arguments 
    std::vector<SubTopic> TopicList;
    // mocap processor instences
    std::vector<OptiTrackFeedBackRigidBody*> MocapList;

    if (argc > 1)// must have at least 1 input argument for topic name
    {
        std::cout<<"++++++++RECEIVEING MOCAP INFO FROM THE FOLLOWING TOPICS:++++++++"<<std::endl;
        for( int i = 1; i< argc; i++) {
            SubTopic topic;
            strcpy (topic.str,"/vrpn_client_node/");
            strcat (topic.str,argv[i]);
            strcat (topic.str,"/pose");
            TopicList.push_back(topic);
            std::cout << topic.str << std::endl;
            //MocapList.push_back(new OptiTrackFeedBackRigidBody(TopicList[i].str,nh,3,3)); 
        }
        std::cout<<"-----------------------------------------------------------------"<<std::endl;
        // assign me
    } else {
        ROS_WARN("NO TOPIC NAME SPECIFIED!");
    }
    OptiTrackFeedBackRigidBody UAV(TopicList[0].str,nh,3,3);
    OptiTrackFeedBackRigidBody Payload("/vrpn_client_node/Payload/pose",nh,3,3);
    int notfeedbackcounter = 3;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        
        ros::spinOnce();

        // get optitrack feedback 
        UAV.FeedbackDetector(1);
        //Payload.FeedbackDetector(3);
        UAV.GetState(UAVstate);
        Payload.GetState(Payloadstate);
        UAV_motion.header.stamp = ros::Time::now();

        for(int i = 0;i<3;i++)
        {
            UAV_motion.position[i] = UAVstate.Position(i);
            UAV_motion.velocity[i] = UAVstate.V_I(i);
            UAV_motion.angular_velocity[i] =  UAVstate.Omega_BI(i);
            Payload_motion.position[i] = Payloadstate.Position(i);
            Payload_motion.velocity[i] = Payloadstate.V_I(i);
            Payload_motion.angular_velocity[i] =  Payloadstate.Omega_BI(i); 
        }
        for(int i = 0;i<4;i++)
        {
            UAV_motion.quaternion[i] = UAVstate.quaternion(i);
            Payload_motion.quaternion[i] = Payloadstate.quaternion(i);
        }
        UAV_motion_pub.publish(UAV_motion);
        Payload_motion_pub.publish(Payload_motion);
        rate.sleep();
    }
    return 0;
}
