#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <OptiTrackFeedBackRigidBody.h>
#include <OptiTrackPublisher.h>
#include <vector>

struct SubTopic
{
    char str[100];
};
struct PubTopic
{
    char str[100];
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "broadcast_position_velocity");
    ros::NodeHandle nh("~");

    // 频率
    ros::Rate rate(60.0);

    const std::string default_msg_type = "Mocap";
    const std::string msg_type = nh.param("msg_type", default_msg_type);
    //
    int notfeedbackcounter = 2;

    //
    // topic list based on arguments 
    std::vector<SubTopic> TopicList;
    /*mocap processor instences:
    NOTICE: unique_ptr is a feature in C++11, to use this, 
    add set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}") to the CMakeList*/ 
    std::vector<std::unique_ptr<OptiTrackFeedBackRigidBody>> MocapList;
    std::vector<std::unique_ptr<OptiTrackPublisher>> PubList;
    std::vector<rigidbody_state> RigidBodyStateList;
    std::vector<optitrack_broadcast::Mocap> MocapMessageList;
    int NumOfChannels = argc-1;
    // must have at least 1 input argument for topic name
    if ( argc > 1) {
        std::cout<<"++++++++RECEIVEING MOCAP INFO FROM THE FOLLOWING TOPICS:++++++++"<<std::endl;
        for( int i = 1; i< argc; i++) {
            SubTopic topic;
            strcpy (topic.str,"/vrpn_client_node/");
            strcat (topic.str,argv[i]);
            strcat (topic.str,"/pose");
            TopicList.push_back(topic);
            std::cout << topic.str;
            /* create an optitrack instance in the heap and push the pointer into the container
               emplace_back is also a feature in C++11 */ 
            MocapList.emplace_back(new OptiTrackFeedBackRigidBody(topic.str,nh,3,3));           
            /* create a list of rigid_body states*/
            rigidbody_state init_state;
            MocapList.back()-> GetState(init_state);
            RigidBodyStateList.push_back(init_state);

            /* creat a list of mocap messages */
            optitrack_broadcast::Mocap init_mocap;

            for ( int i = 0; i < 3; i++) {
                init_mocap.position[i] = 0;
                init_mocap.velocity[i] = 0;
                init_mocap.angular_velocity[i] = 0;
            }
            for ( int i = 0; i < 4; i++) {
                init_mocap.quaternion[i] = init_state.quaternion(i);
            }
            MocapMessageList.push_back(init_mocap);

            /* parse and advertise the publisher */
            PubTopic PubTopic;
            strcpy (PubTopic.str, "/mocap/");
            strcat (PubTopic.str, argv[i]);
            PubList.emplace_back(new OptiTrackPublisher(PubTopic.str,nh,1000, msg_type.c_str()));
            std::cout << " publish processed data to :  "<< PubTopic.str << std::endl;
        }
    } else {
        ROS_WARN("NO TOPIC NAME SPECIFIED!");
    }
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        ros::spinOnce();/********/
        // run the update only when there is at leat 1 instance of feedback specified
        bool isFeedbackStateChanged = false;
        bool isFeedbackAllGood = true;
        if (argc > 1) {
            for ( int k = 0; k < NumOfChannels; k++) {
                /* update the state for each optitrack instance */
                MocapList[k]-> FeedbackDetector(notfeedbackcounter);// store the previous state
                bool FeedBackStateTemp = RigidBodyStateList[k].isFeedbackNomral; 
                MocapList[k]-> GetState(RigidBodyStateList[k]); 
                /* if the buffer number is equal to or above the threshold, set the flag to false*/
                if ( !RigidBodyStateList[k].isFeedbackNomral ) {
                    isFeedbackAllGood = false;
                }
                /* detect any changes to the optitrack feedback*/
                if ( RigidBodyStateList[k].isFeedbackNomral != FeedBackStateTemp) {
                    isFeedbackStateChanged = true;
                }
                PubList[k]-> PublishData(RigidBodyStateList[k]);                      
            }            
        }
        /* If optitrack state changes, print the current info*/
        if ( isFeedbackStateChanged ) {
            std::cout<<"-----------------------------"<<std::endl;
            if ( isFeedbackAllGood ) {
                ROS_INFO("FEEDBACK NORMAL. RECEIVEING VRPN FROM THE FOLLOWING TOPICS: ");
            } else {
                ROS_WARN("FEEDBACK LOST!! RECEIVEING VRPN FROM THE FOLLOWING TOPICS: ");
            }
            for ( int i = 0; i < NumOfChannels; i++) {
                if ( RigidBodyStateList[i].isFeedbackNomral ){
                    ROS_INFO("NORMAL:  %s", TopicList[i].str);
                } else {
                    ROS_WARN("NO DATA: %s", TopicList[i].str);
                }
            }
        }

        rate.sleep();
    }
    return 0;
}
