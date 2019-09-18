#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <Mocap_emulator.h>
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
    ros::init(argc, argv, "emulator_position_velocity");
    ros::NodeHandle nh("~");

    // 频率
    ros::Rate rate(100.0);
    /*mocap processor instences:
    NOTICE: unique_ptr is a feature in C++11, to use this,
    add set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}") to the CMakeList*/
    std::vector<std::unique_ptr<Mocap_emulator>> EmulatorList;

    int NumOfChannels = argc-1;
    // must have at least 1 input argument for topic name
    if ( argc > 1) {
        std::cout<<"++++++++RECEIVEING GAZEBO PLUGIN INFO FROM THE FOLLOWING TOPICS:++++++++"<<std::endl;
        for( int i = 1; i< argc; i++) {
            SubTopic SubTopic;
            PubTopic PubTopic;

            strcpy (SubTopic.str,"/gazebo_ground_truth_");
            strcat (SubTopic.str,argv[i]);
            //strcat (topic.str,"");
            strcpy (PubTopic.str, "/mocap/");
            strcat (PubTopic.str, argv[i]);
            std::cout << SubTopic.str << " publish emulated data to :  "<< PubTopic.str << std::endl;
            /* create an optitrack instance in the heap and push the pointer into the container
               emplace_back is also a feature in C++11 */
            EmulatorList.emplace_back(new Mocap_emulator(PubTopic.str,SubTopic.str,nh,100));
        }
    } else {
        ROS_WARN("NO TOPIC NAME SPECIFIED!");
    }
    ROS_INFO("EMULATOR RUNNING...");
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
