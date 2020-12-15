#include <iostream>

#include "ros/ros.h"
#include "HEAR_core/Timer.hpp"
#include "HEAR_core/std_logger.hpp"
#include "HEAR_nav/Global2Inertial.hpp"
#include "HEAR_nav/RestrictedNormWaypointRefGenerator.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_Factory.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_Optitrack.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_RestNormSettings.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "global2inertial_node");
    if(ros::ok()){
    ros::NodeHandle nh;
    ros::Rate rate(120);
    ROSUnit_Factory ROSUnit_Factory_main(nh);
    ROSUnit* myROSOptitrack = new ROSUnit_Optitrack(nh);
    ROSUnit* rosunit_g2i_position = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "global2inertial/position");
    ROSUnit* rosunit_g2i_orientation = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "global2inertial/orientation");                                                                
    ROSUnit* rosunit_set_height_offset = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                            ROSUnit_msg_type::ROSUnit_Float,
                                                                            "set_height_offset");
    Global2Inertial* myGlobal2Inertial = new Global2Inertial();
    myROSOptitrack->getPorts()[(int)ROSUnit_Optitrack::ports_id::OP_0_OPT]->connect(myGlobal2Inertial->getPorts()[(int)Global2Inertial::ports_id::IP_0_OPTI_MSG]);
    myGlobal2Inertial->getPorts()[(int)Global2Inertial::ports_id::OP_0_OPTIPOS]->connect(rosunit_g2i_position->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    myGlobal2Inertial->getPorts()[(int)Global2Inertial::ports_id::OP_1_OPTIHEADING]->connect(rosunit_g2i_orientation->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    rosunit_set_height_offset->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_0]->connect(myGlobal2Inertial->getPorts()[(int)Global2Inertial::ports_id::IP_1_FLOAT_DATA]);
    std::cout  << "###### GLOBAL2INERTIAL NODE ######" "\n";
    Timer tempo;
    int i = 0;
    while(ros::ok()){
        tempo.tick();
        ros::spinOnce();
        int gone = tempo.tockMicroSeconds();
        if(gone > 8333) {
            std::cout  << i <<  " G2I: " << gone << "\n";
        }
        i++;
        rate.sleep();
    }
    return 0;
    }
}