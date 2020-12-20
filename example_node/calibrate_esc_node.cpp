
#include <iostream>

#include "HEAR_ROS_BRIDGE/ROSUnit_Factory.hpp"
#include "HEAR_NAVIO_Interface/ESCMotor.hpp"
#include "HEAR_math/Ramp.hpp"
#include "HEAR_actuation/EscCalib.hpp"

const int PWM_FREQUENCY = 200;

int main(int argc, char** argv){
    std::cout << "Calibrating ESCs" << std::endl;
    
    ros::init(argc, argv, "Esc_calibration_node");
    ros::NodeHandle nh;
    ros::Rate rate(200);
    ROSUnit_Factory ROSUnit_Factory_main{nh};


    Actuator* M1 = new ESCMotor(0, PWM_FREQUENCY);
    Actuator* M2 = new ESCMotor(1, PWM_FREQUENCY);
    Actuator* M3 = new ESCMotor(2, PWM_FREQUENCY);
    Actuator* M4 = new ESCMotor(3, PWM_FREQUENCY);
    Actuator* M5 = new ESCMotor(4, PWM_FREQUENCY);
    Actuator* M6 = new ESCMotor(5, PWM_FREQUENCY);

    std::vector<Actuator*> actuators{M1, M2, M3, M4, M5, M6};

    EscCalib* esc_calib = new EscCalib(actuators);

    ROSUnit* calib_signal = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                ROSUnit_msg_type::ROSUnit_Bool,
                                                                "calib_signal");
    ROSUnit* set_mode = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                            ROSUnit_msg_type::ROSUnit_Int8, 
                                                            "select_mode");
    ROSUnit* set_channel = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                            ROSUnit_msg_type::ROSUnit_Int8, 
                                                            "select_channel");
                                                            
    auto ramp_signal = new Ramp(1, 100, 1200, 1000);

    calib_signal->getPorts()[ROSUnit_SetBoolSrv::OP_0]->connect(esc_calib->getPorts()[EscCalib::ports_id::IP_1_TRIGGER]);
    calib_signal->getPorts()[ROSUnit_SetBoolSrv::OP_0]->connect(ramp_signal->getPorts()[Ramp::ports_id::IP_0_TRIGGER]);
    set_mode->getPorts()[ROSUnit_SetInt8Srv::OP_0]->connect(esc_calib->getPorts()[EscCalib::ports_id::IP_0_MODE]);
    ramp_signal->getPorts()[Ramp::ports_id::OP_0_DATA]->connect(esc_calib->getPorts()[EscCalib::ports_id::IP_2_SIGNAL]);

    set_channel->getPorts()[ROSUnit_SetInt8Srv::OP_1]->connect(esc_calib->getPorts()[EscCalib::ports_id::IP_3_CHANNEL]);

    ros::spin();

}