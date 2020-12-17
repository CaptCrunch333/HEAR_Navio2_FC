
#include <iostream>

#include "HEAR_ROS_BRIDGE/ROSUnit_Factory.hpp"
#include "HEAR_NAVIO_Interface/ESCMotor.hpp"


int main(int argc, char** argv){
    std::cout << "Calibrating ESCs" << std::endl;

    ROSUnit_Factory ROSUnit_Factory_main{nh};


    Actuator* M1 = new ESCMotor(0, PWM_FREQUENCY);
    Actuator* M2 = new ESCMotor(1, PWM_FREQUENCY);
    Actuator* M3 = new ESCMotor(2, PWM_FREQUENCY);
    Actuator* M4 = new ESCMotor(3, PWM_FREQUENCY);
    Actuator* M5 = new ESCMotor(4, PWM_FREQUENCY);
    Actuator* M6 = new ESCMotor(5, PWM_FREQUENCY);

    std::vector<Actuator*> actuators{M1, M2, M3, M4, M5, M6};

        ROSUnit* ros_slam_pid_trigger_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "slam_pid_switch_z");



}