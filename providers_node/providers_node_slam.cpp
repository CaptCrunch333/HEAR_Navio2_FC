//PROVIDERS_NODE V1.0.0
// 06 Dec 2020
// Muhammad Ahmed Humais

#include <iostream>

#include "HEAR_core/Timer.hpp"
#include "HEAR_core/Mux3D.hpp"
#include "HEAR_core/Demux3D.hpp"
#include "HEAR_core/std_logger.hpp"
#include "HEAR_nav/Global2Inertial.hpp"
#include "HEAR_nav/WrapAroundFunction.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_IMU.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_SLAM.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_Factory.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_Optitrack.hpp"
#include "HEAR_math/Differentiator.hpp"
#include "HEAR_math/ButterFilter_2nd.hpp"
#include "HEAR_math/KalmanFilter.hpp"
#include "HEAR_math/InverseRotateVec.hpp"

const int OPTITRACK_FREQUENCY = 120;

int main(int argc, char **argv){
    ros::init(argc, argv, "providers_node");
    ros::NodeHandle nh;
    ros::Rate rate(200);
    ROSUnit_Factory ROSUnit_Factory_main{nh};
    ROSUnit* rosunit_x_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/x");
    ROSUnit* rosunit_y_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/y");
    ROSUnit* rosunit_z_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/z");
    ROSUnit* rosunit_provider_slam_x_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/slam/x");
    ROSUnit* rosunit_provider_slam_y_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/slam/y");
    ROSUnit* rosunit_provider_slam_z_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/slam/z");
    ROSUnit* rosunit_roll_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/roll");
    ROSUnit* rosunit_pitch_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/pitch");
    ROSUnit* rosunit_yaw_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw");
    ROSUnit* rosunit_provider_slam_yaw_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/slam/yaw");
    ROSUnit* rosunit_yaw_rate_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw_rate");
    ROSUnit* z_KalmanFilter = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "/KalmanFilter/z");
    ROSUnit* rosunit_g2i_position = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "global2inertial/position");
    ROSUnit* rosunit_g2i_orientation = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "global2inertial/orientation");
    ROSUnit* rosunit_imu_acceleration = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_GeoVec,
                                                                    "/imu/acceleration");
    ROSUnit* rosunit_set_map_frame_offset = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                            ROSUnit_msg_type::ROSUnit_Bool,
                                                                            "set_map_frame_offset");
                                                                        
    //***********************ADDING SENSORS********************************
    ROSUnit* myROSUnit_Xsens = new ROSUnit_IMU(nh);

    ROSUnit* SLAM_ROSUnit = new ROSUnit_SLAM(nh);
    rosunit_g2i_position->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(SLAM_ROSUnit->getPorts()[(int)ROSUnit_SLAM::ports_id::IP_0_POS]);
    rosunit_g2i_orientation->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(SLAM_ROSUnit->getPorts()[(int)ROSUnit_SLAM::ports_id::IP_1_ORI]);
    rosunit_set_map_frame_offset->getPorts()[(int)ROSUnit_SetBoolSrv::ports_id::OP_0]->connect(SLAM_ROSUnit->getPorts()[(int)ROSUnit_SLAM::ports_id::IP_2_TRIGGER]);

    //***********************SETTING PROVIDERS**********************************
    Mux3D* mux_provider_x = new Mux3D();
    Mux3D* mux_provider_slam_x = new Mux3D();
    Mux3D* mux_provider_y = new Mux3D();
    Mux3D* mux_provider_slam_y = new Mux3D();
    Mux3D* mux_provider_z = new Mux3D();
    Mux3D* mux_provider_slam_z = new Mux3D();
    Mux3D* mux_provider_roll = new Mux3D();
    Mux3D* mux_provider_pitch = new Mux3D();
    Mux3D* mux_provider_yaw = new Mux3D();
    Mux3D* mux_provider_slam_yaw = new Mux3D();
    Mux3D* mux_provider_yaw_rate = new Mux3D();

    Demux3D* pos_demux = new Demux3D();
    Demux3D* slam_pos_demux = new Demux3D();
    Demux3D* slam_vel_demux = new Demux3D();
    Demux3D* ori_demux = new Demux3D();
    Demux3D* slam_ori_demux = new Demux3D();
    Demux3D* rotated_IMU_demux = new Demux3D();

    KalmanFilter* z_kalmanFilter= new KalmanFilter(1);

    WrapAroundFunction* wrap_around_yaw = new WrapAroundFunction(-M_PI, M_PI);
    WrapAroundFunction* wrap_around_yaw_slam = new WrapAroundFunction(-M_PI, M_PI);
    
    Differentiator* optitrack_x_dot = new Differentiator(1./OPTITRACK_FREQUENCY);
    Differentiator* optitrack_y_dot = new Differentiator(1./OPTITRACK_FREQUENCY);
    Differentiator* optitrack_z_dot = new Differentiator(1./OPTITRACK_FREQUENCY);

    ButterFilter_2nd* filter_x_dot = new ButterFilter_2nd(ButterFilter_2nd::BF_settings::FS120FC5);
    ButterFilter_2nd* filter_y_dot = new ButterFilter_2nd(ButterFilter_2nd::BF_settings::FS120FC5);
    ButterFilter_2nd* filter_z_dot = new ButterFilter_2nd(ButterFilter_2nd::BF_settings::FS120FC5);

    ButterFilter_2nd* filter_roll_dot = new ButterFilter_2nd(ButterFilter_2nd::BF_settings::FS200FC50);
    ButterFilter_2nd* filter_pitch_dot = new ButterFilter_2nd(ButterFilter_2nd::BF_settings::FS200FC50);
    ButterFilter_2nd* filter_yaw_dot = new ButterFilter_2nd(ButterFilter_2nd::BF_settings::FS200FC50);

    InverseRotateVec* rotation_IMU = new InverseRotateVec();

    rosunit_g2i_position->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(pos_demux->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);
    rosunit_g2i_orientation->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(ori_demux->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    SLAM_ROSUnit->getPorts()[(int)ROSUnit_SLAM::ports_id::OP_0_POS]->connect(slam_pos_demux->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);
    SLAM_ROSUnit->getPorts()[(int)ROSUnit_SLAM::ports_id::OP_1_ORI]->connect(slam_ori_demux->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);
    SLAM_ROSUnit->getPorts()[(int)ROSUnit_SLAM::ports_id::OP_2_VEL]->connect(slam_vel_demux->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    // Setting Provider -> Always leave the pv connection last. Do pv_dot and pv_dot_dor first.
    // X Provider
    pos_demux->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(optitrack_x_dot->getPorts()[(int)Differentiator::ports_id::IP_0_DATA]);
    optitrack_x_dot->getPorts()[(int)Differentiator::ports_id::OP_0_DATA]->connect(((Block*)filter_x_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::IP_0_DATA]);
    ((Block*)filter_x_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::OP_0_DATA]->connect(mux_provider_x->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    pos_demux->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(mux_provider_x->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);

    slam_pos_demux->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(mux_provider_slam_x->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    slam_vel_demux->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(mux_provider_slam_x->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);

    //Y Provider
    pos_demux->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(optitrack_y_dot->getPorts()[(int)Differentiator::ports_id::IP_0_DATA]);
    optitrack_y_dot->getPorts()[(int)Differentiator::ports_id::OP_0_DATA]->connect(((Block*)filter_y_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::IP_0_DATA]);
    ((Block*)filter_y_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::OP_0_DATA]->connect(mux_provider_y->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    pos_demux->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(mux_provider_y->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);

    slam_pos_demux->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(mux_provider_slam_y->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    slam_vel_demux->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(mux_provider_slam_y->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);

    //Z Provider
    pos_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(optitrack_z_dot->getPorts()[(int)Differentiator::ports_id::IP_0_DATA]);
    optitrack_z_dot->getPorts()[(int)Differentiator::ports_id::OP_0_DATA]->connect(((Block*)filter_z_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::IP_0_DATA]);
    ((Block*)filter_z_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::OP_0_DATA]->connect(mux_provider_z->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    pos_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_1_POS]);
    rotated_IMU_demux->getPorts()[Demux3D::ports_id::OP_2_DATA]->connect(z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_0_ACC]);
    pos_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(mux_provider_z->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_0_VEL]->connect(z_KalmanFilter->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);

    slam_pos_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(mux_provider_slam_z->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    slam_vel_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(mux_provider_slam_z->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);

    //Roll Provider
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_2_ROLL_RATE]->connect(((Block*)filter_roll_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::IP_0_DATA]);
    ((Block*)filter_roll_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::OP_0_DATA]->connect(mux_provider_roll->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_0_ROLL]->connect(mux_provider_roll->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);

    //Pitch Provider
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_3_PITCH_RATE]->connect(((Block*)filter_pitch_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::IP_0_DATA]);
    ((Block*)filter_pitch_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::OP_0_DATA]->connect(mux_provider_pitch->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_1_PITCH]->connect(mux_provider_pitch->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);

    //Yaw Provider
    slam_ori_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(wrap_around_yaw_slam->getPorts()[(int)WrapAroundFunction::ports_id::IP_0_DATA]);
    wrap_around_yaw_slam->getPorts()[(int)WrapAroundFunction::ports_id::OP_0_DATA]->connect(mux_provider_slam_yaw->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    ///**** Should I change this to come form ROSUnit_SLAM ? ********///
    ori_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(wrap_around_yaw->getPorts()[(int)WrapAroundFunction::ports_id::IP_0_DATA]);
    wrap_around_yaw->getPorts()[(int)WrapAroundFunction::ports_id::OP_0_DATA]->connect(mux_provider_yaw->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);

    //Yaw Rate Provider
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_4_YAW_RATE]->connect(((Block*)filter_yaw_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::IP_0_DATA]);
    ((Block*)filter_yaw_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::OP_0_DATA]->connect(mux_provider_yaw_rate->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    
    //Rotated imu vector
    rosunit_imu_acceleration->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_2]->connect(rotation_IMU->getPorts()[(int)InverseRotateVec::ports_id::IP_0_VEC]);
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_0_ROLL]->connect(rotation_IMU->getPorts()[(int)InverseRotateVec::ports_id::IP_1_ROLL]);
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_1_PITCH]->connect(rotation_IMU->getPorts()[(int)InverseRotateVec::ports_id::IP_2_PITCH]);
    wrap_around_yaw->getPorts()[(int)WrapAroundFunction::ports_id::OP_0_DATA]->connect(rotation_IMU->getPorts()[(int)InverseRotateVec::ports_id::IP_3_YAW]);
    rotation_IMU->getPorts()[(int)InverseRotateVec::ports_id::OP_0_DATA]->connect(rotated_IMU_demux->getPorts()[Demux3D::ports_id::IP_0_DATA]);

    mux_provider_x->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_x_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_slam_x->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_provider_slam_x_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_y->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_y_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_slam_y->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_provider_slam_y_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_z->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_z_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_slam_z->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_provider_slam_z_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_roll->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_roll_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_pitch->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_pitch_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_yaw->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_yaw_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_slam_yaw->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_provider_slam_yaw_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_yaw_rate->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_yaw_rate_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);

    std::cout  << "###### PROVIDERS NODE ######" "\n";
    
    Timer tempo;
    int i = 0;
    while(ros::ok()){
        tempo.tick();

        ros::spinOnce();
       
        int gone = tempo.tockMicroSeconds();
        if(gone > 5000) {
             std::cout  << i << " PROV: " << gone << "\n";
        }
        i++;

        rate.sleep();

    }

    return 0;
}