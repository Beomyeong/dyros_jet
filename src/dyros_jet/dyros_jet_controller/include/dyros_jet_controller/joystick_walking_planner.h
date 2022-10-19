#ifndef JOYSTICK_WALKING_PLANNER_H
#define JOYSTICK_WALKING_PLANNER_H


#include "dyros_jet_controller/dyros_jet_model.h"
#include "math_type_define.h"
#include <vector>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <thread>
#include <mutex>

#include <qpOASES.hpp>

#define ZERO_LIBRARY_MODE

using namespace qpOASES;
using namespace dyros_jet_controller;

#define WA_BEGIN 12
#define RA_BEGIN 21
#define LA_BEGIN 14
#define RF_BEGIN 6
#define LF_BEGIN 0

#define WA_LINK 13
#define RA_LINK 22
#define LA_LINK 15
#define RF_LINK 7
#define LF_LINK 1

#define DEGREE	(0.01745329251994329576923690768489)

const int FILE_CNT1 = 1;

const std::string FILE_NAMES1[FILE_CNT1] =
{
    "/home/beom/data/walking/00_zmp_desired_measured.txt",

};

using namespace std;
namespace dyros_jet_controller
{

class JoyWalkingController
{
public:
    fstream file[FILE_CNT1];

    static constexpr unsigned int PRIORITY = 8;

    JoyWalkingController(DyrosJetModel& model, const VectorQd& current_q, const VectorQd& current_q_dot, const double hz, const double& control_time) :
      total_dof_(DyrosJetModel::HW_TOTAL_DOF), model_(model), current_q_(current_q), current_q_dot_(current_q_dot), hz_(hz), current_time_(control_time), start_time_{}, end_time_{}, slowcalc_thread_(&JoyWalkingController::slowCalc, this), calc_update_flag_(false), calc_start_flag_(false), ready_for_thread_flag_(false), ready_for_compute_flag_(false), foot_step_planner_mode_(false), walking_end_foot_side_ (false), foot_plan_walking_last_(false), foot_last_walking_end_(false)
    {
      Joy_walking_state_send = false;
      Joy_walking_end_ = false;
      for(int i=0; i<FILE_CNT1;i++)
      {
        file[i].open(FILE_NAMES1[i].c_str(),ios_base::out);
      }
    }//Walking Controller ::~JoyWalkingController();

    void JoyCompute();
    void JoysetEnable(bool enable);
    void JoyupdateControlMask(unsigned int *mask);
    void JoywriteDesired(const unsigned int *mask, VectorQd& desired_q);
    void JoyupdateInitialState();
    void JoygetRobotState();
    void getJoystick(double x, double y, double z);
    void setJoystickWalking(int walk_mode, bool hip_compensation, bool lqr, int ik_mode, bool heel_toe,
                            bool is_right_foot_swing, double x, double y, double z, double height, double theta,
                            double step_length, double step_length_y);
    void JoygetComTrajectory();
    void JoyupdateNextStepTime();
    void JoystickWalkingPlanning();
    void Joyori();
    void JoyoriStop(double dlength, double width);
    void JoyFootstepGoon();
    void JoyFootstepStop();
    void JoyZMPtrajectory();
    void JoyfloatToSupportFootstep();
    void JoyZMPGenerator(const unsigned int norm_size, const unsigned planning_step_num);
    void JoyonestepZmp(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py);
    void JoystopstepZmp(unsigned int tick, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py);
    void JoyaddZmpOffset();
    void JoyFootTrajectory();
    void JoysupportToFloatPattern();


    void parameterSetting();

    void JoygetPelvTrajectory();
// function  in compute();
    void computeIkControl(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& desired_leg_q);
    void computeJacobianControl(Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector3d float_lleg_transform_euler, Eigen::Vector3d float_rleg_transform_euler, Eigen::Vector12d& desired_leg_q_dot);
    void compensator();

    //functions in compensator()
    void hipCompensator(); //reference Paper: http://dyros.snu.ac.kr/wp-content/uploads/2017/01/ICHR_2016_JS.pdf
    void hipCompensation();

    void getZmpReal();

    //PreviewController
    void modifiedPreviewControl();
    void previewControl(double dt, int NL, int tick, double x_i, double y_i, Eigen::Vector3d xs,
                        Eigen::Vector3d ys, double ux_1 , double uy_1 ,
                        double& ux, double& uy, double gi, Eigen::VectorXd gp_l,
                        Eigen::Matrix1x3d gx, Eigen::Matrix3d a, Eigen::Vector3d b,
                        Eigen::Matrix1x3d c, Eigen::Vector3d &xd, Eigen::Vector3d &yd);
    void previewControlParameter(double dt, int NL, Eigen::Matrix4d& k, Eigen::Vector3d com_support_init_,
                                 double& gi, Eigen::VectorXd& gp_l, Eigen::Matrix1x3d& gx, Eigen::Matrix3d& a,
                                 Eigen::Vector3d& b, Eigen::Matrix1x3d& c);
    void vibrationControl(const Eigen::Vector12d desired_leg_q, Eigen::Vector12d &output);
    void massSpringMotorModel(double spring_k, double damping_d, double motor_k, Eigen::Matrix12d & mass, Eigen::Matrix<double, 36, 36>& a, Eigen::Matrix<double, 36, 12>& b, Eigen::Matrix<double, 12, 36>& c);
    void discreteModel(Eigen::Matrix<double, 36, 36>& a, Eigen::Matrix<double, 36, 12>& b, Eigen::Matrix<double, 12, 36>& c, int np, double dt,
                       Eigen::Matrix<double, 36, 36>& ad, Eigen::Matrix<double, 36, 12>& bd, Eigen::Matrix<double, 12, 36>& cd,
                       Eigen::Matrix<double, 48, 48>& ad_total, Eigen::Matrix<double, 48, 12>& bd_total);
    void riccatiGain(Eigen::Matrix<double, 48, 48>& ad_total, Eigen::Matrix<double, 48, 12>& bd_total, Eigen::Matrix<double, 48, 48>& q, Eigen::Matrix12d& r, Eigen::Matrix<double, 12, 48>& k);
    void slowCalc();
    void slowCalcContent();

    void discreteRiccatiEquationInitialize(Eigen::MatrixXd a, Eigen::MatrixXd b);
    Eigen::MatrixXd discreteRiccatiEquationLQR(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd R, Eigen::MatrixXd Q);
    Eigen::MatrixXd discreteRiccatiEquationPrev(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd r, Eigen::MatrixXd q);



    //bool type declare
    bool Joy_walking_state_send ;
    bool Joy_walking_end_;
    bool walking_end_foot_side_;
    bool foot_plan_walking_last_;
    bool foot_last_walking_end_;

    // const double & current time ;
    const unsigned int total_dof_;
    double start_time_[DyrosJetModel::HW_TOTAL_DOF];
    double end_time_[DyrosJetModel::HW_TOTAL_DOF];

    Eigen::Matrix6d current_leg_jacobian_l_;
    Eigen::Matrix6d current_leg_jacobian_r_;
    Eigen::Matrix<double, 6, 7> current_arm_jacobian_l_;
    Eigen::Matrix<double, 6, 7> current_arm_jacobian_r_;
    Eigen::Matrix<double, 6, 2> current_waist_jacobian_[2];
    Eigen::Matrix6d                   current_leg_jacobian_l_inv_, current_leg_jacobian_r_inv_;
    DyrosJetModel &model_;

    //joint declare
    VectorQd start_q_;
    VectorQd desired_q_;
    VectorQd target_q_;
    const VectorQd& current_q_;
    const VectorQd& current_q_dot_;
    VectorQd  p_q_;

private:

    const double hz_;
    const double &current_time_; // updated by control_base
    unsigned int walking_tick_ = 0;
    double walking_time_ = 0;

    //sensorData
    Eigen::Vector6d r_ft_;
    Eigen::Vector6d l_ft_;
    Eigen::Vector6d pre_r_ft_;
    Eigen::Vector6d pre_l_ft_;
    Eigen::Vector3d imu_acc_;
    Eigen::Vector3d imu_ang_;
    Eigen::Vector3d imu_grav_rpy_;

    //parameterSetting()
    double t_last_;
    double t_start_;
    double t_start_real_;
    double t_temp_;
    double t_imp_;
    double t_rest_init_;
    double t_rest_last_;
    double t_double1_;
    double t_double2_;
    double t_total_;
    double foot_height_;
    double com_height_;

    bool com_control_mode_;
    bool com_update_flag_; // frome A to B
    bool gyro_frame_flag_;
    bool ready_for_thread_flag_;
    bool ready_for_compute_flag_;
    bool estimator_flag_;


    int ik_mode_;
    int walk_mode_;
    bool hip_compensator_mode_;
    bool lqr_compensator_mode_;
    int heel_toe_mode_;
    int is_right_foot_swing_;
    bool foot_step_planner_mode_;


    bool joy_walking_enable_;
    bool joint_enable_[DyrosJetModel::HW_TOTAL_DOF];
    double step_length_x_;
    double step_length_y_;

    double target_x_;
    double target_y_;
    double target_z_;
    double target_theta_;
    double total_step_num_;
    double total_step_num_joy_;
    double current_step_num_;
    int foot_step_plan_num_;
    int foot_step_start_foot_;

    Eigen::MatrixXd foot_step_joy_;
    Eigen::MatrixXd foot_step_joy_support_frame_;
    Eigen::MatrixXd foot_step_joy_support_frame_offset_;
    Eigen::MatrixXd ref_zmp_joy_;

    //vibrationCotrol
    std::mutex slowcalc_mutex_;
    std::thread slowcalc_thread_;

    VectorQd thread_q_;
    unsigned int thread_tick_;

    Eigen::Matrix<double, 48, 1> x_bar_right_;
    Eigen::Matrix<double, 12, 48> kkk_copy_;
    Eigen::Matrix<double, 48, 48> ad_total_copy_;
    Eigen::Matrix<double, 48, 12> bd_total_copy_;
    Eigen::Matrix<double, 36, 36> ad_copy_;
    Eigen::Matrix<double, 36, 12> bd_copy_;

    Eigen::Matrix<double, 36, 36> ad_right_;
    Eigen::Matrix<double, 36, 12> bd_right_;
    Eigen::Matrix<double, 48, 48> ad_total_right_;
    Eigen::Matrix<double, 48, 12> bd_total_right_;
    Eigen::Matrix<double, 12, 48> kkk_motor_right_;

    Eigen::Matrix<double, 18, 18> mass_matrix_;
    Eigen::Matrix<double, 18, 18> mass_matrix_pc_;
    Eigen::Matrix<double, 12, 12> mass_matrix_sel_;
    Eigen::Matrix<double, 36, 36> a_right_mat_;
    Eigen::Matrix<double, 36, 12> b_right_mat_;
    Eigen::Matrix<double, 12, 36> c_right_mat_;
    Eigen::Matrix<double, 36, 36> a_disc_;
    Eigen::Matrix<double, 36, 12> b_disc_;
    Eigen::Matrix<double, 48, 48> a_disc_total_;
    Eigen::Matrix<double, 48, 12> b_disc_total_;
    Eigen::Matrix<double, 48, 48> kkk_;
    Eigen::Vector12d dist_prev_;


    //Gravitycompensate
    Eigen::Vector12d joint_offset_angle_;
    Eigen::Vector12d grav_ground_torque_;

    bool calc_update_flag_;
    bool calc_start_flag_;

    //Step initial state variable//
    Eigen::Isometry3d pelv_support_init_;
    Eigen::Isometry3d lfoot_support_init_;
    Eigen::Isometry3d rfoot_support_init_;
    Eigen::Isometry3d pelv_float_init_;
    Eigen::Isometry3d lfoot_float_init_;
    Eigen::Isometry3d rfoot_float_init_;

    Eigen::Vector3d pelv_support_euler_init_;
    Eigen::Vector3d lfoot_support_euler_init_;
    Eigen::Vector3d rfoot_support_euler_init_;
    Eigen::Vector3d lfoot_float_euler_init_;
    Eigen::Vector3d rfoot_float_euler_init_;
    VectorQd q_init_;

    Eigen::Vector6d supportfoot_float_init_;
    Eigen::Vector6d supportfoot_support_init_;
    Eigen::Vector6d supportfoot_support_init_offset_;
    Eigen::Vector6d swingfoot_float_init_;
    Eigen::Vector6d swingfoot_support_init_;
    Eigen::Vector6d swingfoot_support_init_offset_;

    Eigen::Isometry3d pelv_suppprt_start_;

    Eigen::Vector3d com_float_init_;
    Eigen::Vector3d com_support_init_;

    double lfoot_zmp_offset_;   //have to be initialized
    double rfoot_zmp_offset_;
    Eigen::Vector3d com_offset_;

    //Step current state variable//
    Eigen::Vector3d com_support_current_;
    Eigen::Vector3d com_support_dot_current_;//from support foot

    ///simulation
    Eigen::Vector3d com_sim_current_;
    Eigen::Vector3d com_sim_dot_current_;
    Eigen::Isometry3d lfoot_sim_global_current_;
    Eigen::Isometry3d rfoot_sim_global_current_;
    Eigen::Isometry3d base_sim_global_current_;
    Eigen::Isometry3d lfoot_sim_float_current_;
    Eigen::Isometry3d rfoot_sim_float_current_;
    Eigen::Isometry3d supportfoot_float_sim_current_;

    Eigen::Vector3d gyro_sim_current_;
    Eigen::Vector3d accel_sim_current_;
    Eigen::Vector3d angle_;
    Eigen::Vector3d angle_init_;

    Eigen::Isometry3d supportfoot_float_current_;
    Eigen::Isometry3d pelv_support_current_;
    Eigen::Isometry3d lfoot_support_current_;
    Eigen::Isometry3d rfoot_support_current_;

    Eigen::Vector3d com_float_current_;
    Eigen::Isometry3d pelv_float_current_;
    Eigen::Isometry3d lfoot_float_current_;
    Eigen::Isometry3d rfoot_float_current_;


    Eigen::Vector3d com_float_old_;
    Eigen::Vector3d com_float_dot_old_;
    Eigen::Vector3d com_support_old_;
    Eigen::Vector3d com_support_dot_old_;
    Eigen::Vector3d com_sim_old_;
    Eigen::Vector2d com_support_dot_old_estimation_;
    Eigen::Vector2d com_support_old_estimation_;

    Eigen::Vector12d current_motor_q_leg_;
    Eigen::Vector12d current_link_q_leg_;
    Eigen::Vector12d pre_motor_q_leg_;
    Eigen::Vector12d pre_link_q_leg_;
    Eigen::Vector12d lqr_output_;
    Eigen::Vector12d lqr_output_pre_;

    Eigen::Vector2d zmp_measured_;

    //desired variables
    Eigen::Vector12d desired_leg_q_;
    Eigen::Vector12d desired_leg_q_dot_;
    VectorQd desired_q_not_compensated_;
    Eigen::Vector12d pre_desired_leg_q_dot_;
    Eigen::Vector12d desired_leg_q_dot_filtered_;
    Eigen::Vector3d com_desired_;
    Eigen::Vector3d com_dot_desired_;
    Eigen::Vector2d zmp_desired_;

    Eigen::Isometry3d rfoot_trajectory_support_;  //local frame
    Eigen::Isometry3d lfoot_trajectory_support_;
    Eigen::Vector3d rfoot_trajectory_euler_support_;
    Eigen::Vector3d lfoot_trajectory_euler_support_;
    Eigen::Vector6d rfoot_trajectory_dot_support_; //x,y,z translation velocity & roll, pitch, yaw velocity
    Eigen::Vector6d lfoot_trajectory_dot_support_;

    Eigen::Isometry3d pelv_trajectory_support_; //local frame
    Eigen::Isometry3d pelv_trajectory_float_; //pelvis frame

    Eigen::Isometry3d rfoot_trajectory_float_;  //pelvis frame
    Eigen::Isometry3d lfoot_trajectory_float_;
    Eigen::Isometry3d pre_rfoot_trajectory_float_;
    Eigen::Isometry3d pre_lfoot_trajectory_float_;
    Eigen::Vector3d rfoot_trajectory_euler_float_;
    Eigen::Vector3d lfoot_trajectory_euler_float_;
    Eigen::Vector3d rfoot_trajectory_dot_float_;
    Eigen::Vector3d lfoot_trajectory_dot_float_;
    Eigen::Vector6d                       lp_;
    Eigen::Vector6d                       rp_;

    //getComTrajectory() variables
    double xi_;
    double yi_;
    Eigen::Vector3d xs_;
    Eigen::Vector3d ys_;
    Eigen::Vector3d xd_;
    Eigen::Vector3d yd_;

    //Preview Control
    double ux_, uy_, ux_1_, uy_1_;
    double zc_;
    double gi_;
    double zmp_start_time_; //원래 코드에서는 start_time, zmp_ref 시작되는 time같음
    Eigen::Matrix4d k_;
    Eigen::VectorXd gp_l_;
    Eigen::Matrix1x3d gx_;
    Eigen::Matrix3d a_;
    Eigen::Vector3d b_;
    Eigen::Matrix1x3d c_;

    //Riccati variable
    Eigen::MatrixXd Z11;
    Eigen::MatrixXd Z12;
    Eigen::MatrixXd Z21;
    Eigen::MatrixXd Z22;
    Eigen::MatrixXd temp1;
    Eigen::MatrixXd temp2;
    Eigen::MatrixXd temp3;
    std::vector<double> eigVal_real; //eigen valueÀÇ real°ª
    std::vector<double> eigVal_img; //eigen valueÀÇ img°ª
    std::vector<Eigen::VectorXd> eigVec_real; //eigen vectorÀÇ real°ª
    std::vector<Eigen::VectorXd> eigVec_img; //eigen vectorÀÇ img°ª
    Eigen::MatrixXd Z;
    Eigen::VectorXd deigVal_real;
    Eigen::VectorXd deigVal_img;
    Eigen::MatrixXd deigVec_real;
    Eigen::MatrixXd deigVec_img;
    Eigen::MatrixXd tempZ_real;
    Eigen::MatrixXd tempZ_img;
    Eigen::MatrixXcd U11_inv;
    Eigen::MatrixXcd X;
    Eigen::MatrixXd X_sol;

    Eigen::EigenSolver<Eigen::MatrixXd>::EigenvectorsType Z_eig;
    Eigen::EigenSolver<Eigen::MatrixXd>::EigenvectorsType es_eig;
    Eigen::MatrixXcd tempZ_comp;
    Eigen::MatrixXcd U11;
    Eigen::MatrixXcd U21;

    ////////////////////////////////////////////////////////////
    //// for joystick control
    /// ////////////////////////////////////////////////////////

    Eigen::Vector4d joystick_input_;
    bool joystick_planning_;
    bool joystick_walking_flag_;
    bool joystick_walking_on_;
    bool halt_;
    int joystick_stop_begin_num_;
    int joystick_rotation_num_;
    int joystick_iter_;

    double pre_theta_;
    double theta_total_;

    Eigen::Isometry3d pre_pelv_trajectory_support_;


};// clss joywalkingcontroller

}//namespace dyros_jet_controller

#endif // JOYSTICK_WALKING_PLANNER_H
