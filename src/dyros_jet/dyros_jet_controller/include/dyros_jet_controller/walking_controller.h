#ifndef WALKING_CONTROLLER_H
#define WALKING_CONTROLLER_H


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

//#define WA_BEGIN 12
//#define RA_BEGIN 21
//#define LA_BEGIN 14
//#define RF_BEGIN 6
//#define LF_BEGIN 0

//#define WA_LINK 13
//#define RA_LINK 22
//#define LA_LINK 15
//#define RF_LINK 7
//#define LF_LINK 1
//#define BASE_LINK 0

#define WA_BEGIN 13
#define RA_BEGIN 21
#define LA_BEGIN 14

#define RF_BEGIN 7
#define LF_BEGIN 1
#define VIRTUAL_JOINT 0

#define WA_LINK 14
#define RA_LINK 22
#define LA_LINK 15

#define RF_LINK 8
#define LF_LINK 2
#define BASE_LINK 1
#define VIRTUAL_LINK 0

#define DEGREE	(0.01745329251994329576923690768489)

const int FILE_CNT =  44 ;

const std::string FILE_NAMES[FILE_CNT] =
{
  ///change this directory when you use this code on the other computer///

  "/home/beom/data/walking/00_zmp_desired_measured.txt",
  "/home/beom/data/walking/01_com_desired_current.txt",
  "/home/beom/data/walking/02_leg_q_desired_current.txt",
  "/home/beom/data/walking/03_upper_q_desired_current.txt",
  "/home/beom/data/walking/04_support_foot_desired_current.txt",
  "/home/beom/data/walking/05_pelvis_support_desired_current.txt",
  "/home/beom/data/walking/06_toe_heel_trajectory.txt",
  "/home/beom/data/walking/07_qpsolution_.txt",
  "/home/beom/data/walking/08_qpsolution with waist_.txt",
  "/home/beom/data/walking/09_float_foot trajectory_.txt",
  "/home/beom/data/walking/10_desired_velocity_.txt",
  "/home/beom/data/walking/11_com_pcheck_.txt",
  "/home/beom/data/walking/12_com_vcheck_.txt",
  "/home/beom/data/walking/13_ft_.txt",
  "/home/beom/data/walking/14_cubiczmppattern_.txt",
  "/home/beom/data/walking/15_mpc_com_.txt",
  "/home/beom/data/walking/16_h_matrix.txt",
  "/home/beom/data/walking/17_future_pel_yaw.txt",
  "/home/beom/data/walking/18_A_eq.txt",
  "/home/beom/data/walking/19_b_eq.txt",
  "/home/beom/data/walking/20_ZMP_of_MPC.txt",
  "/home/beom/data/walking/21_q_dot.txt",
  "/home/beom/data/walking/22_c_temp.txt",
  "/home/beom/data/walking/23_future_comx.txt",
  "/home/beom/data/walking/24_future_l_xfoot_.txt",
  "/home/beom/data/walking/25_future_r_xfoot.txt",
  "/home/beom/data/walking/26_support_polygon.txt",
  "/home/beom/data/walking/27_com after.txt",
  "/home/beom/data/walking/28_foot_step.txt",
  "/home/beom/data/walking/29_future_comy.txt",
  "/home/beom/data/walking/30_zmp_calculated_zmp_measured.txt",
  "/home/beom/data/walking/31_comdesired.txt",
  "/home/beom/data/walking/32_footori.txt",
  "/home/beom/data/walking/33_zmp_.txt",
  "/home/beom/data/walking/34_torque_.txt",
  "/home/beom/data/walking/35_zmp_swing_leg_.txt",
  "/home/beom/data/walking/36_com_swing_leg_.txt",
  "/home/beom/data/walking/37_future_l_yfoot_.txt",
  "/home/beom/data/walking/38_future_r_yfoot_.txt",
  "/home/beom/data/walking/39_future_l_zfoot_.txt",
  "/home/beom/data/walking/40_future_r_zfoot_.txt",
  "/home/beom/data/walking/41_future_comz_.txt",
  "/home/beom/data/walking/42_mpc_com_y.txt",
  "/home/beom/data/walking/43_future_mpc_com_y.txt"


};

using namespace std;
namespace dyros_jet_controller
{

class WalkingController
{
public:
  fstream file[FILE_CNT];


  static constexpr unsigned int PRIORITY = 8;


  WalkingController(DyrosJetModel& model, const VectorQd& current_q, const VectorQd& current_q_dot, const VectorQd& current_torque,const double hz, const double& control_time) :
    total_dof_(DyrosJetModel::HW_TOTAL_DOF), model_(model), current_q_(current_q), current_q_dot_(current_q_dot), current_torque_(current_torque),hz_(hz), current_time_(control_time), start_time_{}, end_time_{}, slowcalc_thread_(&WalkingController::slowCalc, this), calc_update_flag_(false), calc_start_flag_(false), ready_for_thread_flag_(false), ready_for_compute_flag_(false), foot_step_planner_mode_(false), walking_end_foot_side_ (false), foot_plan_walking_last_(false), foot_last_walking_end_(false)
  {
    walking_state_send = false;
    walking_end_ = false;
    for(int i=0; i<FILE_CNT;i++)
    {
      file[i].open(FILE_NAMES[i].c_str(),ios_base::out);
    }
//    file[0]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"zmp_desired_(0)"<<"\t"<<"zmp_desired_(1)"<<"\t"<<"foot_step_(current_step_num_, 0)"<<"\t"<<"foot_step_(current_step_num_, 1)"<<"\t"<<"foot_step_support_frame_(current_step_num_, 0)"<<"\t"<<"foot_step_support_frame_(current_step_num_, 1)"<<"\t"<<"foot_step_support_frame_(current_step_num_, 2)"<<endl;
//    file[1]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"com_desired_(0)"<<"\t"<<"com_desired_(1)"<<"\t"<<"com_desired_(2)"<<"\t"<<"com_dot_desired_(0)"<<"\t"<<"com_dot_desired_(1)"<<"\t"<<"com_dot_desired_(2)"<<"\t"<<"com_support_init_(0)"<<"\t"<<"com_support_init_(0)"<<"\t"<<"com_support_init_(0)"<<endl;
//    file[2]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"desired_leg_q_(0)"<<"\t"<<"desired_leg_q_(1)"<<"\t"<<"desired_leg_q_(2)"<<"\t"<<"desired_leg_q_(3)"<<"\t"<<"desired_leg_q_(4)"<<"\t"<<"desired_leg_q_(5)"<<"\t"<<"desired_leg_q_(6)"<<"\t"<<"desired_leg_q_(7)"<<"\t"<<"desired_leg_q_(8)"<<"\t"<<"desired_leg_q_(9)"<<"\t"<<"desired_leg_q_(10)"<<"\t"<<"desired_leg_q_(11)"<<endl;
//    file[3]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"current_q_(0)"<<"\t"<<"current_q_(1)"<<"\t"<<"current_q_(2)"<<"\t"<<"current_q_(3)"<<"\t"<<"current_q_(4)"<<"\t"<<"current_q_(5)"<<"\t"<<"current_q_(6)"<<"\t"<<"current_q_(7)"<<"\t"<<"current_q_(8)"<<"\t"<<"current_q_(9)"<<"\t"<<"current_q_(10)"<<"\t"<<"current_q_(11)"<<endl;
//    file[4]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"rfoot_trajectory_support_.translation()(0)"<<"\t"<<"rfoot_trajectory_support_.translation()(1)"<<"\t"<<"rfoot_trajectory_support_.translation()(2)"<<"\t"<<"lfoot_trajectory_support_.translation()(0)"<<"\t"<<"lfoot_trajectory_support_.translation()(1)"<<"\t"<<"lfoot_trajectory_support_.translation()(2)"<<"\t"<<"rfoot_support_init_.translation()(0)"<<"\t"<<"rfoot_support_init_.translation()(1)"<<"\t"<<"rfoot_support_init_.translation()(2)"<<endl;
//    file[5]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"pelv_trajectory_support_.translation()(0)"<<"\t"<<"pelv_trajectory_support_.translation()(1)"<<"\t"<<"pelv_trajectory_support_.translation()(2)"<<endl;
//    file[6]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"com_support_current_(0)"<<"\t"<<"com_support_current_(1)"<<"\t"<<"com_support_current_(2)"
//          <<"\t"<<"pelv_support_current_.translation()(0)"<<"\t"<<"pelv_support_current_.translation()(1)"<<"\t"<<"pelv_support_current_.translation()(2)"<<"\t"<<"com_support_dot_current_(0)"<<"\t"<<"com_support_dot_current_(1)"<<"\t"<<"com_support_dot_current_(2)"
//         <<"\t"<<"com_sim_current_(0)"<<"\t"<<"com_sim_current_(1)"<<"\t"<<"com_sim_current_(2)"<<"\t"<<"com_sim_dot_current_(0)"<<"\t"<<"com_sim_dot_current_(1)"<<"\t"<<"com_sim_dot_current_(2)"<<endl;
//    file[7]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"rfoot_support_current_.translation()(0)"<<"\t"<<"rfoot_support_current_.translation()(1)"<<"\t"<<"rfoot_support_current_.translation()(2)"
//          <<"\t"<<"lfoot_support_current_.translation()(0)"<<"\t"<<"lfoot_support_current_.translation()(1)"<<"\t"<<"lfoot_support_current_.translation()(2)"<<endl;
//    file[8]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"vars.x[0]"<<"\t"<<"vars.x[1]"<<"\t"<<"vars.x[2]"<<"\t"<<"vars.x[3]"<<"\t"<<"vars.x[4]"<<"\t"<<"vars.x[5]"<<"\t"<<"zmp_measured_(0)"<<"\t"<<"zmp_measured_(1)"<<"\t"<<"zmp_r_(0)"<<"\t"<<"zmp_r_(1)"<<"\t"<<"zmp_l_(0)"<<"\t"<<"zmp_l_(1)"<<endl;
//    file[9]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"r_ft_(0)"<<"\t"<<"r_ft_(1)"<<"\t"<<"r_ft_(2)"<<"\t"<<"r_ft_(3)"<<"\t"<<"r_ft_(4)"<<"\t"<<"r_ft_(5)"<<"\t"<<"l_ft_(0)"<<"\t"<<"l_ft_(1)"<<"\t"<<"l_ft_(2)"<<"\t"<<"l_ft_(3)"<<"\t"<<"l_ft_(4)"<<"\t"<<"l_ft_(5)"<<endl;
//    file[10]<<"walking_tick_"<<"\t"<<"current_step_num_"<<"\t"<<"current_link_q_leg_(0)"<<"\t"<<"current_link_q_leg_(1)"<<"\t"<<"current_link_q_leg_(2)"<<"\t"<<"current_link_q_leg_(3)"<<"\t"<<"current_link_q_leg_(4)"<<"\t"<<"current_link_q_leg_(5)"<<"\t"<<
//              "current_link_q_leg_(6)"<<"\t"<<"current_link_q_leg_(7)"<<"\t"<<"current_link_q_leg_(8)"<<"\t"<<"current_link_q_leg_(9)"<<"\t"<<"current_link_q_leg_(10)"<<"\t"<<"current_link_q_leg_(11)"<<endl;
//    file[11]<<"walking_tick_"<<"\t"<<"X_hat_post_2_(0)"<<"\t"<<"X_hat_post_2_(1)"<<"\t"<<"X_hat_post_2_(2)"<<"\t"<<"X_hat_post_2_(3)"<<"\t"<<"X_hat_post_2_(4)"<<"\t"<<"X_hat_post_2_(5)"<<"\t"<<"X_hat_post_2_(6)"<<"\t"<<"X_hat_post_2_(7)"<<endl;
//    file[12]<<"walking_tick_"<<"\t"<<"X_hat_post_1_(0)"<<"\t"<<"X_hat_post_1_(1)"<<"\t"<<"X_hat_post_1_(2)"<<"\t"<<"X_hat_post_1_(3)"<<"\t"<<"X_hat_post_1_(4)"<<"\t"<<"X_hat_post_1_(5)"<<endl;
//    file[13]<<"walking_tick_"<<"\t"<<"X_hat_post_3_(0)"<<"\t"<<"X_hat_post_3_(1)"<<"\t"<<"X_hat_post_3_(2)"<<"\t"<<"X_hat_post_3_(3)"<<"\t"<<"X_hat_post_3_(4)"<<"\t"<<"X_hat_post_3_(5)"<<endl;


//    file[0]<<"walking_tick_"<<"\t"<<"zmp_desired_(0)"<<"\t"<<"zmp_desired_(1)"<<"\t"<<"zmp_measured_(0)"<<"\t"<<"zmp_measured_(1)"<<endl;
//    file[1]<<"walking_tick_"<<"\t"<<"com_desired_(0)"<<"\t"<<"com_desired_(1)"<<"\t"<<"com_desired_(2)"<<"\t"<<"com_support_current_(0)"<<"\t"<<"com_support_current_(1)"<<"\t"<<"com_support_current_(2)"<<endl;
//    file[2]<<"walking_tick_"<<"\t"<<"desired_q_(0)"<<"\t"<<"desired_q_(1)"<<"\t"<<"desired_q_(2)"<<"\t"<<"desired_q_(3)"<<"\t"<<"desired_q_(4)"<<"\t"<<"desired_q_(5)"
//             <<"\t"<<"desired_q_(6)"<<"\t"<<"desired_q_(7)"<<"\t"<<"desired_q_(8)"<<"\t"<<"desired_q_(9)"<<"\t"<<"desired_q_(10)"<<"\t"<<"desired_q_(11)"
//            <<"\t"<<"current_q_(0)"<<"\t"<<"current_q_(1)"<<"\t"<<"current_q_(2)"<<"\t"<<"current_q_(3)"<<"\t"<<"current_q_(4)"<<"\t"<<"current_q_(5)"
//            <<"\t"<<"current_q_(6)"<<"\t"<<"current_q_(7)"<<"\t"<<"current_q_(8)"<<"\t"<<"current_q_(9)"<<"\t"<<"current_q_(10)"<<"\t"<<"current_q_(11)"<<endl;
//    file[3]<<"walking_tick_"<<"\t"<<"desired_q_(12)"<<"\t"<<"desired_q_(13)"
//             <<"\t"<<"desired_q_(14)"<<"\t"<<"desired_q_(15)"<<"\t"<<"desired_q_(16)"<<"\t"<<"desired_q_(17)"<<"\t"<<"desired_q_(18)"<<"\t"<<"desired_q_(19)"<<"\t"<<"desired_q_(20)"
//               <<"\t"<<"desired_q_(21)"<<"\t"<<"desired_q_(22)"<<"\t"<<"desired_q_(23)"<<"\t"<<"desired_q_(24)"<<"\t"<<"desired_q_(25)"<<"\t"<<"desired_q_(26)"<<"\t"<<"desired_q_(27)"
//              <<"\t"<<"current_q_(12)"<<"\t"<<"current_q_(13)"
//              <<"\t"<<"current_q_(14)"<<"\t"<<"current_q_(15)"<<"\t"<<"current_q_(16)"<<"\t"<<"current_q_(17)"<<"\t"<<"current_q_(18)"<<"\t"<<"current_q_(19)"<<"\t"<<"current_q_(20)"
//              <<"\t"<<"current_q_(21)"<<"\t"<<"current_q_(22)"<<"\t"<<"current_q_(23)"<<"\t"<<"current_q_(24)"<<"\t"<<"current_q_(25)"<<"\t"<<"current_q_(26)"<<"\t"<<"current_q_(27)"<<endl;
//    file[4]<<"walking_tick_"<<"\t"<<"rfoot_trajectory_support_.translation()(0)"<<"\t"<<"rfoot_trajectory_support_.translation()(1)"<<"\t"<<"rfoot_trajectory_support_.translation()(2)"
//             <<"\t"<<"lfoot_trajectory_support_.translation()(0)"<<"\t"<<"lfoot_trajectory_support_.translation()(1)"<<"\t"<<"lfoot_trajectory_support_.translation()(2)"
//            <<"\t"<<"rfoot_support_current_.translation()(0)"<<"\t"<<"rfoot_support_current_.translation()(1)"<<"\t"<<"rfoot_support_current_.translation()(2)"
//           <<"\t"<<"lfoot_support_current_.translation()(0)"<<"\t"<<"lfoot_support_current_.translation()(1)"<<"\t"<<"lfoot_support_current_.translation()(2)"<<endl;
//    file[5]<<"walking_tick_"<<"\t"<<"pelv_trajectory_support_.translation()(0)"<<"\t"<<"pelv_trajectory_support_.translation()(1)"<<"\t"<<"pelv_trajectory_support_.translation()(2)"
//          <<"\t"<<"pelv_support_current_.translation()(0)"<<"\t"<<"pelv_support_current_.translation()(1)"<<"\t"<<"pelv_support_current_.translation()(2)"<<endl;

  }
  //WalkingController::~WalkingController()
  //{
  //  for(int i=0; i<FILE_CNT;i++)
  //  {
  //    if(file[i].is_open())
  //      file[i].close();
  //  }
  //}

  void getCOMvelocity();
  void getCOMestimation();

  void compute();

  void setTarget(int walk_mode, bool hip_compensation, bool lqr, int ik_mode, bool heel_toe,
                 bool is_right_foot_swing, double x, double y, double z, double height, double theta,
                 double step_length, double step_length_y, bool walking_pattern);
  void setEnable(bool enable);
  void setFootPlan(int footnum, int startfoot, Eigen::MatrixXd footpose);
  void updateControlMask(unsigned int *mask);
  void writeDesired(const unsigned int *mask, VectorQd& desired_q);

  //// for joystick walking
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
  void JoygetPelvTrajectory();
  void JoysupportToFloatPattern();  

  void ZMP_2(Eigen::Vector6d ft_l, Eigen::Vector6d ft_r, Eigen::Vector3d l_pos, Eigen::Vector3d r_pos, Eigen::Vector3d& ZMP);

  void getZmpReal();
  void computeZmp();
  void zmpTest(const unsigned int norm_size, const unsigned planning_step_num);
  void onestepTest(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py);
  void calculateStride();


  void getPelvOriTrajectory();

  void parameterSetting();
  //functions in compute
  void getRobotState();
  void getComTrajectory();
  void getZmpTrajectory();
  void getPelvTrajectory();
  void getFootTrajectory();
  void getAdaptiveFootTrajectory();
  void getFootSinTrajectory();
  void getFootTrajectory2();
  void computeIkControl(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& desired_leg_q);
  void computeJacobianControl(Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector3d float_lleg_transform_euler, Eigen::Vector3d float_rleg_transform_euler, Eigen::Vector12d& desired_leg_q_dot);
  void compensator();

  void supportToFloatPattern();
  void updateNextStepTime();
  void updateInitialState();

  //functions for getFootStep()
  void calculateFootStepTotal();
  void calculateFootStepSeparate();
  void usingFootStepPlanner();
  void calculateFootStepTotal_MJ();

  //functions for getZMPTrajectory()
  void floatToSupportFootstep();
  void addZmpOffset();
  void zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num);
  void onestepZmp(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py);
  void onestepZmp_modified(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py);

  //functions in compensator()
  void hipCompensator(bool support_foot); //reference Paper: http://dyros.snu.ac.kr/wp-content/uploads/2017/01/ICHR_2016_JS.pdf
  void hipCompensation();

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
  void vibrationControl(const Eigen::Vector12d desired_leg_q, Eigen::Vector12d &output, bool left_support);
  void massSpringMotorModel(double spring_k, double damping_d, double motor_k, Eigen::Matrix12d & mass, Eigen::Matrix<double, 36, 36>& a, Eigen::Matrix<double, 36, 12>& b, Eigen::Matrix<double, 12, 36>& c);
  void discreteModel(Eigen::Matrix<double, 36, 36>& a, Eigen::Matrix<double, 36, 12>& b, Eigen::Matrix<double, 12, 36>& c, int np, double dt,
                     Eigen::Matrix<double, 36, 36>& ad, Eigen::Matrix<double, 36, 12>& bd, Eigen::Matrix<double, 12, 36>& cd,
                     Eigen::Matrix<double, 48, 48>& ad_total, Eigen::Matrix<double, 48, 12>& bd_total);
  void riccatiGain(Eigen::Matrix<double, 48, 48>& ad_total, Eigen::Matrix<double, 48, 12>& bd_total, Eigen::Matrix<double, 48, 48>& q, Eigen::Matrix12d& r, Eigen::Matrix<double, 12, 48>& k);
  void slowCalc();
  void slowCalcContent();





  void qpOASES_example();
  void getOptimizationInputMatrix();
  void getOptimizationInputMatrix2();
  void vibrationControl_modified(const Eigen::Vector12d desired_leg_q, Eigen::Vector12d &output);
  void Compliant_control(Eigen::Vector12d desired_leg_q);

  void Jacobian_test(Eigen::Vector12d& desired_leg_q_dot);
  void Jacobian_floating();


  void zmptoInitFloat();

  //////////////////////////////////////////////////
  ///////////////// Home Work //////////////////////

  // for centroidal dynamics
  void Relative_link_position(Eigen::Isometry3d* transform_matrix);
  void Spatial_transform();
  void Projection_Spatial_COM();
  void Relative_link_rotation(Eigen::Isometry3d* transform_matrix);
  void relative_link_trans_matrix(Eigen::Isometry3d* transform_matrix);
  void Relative_Inertia();
  void Joint_Spatial_Inertia();
  void System_Spatial_Inertia();
  void Centroidal_Momentum_Matrix();
  void Centroidal_Dynamics();

  void Spatial_link_transform();
  void relative_link_trans_matrix2(Eigen::Isometry3d* transform_matrix);
  void Link_Spatial_Inertia();
  void Composite_Rigid_Body_Inertia();
  void Spatial_tranform_to_COM();
  void getCentroidal_Momentum_Matrix();

  // for jacobian based control
  void Jacobian_based_IK();

  void UpdateCentroidalMomentumMatrix();

  //////////////////////////////////////////////////

  ////resolved momentum control
  void ResolvedMomentumCtrl();
  void CalculateInertiaMatrix();


  void discreteRiccatiEquationInitialize(Eigen::MatrixXd a, Eigen::MatrixXd b);
  Eigen::MatrixXd discreteRiccatiEquationLQR(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd R, Eigen::MatrixXd Q);
  Eigen::MatrixXd discreteRiccatiEquationPrev(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd r, Eigen::MatrixXd q);

  VectorQd desired_q_not_compensated_;
  VectorQd desired_q_not_compensated2_;

  bool walking_end_foot_side_;
  bool walking_end_;
  bool foot_plan_walking_last_;
  bool foot_last_walking_end_;
  bool walking_state_send;

  Eigen::Vector12d ext_encoder_debug_;

  // functions for swing leg dynamics//
  void getStateMatrix(int sampling_N, double dt, int interval);
  void getSwingLegMatrix();
  void getLegDynamicMatrix(int tick, int N, int interval, Eigen::MatrixXd& foot_pos, Eigen::MatrixXd& foot_acc, Eigen::MatrixXd& Ki_x,Eigen::MatrixXd& Di_x, Eigen::MatrixXd& Ki_y,Eigen::MatrixXd& Di_y);
  void QP_legdynamics();
  void IntrisicMPC();
  void iMPCparameter(int N, double delta);
  void SupportFootReference(Eigen::VectorXd& support_x, Eigen::VectorXd& support_y, int N, double delta);
  void GetConstraintMatrix(Eigen::Matrix<double,23,12> &A_dsp1, Eigen::Matrix<double,18,12> &A_lifting, Eigen::Matrix<double,23,12> &A_landing, Eigen::Matrix<double,24,12> &A_dsp2,
                           Eigen::VectorXd &lbA_dsp1, Eigen::VectorXd  &ubA_dsp1,Eigen::VectorXd &lbA_lifting, Eigen::VectorXd  &ubA_lifting,Eigen::VectorXd &lbA_landing, Eigen::VectorXd  &ubA_landing,Eigen::VectorXd &lbA_dsp2, Eigen::VectorXd  &ubA_dsp2);
  void getDesiredVelocity(Eigen::Vector6d &lp, Eigen::Vector6d &rp, Eigen::Vector6d &lp_toe, Eigen::Vector6d &rp_toe, Eigen::Vector6d &lp_heel, Eigen::Vector6d &rp_heel,
                          Eigen::Vector6d &lp_clik, Eigen::Vector6d &rp_clik, Eigen::Vector6d &lp_toe_clik, Eigen::Vector6d &rp_toe_clik, Eigen::Vector6d &lp_heel_clik, Eigen::Vector6d &rp_heel_clik);

  void MPC_QP_swing();
  void qpIK_test();
  void getFoottrajectory_heel_toe();

  void getMPCMatrix(int full_n, int sampling_n, double dt, int interval);
  void getMPCMatrix2(int full_n, int sampling_n, double dt, int interval);
  void calculateFootMargin(Eigen::MatrixXd& margin_x, Eigen::MatrixXd& margin_y);
  void calculateFootMargin2(Eigen::MatrixXd& margin_x, Eigen::MatrixXd& margin_y);
  void onestepFoot(unsigned int current_step_number, Eigen::VectorXd& temp_f_x, Eigen::VectorXd& temp_f_y);
  void footReferenceGenerator(Eigen::VectorXd& foot_x,  Eigen::VectorXd& foot_y);
  void ObtainMatrix(int NL,int N, double dt, int interval);
  void SupportfootComUpdate(Eigen::Vector3d x_p1,Eigen::Vector3d y_p1, Eigen::Vector3d& New_x_p1, Eigen::Vector3d& New_y_p1);
  void onestepSupportPolygon(unsigned int current_step_number, Eigen::MatrixXd& tempx, Eigen::MatrixXd& tempy);
  void GetSupportPolygon(Eigen::MatrixXd& sp_x, Eigen::MatrixXd& sp_y);
  void Obtain_com_pattern(ifstream& input_x,ifstream& input_y, ifstream& foot_input, Eigen::MatrixXd& com_x,Eigen::MatrixXd& com_y, Eigen::MatrixXd& input_foot);
  void qp3();
  void qp2();
  void qpIK();
  void qpIK_pelvis();
  void qpIK_pelvis_13();
  void qpIK_pel_arm();
  void qpIK_pel_full_arm();
  void qp1();
  void previewQP();
  void MPCwQP();
  void zmpPattern(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py);
  void supportToWaistPattern();
  void getRobotStatefromWaist();
  void updateInitialStatefromWaist();
  void computeWaistJacobianCtrl(Eigen::Isometry3d waist_lleg_transform, Eigen::Isometry3d waist_rleg_transform, Eigen::Vector3d waist_lleg_transform_euler, Eigen::Vector3d waist_rleg_transform_euler,Eigen::VectorXd& desired_q_dot);


  //for heel to toe edge jacobian control......
  void calculateFootEdgeJaco();
  void SettingJointLimit();
  void getToeHeelTrajectory();
  void getOptimizedFootTrajectory();
  void comHeight();
  void MovingZMPtrajectory();
  void MovingZMPGenerator(const unsigned int norm_size, const unsigned planning_step_num);
  void MovingonestepZMP(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py);
  void SettingVirtualJointLimit();

  // for controlled LIPM // updated by myeongju
  void getComTrajectory_MJ();
  void modifiedPreviewControl_MJ();

  void previewParam_MJ_CPM(double dt, int NL, Eigen::Matrix3d& K, Eigen::Vector3d com_support_init_, Eigen::MatrixXd& Gi, Eigen::VectorXd& Gd, Eigen::MatrixXd& Gx,
  Eigen::MatrixXd& A, Eigen::VectorXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D, Eigen::MatrixXd& A_bar, Eigen::VectorXd& B_bar);

  void preview_MJ_CPM(double dt, int NL, int tick, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double& UX, double& UY,
       Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd A_bar, Eigen::VectorXd B_bar, Eigen::Vector2d &XD, Eigen::Vector2d &YD, Eigen::VectorXd& X_bar_p, Eigen::VectorXd& Y_bar_p);

  void getRobotState_MJ();
  void CalculateCenterOfMassSupportBody();

  void SupportfootComUpdate();
  void FutureFoottrajectory(int N_smpl, int T_int);


  void qp31();
  void MPC_pel_yaw();
  void get_mpc_pel_yaw_matrix(int future_horizon, int N_smpl, double dt, int interval);


  void MPC_com();
  void MPC_Matrix_Update(int sampling_n, double dt, int interval);
  void future_trajectory_update(int N_smpl,int interval);
  void FutureSingularityCheck(int N_smpl, int interval);
  void FuturePelYawReference();
  void CalculateFuturePelAngle();
  void QP_momentum();



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

  Eigen::Vector6d r_ft_pre_;
  Eigen::Vector6d l_ft_pre_;
  Eigen::Vector6d r_ft_ppre_;
  Eigen::Vector6d l_ft_ppre_;

  Eigen::Vector6d r_ft_filtered_;
  Eigen::Vector6d l_ft_filtered_;
  Eigen::Vector6d r_ft_filtered_pre_;
  Eigen::Vector6d l_ft_filtered_pre_;
  Eigen::Vector6d r_ft_filtered_ppre_;
  Eigen::Vector6d l_ft_filtered_ppre_;

  Eigen::Vector3d zmp_measured_pre_;
  Eigen::Vector3d zmp_measured_ppre_;

    Eigen::Vector3d f_ft_support_;


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

  double t_total_init_;
  double t_double1_init_;

  bool com_control_mode_;
  bool com_update_flag_; // frome A to B
  bool gyro_frame_flag_;
  bool ready_for_thread_flag_;
  bool ready_for_compute_flag_;
  bool estimator_flag_;
  bool swing_foot_flag_;
  double t_swing_start_;
  bool matrix_get_flag_;


  int ik_mode_;
  int walk_mode_;
  bool hip_compensator_mode_;
  bool lqr_compensator_mode_;
  int heel_toe_mode_;
  int is_right_foot_swing_;
  bool foot_step_planner_mode_;

  bool walking_enable_;  
  bool joint_enable_[DyrosJetModel::HW_TOTAL_DOF];
  double step_length_x_;
  double step_length_y_;

  //double step_angle_theta_;
  double target_x_;
  double target_y_;
  double target_z_;
  double target_theta_;
  double total_step_num_;
  double total_step_num_joy_;
  double current_step_num_;
  int foot_step_plan_num_;
  int foot_step_start_foot_;
  Eigen::MatrixXd foot_pose_;

  Eigen::MatrixXd foot_step_;
  Eigen::MatrixXd foot_step_support_frame_;
  Eigen::MatrixXd foot_step_support_frame_offset_;

  Eigen::MatrixXd foot_step_joy_;
  Eigen::MatrixXd foot_step_joy_support_frame_;
  Eigen::MatrixXd foot_step_joy_support_frame_offset_;

  Eigen::MatrixXd ref_zmp_;
  Eigen::MatrixXd ref_zmp_joy_;
  Eigen::Vector2d final_ref_zmp_;
  double    zmp_margin_;


  VectorQd start_q_;
  VectorQd desired_q_;
  VectorQd target_q_;
  const VectorQd& current_q_;
  const VectorQd& current_q_dot_;
  const VectorQd& current_torque_;



  //const double &current_time_;
  const unsigned int total_dof_;
  double start_time_[DyrosJetModel::HW_TOTAL_DOF];
  double end_time_[DyrosJetModel::HW_TOTAL_DOF];

  //Step initial state variable//
  Eigen::Isometry3d pelv_support_init_;
  Eigen::Isometry3d lfoot_support_init_;
  Eigen::Isometry3d rfoot_support_init_;
  Eigen::Isometry3d pelv_float_init_;
  Eigen::Isometry3d lfoot_float_init_;
  Eigen::Isometry3d rfoot_float_init_;

  Eigen::Isometry3d lfoot_float_init_0_;
  Eigen::Isometry3d rfoot_float_init_0_;

  Eigen::Isometry3d lfoot_support_init_0_;
  Eigen::Isometry3d rfoot_support_init_0_;
  Eigen::Isometry3d pelv_support_init_0_;

  Eigen::Isometry3d lfoot_support_init_2_;
  Eigen::Isometry3d rfoot_support_init_2_;
  Eigen::Isometry3d pelv_support_init_2_;

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
  Eigen::Vector3d com_support_body_;

  Eigen::Vector3d com_support_pre_;
  Eigen::Vector3d com_support_pre_pre_;
  Eigen::Vector2d com_support_pre_vel_;
  Eigen::Vector2d com_support_acc_pre_;
  Eigen::Vector2d com_support_vel_;
  Eigen::Vector2d com_support_acc_;
  Eigen::Vector2d zmp_est_;

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
  Eigen::Isometry3d pre_pelv_trajectory_support_;
  Eigen::Isometry3d lfoot_support_current_;
  Eigen::Isometry3d rfoot_support_current_;
  Eigen::Vector3d lfoot_float_current_euler_;
  Eigen::Vector3d rfoot_float_current_euler_;

  Eigen::Vector3d com_float_current_;
  Eigen::Isometry3d pelv_float_current_;
  Eigen::Isometry3d lfoot_float_current_;
  Eigen::Isometry3d rfoot_float_current_;

  Eigen::Vector3d com_dot_float_current_;


  //from waist coordinate
  Eigen::Isometry3d supportfoot_waist_current_;
  Eigen::Isometry3d waist_support_current_;
  Eigen::Isometry3d lfoot_waist_current_;
  Eigen::Isometry3d rfoot_waist_current_;
  Eigen::Matrix<double, 6, 7> current_leg_waist_l_jacobian_;
  Eigen::Matrix<double, 6, 7> current_leg_waist_r_jacobian_;
  Eigen::Matrix<double, 7, 6> current_leg_waist_l_jacobian_inv_;
  Eigen::Matrix<double, 7, 6> current_leg_waist_r_jacobian_inv_;
  Eigen::Isometry3d rfoot_trajectory_waist_;
  Eigen::Isometry3d lfoot_trajectory_waist_;
  Eigen::Isometry3d pre_rfoot_trajectory_waist_;
  Eigen::Isometry3d pre_lfoot_trajectory_waist_;
  Eigen::Isometry3d waist_trajectory_support_;
  Eigen::Isometry3d waist_trajectory_float_;
  Eigen::Vector3d lfoot_trajectory_euler_waist_;
  Eigen::Vector3d rfoot_trajectory_euler_waist_;
  Eigen::Vector6d rfoot_trajectory_dot_waist_; //x,y,z translation velocity & roll, pitch, yaw velocity
  Eigen::Vector6d lfoot_trajectory_dot_waist_;

  //initial setting for waist coordinate
  Eigen::Isometry3d waist_float_init_;
  Eigen::Isometry3d lfoot_waist_init_;
  Eigen::Isometry3d rfoot_waist_init_;



  Eigen::Matrix6d current_leg_jacobian_l_;
  Eigen::Matrix6d current_leg_jacobian_r_;
  Eigen::Matrix<double, 6, 7> current_arm_jacobian_l_;
  Eigen::Matrix<double, 6, 7> current_arm_jacobian_r_;
  Eigen::Matrix<double, 6, 2> current_waist_jacobian_[2];
  DyrosJetModel &model_;


  //desired variables
  Eigen::Vector12d desired_leg_q_;
  Eigen::Vector12d desired_leg_q_dot_;
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


  //resolved momentum control
  Eigen::Vector3d p_ref_;
  Eigen::Vector3d l_ref_;

  //Gravitycompensate
  Eigen::Vector12d joint_offset_angle_;
  Eigen::Vector12d grav_ground_torque_;

  //vibrationCotrol
  std::mutex slowcalc_mutex_;
  std::thread slowcalc_thread_;

  Eigen::Vector12d current_motor_q_leg_;
  Eigen::Vector12d current_link_q_leg_;
  Eigen::Vector12d pre_motor_q_leg_;
  Eigen::Vector12d pre_link_q_leg_;
  Eigen::Vector12d lqr_output_;
  Eigen::Vector12d lqr_output_pre_;

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

  Eigen::Vector12d dist_prev_;

  bool calc_update_flag_;
  bool calc_start_flag_;


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


  //////////////////QP based StateEstimation/////////////////////
  Eigen::Matrix<double, 18, 6> a_total_;
  Eigen::Matrix<double, 2, 6> a_kin_;
  Eigen::Matrix<double, 2, 6> a_c_dot_;
  Eigen::Matrix<double, 2, 6> a_c_;
  Eigen::Matrix<double, 2, 6> a_zmp_;
  Eigen::Matrix<double, 2, 6> a_c_c_dot_;
  Eigen::Matrix<double, 2, 6> a_f_;
  Eigen::Matrix<double, 6, 6> a_noise_;
  Eigen::Matrix<double, 18, 1> b_total_;
  Eigen::Matrix<double, 2, 1> b_kin_;
  Eigen::Matrix<double, 2, 1> b_c_dot_;
  Eigen::Matrix<double, 2, 1> b_c_;
  Eigen::Matrix<double, 2, 1> b_zmp_;
  Eigen::Matrix<double, 2, 1> b_c_c_dot_;
  Eigen::Matrix<double, 2, 1> b_f_;
  Eigen::Matrix<double, 6, 1> b_noise_;


  Eigen::Vector3d com_float_old_;
  Eigen::Vector3d com_float_dot_old_;
  Eigen::Vector3d com_support_old_;
  Eigen::Vector3d com_support_dot_old_;
  Eigen::Vector3d com_sim_old_;
  Eigen::Vector2d com_support_dot_old_estimation_;
  Eigen::Vector2d com_support_old_estimation_;


  Eigen::Vector2d zmp_r_;
  Eigen::Vector2d zmp_l_;
  Eigen::Vector3d zmp_measured_;
  Eigen::Vector2d zmp_old_estimation_;
  Eigen::Vector3d ZMP_2_;

  Eigen::Vector6d x_estimation_;
  Eigen::Vector3d zmp_measured1_;

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

 // void getQpEstimationInputMatrix();
  ////////////////////////////////////////////////////////


  /////////////////////////Kalman Filter1///////////////////////
  Eigen::Matrix<double, 6, 6> Ad_1_;
  Eigen::Matrix<double, 6, 2> Bd_1_;
  Eigen::Matrix<double, 4, 6> Cd_1_;
  Eigen::Matrix<double, 6, 6> Q_1_;
  Eigen::Matrix<double, 4, 4> R_1_;


  Eigen::Matrix<double, 6, 1> X_hat_prio_1_;
  Eigen::Matrix<double, 6, 1> X_hat_post_1_;
  Eigen::Matrix<double, 6, 1> X_hat_prio_old_1_;
  Eigen::Matrix<double, 6, 1> X_hat_post_old_1_;

  Eigen::Matrix<double, 4, 1> Y_1_;



  Eigen::Matrix<double, 6, 6> P_prio_1_;
  Eigen::Matrix<double, 6, 6> P_post_1_;
  Eigen::Matrix<double, 6, 6> P_prio_old_1_;
  Eigen::Matrix<double, 6, 6> P_post_old_1_;

  Eigen::Matrix<double, 6, 4> K_1_;
  Eigen::Matrix<double, 6, 4> K_old_1_;


  Eigen::Matrix<double, 2, 1> u_old_1_;

//  void kalmanFilter1();
//  void kalmanStateSpace1();
  //////////////////////////////////////////////////////////////


  /////////////////////////Kalman Filter2///////////////////////

  Eigen::Matrix<double, 8, 8> Ad_2_;
  Eigen::Matrix<double, 8, 2> Bd_2_;
  Eigen::Matrix<double, 4, 8> Cd_2_;
  Eigen::Matrix<double, 8, 8> Q_2_;
  Eigen::Matrix<double, 4, 4> R_2_;


  Eigen::Matrix<double, 8, 1> X_hat_prio_2_;
  Eigen::Matrix<double, 8, 1> X_hat_post_2_;
  Eigen::Matrix<double, 8, 1> X_hat_prio_old_2_;
  Eigen::Matrix<double, 8, 1> X_hat_post_old_2_;

  Eigen::Matrix<double, 4, 1> Y_2_;



  Eigen::Matrix<double, 8, 8> P_prio_2_;
  Eigen::Matrix<double, 8, 8> P_post_2_;
  Eigen::Matrix<double, 8, 8> P_prio_old_2_;
  Eigen::Matrix<double, 8, 8> P_post_old_2_;

  Eigen::Matrix<double, 8, 4> K_2_;
  Eigen::Matrix<double, 8, 4> K_old_2_;


  Eigen::Matrix<double, 2, 1> u_old_2_;


//  void kalmanFilter2();
 // void kalmanStateSpace2();
  //////////////////////////////////////////////////////////////


  /////////////////////////Kalman Filter3///////////////////////

  Eigen::Matrix<double, 10, 10> Ad_3_;
  Eigen::Matrix<double, 10, 2> Bd_3_;
  Eigen::Matrix<double, 6, 10> Cd_3_;
  Eigen::Matrix<double, 10, 10> Q_3_;
  Eigen::Matrix<double, 6, 6> R_3_;


  Eigen::Matrix<double, 10, 1> X_hat_prio_3_;
  Eigen::Matrix<double, 10, 1> X_hat_post_3_;
  Eigen::Matrix<double, 10, 1> X_hat_prio_old_3_;
  Eigen::Matrix<double, 10, 1> X_hat_post_old_3_;

  Eigen::Matrix<double, 6, 1> Y_3_;



  Eigen::Matrix<double, 10, 10> P_prio_3_;
  Eigen::Matrix<double, 10, 10> P_post_3_;
  Eigen::Matrix<double, 10, 10> P_prio_old_3_;
  Eigen::Matrix<double, 10, 10> P_post_old_3_;

  Eigen::Matrix<double, 10, 6> K_3_;
  Eigen::Matrix<double, 10, 6> K_old_3_;


  Eigen::Matrix<double, 2, 1> u_old_3_;

 // void kalmanFilter3();
 // void kalmanStateSpace3();
  //////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////
  ////////////////////dynamic information from rbdl ////////////////////////
  Eigen::Isometry3d link_transform_[29];
  double link_mass_[29];
  Eigen::Matrix<double, 29, 1> link_mass_vector_;
  double total_mass_;
  Eigen::Matrix3d link_inertia_[29];
  Eigen::Vector3d link_local_com_position_[29];

  Eigen::Isometry3d relative_link_transform_[29];
  Eigen::Isometry3d link_from_prior_transform_[29];


  //////////////////////////////////////////////////////////////
  /////////////////////////Centroidal Dynamics/////////////////
  Eigen::Vector3d                        relative_distance_[28];
  Eigen::Matrix<double, 6, 6>            Spatial_Matrix_[29];
  Eigen::Matrix<double, 3, 3>            relative_rotation_[28];
  Eigen::Matrix<double, 6, 6>            System_Inertia_[29];
  Eigen::Matrix<double, 6, 6>            System_Inertia_0_;
  Eigen::Matrix<double, 6, 6>            Spatial_Inertia_[29];
  Eigen::Matrix<double, 6, 6>            COM_Projection_Matrix_[29];
  Eigen::Vector6d                        Centroidal_Momentum_Matrix_[29];
  Eigen::Vector6d                        Centroidal_Momentum_[29];

  Eigen::Vector6d                        Centroidal_Momentum_leg_[2];
  Eigen::Vector6d                        Centroidal_Momentum_Upper_;
  Eigen::Vector6d                        Centroidal_Momentum_total_;

  Eigen::Vector6d                        pre_Centroidal_Momentum_Matrix_[29];
  Eigen::Vector6d                        pre_Centroidal_Momentum_[29];
  Eigen::Vector28d                       q_dot_CMM_;
  Eigen::Vector6d                        CM_R_leg_, CM_L_leg_;
  Eigen::Vector6d                        pre_CM_R_leg_, pre_CM_L_leg_;
  VectorQd                               pre_q_dot_;
  Eigen::Matrix<double, 6, 6>            CCRBI_;
  Eigen::Vector6d                        average_spatial_velocity_;

  Eigen::Vector3d                       current_Angular_momentum_;
  Eigen::Vector3d                       pre_current_Angular_momentum_;

  Eigen::Vector3d                       relative_link_distance_[29];
  Eigen::Matrix3d                       relative_link_rotation_[29];
  Eigen::Matrix6d                       Spatial_link_transform_[29];
  Eigen::Matrix6d                       Spatial_link_Inertia_[29];
  Eigen::Matrix6d                       CCRB_Inertia_[29];
  Eigen::Matrix6d                       Spatial_COM_transform_[29];


  Eigen::Vector3d                       y_temp_;
  Eigen::Matrix3d                       A_c_;

  Eigen::Vector6d                       lp_;
  Eigen::Vector6d                       rp_;

  Eigen::Matrix<double, 6, 28>          Augmented_Centroidal_Momentum_Matrix_;
  //////////////////////////////////////////////////////////////
  /// //////////////////////////////////////////////////////////////

  VectorQd                       p_q_;
  VectorQd                       q_dot_;
  VectorQd                       q_dot_2;
  Eigen::Vector6d                       CM_dot_;
  Eigen::Vector6d                       pre_CM_dot_;
  Eigen::Vector6d                       CM_leg_dot_;
  Eigen::Vector6d                       pre_CM_leg_dot_;
  Eigen::Vector6d Left_foot_CM_, Right_foot_CM_, Left_arm_CM_, Right_arm_CM_, waist_CM_;

  void settingParameter();


  /////////////////////////////////////////////////////////////
  //////////////////////////// QP Problems/////////////////////

  Eigen::MatrixXd Ak_;
  Eigen::MatrixXd bk_;
  Eigen::MatrixXd coef_inverse_;
  Eigen::MatrixXd coef_x_;
  Eigen::MatrixXd p_temp_;
  Eigen::MatrixXd Px_;
  Eigen::MatrixXd Py_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::Vector3d b1_;
  Eigen::MatrixXd pu_t_;
  Eigen::MatrixXd pu_;
  Eigen::MatrixXd Av_;
  Eigen::MatrixXd bv_;
  Eigen::MatrixXd Av0_;
  Eigen::MatrixXd Selec_delta_;
  Eigen::MatrixXd Qx_inverse_;
  Eigen::MatrixXd Qy_inverse_;
  Eigen::MatrixXd bvt_sdeltat_;
  Eigen::MatrixXd Qx_;
  Eigen::MatrixXd Qy_;
  Eigen::MatrixXd Qu_;
  Eigen::MatrixXd Au_;
  Eigen::MatrixXd Velocity_delta_x_;
  Eigen::MatrixXd Velocity_delta_y_;
  Eigen::MatrixXd selec_bv_;
  Eigen::MatrixXd selec_delta_v_;

  Eigen::MatrixXd Selec_delta_v_reduced_;
  Eigen::MatrixXd Selec_B_vi_reduced_;
  Eigen::MatrixXd Selec_A_vi_reduced_;
  Eigen::MatrixXd Selec_delta_a_reduced_;
  Eigen::MatrixXd B_ai_reduced_;
  Eigen::MatrixXd A_ai_reduced_;
  Eigen::MatrixXd PA_reduced_;
  Eigen::MatrixXd PB_reduced_;

  Eigen::MatrixXd Av_variance_;
  Eigen::MatrixXd bv_variance_;
  Eigen::MatrixXd Qx_variance_;
  Eigen::MatrixXd Qx_variance_inverse_;
  Eigen::MatrixXd Qy_variance_;
  Eigen::MatrixXd Qy_variance_inverse_;
  double alpha_x_, beta_x_,gamma_x_;
  double alpha_y_, beta_y_,gamma_y_;
  double alpha_, beta_, gamma_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Q_inverse_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd Velocity_delta_;
  Eigen::MatrixXd Q_variance_;
  Eigen::MatrixXd Q_variance_inverse_;

  Eigen::MatrixXd swing_foot_pos_;
  Eigen::MatrixXd swing_foot_acc_;
  Eigen::MatrixXd Ki_x_;
  Eigen::MatrixXd Ki_y_;
  Eigen::MatrixXd Di_x_;
  Eigen::MatrixXd Di_y_;
  Eigen::MatrixXd Px_swing_;
  Eigen::MatrixXd Pu_swing_;
  Eigen::MatrixXd Px_swing_0_;
  Eigen::MatrixXd Pu_swing_0_;

  double Mass_sb_;// mass of support body
  double Mass_sl_;// mass of swing leg



  real_t lbA_[3];
  real_t ubA_[3];

  real_t H_[3*3];
 // real_t A_[3*3];
  real_t xOpt_[3];
  real_t g_[3];
  real_t lb_[3];
  real_t ub_[3];

//  real_t    lbA_[3];
//  real_t    ubA_[3];

//  real_t    H_[9*9];
//  real_t    A_[3*9];
//  real_t    xOpt_[9];
//  real_t    g_[9];
//  real_t    lb_[9];
//  real_t    ub_[9];

  real_t _init_H_[9*9];
  real_t _init_A_[1*9];
  real_t _init_g_[9];
  real_t _init_lb_[9];
  real_t _init_ub_[9];
  real_t _init_lbA_[1];
  real_t _init_ubA_[1];

  ///////////////////////////////////////////////////////////////
  /////////////// Resolved momentum control////////////////////

  Eigen::Vector3d                   inertia_m_vector_[29];
  Eigen::Vector3d                   inertia_h_0_vector_[29];
  Eigen::Vector3d                   inertia_h_vector_[29];
  Eigen::Matrix6d                   inertia_body_;
  Eigen::Matrix6d                   inertia_right_leg_;
  Eigen::Matrix6d                   inertia_left_leg_;
  Eigen::Matrix3d                   total_inertia_;
  VectorQd                          target_speed_;
  Eigen::Vector6d                   momentum_;

  Eigen::Matrix6d                   current_leg_jacobian_l_inv_, current_leg_jacobian_r_inv_;

   bool walking_cmd_;

  ////////////////////////////parameter setting of MPC with qp oases /////////////////////
    Eigen::Vector3d x_p1_;
    Eigen::Vector3d y_p1_;
    Eigen::Vector3d x_d1_;
    Eigen::Vector3d y_d1_;
    Eigen::MatrixXd com_data_;
    Eigen::MatrixXd com_data2_;
    Eigen::MatrixXd foot_data_;
//    Eigen::Vector2d opt_x_;
//    Eigen::Vector2d opt_y_;
    double opt_x_;
    double opt_y_;
    bool MPC_Matrix_cal_;

    //////////////////////////////// parameter setting for swing foot trajectory optimization ////////////////////

    //////////////////////////////// parameter setting for foot edge control /////////////////////////////
    Eigen::Matrix6d current_left_toe_jacobian_;
    Eigen::Matrix6d current_left_heel_jacobian_;
    Eigen::Matrix6d current_right_toe_jacobian_;
    Eigen::Matrix6d current_right_heel_jacobian_;

    /////// parameter for intrisically stable MPC////////

    Eigen::VectorXd p_vector_;
    Eigen::MatrixXd P_matrix_;
    Eigen::VectorXd Lambda_vector_;

    Eigen::Vector3d x_p_temp_;
    Eigen::Vector3d y_p_temp_;

    ///// parameter for controlled LIPM updated by myeongju

    //CP Feedback MJ
    double cp_err_x = 0, cp_err_y = 0;
    double del_px_cp = 0, del_py_cp = 0;

    double x_cp_ref = 0, y_cp_ref = 0;
    double x_cp_act = 0, y_cp_act = 0;
    double Wn;

    double UX_, UY_;

    double py_err, y_des, y_ddot_des;

    double XD_b = 0, YD_b = 0, XD_bb = 0, YD_bb = 0;
    double X_b = 0, Y_b = 0, X_bb = 0, Y_bb = 0;
    double XD_vel = 0, YD_vel = 0, XD_vel_b = 0, YD_vel_b = 0;
    double XA_vel = 0, YA_vel = 0, XA_vel_b = 0, YA_vel_b = 0;


    //Step current state variable//

    Eigen::Vector3d com_support_current_b;
    Eigen::Vector3d com_support_current_dot;
//    Eigen::Vector3d com_support_current_;
    Eigen::Vector3d com_support_current_CLIPM_b;
    Eigen::Vector3d com_support_current_CLIPM_;
    Eigen::Vector3d com_middle_support_current_;
//    Eigen::Vector3d com_support_dot_current_;//from support foot
    Eigen::Vector3d com_support_ddot_current_;//from support foot
    Eigen::Isometry3d com_support_current_MJ;

    Eigen::MatrixXd modified_ref_zmp_;

    Eigen::Vector3d com_support_fixed_;
    Eigen::Isometry3d pelv_support_fixed_;

    Eigen::VectorXd B_vector_;
    Eigen::MatrixXd C_;
    Eigen::MatrixXd D_;
    Eigen::Matrix3d K_;
    Eigen::MatrixXd Gi_;
    Eigen::MatrixXd Gx_;
    Eigen::VectorXd Gd_;
    Eigen::MatrixXd A_bar_;
    Eigen::VectorXd B_bar_;
    Eigen::Vector2d Preview_X, Preview_Y, Preview_X_b, Preview_Y_b;
    Eigen::VectorXd X_bar_p_, Y_bar_p_;
    Eigen::Vector2d XD_;
    Eigen::Vector2d YD_;


    /////////////////// for heel toe optimization //////////////////////
    Eigen::Isometry3d ltoe_float_current_;
    Eigen::Isometry3d lheel_float_current_;
    Eigen::Isometry3d rtoe_float_current_;
    Eigen::Isometry3d rheel_float_current_;

    Eigen::VectorXd q_leg_min_;
    Eigen::VectorXd q_leg_max_;

    Eigen::VectorXd q_upper_min_;
    Eigen::VectorXd q_upper_max_;


    Eigen::Isometry3d pre_ltoe_trajectory_;
    Eigen::Isometry3d pre_rtoe_trajectory_;
    Eigen::Isometry3d pre_lheel_trajectory_;
    Eigen::Isometry3d pre_rheel_trajectory_;

    Eigen::Isometry3d ltoe_float_init_;
    Eigen::Isometry3d rtoe_float_init_;
    Eigen::Isometry3d lheel_float_init_;
    Eigen::Isometry3d rheel_float_init_;

    Eigen::Vector12d pre_toe_q;
    Eigen::Vector6d ltoe_p_;
    Eigen::Vector6d rtoe_p_;
    Eigen::Vector6d lheel_p_;
    Eigen::Vector6d rheel_p_;

    Eigen::Isometry3d ltoe_trajectory_float_;
    Eigen::Isometry3d rtoe_trajectory_float_;
    Eigen::Isometry3d lheel_trajectory_float_;
    Eigen::Isometry3d rheel_trajectory_float_;

    Eigen::Isometry3d ltoe_trajectory_support_;
    Eigen::Isometry3d rtoe_trajectory_support_;
    Eigen::Isometry3d lheel_trajectory_support_;
    Eigen::Isometry3d rheel_trajectory_support_;


    Eigen::Vector3d ltoe_trajectory_euler_;
    Eigen::Vector3d rtoe_trajectory_euler_;
    Eigen::Vector3d lheel_trajectory_euler_;
    Eigen::Vector3d rheel_trajectory_euler_;

    Eigen::Vector6d lp_clik_;
    Eigen::Vector6d rp_clik_;
    Eigen::Vector6d ltoe_clik_;
    Eigen::Vector6d rtoe_clik_;
    Eigen::Vector6d lheel_clik_;
    Eigen::Vector6d rheel_clik_;

    Eigen::Vector3d rfoot_support_current_euler_;
    Eigen::Vector3d lfoot_support_current_euler_;
    Eigen::Isometry3d rfoot_support_heel_;
    Eigen::Isometry3d lfoot_support_heel_;

    Eigen::Vector12d pre_q_sol_;
    Eigen::Vector3d lfoot_lifting_float_euler_init_;
    Eigen::Vector3d rfoot_lifting_float_euler_init_;
    Eigen::Vector3d lfoot_landing_float_euler_init_;
    Eigen::Vector3d rfoot_landing_float_euler_init_;
    Eigen::Vector3d lfoot_dsp2_float_euler_init_;
    Eigen::Vector3d rfoot_dsp2_float_euler_init_;

    Eigen::Vector3d lfoot_lifting_support_euler_init_;
    Eigen::Vector3d rfoot_lifting_support_euler_init_;
    Eigen::Vector3d lfoot_landing_support_euler_init_;
    Eigen::Vector3d rfoot_landing_support_euler_init_;
    Eigen::Vector3d lfoot_dsp2_support_euler_init_;
    Eigen::Vector3d rfoot_dsp2_support_euler_init_;

    Eigen::Isometry3d lfoot_lifting_float_init_;
    Eigen::Isometry3d rfoot_lifting_float_init_;
    Eigen::Isometry3d lfoot_DSP2_float_init_;
    Eigen::Isometry3d rfoot_DSP2_float_init_;

    Eigen::Isometry3d lfoot_lifting_support_init_;
    Eigen::Isometry3d rfoot_lifting_support_init_;
    Eigen::Isometry3d lfoot_DSP2_support_init_;
    Eigen::Isometry3d rfoot_DSP2_support_init_;

    Eigen::Isometry3d lfoot_DSP1_support_init_;
    Eigen::Isometry3d rfoot_DSP1_support_init_;

    Eigen::Isometry3d ltoe_DSP1_support_init_;
    Eigen::Isometry3d rtoe_DSP1_support_init_;

    double pushing_force_;
    double pre_pushing_force_;

    ///// for state estimator declare
    ///

    void getQpEstimationInputMatrix();
    void kalmanStateSpace1();
    void kalmanFilter1();
    void kalmanStateSpace2();
    void kalmanFilter2();
    void kalmanStateSpace3();
    void kalmanFilter3();

    Eigen::Vector2d zmp_measured_1_;

    // for DOB modified control //
    Eigen::Vector12d DOB_IK_output_;
    Eigen::Vector12d DOB_IK_output_b_;
    Eigen::Vector12d d_hat_b_;
    Eigen::Matrix<double, 6, 7> current_leg_jacobian_l_floating_;
    Eigen::Matrix<double, 6, 7> current_leg_jacobian_r_floating_;
    double          floating_joint_;
    double          floating_joint_init_;
    double          pre_floating_joint_;

    /// using floating frame above the pelvis
    ///
    Eigen::Isometry3d   pelvis_floatingbase_current_;
    Eigen::Isometry3d   pelvis_floatingbase_init_;

    Eigen::Vector3d     pelvis_floatingbase_current_euler_;
    Eigen::Vector3d     pelvis_floatingbase_current_euler_init_;

    Eigen::Isometry3d   floating_support_init_;
    Eigen::Isometry3d   floating_support_current_;
    Eigen::Isometry3d   floating_trajectory_support_;

    Eigen::MatrixXd     future_lfoot_trajectory_support_;
    Eigen::MatrixXd     future_rfoot_trajectory_support_;

    Eigen::MatrixXd     future_com_;
    Eigen::MatrixXd     future_lhip_;
    Eigen::MatrixXd     future_rhip_;
    Eigen::VectorXd     future_pel_yaw_;
    Eigen::VectorXd     future_pel_est_;

    Eigen::Matrix2d     a_pel_;
    Eigen::Vector2d     b_pel_;

    Eigen::MatrixXd     A_pel_;
    Eigen::MatrixXd     B_pel_;

    Eigen::MatrixXd     Q_pel_;
    Eigen::VectorXd     g_pel_;

    double              alpha_pel_;
    double              beta_pel_;

    Eigen::Vector2d     pre_pel_yaw_;

    ///////// COM generator with MPC revision //////////////
    Eigen::MatrixXd     Qu_inverse_;

    Eigen::Matrix3d     a_mpc_;
    Eigen::Vector3d     b_mpc_;

    Eigen::MatrixXd     Px_mpc_;
    Eigen::MatrixXd     Pu_mpc_;

    Eigen::MatrixXd     Pax_mpc_;
    Eigen::MatrixXd     Pau_mpc_;

    Eigen::MatrixXd     Pzx_mpc_;
    Eigen::MatrixXd     Pzu_mpc_;

    Eigen::MatrixXd     Qx_mpc_;
    Eigen::MatrixXd     Qy_mpc_;

    double              alpha_mpc_;
    double              beta_mpc_;
    double              gamma_mpc_;

    Eigen::Vector3d     pre_mpc_x_;
    Eigen::Vector3d     pre_mpc_y_;
    Eigen::Vector3d     mpc_x_;
    Eigen::Vector3d     mpc_y_;
    Eigen::MatrixXd     future_com_y_mpc_;
    Eigen::MatrixXd     future_com_x_mpc_;
    Eigen::VectorXd     future_pel_yaw3_;
    Eigen::VectorXd     pre_future_pel_yaw3_;
    Eigen::VectorXd     future_pel_yaw2_;

    double              pre_future_pel_yaw2_;
    double              pre_future_pel_ref_;

    int                 N_first_;
    int                 N_second_;
    int                 N_third_;
    int                 N_total_;
    bool                future_singular_;

    int                 N_;
    int                 interval_;

    double              first_pel_yaw_ref_;
    double              second_pel_yaw_ref_;
    double              third_pel_yaw_ref_;

    Eigen::VectorXd     future_pel_yaw_ref_;

};

}
#endif // WALKING_CONTROLLER_H



