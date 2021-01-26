#include <ros/ros.h>
#include <math.h>
#include <iostream> 
#include <string>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include "quadruped_ctrl/commandDes.h"
#include "quadruped_ctrl/QuadrupedCmd.h"
#include <std_msgs/Float64MultiArray.h>
#include "MPC_Ctrl/ConvexMPCLocomotion.h"
#include "Controllers/RobotLegState.h"
#include "Controllers/ControlFSMData.h"
#include "Dynamics/MiniCheetah.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Utilities/IMUTypes.h"
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/PositionVelocityEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Controllers/ContactEstimator.h"
#include "calculateTool.h"
#include "WBC_Ctrl/WBC_Ctrl.hpp"
#include <planner_msgs/foothold_request.h>
#include <time.h>

#define freq 400.0
#define pai 3.1415926
  float plan_f_foot_pos[22] = {0.370636,0.470636,0.570636,0.670636,0.770636,0.870636,0.95,1.07,1.17,1.25,1.37,1.47,1.55,1.67,1.76,1.83, 1.93, 2.03, 2.13, 2.23,2.33,2.43};
float plan_b_foot_pos[22] = {-0.0172972,0.0827028,0.1827028,0.2827028,0.3827028,0.4827028,0.5827028,0.6827028,0.7827028,0.8827028,0.95,
                            1.07,1.17,1.25,1.25,1.37, 1.47, 1.57, 1.67, 1.77, 1.87, 1.97};
int iter = 0;
int iterr = 0;
int iterr1 = 0;
     int iteriter = 1;
bool swingstate = false;
bool standstate = false;
int set_robot_mode, set_gait_type;
std::vector<double> _gamepadCommand;
std::vector< Vec3<float> > _ini_foot_pos;
Vec4<float> ctrlParam_sim, ctrlParam_robot;
static int foothold_request_id=0;
double v_cmd=0.0;
static planner_msgs::foothold_request foothold_request_msg;//lt
static planner_msgs::foothold_request foothold_all_msg;

float init_joint_pos[12] = {   -0.002856431546828908, 0.8471560427609397,-1.6763175043898286,
                                0.00232150030099465, 0.8489298567967767,  -1.6735534674729875,
                                -0.001352735573201258, 0.844812465289789, -1.6771619146942545,
                                0.00039701748856746377, 0.8414008351135207,-1.6739155945140425};

std::string joint_name[12] = { "FR_hip_joint", "FR_thigh_joint","FR_calf_joint",
                               "FL_hip_joint", "FL_thigh_joint","FL_calf_joint",
                               "RR_hip_joint", "RR_thigh_joint","RR_calf_joint",
                               "RL_hip_joint", "RL_thigh_joint","RL_calf_joint" };




ConvexMPCLocomotion *convexMPC;
LegController<float>* _legController;
StateEstimatorContainer<float>* _stateEstimator;
LegData legdata;
LegCommand legcommand;
ControlFSMData<float> control_data;
//VectorNavData _vectorNavData;
CheaterState<double>* cheaterState;
StateEstimate<float> _stateEstimate;
RobotControlParameters* controlParameters;
DesiredStateCommand<float>* _desiredStateCommand;
ControlFSMData<float>* _data;
WBC_Ctrl<float>* _wbc_ctrl;
LocomotionCtrlData<float>* _wbc_data;
MIT_UserParameters* userParameters;

void QuadrupedCtrl(){
  

}

void velCmdCallBack(const geometry_msgs::Twist& msg){
  if(abs(msg.linear.x) < 0.1){
    _gamepadCommand[0] = 0.0;
  }else{
    _gamepadCommand[0] = msg.linear.x;
  }

  if(abs(msg.linear.y) < 0.1){
    _gamepadCommand[1] = 0.0;
  }else{
    _gamepadCommand[1] = msg.linear.y * 0.5;
  }

  if(abs(msg.angular.x) < 0.1){
    _gamepadCommand[2] = 0.0;
  }else{
    _gamepadCommand[2] = msg.angular.x * 2.0;
  }

  if(abs(msg.angular.y) < 0.1){
    _gamepadCommand[3] = 0.0;
  }else{
    _gamepadCommand[3] = msg.angular.y * 2.0;
  }  
  
}

void footholdPlanVelCmdCallBack(const geometry_msgs::Twist& msg){
  std::cout << " msg.linear.x" <<  msg.linear.x <<endl;
  v_cmd = msg.linear.x;
}

void ImuCmdCallBack(const sensor_msgs::Imu& msg){//IMU数据
  _vectorNavData.accelerometer(0, 0) = msg.linear_acceleration.x;
  _vectorNavData.accelerometer(1, 0) = msg.linear_acceleration.y;
  _vectorNavData.accelerometer(2, 0) = msg.linear_acceleration.z;
//std::cout << "   xo  " <<_vectorNavData.accelerometer(0, 0) << "   yo  " << _vectorNavData.accelerometer(0, 0) << "  zo  " << _vectorNavData.accelerometer(0, 0) <<  "  w "<< msg.orientation.w << std::endl;

  _vectorNavData.quat(0, 0) = msg.orientation.x;
  _vectorNavData.quat(1, 0) = msg.orientation.y;
  _vectorNavData.quat(2, 0) = msg.orientation.z;
  _vectorNavData.quat(3, 0) = msg.orientation.w;
  //std::cout << "msgr " << _vectorNavData.quat(0, 0) << "  msgp " << _vectorNavData.quat(1, 0) << " msgy "<< _vectorNavData.quat(2, 0) << std::endl;

  _vectorNavData.gyro(0, 0) = msg.angular_velocity.x;
  _vectorNavData.gyro(1, 0) = msg.angular_velocity.y;
  _vectorNavData.gyro(2, 0) = msg.angular_velocity.z;
}

void jointStateCmdCallBack(const sensor_msgs::JointState& msg){//反馈的关节角度和速度

    legdata.q_abad[0] = msg.position[1 * 3 + 1];
    legdata.q_hip[0] = -msg.position[1 * 3 + 2];
    legdata.q_knee[0] = -msg.position[1 * 3 ];
    legdata.qd_abad[0] = msg.velocity[1 * 3 + 1];
    legdata.qd_hip[0] = -msg.velocity[1 * 3 + 2];
    legdata.qd_knee[0] = -msg.velocity[1 * 3 ];
    legdata.q_abad[1] = msg.position[0 * 3 + 1];
    legdata.q_hip[1] = -msg.position[0 * 3 + 2];
    legdata.q_knee[1] = -msg.position[0 * 3 ];
    legdata.qd_abad[1] = msg.velocity[0 * 3 + 1];
    legdata.qd_hip[1] = -msg.velocity[0 * 3 + 2];
    legdata.qd_knee[1] = -msg.velocity[0 * 3 ];
    legdata.q_abad[2] = msg.position[3 * 3 + 1];
    legdata.q_hip[2] = -msg.position[3 * 3 + 2];
    legdata.q_knee[2] = -msg.position[3 * 3 ];
    legdata.qd_abad[2] = msg.velocity[3 * 3 + 1];
    legdata.qd_hip[2] = -msg.velocity[3 * 3 + 2];
    legdata.qd_knee[2] = -msg.velocity[3 * 3 ];
    legdata.q_abad[3] = msg.position[2 * 3 + 1];
    legdata.q_hip[3] = -msg.position[2 * 3 + 2];
    legdata.q_knee[3] = -msg.position[2 * 3 ];
    legdata.qd_abad[3] = msg.velocity[2 * 3 + 1];
    legdata.qd_hip[3] = -msg.velocity[2 * 3 + 2];
    legdata.qd_knee[3] = -msg.velocity[2 * 3 ];
    
}

void RobotComCallBack(const nav_msgs::Odometry &msg){//反馈的关节角度和速度

  _vectorNavData.com_pos(0, 0) = msg.pose.pose.position.x;
  _vectorNavData.com_pos(1, 0) = msg.pose.pose.position.y;
  _vectorNavData.com_pos(2, 0) = msg.pose.pose.position.z;
  //std::cout << "Y位置 " << _vectorNavData.com_pos(1, 0) << "Y速度 " << _vectorNavData.com_vel(1, 0) << std::endl;
  _vectorNavData.com_vel(0, 0) = msg.twist.twist.linear.x;
  _vectorNavData.com_vel(1, 0) = msg.twist.twist.linear.y;
  _vectorNavData.com_vel(2, 0) = msg.twist.twist.linear.z;
  //std::cout << _vectorNavData.com_pos << std::endl;
}

void FLFootposCallBack(const visualization_msgs::Marker &marker1){//规划的落脚点
  
    footplan[1] = marker1.pose.position.x;
    std::cout << "规划flx " << footplan[1] << "  fly " << footplany[1]<< "  flz  " << footplanz[1]  << std:: endl;
    footplany[1] = marker1.pose.position.y;
    footplanz[1] = marker1.pose.position.z;


  /*
  visualization_msgs::Marker marker;
  for(int i = 0; i < sizeof(marker_array); i++)
  {
    marker = marker_array.markers[i];
    if(marker.ns == "FR_foothold_marker"){
      footplan[0] = marker.pose.position.x;
      std::cout << "footpos00  " << footplan[0] <<std::endl;
      footplany[0] = marker.pose.position.y;
      }
    else if (marker.ns == "FL_foothold_marker")
    {
      footplan[1] = marker.pose.position.x;
      footplany[1] = marker.pose.position.y;
    }else if (marker.ns == "RR_foothold_marker")
    {
      footplan[2] = marker.pose.position.x;
      footplany[2] = marker.pose.position.y;
    }else if(marker.ns == "RL_foothold_marker")
    {
      footplan[3] = marker.pose.position.x;
      footplany[3] = marker.pose.position.y;
    }
    //std::cout << sizeof(marker_array) << std::endl;
  }
  std::cout << "footpos  " << footplan[0] <<std::endl;
  */
  
}
void FRFootposCallBack(const visualization_msgs::Marker &marker){//规划的落脚点
  

      footplan[0] = marker.pose.position.x;
      std::cout << "规划frx " << footplan[0] << "  fry " << footplany[0]<< "  frz  " << footplanz[0]  << std:: endl;

      footplany[0] = marker.pose.position.y;
      footplanz[0] = marker.pose.position.z;

  
}
void RLFootposCallBack(const visualization_msgs::Marker &marker3){//规划的落脚点
  

      footplan[3] = marker3.pose.position.x;
      std::cout << "规划rlx " << footplan[3] << "  rly " << footplany[3]<< "  rlz  " << footplanz[3]  << std:: endl;

      footplany[3] = marker3.pose.position.y;
      footplanz[3] = marker3.pose.position.z;


  
}
void RRFootposCallBack(const visualization_msgs::Marker &marker2){//规划的落脚点
  

      footplan[2] = marker2.pose.position.x;
      std::cout << "规划rrx " << footplan[2] << "  rry " << footplany[2]<< "  rrz  " << footplanz[3]  << std:: endl;
      footplany[2] = marker2.pose.position.y;
      footplanz[2] = marker2.pose.position.z;


  
}

bool setRobotMode(quadruped_ctrl::QuadrupedCmd::Request  &req,
                  quadruped_ctrl::QuadrupedCmd::Response &res)
{
  set_robot_mode = req.cmd;

  return true;
}

bool setGaitType(quadruped_ctrl::QuadrupedCmd::Request  &req,
                  quadruped_ctrl::QuadrupedCmd::Response &res)
{
  set_gait_type = req.cmd;

  return true;
}


int main(int argc, char **argv) {
  float hMax = 0.35;
  _gamepadCommand.clear();
  _gamepadCommand.resize(4);
  _ini_foot_pos.clear();
  _ini_foot_pos.resize(4);

  clock_t start, finish; 

  Quadruped<float> _quadruped;
  FloatingBaseModel<float> _model;
  convexMPC = new ConvexMPCLocomotion(1/freq, 20);
  
  _quadruped = buildMiniCheetah<float>();
  _model = _quadruped.buildModel();
  _wbc_ctrl = new LocomotionCtrl<float>(_model);
  _wbc_data = new LocomotionCtrlData<float>();
  _legController = new LegController<float>(_quadruped);

  _stateEstimator = new StateEstimatorContainer<float>(
      cheaterState, &_vectorNavData, _legController->datas,
      &_stateEstimate, controlParameters);

  //reset the state estimator
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);

  //源代码中中对姿态和位置速度的估计，添加到状态估计器中
  _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
  _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
  
  _desiredStateCommand =
    new DesiredStateCommand<float>(1/freq);

  ros::init(argc, argv, "quadruped");
  ros::NodeHandle n;

  ros::Rate loop_rate(freq);
  //sensor_msgs::JointState joint_state;
  std_msgs::Float64MultiArray joint_effort;
  ros::Publisher pub_effort = n.advertise<std_msgs::Float64MultiArray>("/joint_group_position_controller/command", 1000);//发布力矩
  //ros::Publisher pub_joint = n.advertise<sensor_msgs::JointState>("/pegasus_mini_gazebo/joint_states", 1000);  //下发给simulator或者robot的关节控制数据（关节扭矩）
  // ros::ServiceClient jointCtrlMode = n.serviceClient<quadruped_ctrl::QuadrupedCmd>("set_jm");
  ros::ServiceServer robotMode = n.advertiseService("robot_mode", setRobotMode);     //接收机器人状态设置
  ros::ServiceServer gaitType = n.advertiseService("gait_type", setGaitType);        //接收机器人步态选择切换
  ros::Subscriber sub_vel = n.subscribe("cmd_vel", 1000, velCmdCallBack);           //接收的手柄速度信息
  ros::Subscriber sub_imu = n.subscribe("/pegasus_mini_imu", 1000, ImuCmdCallBack);          // imu反馈的身体位置和姿态
  ros::Subscriber sub_jointstate = n.subscribe("/joint_states", 1000, jointStateCmdCallBack);   //simulator或者robot反馈的关节信息（位置、速度等）
  ros::Subscriber sub_com_state = n.subscribe("/pegasus_mini_gazebo/ground_truth", 1000, RobotComCallBack);
  ros::Publisher foothold_request_pub = n.advertise<planner_msgs::foothold_request>("/pegasus_mini/foothold_request", 1);
  ros::Publisher foothold_all_pub = n.advertise<planner_msgs::foothold_request>("/pegasus_mini/foothold_all", 1);
  ros::Subscriber sub_v_cmd = n.subscribe("/v_cmd", 1000, footholdPlanVelCmdCallBack);   //接收的落脚点反馈回来的速度信息
  ros::Subscriber sub_footfl = n.subscribe("/FL_foothold_marker", 1000, FLFootposCallBack);
  ros::Subscriber sub_footfr = n.subscribe("/FR_foothold_marker", 1000, FRFootposCallBack);
  ros::Subscriber sub_footrl = n.subscribe("/RL_foothold_marker", 1000, RLFootposCallBack);
  ros::Subscriber sub_footrr = n.subscribe("/RR_foothold_marker", 1000, RRFootposCallBack);

  //control parameters in simulation
  n.getParam("/simulation/stand_kp", ctrlParam_sim(0));
  n.getParam("/simulation/stand_kd", ctrlParam_sim(1));
  n.getParam("/simulation/joint_kp", ctrlParam_sim(2));
  n.getParam("/simulation/joint_kd", ctrlParam_sim(3));

  //control parameters in simulation
  n.getParam("/robot/stand_kp", ctrlParam_robot(0));
  n.getParam("/robot/stand_kd", ctrlParam_robot(1));
  n.getParam("/robot/joint_kp", ctrlParam_robot(2));
  n.getParam("/robot/joint_kd", ctrlParam_robot(3));


  usleep(1000 * 1000);

  ros::spinOnce();

  while (ros::ok()){
    //joint_state.header.stamp = ros::Time::now();
    //joint_effort.header.stamp = ros::Time::now();

    //joint_state.name.resize(12);
    //joint_state.position.resize(12);
    //joint_state.effort.resize(12);
    joint_effort.data.resize(12);
    
    _stateEstimator->run();//Run the state estimator step
    
    // Update the data from the robot
    _legController->updateData(&legdata); 

    // Setup the leg controller for a new iteration
    _legController->zeroCommand();
    _legController->setEnabled(true);
    _legController->setMaxTorqueCheetah3(208.5);

    // Find the desired state trajectory
    _desiredStateCommand->convertToStateCommands(_gamepadCommand);
   
    convexMPC->run(_quadruped, *_legController, *_stateEstimator, *_desiredStateCommand, _gamepadCommand, set_gait_type,v_cmd);
    
    // start = clock();
    // finish = clock();
    // double duration;
    // duration = (double)(finish - start) / CLOCKS_PER_SEC;
    // printf( "%f seconds\n", duration );
    // for(int i = 0; i < 3; i++){
    //   std::cout << "get com position" << legdata.robot_com_position[i] << std::endl;
    // }

    
  if(use_wbc){
    _wbc_data->pBody_des = convexMPC->pBody_des;
    _wbc_data->vBody_des = convexMPC->vBody_des;
    _wbc_data->aBody_des = convexMPC->aBody_des;
    _wbc_data->pBody_RPY_des = convexMPC->pBody_RPY_des;
    _wbc_data->vBody_Ori_des = convexMPC->vBody_Ori_des;
    
    for(size_t i(0); i<4; ++i){
      _wbc_data->pFoot_des[i] = convexMPC->pFoot_des[i];
      _wbc_data->vFoot_des[i] = convexMPC->vFoot_des[i];
      _wbc_data->aFoot_des[i] = convexMPC->aFoot_des[i];
      _wbc_data->Fr_des[i] = convexMPC->Fr_des[i]; 
        //std::cout << "foot " << i << " pos "<<_wbc_data->pFoot_des[i] << std::endl;
      //std::cout << "foot " << i << " v "<<_wbc_data->vFoot_des[i] << std::endl;
    }
    _wbc_data->contact_state = convexMPC->contact_state;
    _wbc_ctrl->run(_wbc_data, _quadruped, *_legController, *_stateEstimator, *_desiredStateCommand, userParameters);
   for(int leg(0); leg<4; ++leg){
    //_legController->commands[leg].pDes = pDes_backup[leg];
    _legController->commands[leg].vDes = convexMPC->vDes_backup[leg];
    //_legController->commands[leg].kpCartesian = Kp_backup[leg];
    _legController->commands[leg].kdCartesian = convexMPC->Kd_backup[leg];
    }

    foothold_all_msg.FR_status.data=1;
    foothold_all_msg.FR_foot.x=convexMPC->pub_Pf[0][0];
    foothold_all_msg.FR_foot.y=convexMPC->pub_Pf[0][1];
    foothold_all_msg.FR_foot.z=convexMPC->pub_Pf[0][2];

    foothold_all_msg.RL_status.data=1;
    foothold_all_msg.RL_foot.x=convexMPC->pub_Pf[3][0];
    foothold_all_msg.RL_foot.y=convexMPC->pub_Pf[3][1];
    foothold_all_msg.RL_foot.z=convexMPC->pub_Pf[3][2];

    foothold_all_msg.RR_status.data=1;
    foothold_all_msg.RR_foot.x=convexMPC->pub_Pf[2][0];
    foothold_all_msg.RR_foot.y=convexMPC->pub_Pf[2][1];
    foothold_all_msg.RR_foot.z=convexMPC->pub_Pf[2][2];

    foothold_all_msg.FL_status.data=1;
    foothold_all_msg.FL_foot.x=convexMPC->pub_Pf[1][0];
    foothold_all_msg.FL_foot.y=convexMPC->pub_Pf[1][1];
    foothold_all_msg.FL_foot.z=convexMPC->pub_Pf[1][2];

    
    foothold_all_pub.publish(foothold_all_msg);
  
    if(convexMPC->contact_state[0] <= 0.0)
    {
      foothold_request_msg.FR_status.data=0;
    }else
    {
      foothold_request_msg.FR_status.data=1;
    }
    if(convexMPC->contact_state[1] <= 0.0)
    {
      foothold_request_msg.FL_status.data=0;
    }else
    {
      foothold_request_msg.FL_status.data=1;
    }
    if(convexMPC->contact_state[2] <= 0.0)
    {
      foothold_request_msg.RR_status.data=0;
    }else
    {
      foothold_request_msg.RR_status.data=1;
    }
    if(convexMPC->contact_state[3] <= 0.0)
    {
      foothold_request_msg.RL_status.data=0;
    }else
    {
      foothold_request_msg.RL_status.data=1;
    }

    
    foothold_request_msg.FR_foot.x=convexMPC->pub_Pf[0][0];
    foothold_request_msg.FR_foot.y=convexMPC->pub_Pf[0][1];
    foothold_request_msg.FR_foot.z=convexMPC->pub_Pf[0][2];
    
    foothold_request_msg.RL_foot.x=convexMPC->pub_Pf[3][0];
    foothold_request_msg.RL_foot.y=convexMPC->pub_Pf[3][1];
    foothold_request_msg.RL_foot.z=convexMPC->pub_Pf[3][2];
    
    foothold_request_msg.RR_foot.x=convexMPC->pub_Pf[2][0];
    foothold_request_msg.RR_foot.y=convexMPC->pub_Pf[2][1];
    foothold_request_msg.RR_foot.z=convexMPC->pub_Pf[2][2];
    
    foothold_request_msg.FL_foot.x=convexMPC->pub_Pf[1][0];
    foothold_request_msg.FL_foot.y=convexMPC->pub_Pf[1][1];
    foothold_request_msg.FL_foot.z=convexMPC->pub_Pf[1][2];
    
    if(foothold_all_msg.RR_status.data==1 && foothold_all_msg.FR_status.data==1 &&
    foothold_all_msg.RL_status.data==1 && foothold_all_msg.FL_status.data==1 
      && swingstate == false && standstate == false && set_gait_type == 9){
      foothold_request_id=-1;
      // footplan[0] =  0.24;
      // footplan[3] =  -0.05;
      // footplan[1] =  0.24;
      // footplan[2] =  -0.05;
      
      iterr ++;
      std::cout << "第 "<< iterr <<" 次 "<<"00000000"<<std::endl;
      foothold_request_msg.foothold_request_id.data = foothold_request_id;
      foothold_request_pub.publish(foothold_request_msg);

      // footplan[0] = plan_f_foot_pos[0];
      // footplan[1] = plan_f_foot_pos[0];
      // footplan[2] = plan_b_foot_pos[0];
      // footplan[3] = plan_b_foot_pos[0];
     
      //Pf[2] = footplanz[0];
      standstate = true;
      
    }
    else if (foothold_request_msg.FR_status.data == 0 && foothold_request_msg.RL_status.data == 0 && convexMPC->vFoot_des[0][0] != 0)
    {
      swingstate = true;
    }
    
    if(foothold_request_msg.FR_status.data == 1 && foothold_request_msg.RL_status.data == 1 && foothold_request_msg.FL_status.data == 1 && foothold_request_msg.RR_status.data == 1 
     && swingstate == true && standstate == true)
    {
       foothold_request_id++;
       foothold_request_msg.foothold_request_id.data = foothold_request_id;
       foothold_request_pub.publish(foothold_request_msg);
      // // footplan[0] +=  0.10;
      // // footplan[3] +=  0.10;
      iterr1 ++;

      std::cout << "第 "<< iterr1 <<" 次 "<<std::endl;
      std::cout << "1111111111111111111foothold_request_id:" << foothold_request_id<<std::endl;
      std::cout << "当前flx " << convexMPC->pub_Pf[1][0] << "  fly " << convexMPC->pub_Pf[1][1]<< "  flz  " << convexMPC->pub_Pf[1][2]  << std:: endl;
      std::cout << "当前frx " << convexMPC->pub_Pf[0][0] << "  fry " << convexMPC->pub_Pf[0][1]<< "  frz  " << convexMPC->pub_Pf[0][2]  << std:: endl;
      std::cout << "当前rlx " << convexMPC->pub_Pf[3][0] << "  rly " << convexMPC->pub_Pf[3][1]<< "  rlz  " << convexMPC->pub_Pf[3][2]  << std:: endl;
      std::cout << "当前rrx " << convexMPC->pub_Pf[2][0] << "  rry " << convexMPC->pub_Pf[2][1]<< "  rrz  " << convexMPC->pub_Pf[2][2]  << std:: endl;
      // std::cout << "控制器flx " << convexMPC->planfootpoint[1][0] << "  fly " << convexMPC->planfootpoint[1][1]<< "  flz  " << convexMPC->planfootpoint[1][2]  << std:: endl;
      // std::cout << "控制器frx " << convexMPC->planfootpoint[0][0] << "  fry " << convexMPC->planfootpoint[0][1]<< "  frz  " << convexMPC->planfootpoint[0][2]  << std:: endl;
      // std::cout << "控制器rlx " << convexMPC->planfootpoint[3][0] << "  rly " << convexMPC->planfootpoint[3][1]<< "  rlz  " << convexMPC->planfootpoint[3][2]  << std:: endl;
      // std::cout << "控制器rrx " << convexMPC->planfootpoint[2][0] << "  rry " << convexMPC->planfootpoint[2][1]<< "  rrz  " << convexMPC->planfootpoint[2][2]  << std:: endl;
      std::cout << "当flx " << convexMPC->pubFoot[1][0] << "  fly " << convexMPC->pubFoot[1][1]<< "  flz  " << convexMPC->pubFoot[1][2]  << std:: endl;
      std::cout << "当frx " << convexMPC->pubFoot[0][0] << "  fry " << convexMPC->pubFoot[0][1]<< "  frz  " << convexMPC->pubFoot[0][2]  << std:: endl;
      std::cout << "当rlx " << convexMPC->pubFoot[3][0] << "  rly " << convexMPC->pubFoot[3][1]<< "  rlz  " << convexMPC->pubFoot[3][2]  << std:: endl;
      std::cout << "当rrx " << convexMPC->pubFoot[2][0] << "  rry " << convexMPC->pubFoot[2][1]<< "  rrz  " << convexMPC->pubFoot[2][2]  << std:: endl;

      swingstate = false;
 
      // if (iteriter <=22)
      // {
  
      // footplan[0] = plan_f_foot_pos[iteriter];
      // footplan[1] = plan_f_foot_pos[iteriter];
      // footplan[2] = plan_b_foot_pos[iteriter];
      // footplan[3] = plan_b_foot_pos[iteriter];
      
      // }
      

      // iteriter++;
      // std::cout << "控xxx " << " " << footplan[0] << "  fly " <<footplan[1]<< "  flz  " << footplan[2]  << std:: endl;
    }


    


      // // footplan[0] +=  0.10;
      // // footplan[3] +=  0.10;
     
     



 
    // }else if(foothold_request_msg.FL_status.data == 1 && foothold_request_msg.RR_status.data == 1)
    // {
    //   foothold_request_id++;
    //    foothold_request_msg.foothold_request_id.data = foothold_request_id;
    //   foothold_request_pub.publish(foothold_request_msg);
    //   // footplan[1] +=  0.10;
    //   // footplan[2] +=  0.10;
    //   std::cout << "222222222222 foothold_request_id:" <<foothold_request_id<< std::endl;
    //   std::cout << "当前flx " << convexMPC->pub_Pf[1][0] << "  fly " << convexMPC->pub_Pf[1][1]<< "  flz  " << convexMPC->pub_Pf[1][2]  << std:: endl;
    //   std::cout << "当前rrx " << convexMPC->pub_Pf[2][0] << "  rry " << convexMPC->pub_Pf[2][1]<< "  rrz  " << convexMPC->pub_Pf[2][2]  << std:: endl;
    //   std::cout << "规划flx " << footplan[1] << "  fly " << footplany[1]<< "  flz  " << footplanz[1]  << std:: endl;
    //   std::cout << "规划rrx " << footplan[2] << "  rry " << footplany[2]<< "  rrz  " << footplanz[3]  << std:: endl;

    // }

    // foothold_request_msg.foothold_request_id.data = foothold_request_id;
    // foothold_request_pub.publish(foothold_request_msg);
    // std::cout << "fr " << convexMPC->contact_state[0] << "  fl " << convexMPC->contact_state[1] << "  RR  " << convexMPC->contact_state[2]  << " RL " 
    //  << convexMPC->contact_state[3] << std:: endl;

    
    
}
    
    _legController->updateCommand(&legcommand, ctrlParam_sim);

    for(int i = 0; i < 4; i++){
      joint_effort.data[i * 3] = legcommand.tau_abad_ff[i];
      joint_effort.data[i * 3 + 1] = -legcommand.tau_hip_ff[i];
      joint_effort.data[i * 3 + 2] = -legcommand.tau_knee_ff[i];  
    }


    iter++;
    
    if(iter > 2){
      pub_effort.publish(joint_effort);  //第一次的值有问题，1和2腿的commandPes的值为0
      
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
