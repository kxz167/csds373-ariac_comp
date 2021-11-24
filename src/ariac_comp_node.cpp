#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/VacuumGripperControl.h"
#include "osrf_gear/SubmitShipment.h"
#include "osrf_gear/Model.h"
#include "geometry_msgs/Pose.h"
#include <vector>
#include <string>
#include <map>
#include <limits>
// #include "angles/angles.h"

//Lab 6
#include "ur_kinematics/ur_kin.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

//TRANSFORMS:
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

//ActionServers
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

//TO VIEW FRAMES:
// rosrun tf tf_echo /world <anything>

//CONSTANTS:
std::vector<std::string> JOINT_NAMES{
    "linear_arm_actuator_joint",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
};

//Global Variables
//Order tracking
ros::ServiceClient gc_global;
ros::ServiceClient submitter_global;

int order_count = 0;
int msg_count = 0;
std::vector<osrf_gear::Order> order_vector;

//Items
std::map<std::string, std::vector<osrf_gear::Model>> items_bin; //Holds products in bins
std::map<std::string, std::vector<osrf_gear::Model>> items_agv; //Holds products on the conveyerbelts
std::map<std::string, std::vector<osrf_gear::Model>> items_qcs; //Holds faulty projects (on trays)
bool populated = false;
//Joint states:
sensor_msgs::JointState joint_states;
std::map<std::string, geometry_msgs::Pose> agv_pose;

actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *trajectory_actionserver; 
control_msgs::FollowJointTrajectoryAction joint_trajectory_as;

//BIN LOCATIONS <ID, y location>
std::map<std::string, double> bin_pos = {
    {"bin4", .383}, //Or .383
    {"bin5", 1.15},
    {"bin6", 1.4},
    {"agv1", 2.2},
    {"agv2", -2.2}};

trajectory_msgs::JointTrajectory base_trajectory(bool default_pose){
    trajectory_msgs::JointTrajectory joint_trajectory;

    joint_trajectory.header.seq = msg_count++;
    joint_trajectory.header.stamp = ros::Time::now();
    joint_trajectory.header.frame_id = "/world";
    joint_trajectory.joint_names = JOINT_NAMES;

    if(default_pose){
        joint_trajectory.points.resize(1);
        joint_trajectory.points[0].positions.resize(7);
        joint_trajectory.points[0].positions[1] = 3.1415;
        joint_trajectory.points[0].positions[2] = -1.8;
        joint_trajectory.points[0].positions[3] = 2.85;
        joint_trajectory.points[0].positions[4] = 3.7;//0
        joint_trajectory.points[0].positions[5] = 4.7;//1.5
        joint_trajectory.points[0].positions[6] = 0;

        joint_trajectory.points[0].time_from_start = ros::Duration(3);
    }

    return joint_trajectory;
}


void patch_trajectory_point(trajectory_msgs::JointTrajectoryPoint &trajectory_point){
    //Prevents running into the bins
    if (trajectory_point.positions[2] > 3.14) {
        trajectory_point.positions[2] -= 6.28;
    }

    // Prevents excessive turning
    if (trajectory_point.positions[3] > 3.14) {
        trajectory_point.positions[3] -= 6.28;
    }

    //Prevents wrist clipping
    if (trajectory_point.positions[4] > 3.14) {
        // trajectory_point.positions[4] -= 6.28;
    }
}

void patch_trajectory(trajectory_msgs::JointTrajectory &trajectory){
    for (int i = 0; i < trajectory.points.size(); i++)
    {
        patch_trajectory_point(trajectory.points[i]);
    }
}
double actuator_position(geometry_msgs::Pose model_pose, std::string product_location){
    double limit = 2.5;
    double offset = 0.75;
    return (model_pose.position.x > 0) ? std::min(bin_pos[product_location] + offset, limit) : std::max(bin_pos[product_location] - offset, -1 * limit);
}

//Helper method to print out a Pose into a useful string.
void print_pose(const geometry_msgs::Pose pose)
{
    ROS_WARN("xyz = (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z);
    ROS_WARN("wxyz = (%f, %f, %f, %f)", pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

void print_trajectory_points(const trajectory_msgs::JointTrajectory traj)
{
    ROS_WARN("Size of: %s", std::to_string(traj.points.size()).c_str());
    for (trajectory_msgs::JointTrajectoryPoint point : traj.points)
    {

        ROS_WARN("-----------------");
        ROS_WARN("%f, %f, %f, %f, %f, %f, %f", point.positions[0], point.positions[1], point.positions[2], point.positions[3], point.positions[4], point.positions[5], point.positions[6]);
        ROS_WARN("-----------------");
    }
    ROS_WARN("Done printing");
}

void print_solutions(double possible_sol[8][6]) {
    for (int i = 0; i < 8; i++) {
        ROS_WARN("%f, %f, %f, %f, %f, %f", possible_sol[i][0], possible_sol[i][1], possible_sol[i][2], possible_sol[i][3], possible_sol[i][4], possible_sol[i][5]);
    }
}

// Store the order whenever it is received.
void order_callback(const osrf_gear::Order::ConstPtr &msg)
{
    ROS_INFO("Order %d Received.", ++order_count);
    order_vector.push_back(*msg);
}

// Camera callback to store the product (models) based on types.
void cameraCallback(
    const osrf_gear::LogicalCameraImage::ConstPtr &msg,
    std::map<std::string, std::vector<osrf_gear::Model>> *itemMap //Must use pointers to avoid segfault
)
{
    //ROS_INFO("Camera callback happened");
    if (msg->models.empty())
    {
        // Return if no parts below.
        return;
    }

    //Then get the type if there are any
    std::string type = msg->models.front().type;

    //For QC cameras, if no type, set type to faulty:
    if (type.empty())
    {
        type = "faulty";
    }

    //Create a copy of each of the models:
    std::vector<osrf_gear::Model> product_models;
    for (osrf_gear::Model product_model : msg->models)
    {
        product_models.push_back(osrf_gear::Model(product_model));
    }
    (*itemMap)[type] = product_models;
    populated = true;
}

void agv_callback(
    const osrf_gear::LogicalCameraImage::ConstPtr &msg,
    std::string agv_name //Must use pointers to avoid segfault
)
{
    agv_pose[agv_name] = msg->pose;
}

void armJointCallback(
    const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_states = sensor_msgs::JointState(*msg);
}

//Find the RMS distance of a single solution
double dist(double solution[6])
{
    double result = 0.0;
    result += pow(solution[0] - joint_states.position[3], 2);
    result += pow(solution[1] - joint_states.position[2], 2);
    result += pow(solution[2] - joint_states.position[0], 2);
    result += pow(solution[3] - joint_states.position[4], 2);
    result += pow(solution[4] - joint_states.position[5], 2);
    result += pow(solution[5] - joint_states.position[6], 2);
    result /= 6;
    return sqrt(result);
}

// Filter out certain angles depending on where it is.
int optimal_solution_index(double possible_sol[8][6])
{
    //shoulder pan
    // away     -> x < pi/2 || x > 3pi/2    Means that the shoulder must
    // towards  -> x > pi/2 && x < 3pi/2
    //shoulder lift     YOU ALWAYS WANT THIS TO BE greater than pi
    // away     -> x < 3pi/2       true if shoulder pan is away
    // towards  -> x > 3pi/2       true if shoulder pan is towards.
    //elbow joint
    // away     -> x > pi       true if shoulder pan is away
    // towards  -> x < pi       true if shoulder pan is towards
    //wrist1
    //wrist2
    //wrist3

    for (int i = 0; i < 8; i++)
    {
        ROS_WARN("%f, %f, %f, %f, %f, %f", possible_sol[i][0], possible_sol[i][1], possible_sol[i][2], possible_sol[i][3], possible_sol[i][4], possible_sol[i][5]);
    }

    double pi = 3.1415;

    bool op_sol[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 8; i++)
    {
        double shoulder_pan = possible_sol[i][0];
        double shoulder_lift = possible_sol[i][1];
        double elbow = possible_sol[i][2];
        double wrist1 = possible_sol[i][3];
        double wrist2 = possible_sol[i][4];
        double wrist3 = possible_sol[i][5];
        bool valid_s = false;

        //shoulder pan
        if (shoulder_pan < pi / 2 || shoulder_pan > 3 * pi / 2)
        {
            ROS_INFO("sp1");
            // away     -> x < pi/2 || x > 3pi/2    Means that the shoulder must
            //shoulder lift     YOU ALWAYS WANT THIS TO BE greater than pi
            if (shoulder_lift < 3 * pi / 2 && elbow > pi)
            {
                    ROS_INFO("sp2");
                    if (wrist2 > pi) {
                        // valid_s = true;
                    }
                    
            }
            // away     -> x < 3pi/2       true if shoulder pan is away
            //elbow joint
            // away     -> x > pi       true if shoulder pan is away
        }
        else
        {
            // ROS_INFO("sp3");
            // towards  -> x > pi/2 && x < 3pi/2
            //shoulder lift     YOU ALWAYS WANT THIS TO BE greater than pi
            // towards  -> x > 3pi/2       true if shoulder pan is towards.
            //elbow joint
            // towards  -> x < pi       true if shoulder pan is towards
            if (shoulder_lift > 3 * pi / 2 && elbow < pi)
            {
                ROS_INFO("sp4");
                    ROS_INFO("sp2");
                    if (wrist2 > pi) {
                        valid_s = true;
                    }
                    
            }
        }

        if (valid_s)
        {
            // ROS_INFO("%d", i);
            return i;
        }
    }

    return -1;
}

int agv_filter(double possible_sol[8][6])
{
    for (int i = 0; i < 8; i++)
    {
        ROS_WARN("%f, %f, %f, %f, %f, %f", possible_sol[i][0], possible_sol[i][1], possible_sol[i][2], possible_sol[i][3], possible_sol[i][4], possible_sol[i][5]);
    }

    double pi = 3.1415;

    for (int i = 0; i < 8; i++)
    {
        double shoulder_pan = possible_sol[i][0];
        double shoulder_lift = possible_sol[i][1];
        double elbow = possible_sol[i][2];
        double wrist1 = possible_sol[i][3];
        double wrist2 = possible_sol[i][4];
        double wrist3 = possible_sol[i][5];
        bool valid_s = false;
        if(wrist2 > pi){
            // ROS_WARN("%f", i);
            return i;
        }
    }

    ROS_WARN("FAILED");
    return 0;
}

// Choose the solution with the least difference in angles
int shortest_solution(double possible_sol[8][6]) {
    int min_dist_idx = -1;
    double min_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 8; i++)
    {
        double temp_dist = dist(possible_sol[i]);
        if (temp_dist < min_dist)
        {
            min_dist_idx = i;
            min_dist = temp_dist;
        }
    }
    return min_dist_idx;
}

trajectory_msgs::JointTrajectoryPoint ik_point(geometry_msgs::Pose desired_pose, double height, int duration){
    //LAB 6 Arm Movement:
    double T_des[4][4] = {0.0};
    double q_des[8][6] = {0.0};

    // What we are shooting for
    T_des[0][3] = desired_pose.position.x;
    T_des[1][3] = desired_pose.position.y;
    T_des[2][3] = desired_pose.position.z + height; //Place slightly above model
    T_des[3][3] = 1.0;

    //End effector ROTATION
    T_des[2][0] = -1.0;
    T_des[0][1] = -1.0;
    T_des[1][2] = 1.0;

    //InverseKinematics to find joint angles:
    int num_sols = ur_kinematics::inverse(&T_des[0][0], &q_des[0][0], 0.0);

    //Trajectory Command Message:
    int q_des_indx = optimal_solution_index(q_des);

    //Populate the point
    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions.resize(7);
    for (int indy = 0; indy < 6; indy++)
    {   
        trajectory_point.positions[indy + 1] = q_des[q_des_indx][indy];
        ROS_INFO("%f", q_des[q_des_indx][indy]);
    }
    ROS_INFO(" ");
    //Modify the elbow to prevent crashing into the big   
    patch_trajectory_point(trajectory_point);
    //Set speed
    trajectory_point.time_from_start = ros::Duration(duration);

    return trajectory_point;
}

trajectory_msgs::JointTrajectoryPoint ik_point_2(geometry_msgs::Pose desired_pose, double height, int duration){
    //LAB 6 Arm Movement:
    double T_des[4][4] = {0.0};
    double q_des[8][6] = {0.0};

    // What we are shooting for
    T_des[0][3] = desired_pose.position.x;
    T_des[1][3] = desired_pose.position.y;
    T_des[2][3] = desired_pose.position.z + height; //Place slightly above model
    T_des[3][3] = 1.0;

    //End effector ROTATION
    T_des[2][0] = -1.0;
    T_des[0][1] = -1.0;
    T_des[1][2] = 1.0;

    //InverseKinematics to find joint angles:
    int num_sols = ur_kinematics::inverse(&T_des[0][0], &q_des[0][0], 0.0);

    //Trajectory Command Message:
    // int q_des_indx = optimal_solution_index(q_des);
    int q_des_indx = agv_filter(q_des);

    //Populate the point
    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions.resize(7);
    for (int indy = 0; indy < 6; indy++)
    {   
        trajectory_point.positions[indy + 1] = q_des[q_des_indx][indy];
        ROS_INFO("%f", q_des[q_des_indx][indy]);
    }
    ROS_INFO(" ");
    //Modify the elbow to prevent crashing into the big   
    patch_trajectory_point(trajectory_point);
    //Set speed
    trajectory_point.time_from_start = ros::Duration(duration);

    return trajectory_point;
}

geometry_msgs::Pose pose_wrt_arm(geometry_msgs::Pose product_pose, std::string camera) {
    //LOOK FOR THE TRANSFORM
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);

    geometry_msgs::TransformStamped camera_to_base;
    camera_to_base = tf_buffer.lookupTransform("arm1_base_link", camera, ros::Time(0), ros::Duration(1.0));

    //Perform the transform into the correct frame:
    geometry_msgs::Pose new_pose;
    tf2::doTransform(product_pose, new_pose, camera_to_base); // robot_pose is the PoseStamped I want to transform

    return new_pose;
}

geometry_msgs::Pose pose_wrt_base(geometry_msgs::Pose product_pose, std::string camera) {
    //LOOK FOR THE TRANSFORM
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);

    geometry_msgs::TransformStamped camera_to_base;
    camera_to_base = tf_buffer.lookupTransform("arm1_base", camera, ros::Time(0), ros::Duration(1.0));

    //Perform the transform into the correct frame:
    geometry_msgs::Pose new_pose;
    tf2::doTransform(product_pose, new_pose, camera_to_base); // robot_pose is the PoseStamped I want to transform

    return new_pose;
}

void send_trajectory(trajectory_msgs::JointTrajectory traj, bool blocking) {
    
    joint_trajectory_as.action_goal.goal.trajectory = traj;

    actionlib::SimpleClientGoalState act_state =
        trajectory_actionserver->sendGoalAndWait(joint_trajectory_as.action_goal.goal,
                                        ros::Duration(30.0), ros::Duration(30.0));
    ROS_INFO("Action Server returned with status [%i] %s", act_state.state_, act_state.toString().c_str());
}

void set_gripper(bool status){
    osrf_gear::VacuumGripperControl srv;
    srv.request.enable = status;
    
    if(gc_global.call(srv)) {
        while(!srv.response.success) {
            gc_global.call(srv);
        }
        ROS_INFO("VACUUM %s", status ? "ENABLED" : "DISABLED");       
    }
    else {
        ROS_WARN("Could not reach the vacuum service.");
    }
}

void enable_gripper(){
    set_gripper(true);
}

void disable_gripper(){
    set_gripper(false);
}

void pickup_part(geometry_msgs::Pose desired_pose, double actuator){
    enable_gripper();
    //Position right at
    trajectory_msgs::JointTrajectory grip_joint_trajectory = base_trajectory(false);

    //Set the next point to be the optimal ik point
    grip_joint_trajectory.points.resize(2);
    ROS_INFO("From up");
    grip_joint_trajectory.points[0] = ik_point(desired_pose, 0.05, 1); // pause above product
    grip_joint_trajectory.points[0].positions[0] = actuator;

    ROS_INFO("Going down");
    grip_joint_trajectory.points[1] = ik_point(desired_pose, 0.015, 2); // pause above product
    grip_joint_trajectory.points[1].positions[0] = actuator;

    //TODO FIGURE OUT HOW TO GENERALIZE THIS, PUT AS into global;
    //position above.

    send_trajectory(grip_joint_trajectory, true);
    sleep(1);

    grip_joint_trajectory = base_trajectory(false);
    grip_joint_trajectory.points.resize(2);
    ROS_INFO("From down");
    grip_joint_trajectory.points[0] = ik_point(desired_pose, 0.02, 1); // pause at product
    grip_joint_trajectory.points[0].positions[0] = actuator;

    ROS_INFO("Going up");
    grip_joint_trajectory.points[1] = ik_point(desired_pose, 0.3, 2); // move up with product
    grip_joint_trajectory.points[1].positions[0] = actuator;

    send_trajectory(grip_joint_trajectory, true);

}

//Return all of the potential poses for the querried AGV camera
void place_part (std::string agv_name){
    trajectory_msgs::JointTrajectory act_joint_trajectory;

    //Head to the agv.
    act_joint_trajectory = base_trajectory(true);
    act_joint_trajectory.points[0].positions[0] = bin_pos[agv_name];
    act_joint_trajectory.points[0].positions[1] = 1.57;
    send_trajectory(act_joint_trajectory, true);
    
    // CONVERT FRAME TO LOGICAL CAMERA AGV1
    geometry_msgs::Pose agv_target_pose = pose_wrt_arm(agv_pose[agv_name], "world");
    ROS_WARN("Camera Position Relative to arm1:");
    // print_pose(new_pose);
    // ROS_WARN("%f", agv_pose[agv_name].position.x);

    //Reach out and place it right below the camera
    trajectory_msgs::JointTrajectory grip_joint_trajectory = base_trajectory(false);
    grip_joint_trajectory.points.resize(1);
    grip_joint_trajectory.points[0] = ik_point_2(agv_target_pose, -.4, 4); // pause above product
    grip_joint_trajectory.points[0].positions[0] = bin_pos[agv_name];
    // print_trajectory_points(grip_joint_trajectory);
    send_trajectory(grip_joint_trajectory, true);

    //Drop the part
    disable_gripper();

    grip_joint_trajectory = base_trajectory(true);
    grip_joint_trajectory.points.resize(1);
    grip_joint_trajectory.points[0].positions[0] = bin_pos[agv_name];
    send_trajectory(grip_joint_trajectory, true);
}

int main(int argc, char **argv)
{
    //Initialize ros:
    ros::init(argc, argv, "ariac_comp_node");

    /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle n;

    //Create trigger for the beginning of the competition:
    std_srvs::Trigger begin_comp;
    ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");


    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_as("/ariac/arm1/arm/follow_joint_trajectory", true); 
	trajectory_actionserver = &trajectory_as;

    ros::ServiceClient gripper_client = n.serviceClient<osrf_gear::VacuumGripperControl>("ariac/arm1/gripper/control");
    gc_global = gripper_client;

    ros::ServiceClient submit_client = n.serviceClient<osrf_gear::SubmitShipment>("ariac/submit_shipment");
    submitter_global = submit_client;

    //Subscribe to cameras over product bins:
    //ITEMS:
    ros::Subscriber camera_sub_b1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin1", 1000, boost::bind(cameraCallback, _1, &items_bin));
    ros::Subscriber camera_sub_b2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin2", 1000, boost::bind(cameraCallback, _1, &items_bin));
    ros::Subscriber camera_sub_b3 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin3", 1000, boost::bind(cameraCallback, _1, &items_bin));
    ros::Subscriber camera_sub_b4 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin4", 1000, boost::bind(cameraCallback, _1, &items_bin));
    ros::Subscriber camera_sub_b5 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin5", 1000, boost::bind(cameraCallback, _1, &items_bin));
    ros::Subscriber camera_sub_b6 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin6", 1000, boost::bind(cameraCallback, _1, &items_bin));
    //AGV:
    ros::Subscriber camera_sub_a1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv1", 1000, boost::bind(agv_callback, _1, "agv1"));
    ros::Subscriber camera_sub_a2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv2", 1000, boost::bind(agv_callback, _1, "agv2"));
    //TRAYS:
    ros::Subscriber camera_sub_q1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_1", 1000, boost::bind(cameraCallback, _1, &items_qcs));
    ros::Subscriber camera_sub_q2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_2", 1000, boost::bind(cameraCallback, _1, &items_qcs));

    //Subscribe to incoming orders:
    ros::Subscriber orderSub = n.subscribe("/ariac/orders", 1000, order_callback);

    //Create the material location service for determining location of products
    ros::ServiceClient material_location_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

    //Subscribe to joint states:
    ros::Subscriber joint_states_h = n.subscribe("/ariac/arm1/joint_states", 10, armJointCallback);

    //Publish to a topic:
    ros::Publisher trajectory_mover = n.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 10);
    //Begin the competition
    int service_call_succeeded;
    service_call_succeeded = begin_client.call(begin_comp);
    if (!service_call_succeeded)
    {
        //Something went wrong calling the service itself
        ROS_ERROR("Competition service call failed! LOSER!");
        ros::shutdown();
    }
    else
    {
        if (!begin_comp.response.success)
        {
            //Service was called, but could not be started
            ROS_WARN("Start Failure: %s", begin_comp.response.message.c_str());
        }
        else
        {
            //Service was called and competition started successfully
            ROS_INFO("Start Success: %s", begin_comp.response.message.c_str());
        }
    }

    //Clear out the orders in case of previous artifacting.
    order_vector.clear();

    //Define checks
    ros::Rate loop_rate(10);
    // int msg_count = 0;

    while (ros::ok())
    {
        //Look and see if there are any waiting:
        if (!order_vector.empty() && populated)
        {
            ROS_INFO("------------------------");
            ROS_INFO("Processing Order Begin: ");

            /**
             * FIRST CODE TRY, loops through each order, and each subsequent product, locations, etc. Kept for future assignments
             */
            // Loop through each shipment
            for(osrf_gear::Shipment shipment : order_vector.front().shipments){ //Loop through all shipments
                //
                
                //And each product within
                for(osrf_gear::Product product : shipment.products){
                    std::string product_type = product.type;
                    ROS_INFO("One product of type: %s required for shipment...", product_type.c_str());
                    osrf_gear::GetMaterialLocations srv;
                    srv.request.material_type = product_type;
                    // int call_succeeded = material_location_client.call(srv);
                    // ROS_INFO("%d", call_succeeded);
                    if(material_location_client.call(srv)){
                        //GET THE FIRST NON BELT PRODUCT LOCATION:
                        std::string product_location;
                        ROS_INFO("Located In   : %s", product_location.c_str());
                        for(osrf_gear::StorageUnit unit : srv.response.storage_units){
                            if(unit.unit_id != "belt"){ // WE ARE NOT REPSONSIBLE FOR THE BELT, get first not belt
                                product_location = unit.unit_id;
                                break;
                            }
                        }
                        ROS_INFO("Product Location: %s", product_location.c_str());

                        //GET ALL ITEMS IN CORRESPONDING BIN: 
                        std::vector<osrf_gear::Model> bin_all_items = items_bin[product_type];

                        disable_gripper();

                        //Find the first product available.
                        osrf_gear::Model curr_model = bin_all_items.front();
                        ROS_WARN("Product Position Relative to camera:");
                        print_pose(curr_model.pose);

                        // STOWED POSITION
                        trajectory_msgs::JointTrajectory act_joint_trajectory = base_trajectory(true);
                        act_joint_trajectory.points[0].positions[0] = actuator_position(curr_model.pose, product_location);
                        send_trajectory(act_joint_trajectory, true);

                        // Convert reference frame
                        geometry_msgs::Pose new_pose = pose_wrt_arm(curr_model.pose, "logical_camera_bin4_frame");
                        ROS_WARN("Product Position Relative to arm1:");
                        print_pose(new_pose);

                        trajectory_msgs::JointTrajectory joint_trajectory = base_trajectory(false);

                        //Set the next point to be the optimal ik point
                        joint_trajectory.points.resize(1);
                        joint_trajectory.points[0] = ik_point(new_pose, 0.05, 3); // pause above product
                        joint_trajectory.points[0].positions[0] = actuator_position(curr_model.pose, product_location);

                        //TODO FIGURE OUT HOW TO GENERALIZE THIS, PUT AS into global;
                        send_trajectory(joint_trajectory, true);
                        
                        pickup_part(new_pose, actuator_position(curr_model.pose, product_location));
                        
                        // GO TO STOWED POSITION
                        act_joint_trajectory = base_trajectory(true);
                        act_joint_trajectory.points[0].positions[0] = actuator_position(curr_model.pose, product_location);
                        send_trajectory(act_joint_trajectory, true);

                        // GO TO AGV1
                        // act_joint_trajectory = base_trajectory(true);
                        // act_joint_trajectory.points[0].positions[0] = actuator_position(curr_model.pose, "agv1");
                        // act_joint_trajectory.points[0].positions[1] = 1.57;
                        // send_trajectory(act_joint_trajectory, true);
                        
                        // // CONVERT FRAME TO LOGICAL CAMERA AGV1
                        // geometry_msgs::Pose agv_target_pose = pose_wrt_arm(agv_pose["agv1"], "world");
                        // ROS_WARN("Camera Position Relative to arm1:");
                        // print_pose(new_pose);
                        // // ROS_WARN("%f", agv_pose["agv1"].position.x);

                        // trajectory_msgs::JointTrajectory grip_joint_trajectory = base_trajectory(false);
                        // //Set the next point to be the optimal ik point
                        // grip_joint_trajectory.points.resize(1);
                        // grip_joint_trajectory.points[0] = ik_point_2(agv_target_pose, -.4, 10); // pause above product
                        // grip_joint_trajectory.points[0].positions[0] = actuator_position(curr_model.pose, "agv1");
                        // print_trajectory_points(grip_joint_trajectory);
                        // send_trajectory(grip_joint_trajectory, true);
                        // disable_gripper();

                        place_part("agv1");
                    }
                }
            }

            ROS_INFO("OUT OF LOOP");
            sleep(10);

            std::string shipment_name = order_vector.front().shipments.front().shipment_type; 
            std::string destination = order_vector.front().shipments.front().agv_id;
            
            
            //Get the first product (first order's first shipment's first product)
            osrf_gear::Product first_product = order_vector.front().shipments.front().products.front();



            //Get the type
            std::string product_type = first_product.type;
            ROS_INFO("Product Type : %s", product_type.c_str());

            //Create a request to the material location service:
            osrf_gear::GetMaterialLocations srv;
            srv.request.material_type = product_type;

            //Call the request
            if (material_location_client.call(srv)) {
                //Were able to find a product location:
                std::string product_location = srv.response.storage_units.front().unit_id;
                ROS_INFO("Located In   : %s", product_location.c_str());

                // if (items_bin[product_type.c_str()].empty()) {
                //     ROS_WARN("Camera can not confirm %s is in %s", product_type.c_str(), product_location.c_str());
                //     break;
                // }

                std::vector<osrf_gear::Model> bin_all_items = items_bin[product_type.c_str()];
                // for(int i = 0; i < bin_all_items.size(); ++i) {
                int i = 0;
                disable_gripper();
                osrf_gear::Model curr_model = bin_all_items.at(i);
                ROS_WARN("Product Position Relative to camera:");
                print_pose(curr_model.pose);

                // STOWED POSITION
                trajectory_msgs::JointTrajectory act_joint_trajectory = base_trajectory(true);
                act_joint_trajectory.points[0].positions[0] = actuator_position(curr_model.pose, product_location);
                send_trajectory(act_joint_trajectory, true);

                // Convert reference frame
                geometry_msgs::Pose new_pose = pose_wrt_arm(curr_model.pose, "logical_camera_bin4_frame");
                ROS_WARN("Product Position Relative to arm1:");
                print_pose(new_pose);

                trajectory_msgs::JointTrajectory joint_trajectory = base_trajectory(false);

                //Set the next point to be the optimal ik point
                joint_trajectory.points.resize(1);
                joint_trajectory.points[0] = ik_point(new_pose, 0.05, 3); // pause above product
                joint_trajectory.points[0].positions[0] = actuator_position(curr_model.pose, product_location);

                //TODO FIGURE OUT HOW TO GENERALIZE THIS, PUT AS into global;
                send_trajectory(joint_trajectory, true);
                
                pickup_part(new_pose, actuator_position(curr_model.pose, product_location));
                
                // GO TO STOWED POSITION
                act_joint_trajectory = base_trajectory(true);
                act_joint_trajectory.points[0].positions[0] = actuator_position(curr_model.pose, product_location);
                send_trajectory(act_joint_trajectory, true);

                // GO TO AGV1
                // act_joint_trajectory = base_trajectory(true);
                // act_joint_trajectory.points[0].positions[0] = actuator_position(curr_model.pose, "agv1");
                // act_joint_trajectory.points[0].positions[1] = 1.57;
                // send_trajectory(act_joint_trajectory, true);
                
                // // CONVERT FRAME TO LOGICAL CAMERA AGV1
                // geometry_msgs::Pose agv_target_pose = pose_wrt_arm(agv_pose["agv1"], "world");
                // ROS_WARN("Camera Position Relative to arm1:");
                // print_pose(new_pose);
                // // ROS_WARN("%f", agv_pose["agv1"].position.x);

                // trajectory_msgs::JointTrajectory grip_joint_trajectory = base_trajectory(false);
                // //Set the next point to be the optimal ik point
                // grip_joint_trajectory.points.resize(1);
                // grip_joint_trajectory.points[0] = ik_point_2(agv_target_pose, -.4, 10); // pause above product
                // grip_joint_trajectory.points[0].positions[0] = actuator_position(curr_model.pose, "agv1");
                // print_trajectory_points(grip_joint_trajectory);
                // send_trajectory(grip_joint_trajectory, true);
                // disable_gripper();

                place_part("agv1");

                //

                // EXTEND IN FRONT OF CAMERA FACING DOWN

                // DROP PART

                // enable_gripper();
                // //Position right at
                // trajectory_msgs::JointTrajectory grip_joint_trajectory = base_trajectory(false);

                // //Set the next point to be the optimal ik point
                // grip_joint_trajectory.points.resize(2);
                // grip_joint_trajectory.points[0] = ik_point(new_pose, 0.02, 3); // pause above product
                // grip_joint_trajectory.points[0].positions[0] = actuator_position(curr_model.pose, product_location);
                // grip_joint_trajectory.points[1] = ik_point(new_pose, 0.05, 7); // pause above product
                // grip_joint_trajectory.points[1].positions[0] = actuator_position(curr_model.pose, product_location);

                // //TODO FIGURE OUT HOW TO GENERALIZE THIS, PUT AS into global;
                // send_trajectory(grip_joint_trajectory, true);
                // //position above.

                // // send_trajectory(joint_trajectory, true);
                
                // disable_gripper();


                // DROP PART

                // SUBMIT AGV TRAY
                osrf_gear::SubmitShipment submit_srv;
                submit_srv.request.destination_id = "1";
                submit_srv.request.shipment_type = shipment_name;
                if (submit_client.call(submit_srv)) {
                    while (!submit_srv.response.success) {
                        // ROS_INFO("Retrying submission");
                        submit_client.call(submit_srv);
                    }
                    ROS_INFO("Submitted shipment %s", shipment_name.c_str());
                    ROS_INFO("Inspection Result %f", submit_srv.response.inspection_result);
                }
                else {
                    ROS_WARN("Destination not specified");
                }
                // }
            }
            else
            {
                //Call failed
                ROS_WARN("Could not reach the material service.");
            }
            
            //Clear out the order.
            order_vector.erase(order_vector.begin());
            ROS_INFO("Processing Order End:   ");
            ROS_INFO("------------------------");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
