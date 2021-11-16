#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Model.h"
#include "geometry_msgs/Pose.h"
#include <vector>
#include <string>
#include <map>
#include <limits>

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

//CONSTANTS:

//Global Variables
//Order tracking
int order_count = 0;
std::vector<osrf_gear::Order> order_vector;

//Items
std::map<std::string, std::vector<osrf_gear::Model>> items_bin; //Holds products in bins
std::map<std::string, std::vector<osrf_gear::Model>> items_agv; //Holds products on the conveyerbelts
std::map<std::string, std::vector<osrf_gear::Model>> items_qcs; //Holds faulty projects (on trays)
bool populated = false;
//Joint states:
sensor_msgs::JointState joint_states;

//BIN LOCATIONS <ID, y location>
std::map<std::string, double> bin_pos = {
    {"bin4", .383}, //Or .383
    {"bin5", 1.15},
    {"bin6", 1.916}};

//Helper method to print out a Pose into a useful string.
void printPose(const geometry_msgs::Pose pose)
{
    ROS_WARN("xyz = (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z);
    ROS_WARN("wxyz = (%f, %f, %f, %f)", pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

void negatify_trajectory_point(trajectory_msgs::JointTrajectory &traj)
{
    // ROS_WARN("Size of: %s", std::to_string(traj.points.size()).c_str());
    for (int i = 0; i < 10; i++)
    {
        // for(int j = 2; j < 6; j++){
        //
        // }

        //Modify:
        if (traj.points[i].positions[2] > 3.14)
        {
            traj.points[i].positions[2] -= 6.28;
        }
        // ROS_WARN("-----------------");
        ROS_WARN("%f, %f, %f, %f, %f, %f, %f", traj.points[i].positions[0], traj.points[i].positions[1], traj.points[i].positions[2], traj.points[i].positions[3], traj.points[i].positions[4], traj.points[i].positions[5], traj.points[i].positions[6]);
        // ROS_WARN("-----------------");
    }
    // ROS_WARN("Done printing");
}

void print_trajectory_point(const trajectory_msgs::JointTrajectory traj)
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

// Store the order whenever it is received.
void orderCallback(const osrf_gear::Order::ConstPtr &msg)
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

void armJointCallback(
    const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_states = sensor_msgs::JointState(*msg);
}

//Find the distance of a single solution
double dist(double solution[6])
{
    double result = 0.0;
    result += pow(solution[0] - joint_states.position[3], 2);
    result += pow(solution[1] - joint_states.position[2], 2);
    result += pow(solution[2] - joint_states.position[0], 2);
    result += pow(solution[3] - joint_states.position[4], 2);
    result += pow(solution[4] - joint_states.position[5], 2);
    result += pow(solution[5] - joint_states.position[6], 2);
    return sqrt(result);
}

// Filter out certain angles depending on where it is.
int sol_filter(double possible_sol[8][6])
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

    int op_sol = 0;
    for (int i = 0; i < 8; i++)
    {
        double shoulder_pan = possible_sol[i][0];
        double shoulder_lift = possible_sol[i][1];
        double elbow = possible_sol[i][2];

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

                valid_s = true;
            }
            // away     -> x < 3pi/2       true if shoulder pan is away
            //elbow joint
            // away     -> x > pi       true if shoulder pan is away
        }
        else
        {
            ROS_INFO("sp3");
            // towards  -> x > pi/2 && x < 3pi/2
            //shoulder lift     YOU ALWAYS WANT THIS TO BE greater than pi
            // towards  -> x > 3pi/2       true if shoulder pan is towards.
            //elbow joint
            // towards  -> x < pi       true if shoulder pan is towards
            if (shoulder_lift > 3 * pi / 2 && elbow < pi)
            {
                ROS_INFO("sp4");

                valid_s = true;
            }
        }

        if (valid_s)
        {
            ROS_INFO("%d", i);
            return i;
        }
    }

    return 0;
}

// Find the shortest distance
int optimal_solution(double possible_sol[8][6], int num_solutions)
{
    int min_dist_idx = -1;
    double min_dist = std::numeric_limits<double>::infinity();

    for (int i = 0; i < 8; i++)
    {
        ROS_WARN("%f, %f, %f, %f, %f, %f", possible_sol[i][0], possible_sol[i][1], possible_sol[i][2], possible_sol[i][3], possible_sol[i][4], possible_sol[i][5]);
    }

    // rosmsg info sensor_msgs::JointState
    // rostopic echo -n 1 /ariac/arm1/joint_states

    for (int i = 0; i < num_solutions; i++)
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

// DEPRECATED
int valid_solution(double possible_sol[8][6])
{

    for (int i = 0; i < 8; i++)
    {
        ROS_WARN("%f, %f, %f, %f, %f, %f", possible_sol[i][0], possible_sol[i][1], possible_sol[i][2], possible_sol[i][3], possible_sol[i][4], possible_sol[i][5]);
    }
    int op_sol = 0;
    //CAUSES SEGFAULT:
    for (int i = 0; i < 8; i++)
    {
        // ROS_WARN("%f, %f, %f, %f, %f, %f", possible_sol[i][0], possible_sol[i][1], possible_sol[i][2], possible_sol[i][3], possible_sol[i][4], possible_sol[i][5]);

        if (possible_sol[i][1] > 3.14159 || possible_sol[i][2] > 3.14)
        {
            op_sol++;
        }
        else
        {
            return 0;
        }
    }
    // possible_sol[i]

    // for(auto& solution : possible_sol){
    //     if( solution[0] < -1.57 || 1.57 < solution[0]){
    //         op_sol++;
    //     }
    //     else{
    //         break;
    //     }

    // }

    return 0;
}

//LINEAR MOVEMENT:
void linear_move(double target_x, double target_y, double target_z, int npts, trajectory_msgs::JointTrajectory &traj)
{
    traj.points.resize(npts);
    // double target_x = T_goal[0][3];
    // double target_y = T_goal[1][3];
    // double target_z = T_goal[2][3];
    double curr_pose[4][4];
    double angles[6] = {joint_states.position[3], joint_states.position[2], joint_states.position[0],
                        joint_states.position[4], joint_states.position[5], joint_states.position[6]};
    ur_kinematics::forward(&angles[0], &curr_pose[0][0]);
    double curr_loc_x = curr_pose[0][3];
    double curr_loc_y = curr_pose[1][3];
    double curr_loc_z = curr_pose[2][3];
    double ray[3]{target_x - curr_loc_x,
                  target_y - curr_loc_y,
                  target_z - curr_loc_z};
    double tr[3] = {ray[0] / npts, ray[1] / npts, ray[2] / npts};
    trajectory_msgs::JointTrajectoryPoint linear_traj[npts];
    for (int i = 0; i < npts; i++)
    {
        linear_traj[i].positions.resize(7);
        traj.points[i].positions.resize(7);
        double intermediate_goal[3] = {curr_loc_x + tr[0] * (i + 1),
                                       curr_loc_y + tr[1] * (i + 1),
                                       curr_loc_z + tr[2] * (i + 1)};
        double q_inter[8][6];
        double T_inter[4][4];

        T_inter[0][3] = intermediate_goal[0];
        T_inter[1][3] = intermediate_goal[1];
        T_inter[2][3] = intermediate_goal[2];
        T_inter[3][3] = 1.0;
        T_inter[2][0] = -1.0;
        T_inter[0][1] = -1.0;
        T_inter[1][2] = 1.0;
        ur_kinematics::inverse(&T_inter[0][0], &q_inter[0][0], 0.0);
        int sol = sol_filter(q_inter);
        traj.points[i].positions[0] = joint_states.position[1];
        for (int ii = 0; ii < 6; ii++) //ii=0 -> 5, i = 0 -> 9
        {
            // ROS_WARN("FAIL");
            // traj.points[i].positions[ii + 1] = 1;
            traj.points[i].positions[ii + 1] = q_inter[sol][ii];
            // ROS_WARN("FAIL");
        }
        traj.points[i].time_from_start = ros::Duration(2*i + 1);
    }

    // traj.points = linear_traj;
    // return linear_traj;
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

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
        trajectory_as("/ariac/arm1/arm/follow_joint_trajectory", true);
    control_msgs::FollowJointTrajectoryAction joint_trajectory_as;

    //Subscribe to cameras over product bins:
    //ITEMS:
    ros::Subscriber camera_sub_b1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin1", 1000, boost::bind(cameraCallback, _1, &items_bin));
    ros::Subscriber camera_sub_b2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin2", 1000, boost::bind(cameraCallback, _1, &items_bin));
    ros::Subscriber camera_sub_b3 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin3", 1000, boost::bind(cameraCallback, _1, &items_bin));
    ros::Subscriber camera_sub_b4 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin4", 1000, boost::bind(cameraCallback, _1, &items_bin));
    ros::Subscriber camera_sub_b5 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin5", 1000, boost::bind(cameraCallback, _1, &items_bin));
    ros::Subscriber camera_sub_b6 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin6", 1000, boost::bind(cameraCallback, _1, &items_bin));
    //AGV:
    ros::Subscriber camera_sub_a1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv1", 1000, boost::bind(cameraCallback, _1, &items_agv));
    ros::Subscriber camera_sub_a2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv2", 1000, boost::bind(cameraCallback, _1, &items_agv));
    //TRAYS:
    ros::Subscriber camera_sub_q1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_1", 1000, boost::bind(cameraCallback, _1, &items_qcs));
    ros::Subscriber camera_sub_q2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_2", 1000, boost::bind(cameraCallback, _1, &items_qcs));

    //Subscribe to incoming orders:
    ros::Subscriber orderSub = n.subscribe("/ariac/orders", 1000, orderCallback);

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

    int msg_count = 0;

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
            //Loop through each shipment
            // for(osrf_gear::Shipment shipment : order_vector.front().shipments){
            //     //And each product within
            //     for(osrf_gear::Product product : shipment.products){
            //         std::string product_type = product.type.c_str();
            //         ROS_INFO("%s", product_type);
            //         // std_srvs::Trigger mat_loc;
            //         osrf_gear::GetMaterialLocations srv;
            //         srv.request.material_type = product_type;
            //         // int call_succeeded = material_location_client.call(srv);
            //         // ROS_INFO("%d", call_succeeded);
            //         if(material_location_client.call(srv)){
            //             for(osrf_gear::StorageUnit unit : srv.response.storage_units){
            //                 ROS_INFO("%s", unit.unit_id.c_str());
            //             }
            //         }
            //     }
            // }

            //Get the first product (first order's first shipment's first product)
            osrf_gear::Product first_product = order_vector.front().shipments.front().products.front();

            //Get the type
            std::string product_type = first_product.type;
            ROS_INFO("Product Type : %s", product_type.c_str());

            //Create a request to the material location service:
            osrf_gear::GetMaterialLocations srv;
            srv.request.material_type = product_type;

            //Call the request
            if (material_location_client.call(srv))
            {
                //Were able to find a product location:
                std::string product_location = srv.response.storage_units.front().unit_id;
                ROS_INFO("Located In   : %s", product_location.c_str());

                if (items_bin.count(product_type.c_str()) == 0)
                {
                    ROS_WARN("Camera does not confirm %s in %s", product_type.c_str(), product_location.c_str());
                    break;
                }
                osrf_gear::Model first_model = items_bin[product_type.c_str()].front();
                // geometry_msgs::Pose first_pose = items_bin[product_type.c_str()][7];
                // geometry_msgs::PoseStamped first_pose = items_bin[product_type.c_str()].front();
                //Output the pose
                ROS_WARN("Product Position:");
                printPose(first_model.pose);
                // printPose(first_pose);

                // STOWED POSITION

                // //MOVE TO LOCATION
                // //Trajectory Command Message:
                trajectory_msgs::JointTrajectory act_joint_trajectory;
                ROS_INFO("SERVER");

                //     //Bootsrap
                act_joint_trajectory.header.seq = msg_count++;
                act_joint_trajectory.header.stamp = ros::Time::now();
                act_joint_trajectory.header.frame_id = "/world"; //TODO World correct?
                ROS_INFO("SERVER");

                // act_joint_trajectory.joint_names.clear();
                act_joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
                act_joint_trajectory.joint_names.push_back("shoulder_pan_joint");
                act_joint_trajectory.joint_names.push_back("shoulder_lift_joint");
                act_joint_trajectory.joint_names.push_back("elbow_joint");
                act_joint_trajectory.joint_names.push_back("wrist_1_joint");
                act_joint_trajectory.joint_names.push_back("wrist_2_joint");
                act_joint_trajectory.joint_names.push_back("wrist_3_joint");

                // ROS_INFO();

                // act_joint_trajectory.points.clear();
                act_joint_trajectory.points.resize(1);

                // // DON'T MOVE ARM:
                //act_joint_trajectory.points[0].positions.resize(act_joint_trajectory.joint_names.size());
                //for (int indy =0; indy < act_joint_trajectory.joint_names.size(); indy++){
                //    for(int indz = 0; indz < joint_states.name.size(); indz++){
                //        if(act_joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
                //           act_joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
                //            break;
                //        }
                //   }
                //}

                trajectory_msgs::JointTrajectoryPoint stowed_point;

                // new_point.positions[0] = 1;
                stowed_point.positions.resize(7);
                // stowed_point.positions[0] = bin_pos[product_location] + .6;
                double limit = 2.5;
                double offset = 0.75;
                stowed_point.positions[0] = (first_model.pose.position.x > 0) ? std::min(bin_pos[product_location] + offset, limit) : std::max(bin_pos[product_location] - offset, -1 * limit);

                stowed_point.positions[1] = 3.1415;
                stowed_point.positions[2] = -1.8;
                stowed_point.positions[3] = 2.2;
                stowed_point.positions[4] = 0;
                stowed_point.positions[5] = 1.5;
                stowed_point.positions[6] = 0;

                // stowed_point.positions[1] = 3.14;
                // stowed_point.positions[2] = -1.5;
                // stowed_point.positions[3] = 3.14;
                // stowed_point.positions[1] = 0;
                // stowed_point.positions[2] = 0;
                // stowed_point.positions[3] = 0;
                // stowed_point.positions[4] = 0;
                // stowed_point.positions[5] = 0;
                // stowed_point.positions[6] = 0;

                stowed_point.time_from_start = ros::Duration(5.0);

                act_joint_trajectory.points[0] = stowed_point;

                joint_trajectory_as.action_goal.goal.trajectory = act_joint_trajectory;

                actionlib::SimpleClientGoalState act_state =
                    trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal,
                                                  ros::Duration(30.0), ros::Duration(30.0));
                ROS_INFO("Action Server returned with status [%i] %s", act_state.state_, act_state.toString().c_str());

                sleep(2);

                ROS_INFO("SERVER");

                //SECOND LOCATION

                //LOOK FOR THE TRANSFORM
                tf2_ros::Buffer tf_buffer;
                tf2_ros::TransformListener tf2_listener(tf_buffer);
                // tf2_ros::TransformListener tf2_listener(tf_buffer2);

                geometry_msgs::TransformStamped camera_to_base;
                camera_to_base = tf_buffer.lookupTransform("arm1_base_link", "logical_camera_bin4_frame", ros::Time(0), ros::Duration(1.0));

                //Perform the transpose:
                geometry_msgs::Pose new_pose;
                geometry_msgs::Pose model_pose = first_model.pose;
                tf2::doTransform(model_pose, new_pose, camera_to_base); // robot_pose is the PoseStamped I want to transform
                // printPose(tempPose);

                // TRANSFORM TO WORLD COORDINATES FOR DEBUG
                // geometry_msgs::TransformStamped camera_to_world;
                // geometry_msgs::Pose tempPose2;
                // camera_to_world = tf_buffer.lookupTransform("world", "logical_camera_bin4_frame", ros::Time(0), ros::Duration(1.0) );
                // tf2::doTransform(m_pose, tempPose2, camera_to_world); // robot_pose is the PoseStamped I want to transform
                // printPose(tempPose2);

                //LAB 6 Arm Movement:
                double T_des[4][4] = {0.0};
                double q_des[8][6] = {0.0};

                //Where is our product?
                // WHAT WE ARE SHOOTING FOR
                T_des[0][3] = new_pose.position.x;
                T_des[1][3] = new_pose.position.y;
                T_des[2][3] = new_pose.position.z + .15; //Place slightly above model
                T_des[3][3] = 1.0;
                //End effector ROTATION
                T_des[2][0] = -1.0;
                T_des[0][1] = -1.0;
                T_des[1][2] = 1.0;

                //InverseKinematics to find joint angles:
                int num_sols;
                num_sols = ur_kinematics::inverse(&T_des[0][0], &q_des[0][0], 0.0);
                // ROS_INFO("%d", num_sols);

                //Trajectory Command Message:
                trajectory_msgs::JointTrajectory joint_trajectory;

                //Bootsrap
                joint_trajectory.header.seq = msg_count++;
                joint_trajectory.header.stamp = ros::Time::now();
                joint_trajectory.header.frame_id = "/world"; //TODO World correct?

                joint_trajectory.joint_names.clear();
                joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
                joint_trajectory.joint_names.push_back("shoulder_pan_joint");
                joint_trajectory.joint_names.push_back("shoulder_lift_joint");
                joint_trajectory.joint_names.push_back("elbow_joint");
                joint_trajectory.joint_names.push_back("wrist_1_joint");
                joint_trajectory.joint_names.push_back("wrist_2_joint");
                joint_trajectory.joint_names.push_back("wrist_3_joint");

                // joint_trajectory.points.clear();

                // int n = 10; // percent change per movement
                // joint_trajectory.points.resize(n);

                // POINT 1: NOT NECESSARY:
                // joint_trajectory.points[0].time_from_start = ros::Duration(0.0);

                // // populate startpooint, point 0, with current position given by joint_states
                // joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
                // for (int indy =0; indy < joint_trajectory.joint_names.size(); indy++){
                //     for(int indz = 0; indz < joint_states.name.size(); indz++){
                //         if(joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
                //             joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
                //             break;
                //         }
                //     }
                // }

                // time to start moving from startpoint
                // we want to move immediately so 0 seconds

                // SET WAYPOINT TARGET
                // int q_des_indx = optimal_solution(q_des, num_sols);
                // int q_des_indx = optimal_solution(q_des, num_sols);
                int q_des_indx = sol_filter(q_des);
                // ROS_WARN("%d", q_des_indx);
                // joint_trajectory.points[0] = stowed_point;

                //Set waypoint size:
                joint_trajectory.points.resize(1);
                joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
                // joint_trajectory.points[0].positions[0] = joint_states.position[1]; //Where the linear actuator is now, replace with bin location
                ROS_WARN("GOOD POSE");
                ROS_WARN("%f, %f, %f, %f, %f, %f, %f", joint_states.position[1], q_des[q_des_indx][0], q_des[q_des_indx][1], q_des[q_des_indx][2], q_des[q_des_indx][3], q_des[q_des_indx][4], q_des[q_des_indx][5]);
                joint_trajectory.points[0].positions[0] = (first_model.pose.position.x > 0) ? std::min(bin_pos[product_location] + offset, limit) : std::max(bin_pos[product_location] - offset, -1 * limit);
                for (int indy = 0; indy < 6; indy++)
                {   
                    joint_trajectory.points[0].positions[indy + 1] = q_des[q_des_indx][indy];
                }

                if (joint_trajectory.points[0].positions[2] > 3.14)
                {
                    joint_trajectory.points[0].positions[2] -= 6.28;
                }                
                joint_trajectory.points[0].time_from_start = ros::Duration(5);



                const double pi = 3.14159; // 180 deg
                const double pi2 = pi / 2; // 90 deg
                const double pi4 = pi / 4; // 45 deg
                // joint_trajectory.points[0].positions[2] = joint_trajectory.points[0].positions[2] - 6.2; // linear arm actuator, position on its own belt

                // ROS_WARN("--------");
                // ROS_WARN("%f", joint_states.position[3]);
                // ROS_WARN("%f", joint_states.position[2]);
                // ROS_WARN("%f", joint_states.position[0]);
                // ROS_WARN("%f", joint_states.position[4]);
                // ROS_WARN("%f", joint_states.position[5]);
                // ROS_WARN("%f", joint_states.position[6]);

                //joint_trajectory.points[0].positions[1] = 0; // shoulder pan swivel base, positive CCW
                // for (int i = 1; i <= n; ++i)
                // {
                //     joint_trajectory.points[i - 1].positions.resize(7);
                //     joint_trajectory.points[i - 1].positions[0] = joint_states.position[1];
                // for (int indy = 0; indy < 6; indy++){
                //     double incr = (q_des[q_des_indx][indy] - joint_states.position[indy])/n;

                //     joint_trajectory.points[i-1].positions[indy+1] = joint_states.position[indy] + i * incr;
                // }

                // double incr = (q_des[q_des_indx][indy] - joint_states.position[indy])/n;
                // ROS_WARN("--------");
                // ROS_WARN("%f", joint_states.position[3] + i * ((q_des[q_des_indx][0] - joint_states.position[3]) / n));
                // ROS_WARN("%f", joint_states.position[2] + i * ((q_des[q_des_indx][1] - joint_states.position[2]) / n));
                // ROS_WARN("%f", joint_states.position[0] + i * ((q_des[q_des_indx][2] - joint_states.position[0]) / n));
                // ROS_WARN("%f", joint_states.position[4] + i * ((q_des[q_des_indx][3] - joint_states.position[4]) / n));
                // ROS_WARN("%f", joint_states.position[5] + i * ((q_des[q_des_indx][4] - joint_states.position[5]) / n));
                // ROS_WARN("%f", joint_states.position[6] + i * ((q_des[q_des_indx][5] - joint_states.position[6]) / n));
                // joint_trajectory.points[i - 1].positions[1] = joint_states.position[3] + i * ((q_des[q_des_indx][0] - joint_states.position[3]) / n);
                // joint_trajectory.points[i - 1].positions[2] = joint_states.position[2] + i * ((q_des[q_des_indx][1] - joint_states.position[2]) / n);
                // joint_trajectory.points[i - 1].positions[3] = joint_states.position[0] + i * ((q_des[q_des_indx][2] - joint_states.position[0]) / n);
                // joint_trajectory.points[i - 1].positions[4] = joint_states.position[4] + i * ((q_des[q_des_indx][3] - joint_states.position[4]) / n);
                // joint_trajectory.points[i - 1].positions[5] = joint_states.position[5] + i * ((q_des[q_des_indx][4] - joint_states.position[5]) / n);
                // joint_trajectory.points[i - 1].positions[6] = joint_states.position[6] + i * ((q_des[q_des_indx][5] - joint_states.position[6]) / n);

                // result += pow(solution[0] - joint_states.position[3], 2);
                // result += pow(solution[1] - joint_states.position[2], 2);
                // result += pow(solution[2] - joint_states.position[0], 2);
                // result += pow(solution[3] - joint_states.position[4], 2);
                // result += pow(solution[4] - joint_states.position[5], 2);
                // result += pow(solution[5] - joint_states.position[6], 2);

                // ROS_WARN("%f", joint_trajectory.points[i-1].positions[0]);
                //     joint_trajectory.points[i - 1].time_from_start = ros::Duration(10 * i);
                // }

                //joint_trajectory.points[0].positions[2] = (joint_trajectory.points[0].positions[2] > pi) ? joint_trajectory.points[0].positions[2] - 2*pi : joint_trajectory.points[0].positions[2]; // shoulder lift, pitch
                //joint_trajectory.points[0].positions[3] = (joint_trajectory.points[0].positions[3] > pi) ?0; // elbow
                //joint_trajectory.points[0].positions[3] - 2*pi : joint_trajectory.points[0].positions[2];
                //joint_trajectory.points[0].positions[4] = (joint_trajectory.points[0].positions[4] > pi) ?0; // wrist 1
                //joint_trajectory.points[0].positions[4] - 2*pi : joint_trajectory.points[0].positions[2];
                //joint_trajectory.points[0].positions[5] = (joint_trajectory.points[0].positions[5] > pi) ?0; // wrist 2
                //joint_trajectory.points[0].positions[5] - 2*pi : joint_trajectory.points[0].positions[2];
                //joint_trajectory.points[0].positions[6] = (joint_trajectory.points[0].positions[6] > pi) ?; // wrist 3 orient vacuum
                //joint_trajectory.points[0].positions[2] - 2*pi : joint_trajectory.points[0].positions[2];

                // joint_trajectory.points[0].time_from_start = ros::Duration(5.0);

                // FOR PUBLISHING TO TOPIC
                // trajectory_mover.publish(joint_trajectory);

                // linear_move(new_pose.position.x, new_pose.position.y, new_pose.position.z + .15, 10, joint_trajectory);
                // ROS_WARN("Original Trajectories");
                // print_trajectory_point(joint_trajectory);

                // ROS_WARN("New Trajectories");
                // ROS_WARN("%f, %f, %f, %f, %f, %f, %f", joint_states.position[1], joint_states.position[3], joint_states.position[2], joint_states.position[0], joint_states.position[4], joint_states.position[5], joint_states.position[6]);
                // negatify_trajectory_point(joint_trajectory);

                joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
                actionlib::SimpleClientGoalState state =
                    trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal,
                                                  ros::Duration(60.0), ros::Duration(60.0));
                ROS_INFO("Action Server returned with status [%i] %s", state.state_, state.toString().c_str());
            }
            else
            {
                //Call failed
            }

            order_vector.erase(order_vector.begin());
            ROS_INFO("Processing Order End:   ");
            ROS_INFO("------------------------");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
