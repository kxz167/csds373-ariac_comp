#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Model.h"
#include "geometry_msgs/Pose.h"
#include <map>

//Lab 6
#include "ur_kinematics/ur_kin.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

//Global Variables
//Order tracking
int order_count = 0;
std::vector<osrf_gear::Order> order_vector;

//Items
std::map<std::string, std::vector<osrf_gear::Model>> items_bin;     //Holds products in bins
std::map<std::string, std::vector<osrf_gear::Model>> items_agv;     //Holds products on the conveyerbelts
std::map<std::string, std::vector<osrf_gear::Model>> items_qcs;     //Holds faulty projects (on trays)


//Joint states:
sensor_msgs::JointState joint_states;


// Store the order whenever it is received.
void orderCallback(const osrf_gear::Order::ConstPtr &msg)
{
    ROS_INFO("Order %d Received.", ++order_count);
    order_vector.push_back(*msg);
}

// Camera callback to store the product (models) based on types.
void cameraCallback(
    const osrf_gear::LogicalCameraImage::ConstPtr &msg, 
    std::map<std::string, std::vector<osrf_gear::Model>> *itemMap       //Must use pointers to avoid segfault
){
    //Get information from cameras
    if(msg -> models.empty()) {
        // Return if no parts below.
        return;
    }
        
    //Then get the type if there are
    std::string type = msg -> models.front().type;

    //For QC cameras, if no type, set type to faulty:
    if(type.empty()){
        type = "faulty";
    }

    //Place into the item map:
    (*itemMap)[type.c_str()] = msg -> models;
}

void armJointCallback(
    const sensor_msgs::JointState::ConstPtr &msg
){
    joint_states = sensor_msgs::JointState(*msg);
}

//Helper method to print out a Pose into a useful string.
void printPose(const geometry_msgs::Pose pose){
    ROS_WARN("Product Position:");
    ROS_WARN("xyz = (%f, %f, %f)",pose.position.x,pose.position.y,pose.position.z);
    ROS_WARN("wxyz = (%f, %f, %f, %f)",pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
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

    //Subscribe to cameras over product bins:
    ros::Subscriber camera_sub_b1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin1" , 1000, boost::bind(cameraCallback, _1, &items_bin));
    ros::Subscriber camera_sub_b2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin2" , 1000, boost::bind(cameraCallback, _1, &items_bin));
    ros::Subscriber camera_sub_b3 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin3" , 1000, boost::bind(cameraCallback, _1, &items_bin));
    ros::Subscriber camera_sub_b4 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin4" , 1000, boost::bind(cameraCallback, _1, &items_bin));
    ros::Subscriber camera_sub_b5 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin5" , 1000, boost::bind(cameraCallback, _1, &items_bin));
    ros::Subscriber camera_sub_b6 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin6" , 1000, boost::bind(cameraCallback, _1, &items_bin));
    ros::Subscriber camera_sub_a1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv1" , 1000, boost::bind(cameraCallback, _1, &items_agv));
    ros::Subscriber camera_sub_a2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv2" , 1000, boost::bind(cameraCallback, _1, &items_agv));
    ros::Subscriber camera_sub_q1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_1" , 1000, boost::bind(cameraCallback, _1, &items_qcs));
    ros::Subscriber camera_sub_q2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_2" , 1000, boost::bind(cameraCallback, _1, &items_qcs));
    
    //Subscribe to incoming orders:
    ros::Subscriber orderSub = n.subscribe("/ariac/orders", 1000, orderCallback);

    //Create the material location service for determining location of products
    ros::ServiceClient material_location_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");


    //Subscribe to joint states:
    ros::Subscriber joint_states_h = n.subscribe("/ariac/arm1/joint_states", 10, armJointCallback);

    //Publish to a topic:
    ros::Publisher trajectory_mover = n.publish("/ariac/arm1/arm/command", 10);
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
        if(!order_vector.empty()){
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
            if(material_location_client.call(srv)){
                //Were able to find a product location:
                std::string product_location = srv.response.storage_units.front().unit_id;
                ROS_INFO("Located In   : %s", product_location.c_str());
                osrf_gear::Model first_model = items_bin[product_type.c_str()].front();

                //Output the pose
                printPose(first_model.pose);

                //LAB 2 PRODUCT PROCESSING
                double T_pose[4][4] = {0.0}, T_des[4][4] = {0.0};
                double q_pose[6] = {0.0}, q_des[8][6] = {0.0};
                // trajectory_msgs::JointTrajectory desired;
                geometry_msgs::Pose desired = first_model.pose;

                //Finds where our sucky thing is right now.
                q_pose[0] = joint_states.position[3];
                q_pose[1] = joint_states.position[2];
                q_pose[2] = joint_states.position[0];
                q_pose[3] = joint_states.position[4];
                q_pose[4] = joint_states.position[5];
                q_pose[5] = joint_states.position[6];

                // ur_kinematics::forward((double*)&q_pose, (double*)&T_pose);

                //Where is our product?
                //LOCATION x/y/z
                T_des[0][3] = desired.position.x;
                T_des[1][3] = desired.position.y;
                T_des[2][3] = desired.position.z + 0.3;
                T_des[3][3] = 1.0;

                //ROTATION
                T_des[2][0] = -1.0;
                T_des[0][1] = -1.0;
                T_des[1][2] = -1.0;

                trajectory_msgs::JointTrajectory joint_trajectory;

                joint_trajectory.header.seq = count++;
                joint_trajectory.header.stamp = ros::Time::now();
                joint_trajectory.header.frame_id = "/world";

                //Construct message:
                joint_trajectory.joint_names.clear();
                joint_trajectory.joint_names.pushBack("linear_arm_actuator_joint");
                joint_trajectory.joint_names.pushBack("shoulder_pan_joint");
                joint_trajectory.joint_names.pushBack("shoulder_lift_joint");
                joint_trajectory.joint_names.pushBack("elbow_joint");
                joint_trajectory.joint_names.pushBack("wrist_1_joint");
                joint_trajectory.joint_names.pushBack("wrist_2_joint");
                joint_trajectory.joint_names.pushBack("wrist_3_joint");

                joint_trajectory.points.resize(2);

                joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
                for (int indy =0; indy < joint_trajectory.joint_names.size(); indy++){
                    for(int indz = 0; indz < joint_states.name.size(); indz++){
                        if(joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
                            joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
                            break;
                        }
                    }
                }

                joint_trajectory.points[0].time_from_start = ros::Duration(0.0);

                int q_des_indx = 0;     
                joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
                joint_trajectory.points[1].positions[0] = joint_states.positions[1];

                for(int indy = 0; indy < 6; indy++) {
                    joint_trajectory.points[1].positions[indy+1] = q_sols[q_sols_indx][indy];
                }
                joint_trajectory.points[1].time_from_start = ros::Duration(1.0);


            }
            else{
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
