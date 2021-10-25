#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Model.h"
#include "geometry_msgs/Pose.h"
#include <map>

//Global order vector
int order_count = 0;
std::vector<osrf_gear::Order> order_vector;
std::map<std::string, std::vector<osrf_gear::Model>> items_bin;
std::map<std::string, std::vector<osrf_gear::Model>> items_agv;
std::map<std::string, std::vector<osrf_gear::Model>> items_qcs;
// std::map<std::string, std::string> CAMERAS;
// m["bin4"]

void orderCallback(const osrf_gear::Order::ConstPtr &msg)
{
    ROS_INFO("Order %d Received", ++order_count);
    order_vector.push_back(*msg);

    //Create a copy of the message:
    // geometry_msgs::Twist msg_copy(*msg);

}

void cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &msg){
    //We get info from subscribed camera:
    if(msg -> models.empty()) {
        // ROS_INFO("No more products");
        return;
    }
        
    std::string type = msg -> models.front().type;
    // ROS_INFO(type.c_str());
    items_bin[type.c_str()] = msg -> models;
}

void agv_cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &msg){
    //We get info from subscribed camera:
    if(msg -> models.empty()) {
        // ROS_INFO("No more products");
        return;
    }
        
    std::string type = msg -> models.front().type;
    // ROS_INFO(type.c_str());
    items_agv[type.c_str()] = msg -> models;
}

void qcs_cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &msg){
    //We get info from subscribed camera:
    if(msg -> models.empty()) {
        // ROS_INFO("No more products");
        return;
    }
        
    std::string type = msg -> models.front().type;
    // ROS_INFO(type.c_str());
    items_qcs[type.c_str()] = msg -> models;
}

void printPose(const geometry_msgs::Pose pose){
    ROS_WARN("\nProduct Position:\nxyz = (%f, %f, %f) \nwxyz = (%f, %f, %f, %f)",pose.position.x,pose.position.y,pose.position.z,pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
}

int main(int argc, char **argv)
{
    /**
   * Initialize ros, name the node
   */
    ros::init(argc, argv, "ariac_comp_node");

    /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle n;

    /**
     * Trigger the beginning of the competiton.
     */
    std_srvs::Trigger begin_comp;
    ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

    //Subscribe to cameras over product bins:
    ros::Subscriber camera_sub_b1 = n.subscribe("/ariac/logical_camera_bin1" , 1000, cameraCallback);
    ros::Subscriber camera_sub_b2 = n.subscribe("/ariac/logical_camera_bin2" , 1000, cameraCallback);
    ros::Subscriber camera_sub_b3 = n.subscribe("/ariac/logical_camera_bin3" , 1000, cameraCallback);
    ros::Subscriber camera_sub_b4 = n.subscribe("/ariac/logical_camera_bin4" , 1000, cameraCallback);
    ros::Subscriber camera_sub_b5 = n.subscribe("/ariac/logical_camera_bin5" , 1000, cameraCallback);
    ros::Subscriber camera_sub_b6 = n.subscribe("/ariac/logical_camera_bin6" , 1000, cameraCallback);
    ros::Subscriber camera_sub_a1 = n.subscribe("/ariac/logical_camera_agv1" , 1000, agv_cameraCallback);
    ros::Subscriber camera_sub_a2 = n.subscribe("/ariac/logical_camera_agv2" , 1000, agv_cameraCallback);
    ros::Subscriber camera_sub_q1 = n.subscribe("/ariac/quality_control_sensor_1" , 1000, qcs_cameraCallback);
    ros::Subscriber camera_sub_q2 = n.subscribe("/ariac/quality_control_sensor_2" , 1000, qcs_cameraCallback);

    
    /**
     * Create the material location service for determining location of products
     */
    ros::ServiceClient material_location_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

    int service_call_succeeded;

    service_call_succeeded = begin_client.call(begin_comp);

    if (!service_call_succeeded)
    {
        ROS_ERROR("Competition service call failed! LOSER!");
        ros::shutdown();
    }
    else
    {
        if (!begin_comp.response.success)
        {
            ROS_WARN("Start Failure: %s", begin_comp.response.message.c_str());
        }
        else
        {
            ROS_INFO("Start Success: %s", begin_comp.response.message.c_str());
        }
    }

    order_vector.clear();

    /**
   * Create a publisher
   */
    // commandPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    /**
   * Create the subscribers for  the lidar
   */
    ros::Subscriber orderSub = n.subscribe("/ariac/orders", 1000, orderCallback);

    //Define checks
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // ROS_INFO("HI");
        //Look at orders if there are any waiting:
        if(!order_vector.empty()){
            //Loop through each shipment (UNECESSARY BUT KEPT)
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

            osrf_gear::Product first_product = order_vector.front().shipments.front().products.front();
            
            std::string product_type = first_product.type;
            ROS_INFO("Product Type : %s", product_type.c_str());
            
            osrf_gear::GetMaterialLocations srv;
            srv.request.material_type = product_type;
            if(material_location_client.call(srv)){
                //Were able to find product location:
                std::string product_location = srv.response.storage_units.front().unit_id;
                ROS_INFO("Located In   : %s", product_location.c_str());
                osrf_gear::Model first_model = items_bin[product_type.c_str()].front();

                printPose(first_model.pose);
            }

            order_vector.erase(order_vector.begin());
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
