#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Model.h"
#include "geometry_msgs/Pose.h"
#include <map>

//Global Variables
//Order tracking
int order_count = 0;
std::vector<osrf_gear::Order> order_vector;

//Items
std::map<std::string, std::vector<osrf_gear::Model>> items_bin;     //Holds products in bins
std::map<std::string, std::vector<osrf_gear::Model>> items_agv;     //Holds products on the conveyerbelts
std::map<std::string, std::vector<osrf_gear::Model>> items_qcs;     //Holds faulty projects (on trays)


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
