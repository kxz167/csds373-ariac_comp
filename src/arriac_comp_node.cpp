#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"

//Global order vector
std::vector<osrf_gear::Order> order_vector;

void orderCallback(const osrf_gear::Order::ConstPtr &msg)
{
    ROS_INFO("Callback");
    order_vector.push_back(*msg);

    //Create a copy of the message:
    // geometry_msgs::Twist msg_copy(*msg);

}

int main(int argc, char **argv)
{
    /**
   * Initialize ros, name the node
   */
    ros::init(argc, argv, "arriac_comp_node");

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
    
    /**
     * Create the material location service for determining location of products
     */
    ros::ServiceClient material_location_client = n.serviceClient<std_srvs::Trigger>("/ariac/material_locations");

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
    // ros::Subscriber laserSub = n.subscribe("laser_1", 1000, lidarCallback);

    //Define checks
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ROS_INFO("HI");
        //Look at orders if there are any waiting:
        if(!order_vector.empty()){
            //Loop through each shipment
            for(osrf_gear::Shipment shipment : order_vector.front().shipments){
                //And each product within
                for(osrf_gear::Product product : shipment.products){
                    std::string product_type = product.type;
                    ROS_INFO(product_type.c_str());
                    ROS_INFO(material_location_client.call(product_type).c_str());
                }
            }

            order_vector.erase(order_vector.begin());
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
