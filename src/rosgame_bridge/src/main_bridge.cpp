#include "bridge.hpp"

int main ( int argc, char * argv[] )
{
    // La inicialización de ROS debe ser la primera instrucción.
    rclcpp::init ( argc, argv );

    auto node=std::make_shared<Bridge>();

    rclcpp::Rate rate(10);
    RCLCPP_INFO(node->get_logger(), "Bridge node started. Waiting for brave warriors");
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}