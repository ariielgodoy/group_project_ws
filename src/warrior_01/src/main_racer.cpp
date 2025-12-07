//--------------------------------------------------------------------//
//University of Málaga
//MAPIR Research Group - Machine Perception and Intelligent Robotics
//--------------------------------------------------------------------//

#include "warrior_01/racer.hpp"

int main ( int argc, char * argv[] )
{
    rclcpp::init ( argc, argv );
    
    auto node=std::make_shared<Warrior>();

    rclcpp::Rate rate(10);

    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}