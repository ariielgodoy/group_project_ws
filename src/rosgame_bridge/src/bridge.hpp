#include "rclcpp/rclcpp.hpp"
#include "jsoncpp/json/json.h"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rosgame_msgs/msg/rosgame_twist.hpp"
#include "rosgame_msgs/msg/rosgame_point.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rosgame_bridge/srv/rosgame_register.hpp"
#include <chrono>

class Bridge : public rclcpp::Node
{
public:
    Bridge();

    // Se define una función para enviar cada topic procedente de CoppeliaSim al nodo del jugador correspondiente (o viceversa).
    void cmdvel_info_exchange(const rosgame_msgs::msg::RosgameTwist::SharedPtr msg);
    void goalxy_info_exchange(const rosgame_msgs::msg::RosgamePoint::SharedPtr msg);
    void laser_info_exchange(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void scene_info_exchange(const std_msgs::msg::String::SharedPtr msg);
    
    ~Bridge(){};

private:
    // Se declara el servidor del servicio y una función a la que llamar cuando se solicite.
    rclcpp::Service<rosgame_bridge::srv::RosgameRegister>::SharedPtr server_register_;
    void handle_register_service
    (
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<rosgame_bridge::srv::RosgameRegister::Request> request,
        std::shared_ptr<rosgame_bridge::srv::RosgameRegister::Response> response  
    );

    // Se define un publicador que se usará en CoppeliaSim para generar los robots de los jugadores.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    
    // Función para generar un código alfanumérico aleatorio.
    std::string generate_code(int length);

    struct possible_skills
    {
        bool teleport;
        bool hammer;
        bool shield;
    };

    struct Player
    {
        std::string username;
        std::string coppelia_laser_topic;
        std::string coppelia_scene_topic;
        possible_skills skills;
        bool isbanned;
        
        // Manejadores para pub/sub comandos de velocidad.
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub1_;
        rclcpp::Subscription<rosgame_msgs::msg::RosgameTwist>::SharedPtr sub1_;
        
        // Manejadores para pub/sub información del sensor láser.
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub2_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;

        // Manejadores para pub/sub información de la escena.
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub3_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub3_;

        // Manejadores para pub/sub comandos de posición.
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub4_;
        rclcpp::Subscription<rosgame_msgs::msg::RosgamePoint>::SharedPtr sub4_;
    };

    // La información de los jugadores se guardará en un mapa.
    std::unordered_map<std::string, Player> RosgamePlayers;

    //Timer to check the "security" of the system
    rclcpp::TimerBase::SharedPtr timer;
    void timer_callback();
    std::vector<std::string> colors;
};