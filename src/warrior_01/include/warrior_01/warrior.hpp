//--------------------------------------------------------------------//
//University of Málaga
//MAPIR Research Group - Machine Perception and Intelligent Robotics
//--------------------------------------------------------------------//
#include "rclcpp/rclcpp.hpp"
#include "jsoncpp/json/json.h"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rosgame_msgs/msg/rosgame_twist.hpp"
#include "rosgame_msgs/msg/rosgame_point.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rosgame_bridge/srv/rosgame_register.hpp"

class Warrior : public rclcpp::Node
{
public:
    Warrior();
    ~Warrior();


    bool MOVE_TO_GOAL = true;
    bool OBSTACLE_FOUND = false;
    
    void perform_movement(float angle, float distance, float front_distance);

    void computing_angle(std::pair<float, float> target);

    float compute_euclidean_distance(float x1, float x2, float y1, float y2);

    // Función para procesar el sensor láser.
    void process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    float P_for_aiming(float angle);

    float P_for_velocity(float angle);

    std::pair<float, float> get_nearest_enemy_pos();

    std::pair<float, float> current_resource_target = {
        std::numeric_limits<float>::quiet_NaN(), 
        std::numeric_limits<float>::quiet_NaN()
    };
    void most_density_of_resources();

    void going_for_safest_resource_escape();

    std::pair<float, float> current_attack_target;
    void crashing_into_weakest_enemy();

    void go_for_nearest_charger();

    void FSM(int number_of_players_around, bool enemy_with_advantage_near);

    enum class State {
        SEARCHING_RESOURCES,
        ENGAGING,
        SCAPING,
        RETREATING
    };

    State current_state = State::SEARCHING_RESOURCES;

    void computing_data_for_FSM();

    bool is_item_present_now(const std::vector<float>& last_item_pos, const std::vector<std::vector<float>>& current_skills_list);

    std::vector<std::vector<float>> last_skills_pos_array;
    void process_enemy_and_item_data();

    void avoidance_maneuver(bool front_wall, bool left_wall, bool right_wall, float front_distance);

    // Función para procesar la información de la escena.
    void process_scene_info(const std_msgs::msg::String::SharedPtr msg);

    
    //puedes añadir mas funciones si lo crees oportuno

    //PUBLICADORES
    //============
    // Publicador para enviar comandos de movimiento.
    rclcpp::Publisher<rosgame_msgs::msg::RosgameTwist>::SharedPtr pub1_;

    // Publicador para enviar comandos de posición.
    rclcpp::Publisher<rosgame_msgs::msg::RosgamePoint>::SharedPtr pub2_;

    //SUBSCRIPTORES
    //============
    // Subscripción al topic del sensor láser.
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
    
    // Subscripción al topic con la información de la escena.
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub2_;

    


private:
    // Variables de estado del robot. Puedes añadir las que necesites
    float battery;
    float pos_x, pos_y, gamma;
    bool autopilot_enabled = false, hammer_enabled = false, shield_enabled = false;
    
    //Id code for our robot. 
    std::string code = "-1";
    std::string warrior_nick ="";
    
    // Información de la escena.
    std::vector<std::vector<float>> skills_pos_array;
    std::vector<std::vector<float>> chargers_pos_array;
    std::vector<std::vector<float>> players_pos_array;

    // Variables auxiliares.
    // cualquier otra variable que puedas necesitar
    //Estructura para almacenar información de cada enemigo
    struct EnemyInfo {
        int id;
        float x, y;

        bool has_powerup = false; 
        double powerup_expiry = 50;
        
        double last_seen = 0.0;
    };

    std::map<int, EnemyInfo> tracked_enemies;
    int next_enemy_id = 0;
    const double POWERUP_DURATION = 40.0;

};