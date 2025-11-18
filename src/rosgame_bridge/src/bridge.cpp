
#include "bridge.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::literals::chrono_literals;

Bridge::Bridge(): Node ("bridge")
{
    // La función "handle_register_service" se ejecuta cuando se llama al servicio "RosgameRegister".
    server_register_ = this->create_service<rosgame_bridge::srv::RosgameRegister>
                       ("register_service", std::bind(&Bridge::handle_register_service, this, _1, _2, _3));

    pub_ = this->create_publisher<std_msgs::msg::String>("/create_robot", 10);


    timer = this->create_wall_timer(20s, 
        std::bind(&Bridge::timer_callback, this));

    // Se utiliza la hora actual como semilla para la función rand().
    srand(time(0));
    colors.push_back("RED");
    colors.push_back("GREEN");
    colors.push_back("BLUE");
    colors.push_back("PURPLE");
    colors.push_back("CYAN");

}



 void Bridge::timer_callback()
{
    //If any player is publishing in any /controlRobot topic, it is banned
    for (unsigned int n=1;n<RosgamePlayers.size()+1;n++)    
        {
            for (auto &player : RosgamePlayers)
            {
                if (!player.second.isbanned)
                {
                   std::string topic_name1="/controlRobot" + std::to_string(n) + "/cmd_vel";
                   std::string topic_name2="/controlRobot" + std::to_string(n) + "/goal_x_y";
                
                   //RCLCPP_INFO_STREAM(this->get_logger(),"Checking user [" << player.second.username << "] Inspecting "<< topic_name1);
                   //RCLCPP_INFO_STREAM(this->get_logger(),"Checking user [" << player.second.username << "] Inspecting "<< topic_name2);
                   
                   auto result1 = this->get_publishers_info_by_topic(topic_name1);
                   auto result2 = this->get_publishers_info_by_topic(topic_name2);

                   for(const auto& node : result1)
                     {
                        
                        //RCLCPP_INFO_STREAM(this->get_logger(),node.node_name() << " is publisher of topic name: " << topic_name << std::endl);
                        if (node.node_name()!=this->get_name())
                        {
                            RCLCPP_ERROR_STREAM(this->get_logger(),player.second.username << " is cheatting us!!! ");
                            RCLCPP_ERROR_STREAM(this->get_logger(),player.second.username << " is BANNED. ");
                            player.second.isbanned=true;
                    
                        }
                    
                    }
                        for(const auto& node : result2)
                        {
                            //RCLCPP_INFO_STREAM(this->get_logger(),node.node_name() << " is publisher of topic name: " << topic_name << std::endl);
                            if (node.node_name()!=this->get_name())
                            {
                                RCLCPP_ERROR_STREAM(this->get_logger(),player.second.username << " is cheatting us!!! ");
                                RCLCPP_ERROR_STREAM(this->get_logger(),player.second.username << " is BANNED. ");
                                player.second.isbanned=true;
                        }
                    
                }
            }
            }
    }
}
    



std::string Bridge::generate_code (int length)
{
    // Al definir la variable "alfanum" como estática, su valor se mantendrá constante a lo largo del programa.
    static const char alfanum[] = "0123456789"
                                  "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                  "abcdefghijklmnopqrstuvwxyz";
    
    std::string code = "";


    // El primer caracter de un topic NO puede ser un número.
    code = code + alfanum[(rand() % (sizeof(alfanum) - 11)) + 10];

    for (int i = 1; i < length; i++)
    {   code = code + alfanum[rand() % (sizeof(alfanum) - 1)];  }
    


    return code;
}


void Bridge::handle_register_service(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<rosgame_bridge::srv::RosgameRegister::Request> request,
    std::shared_ptr<rosgame_bridge::srv::RosgameRegister::Response> response)
{ 
    // Comprueba si el usuario se ha registrado previamente.
    RCLCPP_INFO_STREAM(this->get_logger(),"----------------------------------------------------------------------------------");
    RCLCPP_INFO_STREAM(this->get_logger(),"Received a register petition ["<< request_header->sequence_number << "]");
    for (const auto &player : RosgamePlayers)
    {
        if (player.second.username == request->username)
        {
            response->code = "-1";
            RCLCPP_WARN_STREAM(this->get_logger(), "Username already registered.");
            return;
        }
    }
        
    // Crea una nueva instancia de jugador.
    Player new_player;
    
    // Genera la respuesta del servicio con el código aleatorio.
    std::string code = generate_code(8);
    response->code = code;

    int n_robot = RosgamePlayers.size() + 1;

    RCLCPP_INFO_STREAM(this->get_logger(), "Username Color id: "<< colors[n_robot-1]);

    // Se definen los nombres de los topics, teniendo el cuenta el índice del robot.
    std::string coppelia_cmdvel_topic = "/controlRobot" + std::to_string(n_robot) + "/cmd_vel";
    std::string player_cmdvel_topic = "/" + code + "/cmd_vel";
    std::string coppelia_goalxy_topic = "/controlRobot" + std::to_string(n_robot) + "/goal_x_y";
    std::string player_goalxy_topic = "/" + code + "/goal_x_y";
    std::string coppelia_laser_topic = "/Robot" + std::to_string(n_robot) + "/laser_scan";
    std::string player_laser_topic = "/" + code + "/laser_scan";
    std::string coppelia_scene_topic = "/Robot" + std::to_string(n_robot) + "/scene_info";
    std::string player_scene_topic = "/" + code + "/scene_info";
    
    // Guarda la información de los jugadores en el mapa "RosgamePlayers", definiendo publicadores y suscriptores necesarios.
    new_player.username = request->username;
    new_player.isbanned = false;
    new_player.coppelia_laser_topic = coppelia_laser_topic;
    new_player.coppelia_scene_topic = coppelia_scene_topic;
    new_player.skills = {false, false, false};
    new_player.pub1_ = this->create_publisher<geometry_msgs::msg::Twist>( coppelia_cmdvel_topic, 10 );
    new_player.sub1_ = this->create_subscription<rosgame_msgs::msg::RosgameTwist>( player_cmdvel_topic, 10, std::bind(&Bridge::cmdvel_info_exchange, this, _1) );
    new_player.pub2_ = this->create_publisher<sensor_msgs::msg::LaserScan>( player_laser_topic, 10 );
    new_player.sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>( coppelia_laser_topic, 10, std::bind(&Bridge::laser_info_exchange, this, _1) );
    new_player.pub3_ = this->create_publisher<std_msgs::msg::String>( player_scene_topic, 10 );
    new_player.sub3_ = this->create_subscription<std_msgs::msg::String>( coppelia_scene_topic, 10, std::bind(&Bridge::scene_info_exchange, this, _1) );
    new_player.pub4_ = this->create_publisher<geometry_msgs::msg::Point>( coppelia_goalxy_topic, 10 );
    new_player.sub4_ = this->create_subscription<rosgame_msgs::msg::RosgamePoint>( player_goalxy_topic, 10, std::bind(&Bridge::goalxy_info_exchange, this, _1) );
    
    RosgamePlayers[code] = new_player;

    // Crea un nuevo robot en la escena de CoppeliaSim.
    std_msgs::msg::String new_robot;
    new_robot.data = request->username;
    pub_ -> publish(new_robot);

    // DEBUGGING.
    RCLCPP_INFO(this->get_logger(), "Username = '%s'", new_player.username.c_str());
  //  RCLCPP_INFO(this->get_logger(), "CoppeliaSim 'cmd_vel' topic = '%s'", coppelia_cmdvel_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Player 'cmd_vel' topic = '%s'", player_cmdvel_topic.c_str());
  //  RCLCPP_INFO(this->get_logger(), "CoppeliaSim laser topic = '%s'", coppelia_laser_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Player laser topic = '%s'", player_laser_topic.c_str());
  //  RCLCPP_INFO(this->get_logger(), "CoppeliaSim scene info = '%s'", coppelia_scene_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Player scene info = '%s'", player_scene_topic.c_str());

    RCLCPP_INFO_STREAM(this->get_logger(),"----------------------------------------------------------------------------------");
}


void Bridge::cmdvel_info_exchange(const rosgame_msgs::msg::RosgameTwist::SharedPtr msg)
{
    geometry_msgs::msg::Twist velocity;

    // Guarda la información del msg en un nuevo topic.
    velocity.linear.x = msg->vel.linear.x;
    velocity.angular.z = msg->vel.angular.z;

    // En el mapa "RosgamePlayers" se busca al jugador cuyo código coincide con el del mensaje, empleando el método find().
    auto pointer = RosgamePlayers.find(msg->code);
    // Con el iterador que devuelve, accedemos al campo "pub1_" para reenviar el topic al robot de la escena.
    //Compruebo que code es válido para un jugador.
    //TODO.- Si se publica con un code incorrecto o sin code, banear al jugador
    if (pointer!= RosgamePlayers.end())
    {
        if (!pointer->second.isbanned)
            pointer->second.pub1_->publish(velocity);
    }
}


void Bridge::goalxy_info_exchange(const rosgame_msgs::msg::RosgamePoint::SharedPtr msg)
{
    geometry_msgs::msg::Point position;

    // Guarda la información del msg en un nuevo topic.
    position.x = msg->pos.x;
    position.y = msg->pos.y;

    // En el mapa "RosgamePlayers" se busca al jugador cuyo código coincide con el del mensaje, empleando el método find().
    auto pointer = RosgamePlayers.find(msg->code);
    // Con el iterador que devuelve, accedemos al campo "pub4_" para reenviar el topic al robot de la escena.    
    
    if (pointer!= RosgamePlayers.end())
    {
        if (!pointer->second.isbanned)
            pointer->second.pub4_->publish(position);
    }

}


void Bridge::laser_info_exchange(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    sensor_msgs::msg::LaserScan laser_scan;

    // La información del msg se guarda en un nuevo topic.
    laser_scan.angle_min = msg->angle_min;
    laser_scan.angle_max = msg->angle_max;
    laser_scan.angle_increment = msg->angle_increment;
    laser_scan.range_min = msg->range_min;
    laser_scan.range_max = msg->range_max;
    laser_scan.ranges = msg->ranges;

    // En el mapa "RosgamePlayers" se busca el jugador cuyo campo "coppelia_laser_topic" coindice con el "frame_id" del msg.
    for (auto &player : RosgamePlayers)
    {
        if (player.second.coppelia_laser_topic == msg->header.frame_id && !player.second.isbanned)
        {
            // Con el iterador que devuelve, accedemos al campo "pub2_" para reenviar el topic al nodo del jugador.
            player.second.pub2_->publish(laser_scan);
            return;
        }
    }
}


void Bridge::scene_info_exchange(const std_msgs::msg::String::SharedPtr msg)
{
    // Define un objeto para convertir el msg de tipo string con formato a JSON.
    Json::CharReaderBuilder reader;
    Json::Value JsonSceneData;
    std::istringstream jsonStream(msg->data);
    Json::parseFromStream(reader, jsonStream, &JsonSceneData, nullptr);

    // Descomentar para mostar la información de la escena en el terminal.
    // RCLCPP_INFO(this->get_logger(), "Msg: '%s'", msg->data.c_str());

    // Se obtiene el nombre del topic a partir de la clave "Topic".
    // Get the value associated to the "Topic" key.
    std::string topic_name = JsonSceneData["Topic"].asString();

    // Se obtienen las habilidades del JSON a partir de la clave "Skills".
    bool teleport_enabled = JsonSceneData["Skills"]["Teleport"].asBool();
    bool hammer_enabled = JsonSceneData["Skills"]["Hammer"].asBool();
    bool shield_enabled = JsonSceneData["Skills"]["Shield"].asBool();


    // Hay que borrar el campo "Topic" antes de enviar el msg al jugador, pues no puede conocer el nombre del topic de CoppeliaSim.
    JsonSceneData.removeMember("Topic");

    // Se define un objeto para convertir el JSON en un string con formato para poder enviarlo en un topic.
    Json::StreamWriterBuilder writer;
    // El espaciado "indentation" se tiene que establecer como vacío para que no aparezcan caracteres de escape (\n o \t).
    writer["indentation"] = "";
    std::string StringSceneData = Json::writeString(writer, JsonSceneData);

    // La información del msg se guarda en un nuevo topic.
    std_msgs::msg::String scene_info_msg;
    scene_info_msg.data = StringSceneData;

    // En el mapa "RosgamePlayers" se buca al jugador cuyo campo "coppelia_scene_info" coincide con el nombre del topic.
    for (auto &player : RosgamePlayers)
    {
        if (player.second.coppelia_scene_topic == topic_name && !player.second.isbanned)
        {
            // Con el iterador que devuelve, accedemos al campo "pub3_" para reenviar el topic al nodo del jugador.
            player.second.pub3_->publish(scene_info_msg);

            // También se usa este iterador para actualizar las habilidades del jugador.
            player.second.skills.teleport = teleport_enabled;
            player.second.skills.hammer = hammer_enabled;
            player.second.skills.shield = shield_enabled;

            return;
        }
    }
}