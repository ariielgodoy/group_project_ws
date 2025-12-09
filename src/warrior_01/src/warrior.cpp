//--------------------------------------------------------------------//
//University of Málaga
//MAPIR Research Group - Machine Perception and Intelligent Robotics
//--------------------------------------------------------------------//

#include "warrior_01/warrior.hpp"

Warrior::Warrior(): Node ("robot_warrior")
{
    warrior_nick = "JackTheBotRipper";
    int cont = 0;
    
    // Se crea un cliente de servicio y una solicitud para lanzar el servicio.
    auto client = create_client<rosgame_bridge::srv::RosgameRegister>("register_service");
    
    auto request = std::make_shared<rosgame_bridge::srv::RosgameRegister::Request>();
    request -> username = warrior_nick;
    
    // Se espera a que el servicio esté disponible.
    bool service_available = false;
    while(!service_available && rclcpp::ok())
    {
        if (client->wait_for_service(std::chrono::seconds(5)))
        {   service_available = true;   }
        else
        {   RCLCPP_INFO(this->get_logger(), "Service not available. Retrying...");  }
    }
    
    // Se llama al servicio hasta que la respuesta sea diferente a "-1". Este valor indica que ya existe un jugador registrado con el nombre de usuario proporcionado.
    while (code == "-1" && rclcpp::ok())
    {   
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            code = response->code;

            if (code == "-1")
            {
                RCLCPP_WARN(this->get_logger(), "Username already exists. Calling the service again...");
                warrior_nick= warrior_nick + std::to_string(cont);
                request -> username = warrior_nick;
                cont = cont + 1;
            }
            else
            {   
                // Se definen los publicadores y suscriptores necesarios.
                pub1_ = create_publisher<rosgame_msgs::msg::RosgameTwist>( "/" + code + "/cmd_vel", 10 );
                pub2_ = create_publisher<rosgame_msgs::msg::RosgamePoint>( "/" + code + "/goal_x_y", 10 );
                sub1_ = create_subscription<sensor_msgs::msg::LaserScan>( "/" + code + "/laser_scan", 1, std::bind(&Warrior::process_laser_info, this, std::placeholders::_1));
                sub2_ = create_subscription<std_msgs::msg::String>( "/" + code + "/scene_info", 1, std::bind(&Warrior::process_scene_info, this, std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(), "Player registered. Starting simulation.");           
            }
        }
    }




}

Warrior::~Warrior()
{
     RCLCPP_ERROR(this->get_logger(), "Game over for [%s]", warrior_nick.c_str());
}

void Warrior::process_scene_info(const std_msgs::msg::String::SharedPtr msg)
{
    // Se convierte el msg de tipo string con formato en un JSON.
    Json::CharReaderBuilder reader;
    Json::Value JsonSceneData;
    std::istringstream jsonStream(msg->data);
    Json::parseFromStream(reader, jsonStream, &JsonSceneData, nullptr);
        
    // Se obtiene el valor de la batería de la clave "Battery_Level".
    battery = JsonSceneData["Battery_Level"].asFloat();

    // Se obtiene la pose del robot a partir de la clave "Robot_Pose".
    pos_x = JsonSceneData["Robot_Pose"]["x"].asFloat();
    pos_y = JsonSceneData["Robot_Pose"]["y"].asFloat();
    gamma = JsonSceneData["Robot_Pose"]["gamma"].asFloat();

    // Se obtienen las habilidades de la cave "Skills".
    autopilot_enabled = JsonSceneData["Skills"]["Autopilot"].asBool();
    hammer_enabled = JsonSceneData["Skills"]["Hammer"].asBool();
    shield_enabled = JsonSceneData["Skills"]["Shield"].asBool();

    // Se obtienen las posiciones de bloques de habilidades de la clave "Skills_Positions" en el campo "FOV".
    std::vector<std::vector<float>> skills_pos_array_aux;
    const Json::Value &skills_pos = JsonSceneData["FOV"]["Skills_Positions"];
    for (const Json::Value &skill : skills_pos)
    {
        std::vector<float> skillData;
        for (const Json::Value &value : skill)
        {   skillData.push_back(value.asFloat());   }
        skills_pos_array_aux.push_back(skillData);
    }
    skills_pos_array = skills_pos_array_aux;

    // Se obtienen las posiciones de plataformas de recarga de la clave "Chargers_Positions" en el campo "FOV".
    // Las plataformas de recarga están en posiciones fijas durante toda la simulación.
    // Objetivo: buscar las cinco plataformas y almacenarlas todas en la lista "chargers_pos_array".
    const Json::Value &chargers_pos = JsonSceneData["FOV"]["Chargers_Positions"];
    for (const Json::Value &charger : chargers_pos)
    {
        std::vector<float> chargerData;
        for (const Json::Value &value : charger)
        {   chargerData.push_back(value.asFloat());     }

        // Verifica si la posición ya está en "chargers_pos_array".
        // La función "find" devuelve un iterador que apunta al elemento del array si existe o al final del array si no lo ha encontrado.
        if (std::find(chargers_pos_array.begin(), chargers_pos_array.end(), chargerData) == chargers_pos_array.end())
        {   chargers_pos_array.push_back(chargerData);  }
    }

    // Se obtienen las posiciones de los oponentes de la clave "Players_Positions" en el campo "FOV".
    std::vector<std::vector<float>> players_pos_array_aux;
    const Json::Value &players_pos = JsonSceneData["FOV"]["Players_Positions"];
    for (const Json::Value &player : players_pos)
    {
        std::vector<float> playerData;
        for (const Json::Value &value : player)
        {   playerData.push_back(value.asFloat());  }
        players_pos_array_aux.push_back(playerData);
    }
    players_pos_array = players_pos_array_aux;


    RCLCPP_INFO(this->get_logger(), "Msg: '%s'", msg->data.c_str());
    /*if (!skills_pos_array.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Skills Positions Array:");
        for (const auto &skill_pos : skills_pos_array)
        {   RCLCPP_INFO(this->get_logger(), "[X] = '%f', [Y] = '%f'", skill_pos[0], skill_pos[1]);  }
    }*/
    // DEBUGGING
    /*
    RCLCPP_INFO(this->get_logger(), "Msg: '%s'", msg->data.c_str());

    RCLCPP_INFO(this->get_logger(), "Battery level: '%f'", battery);

    RCLCPP_INFO(this->get_logger(), "Is teleport enabled? '%s'", (teleport_enabled ? "True" : "False"));
    RCLCPP_INFO(this->get_logger(), "Is the hammer enabled? '%s'", (hammer_enabled ? "True" : "False"));
    RCLCPP_INFO(this->get_logger(), "Is the shield enabled? '%s'", (shield_enabled ? "True" : "False"));

    if (!skills_pos_array.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Skills Positions Array:");
        for (const auto &skill_pos : skills_pos_array)
        {   RCLCPP_INFO(this->get_logger(), "[X] = '%f', [Y] = '%f'", skill_pos[0], skill_pos[1]);  }
    }

    RCLCPP_INFO(this->get_logger(), "Robot Pose: [X] = '%f', [Y] = '%f', [GAMMA] = '%f'", pos_x, pos_y, gamma);
    */
}

float Warrior::compute_euclidean_distance(float x1, float x2, float y1, float y2){
    
    float distance = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));

    return distance;
}



float Warrior::P_for_aiming(float angle){

    float Kp = 2; //2 ha ido bien
    float P = Kp * angle;

    return P;
}



void Warrior::perform_movement(float angle)
{
    rosgame_msgs::msg::RosgameTwist movement;

    movement.vel.linear.x = 0.2;
    movement.vel.angular.z = this->P_for_aiming(angle);

    movement.code = code;
    this->pub1_->publish(movement);

}


void Warrior::computing_angle(std::pair<float, float> target){
    double x = target.first;
    double y = target.second;

    double x_diff = x - pos_x;
    double y_diff = y - pos_y;

    double global_angle = atan2(y_diff, x_diff);

    double local_angle_diff = global_angle - gamma;
    double real_local_angle_diff = (sin(local_angle_diff), cos(local_angle_diff));


    this->perform_movement(real_local_angle_diff);
}


void Warrior::most_density_of_resources(){
    float best_density_score = -1.0f;
    std::pair<float, float> best_resource_target = {0.0f, 0.0f};
    float radius = 5.0f; //Radio para considerar que las cajas estan agrupadas

    //Iteramos por cada recurso para evaluar su entorno
    for (size_t i = 0; i < skills_pos_array.size(); ++i) {
        float current_x = skills_pos_array[i][0];
        float current_y = skills_pos_array[i][1];
        int count = 0;

        //Comparamos esta caja con todas las demás
        for (size_t j = 0; j < skills_pos_array.size(); ++j) {
            float dist = compute_euclidean_distance(current_x, skills_pos_array[j][0], 
                                                   current_y, skills_pos_array[j][1]);
            if (dist <= radius) {
                count++;
            }
        }

        //Puntuamos: Densidad = número de cajas / distancia al robot (para no irnos muy lejos)
        float dist_to_robot = compute_euclidean_distance(pos_x, current_x, pos_y, current_y);
        float score = (float)count / (dist_to_robot * 0.5f); //El 0.5 pondera la cercanía

        if (score > best_density_score) {
            best_density_score = score;
            best_resource_target = {current_x, current_y};
        }
    }
    this->computing_angle(best_resource_target);
}


std::pair<float, float> Warrior::get_nearest_enemy_pos(){
    float min_dist = 9999.0f;
    std::pair<float, float> nearest_coords = {0.0f, 0.0f};
    
    //Iteramos sobre todos los enemigos detectados
    for (const auto& enemy : players_pos_array) {
        float enemy_x = enemy[0];
        float enemy_y = enemy[1];

        float dist = compute_euclidean_distance(pos_x, enemy_x, pos_y, enemy_y);
        
        if (dist < min_dist) {
            min_dist = dist;
            nearest_coords = {enemy_x, enemy_y};
        }
    }

    return nearest_coords;
}


void Warrior::going_for_safest_resource_escape() {
    float min_dist_to_me = 1000.0f;
    std::pair<float, float> safest_resource_target = {0.0f, 0.0f};
    bool found = false;

    std::pair<float, float> enemy = this->get_nearest_enemy_pos();

    float my_dist_to_enemy = this->compute_euclidean_distance(pos_x, enemy.first, pos_y, enemy.second);

    for (size_t i = 0; i < skills_pos_array.size(); ++i) {
        float res_x = skills_pos_array[i][0];
        float res_y = skills_pos_array[i][1];

        float res_dist_to_enemy = compute_euclidean_distance(res_x, enemy.first, res_y, enemy.second);


        if (res_dist_to_enemy > my_dist_to_enemy) {
            float dist_to_me = this->compute_euclidean_distance(pos_x, res_x, pos_y, res_y);
            
            if (dist_to_me < min_dist_to_me) {
                min_dist_to_me = dist_to_me;
                safest_resource_target = {res_x, res_y};
                found = true;
            }
        }
    }

    if (!found) {
        // Calculamos el vector que va desde el enemigo hacia el robot
        float diff_x = pos_x - enemy.first;
        float diff_y = pos_y - enemy.second;

        // Normalizamos y creamos un punto a 10 metros en esa dirección
        float magnitude = sqrt(diff_x * diff_x + diff_y * diff_y);
        
        // Evitar división por cero
        if (magnitude > 0.1f) {
            safest_resource_target.first = pos_x + (diff_x / magnitude) * 10.0f;
            safest_resource_target.second = pos_y + (diff_y / magnitude) * 10.0f;
        }
    }

    this->computing_angle(safest_resource_target);
}


void Warrior::go_for_nearest_charger(){
    float min_dist = 1000.0f; // Inicializamos con una distancia grande
    std::pair<float, float> nearest_charger = {0.0f, 0.0f};
    bool charger_found = false;

    for (const auto& charger : chargers_pos_array) {
        float charger_x = charger[0];
        float charger_y = charger[1];
        
        float dist = compute_euclidean_distance(pos_x, charger_x, pos_y, charger_y);
        
        if (dist < min_dist) {
            min_dist = dist;
            nearest_charger = {charger_x, charger_y};
            charger_found = true;
        }
    }

    if (charger_found) {
        this->computing_angle(nearest_charger);
    }
}


void Warrior::crashing_into_weakest_enemy(){
    
}

//Maquina de estados para escoger accion
void Warrior::FSM(int number_of_players_around) {
    if (battery < 40.0 && current_state != State::RETREATING) {
        current_state = State::RETREATING;
    }

    switch (current_state) {
        case State::SEARCHING_RESOURCES:
            this->most_density_of_resources();
            
            if (hammer_enabled || shield_enabled) 
                current_state = State::ENGAGING;
            else if (enemy_with_advantage_near) 
                current_state = State::SCAPING;
            break;
        
        case State::SCAPING:
            this->going_for_safest_resource_escape(); 
            
            if (!enemy_with_advantage_near)
                current_state = State::SEARCHING_RESOURCES;
            break;

        case State::ENGAGING:
            this->crashing_into_weakest_enemy();

            if (!hammer_enabled && !shield_enabled) {
                current_state = (number_of_players_around < 2) ? State::ENGAGING : State::SEARCHING_RESOURCES;
            }
            break;

        case State::RETREATING:
            this->go_for_nearest_charger();
            if (battery > 90.0) // Cargamos un poco más para tener margen
                current_state = State::SEARCHING_RESOURCES;
            break;
    }
}


void Warrior::computing_data_for_FSM(){
    int number_of_players_around = 0;
    for (const auto &players_pos : players_pos_array){
        double x = players_pos[0];
        double y = players_pos[1];

        if(this->compute_euclidean_distance(x, pos_x, y, pos_y)<3){
            number_of_players_around++;
        }
    }
    //FSM para las decisiones del robot
    this->FSM(number_of_players_around);
}

void Warrior::process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    int array_size = msg->ranges.size();
    double angle_increment = msg->angle_increment;


    //Flags para el programa de movimiento
    
    double front_distance = msg->range_max;
    int izquierda = 0;
    int centro = 0;
    int derecha = 0;

    bool front_wall = false;
    bool left_wall = false;
    bool right_wall = false;

    
    //Mirar a los lados y al centro
    for(int k = 86; k < 257; k++){
        if(msg->ranges[k]<0.5){
            derecha++;
            if(derecha>10){
                right_wall = true;
            }
            
        }


        if(msg->ranges[k+array_size/2 - 86 - 86] < 0.5){
            centro++;
            if(front_distance > msg->ranges[k+214]){
                front_distance = msg->ranges[k+214];
            }
            if(centro>20){
                front_wall = true;
            }
            
        }

        // El barrido es de 128 y como no me importan los obstaculos superados, pues el barrido es de -90 a 90
        if(msg->ranges[k + array_size - 1 - 86 - 171]<0.5){
            izquierda++;
            if(izquierda>10){
                left_wall = true;
            }
        }
    }

    this->creating_players_data();

    this->computing_data_for_FSM();
}






