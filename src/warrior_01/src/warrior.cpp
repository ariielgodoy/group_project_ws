//--------------------------------------------------------------------//
//University of Málaga
//MAPIR Research Group - Machine Perception and Intelligent Robotics
//--------------------------------------------------------------------//

#include "warrior_01/warrior.hpp"

Warrior::Warrior(): Node ("robot_warrior")
{
    warrior_nick = "warrior_01";
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
                pub1_ = create_publisher<rosgame_msgs::msg::RosgameTwist>( "/" + code + "/cmd_vel", 1 );
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
    chargers_pos_array.clear();
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


    //RCLCPP_INFO(this->get_logger(), "Msg: '%s'", msg->data.c_str());
    /*RCLCPP_INFO(this->get_logger(), "Players Positions Array:");
    for (const auto &skill_pos : players_pos_array)
    {   RCLCPP_INFO(this->get_logger(), "Enemigos: [X] = '%f', [Y] = '%f'", skill_pos[0], skill_pos[1]);  }
       */
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

    float Kp = 0.7;
    float P = Kp * angle;

    return P;
}

float Warrior::P_for_velocity(float angle){

    const float V_MAX = 0.8;
    const float V_MIN = 0.3;


    const float K_REDUCTION = (V_MAX - V_MIN) / M_PI;

    float abs_angle = fabsf(angle);

    float P = V_MAX - K_REDUCTION * abs_angle;

    if (P < V_MIN){
        P = V_MIN;
    }
    
    return P;
}

void Warrior::perform_movement(float angle, float distance, float front_distance)
{
    rosgame_msgs::msg::RosgameTwist movement;
    movement.vel.linear.x = P_for_velocity(angle);

    if (current_state == State::RETREATING && distance < 3 && battery < 70){
        movement.vel.linear.x = 0.1;
    }

    if(front_distance < 0.6) movement.vel.linear.x = - 1.3;

    RCLCPP_INFO(this->get_logger(), "Angulo: %f", angle);
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
    double real_local_angle_diff = atan2(sin(local_angle_diff), cos(local_angle_diff));


    this->perform_movement(real_local_angle_diff, this->compute_euclidean_distance(x, pos_x, y, pos_y), 3);
}


void Warrior::most_density_of_resources(){
    const float HYSTERESIS_MARGIN = 0.5f; 
    
    float best_score = 1e9f;
    std::pair<float, float> new_best_resource_target = {0.0f, 0.0f}; 

    if (skills_pos_array.empty()) {
        RCLCPP_WARN(this->get_logger(), "No hay recursos detectados. Buscando al azar...");
        // Si no hay recursos, limpiamos el objetivo para el siguiente ciclo.
        this->current_resource_target = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
        return; 
    }
    
    // --- 1. APLICAR HISTERESIS AL OBJETIVO ACTUAL ---
    
    // Comprobación segura: ¿El objetivo actual está asignado (no es NAN)?
    bool current_target_valid = !std::isnan(this->current_resource_target.first);

    if (current_target_valid) { 
        float dist_to_current_target = compute_euclidean_distance(pos_x, this->current_resource_target.first, 
                                                                  pos_y, this->current_resource_target.second);
        
        // Damos una ventaja de 0.5m al objetivo actual para que sea más difícil cambiar.
        best_score = dist_to_current_target - HYSTERESIS_MARGIN; 
        new_best_resource_target = this->current_resource_target;
    }
    
    // Si NO es válido, best_score sigue siendo 1e9f, por lo que cualquier recurso ganará.
    
    // --- 2. BÚSQUEDA DEL RECURSO MÁS CERCANO (VORAZ) ---
    // ... (El resto del bucle de búsqueda) ...
    for (const auto& resource : skills_pos_array) {
        float res_x = resource[0];
        float res_y = resource[1];

        float current_distance_score = compute_euclidean_distance(pos_x, res_x, pos_y, res_y);

        if (current_distance_score < best_score) {
            best_score = current_distance_score;
            new_best_resource_target = {res_x, res_y};
        }
    }

    // --- 3. ACTUALIZAR ESTADO PERSISTENTE Y MOVER ---
    
    this->current_resource_target = new_best_resource_target;

    RCLCPP_INFO(this->get_logger(), "Recurso objetivo (Voraz con Histeresis) [X]: %f, [Y]:%f (Distancia efectiva: %f)", 
                new_best_resource_target.first, new_best_resource_target.second, best_score);
    this->computing_angle(new_best_resource_target);
}


std::pair<float, float> Warrior::get_nearest_enemy_pos(){
    float min_dist = 9999.0f;
    std::pair<float, float> nearest_coords = {0.0f, 0.0f};
    
    //Iteramos sobre todos los enemigos detectados
    for (const auto& enemy : players_pos_array) {
        
        float enemy_x = enemy[0];
        float enemy_y = enemy[1];

        RCLCPP_INFO(this->get_logger(), "enemigos [X]: %f, [Y]:%f", enemy_x, enemy_y);
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
        float diff_x = pos_x - enemy.first;
        float diff_y = pos_y - enemy.second;

        float magnitude = sqrt(diff_x * diff_x + diff_y * diff_y);

        if (magnitude > 0.1f) {
            safest_resource_target.first = pos_x + (diff_x / magnitude) * 10.0f;
            safest_resource_target.second = pos_y + (diff_y / magnitude) * 10.0f;
        }
    }
    RCLCPP_INFO(this->get_logger(), "Recurso objetivo [X]: %f , [Y]:%f", safest_resource_target.first, safest_resource_target.second);
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
        
        if (dist + 1 < min_dist) {
            min_dist = dist;
            nearest_charger = {charger_x, charger_y};
            charger_found = true;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Cargador objetivo [X]: %f , [Y]:%f", nearest_charger.first, nearest_charger.second);
    if (charger_found) {
        current_attack_target = nearest_charger;
        this->computing_angle(nearest_charger);
    }
}


void Warrior::crashing_into_weakest_enemy(){
    float min_dist_to_vulnerable = 9999.0f;
    std::pair<float, float> target_coords = {0.0f, 0.0f};
    bool found_vulnerable_target = false;


    for (const auto& pair : tracked_enemies) {
        const EnemyInfo& enemy = pair.second;
        
        if (!enemy.has_powerup) {
            
            float dist = compute_euclidean_distance(pos_x, enemy.x, pos_y, enemy.y);


            if (dist < min_dist_to_vulnerable) {
                min_dist_to_vulnerable = dist;
                target_coords = {enemy.x, enemy.y};
                found_vulnerable_target = true;
            }
        }
    }

    if (found_vulnerable_target) {
        current_attack_target = target_coords;
        this->computing_angle(target_coords);
        
    } 
    else {
        current_attack_target = {NAN, NAN};
        this->current_state = State::SEARCHING_RESOURCES;
    }
}

//Maquina de estados para escoger accion
void Warrior::FSM(int number_of_players_around, bool enemy_with_advantage_near) {
    if (battery < 40.0 && current_state != State::RETREATING) {
        current_state = State::RETREATING;
    }

    if(current_state != State::ENGAGING){
        current_attack_target = {NAN, NAN};
    }

    switch (current_state) {
        case State::SEARCHING_RESOURCES:
            RCLCPP_INFO(this->get_logger(), "Buscando recursos");
            this->most_density_of_resources();
            
            if ((hammer_enabled && shield_enabled) && !players_pos_array.empty()) 
                current_state = State::ENGAGING;
            else if (enemy_with_advantage_near) 
                current_state = State::SCAPING;
            else
                current_state = State::SEARCHING_RESOURCES;
            break;
        
        case State::SCAPING:
            RCLCPP_INFO(this->get_logger(), "Escape");
            this->going_for_safest_resource_escape(); 
            
            if (!enemy_with_advantage_near)
                current_state = State::SEARCHING_RESOURCES;
            break;

        case State::ENGAGING:
            RCLCPP_INFO(this->get_logger(), "Kiere sentirla en el pecho?");
            this->crashing_into_weakest_enemy();

            if (!hammer_enabled && !shield_enabled) {
                current_state = (number_of_players_around < 2 || !enemy_with_advantage_near) ? State::ENGAGING : State::SEARCHING_RESOURCES;
            }
            break;

        case State::RETREATING:
            RCLCPP_INFO(this->get_logger(), "A cargar");
            this->go_for_nearest_charger();
            if (battery > 90.0)
                current_state = State::SEARCHING_RESOURCES;
            break;
    }
}


void Warrior::computing_data_for_FSM(){
    int number_of_players_around = 0;
    bool enemy_with_advantage_near = false;
    const float DANGER_THRESHOLD = 4;

    for (const auto& pair : tracked_enemies) {
        const EnemyInfo& enemy = pair.second;
        
        float dist = this->compute_euclidean_distance(enemy.x, pos_x, enemy.y, pos_y);

        if (dist < DANGER_THRESHOLD) {
            number_of_players_around++;
  
            if (enemy.has_powerup) {
                enemy_with_advantage_near = true;
                break;
            }
        }
    }

    this->FSM(number_of_players_around, enemy_with_advantage_near);
}


bool Warrior::is_item_present_now(
    const std::vector<float>& last_item_pos, 
    const std::vector<std::vector<float>>& current_skills_list) 
{
    const float POSITION_TOLERANCE = 0.1f; 
    
    if (last_item_pos.size() < 2) {
        return false; 
    }

    float last_x = last_item_pos[0];
    float last_y = last_item_pos[1];

    for (const auto& current_pos : current_skills_list) {
        if (current_pos.size() < 2) continue; 

        float current_x = current_pos[0];
        float current_y = current_pos[1];

        float dist = compute_euclidean_distance(last_x, current_x, last_y, current_y);

        if (dist < POSITION_TOLERANCE) {
            return true; 
        }
    }

    return false;
}


void Warrior::process_enemy_and_item_data() {
    double now = this->get_clock()->now().seconds();
    float max_association_dist = 1.0f;
    float pickup_threshold = 0.8f;
    
    for (auto it = tracked_enemies.begin(); it != tracked_enemies.end(); ) {
        if (now - it->second.last_seen > 1.0) {
             it = tracked_enemies.erase(it);
        } else {
            if (it->second.has_powerup && now > it->second.powerup_expiry) {
                it->second.has_powerup = false;
            }
            ++it;
        }
    }


    const auto& last_skills = this->last_skills_pos_array; 
    
    for (const auto& pos : players_pos_array) {
        float new_x = pos[0];
        float new_y = pos[1];
        int associated_id = -1;
        float min_dist = max_association_dist;

        for (auto& [id, info] : tracked_enemies) {
            float dist = compute_euclidean_distance(new_x, info.x, new_y, info.y);
            if (dist < min_dist) {
                min_dist = dist;
                associated_id = id;
            }
        }


        if (associated_id != -1) {
            tracked_enemies[associated_id].x = new_x;
            tracked_enemies[associated_id].y = new_y;
            tracked_enemies[associated_id].last_seen = now;
        } else {
            EnemyInfo new_enemy;
            new_enemy.id = next_enemy_id++;
            new_enemy.x = new_x;
            new_enemy.y = new_y;
            new_enemy.last_seen = now;
            tracked_enemies[new_enemy.id] = new_enemy;
            associated_id = new_enemy.id;
        }

        if (associated_id != -1 && !tracked_enemies[associated_id].has_powerup) {
            
            for (const auto& last_skill_pos : last_skills) {

                if (!is_item_present_now(last_skill_pos, skills_pos_array)) {
                    
                    float dist_to_pickup_spot = compute_euclidean_distance(new_x, last_skill_pos[0], new_y, last_skill_pos[1]);
                    
                    if (dist_to_pickup_spot < pickup_threshold) {
                        tracked_enemies[associated_id].has_powerup = true;
                        tracked_enemies[associated_id].powerup_expiry = now + POWERUP_DURATION; 

                        break; 
                    }
                }
            }
        }
    }

    this->last_skills_pos_array = skills_pos_array;
}


void Warrior::process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    int array_size = msg->ranges.size();
    double angle_max = 120*2*M_PI/360;
    double angle_min = -120*2*M_PI/360;
    double angle_increment = (angle_max - angle_min) / array_size; 

    const double SAFETY_THRESHOLD = 2;
    const double SIDE_DANGER_DIST = 0.8; 
    const float ASSOCIATION_TOLERANCE = 0.8f;
    
    float min_front_dist = msg->range_max;
    float min_right_dist = msg->range_max;
    float min_left_dist = msg->range_max;
    
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float distance = msg->ranges[i];
        double current_angle = angle_min + (i * angle_increment);
        
        if (fabsf(current_angle) <= 0.7) {
            if (distance < min_front_dist) min_front_dist = distance;
        }
        
        else if (current_angle > 0.78 && current_angle <= 1.57) {
            if (distance < min_right_dist) min_right_dist = distance;
        }
        
        else if (current_angle >= -1.57 && current_angle < -0.78) {
            if (distance < min_left_dist) min_left_dist = distance;
        }
    }

    this->process_enemy_and_item_data(); 

    bool target_is_assigned = !std::isnan(current_attack_target.first);
    float precise_angle = NAN;
    float laser_dist_to_target = NAN;
    bool is_target_in_front = false;

    if (current_state == State::ENGAGING && target_is_assigned && compute_euclidean_distance(pos_x, current_attack_target.first, pos_y, current_attack_target.second)) {
        
        if (current_attack_target.first != 0.0f || current_attack_target.second != 0.0f) {
            
            float dist_global = compute_euclidean_distance(pos_x, current_attack_target.first, pos_y, current_attack_target.second);
            double target_global_angle = atan2(current_attack_target.second - pos_y, current_attack_target.first - pos_x);
            double target_local_angle = target_global_angle - gamma;
            target_local_angle = atan2(sin(target_local_angle), cos(target_local_angle));
            
            int target_index = (int)round((target_local_angle - angle_min) / angle_increment);
            
            if (target_index >= 0 && target_index < msg->ranges.size()) {
                float laser_dist = msg->ranges[target_index];
                
                if (laser_dist > 0.1f && fabsf(laser_dist - dist_global) < ASSOCIATION_TOLERANCE) {
                    laser_dist_to_target = laser_dist;
                    precise_angle = target_local_angle;
                    
                    is_target_in_front = (laser_dist_to_target < SAFETY_THRESHOLD);
                }
            }
        }
        
        if (!std::isnan(precise_angle)) {
            RCLCPP_INFO(this->get_logger(), "SEGUIMIENTO LÁSER ACTIVO. Cierre en: %f m", laser_dist_to_target);

            this->perform_movement(precise_angle, 3.0f, laser_dist_to_target);
        }
    }
    

    if (min_front_dist < SAFETY_THRESHOLD) {

        if (current_state == State::ENGAGING && is_target_in_front) {
            RCLCPP_INFO(this->get_logger(), "IGNORANDO OBSTÁCULO: ES EL ENEMIGO PARA COLISIÓN.");
            RCLCPP_INFO(this->get_logger(), "FSM");
            this->computing_data_for_FSM();
            return; 
        }

        RCLCPP_WARN(this->get_logger(), "EVASIÓN PROPORCIONAL. Bloqueo a %f m", min_front_dist);
        
        float turn_command = 0.0f;
        float repulse_diff = 0.0f;
        
        if (min_left_dist < SIDE_DANGER_DIST || min_right_dist < SIDE_DANGER_DIST) {
            repulse_diff = min_right_dist - min_left_dist;
        } else {
            repulse_diff = (min_right_dist > min_left_dist) ? 1.0f : -1.0f; 
        }

        const float Kp_repulsion = 0.8;
        if(fabs(turn_command) < M_PI/2){
            turn_command = Kp_repulsion * repulse_diff;
        }else if(turn_command>0){
            turn_command = M_PI/2;
        }else if(turn_command<0){
            turn_command = -M_PI/2;
        }

        this->perform_movement(turn_command, 3.0, min_front_dist);
        RCLCPP_INFO(this->get_logger(), "FSM");
        this->computing_data_for_FSM();
        return; 
    }
    

    RCLCPP_INFO(this->get_logger(), "FSM");
    this->computing_data_for_FSM();
}