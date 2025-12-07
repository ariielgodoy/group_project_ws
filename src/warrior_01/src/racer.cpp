//--------------------------------------------------------------------//
//University of Málaga
//MAPIR Research Group - Machine Perception and Intelligent Robotics
//--------------------------------------------------------------------//

#include "warrior_01/racer.hpp"

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
    const Json::Value &skills_pos = JsonSceneData["FOV"]["Coins_Positions"];
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

    if(!trajectory_calculated && !skills_pos_array.empty() && battery > 97){
        trajectory = this->trajectory_computation();
        trajectory_calculated = true;
        
        for (const auto &coins_pos : trajectory)
        {   RCLCPP_INFO(this->get_logger(), "[X] = '%f', [Y] = '%f'", coins_pos[0], coins_pos[1]);  }
    }


    //RCLCPP_INFO(this->get_logger(), "Msg: '%s'", msg->data.c_str());
    if (!skills_pos_array.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Skills Positions Array:");
        for (const auto &skill_pos : skills_pos_array)
        {   RCLCPP_INFO(this->get_logger(), "[X] = '%f', [Y] = '%f'", skill_pos[0], skill_pos[1]);  }
    }
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

std::vector<std::vector<float>> Warrior::trajectory_computation()
{
    //Algoritmo de nearest neighbour
    size_t j;
    size_t position_closest_distance;
    float simulated_x_position = pos_x;
    float simulated_y_position = pos_y;
    std::vector<std::vector<float>> fixed_skills_pos_array = skills_pos_array;
    std::vector<std::vector<float>> path;

    if (!skills_pos_array.empty()){

        std::vector<float> distances_coins;
        while(!fixed_skills_pos_array.empty()){
            distances_coins.clear();
            float closest_distance_coins = 500;
            for (const auto &skill_pos : fixed_skills_pos_array)  
            {
                float x = skill_pos[0];
                float y = skill_pos[1];
                float euclidean_distance_coins = compute_euclidean_distance(simulated_x_position, x, simulated_y_position, y);
                distances_coins.push_back(euclidean_distance_coins); //No hace falta usar el contador para acceder a las posiciones
                //Y no va a haber el problema del overflow que habia antes
            }
            //Fin del calculo de las distancias euclideas
            //Inicio del calculo de la trayectoria
            for(j = 0; j < distances_coins.size(); j++){
                if(closest_distance_coins>distances_coins[j])
                {
                    closest_distance_coins = distances_coins[j];;
                    position_closest_distance = j;
                }
            }
            path.push_back(fixed_skills_pos_array[position_closest_distance]);
            simulated_x_position = path.back()[0];
            simulated_y_position = path.back()[1];
            fixed_skills_pos_array.erase(fixed_skills_pos_array.begin() + position_closest_distance);
        }   
    }
    return path;
}


float Warrior::PID_for_aiming(float angle){

    float Kp = 2; //2 ha ido bien
    float P = Kp * angle;


    return P;
}


std::vector<float> Warrior::euclidean_distance_and_angle_to_coin(bool to_coin_or_to_battery, int coin_battery_position){

    if(trajectory.empty()) return {};

    std::vector<float> datos;
    float close_enough;
    float below;

    float x_coin, y_coin;


    if(to_coin_or_to_battery){
        x_coin = trajectory[coin_battery_position][0];
        y_coin = trajectory[coin_battery_position][1];
    }
    else{
        x_coin = chargers_pos_array[coin_battery_position][0];
        y_coin = chargers_pos_array[coin_battery_position][1];
    }

    float euclidean_distance_coins = compute_euclidean_distance(x_coin, pos_x, y_coin, pos_y);
    float x_for_the_angle_coins = x_coin - pos_x;
    float y_for_the_angle_coins = y_coin - pos_y;

    float angle_coins = atan2(y_for_the_angle_coins, x_for_the_angle_coins);
    


    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ////// En vez de hacerlo con los if, lo he cambiado para calcularlo con atan2, que funciona mejor//
    ////// Ya gira en la direccion mas corta para dirigirse a la moneda////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////


    float angle_diff = angle_coins - gamma;
    float computing_real_angle_diff = atan2(sin(angle_diff), cos(angle_diff));

    if (!to_coin_or_to_battery && fabs(computing_real_angle_diff) > (80.0 * M_PI / 180.0)) {
        RCLCPP_INFO(this->get_logger(), 
                    "Ángulo muy grande hacia cargador, salto a moneda");
        return euclidean_distance_and_angle_to_coin(true, 0); 
    }


    if(fabs(computing_real_angle_diff) < 0.26){ //Si el valor absoluto de la diferencia de angulo esta entre -0.14 radianes y 0.14 radianes
                                                //Enviamos close enough y se empieza a calcular el PD para que se mueva el robot y no se pare
                                                // para reajustar la direccion
        close_enough = 1;
        below = 2;
    }else if(computing_real_angle_diff > 0){
        below = 1;
        close_enough = 0;
    }else{
        below = 0;
        close_enough = 0;
    }

    if(euclidean_distance_coins < 0.3 && to_coin_or_to_battery){
        trajectory.erase(trajectory.begin());
    }

    datos.push_back(euclidean_distance_coins);
    datos.push_back(below);
    datos.push_back(close_enough);
    datos.push_back(computing_real_angle_diff);
    return datos;
}


std::pair<bool, int> Warrior::choosing_coin_or_battery() {
    bool to_coin_or_to_battery = true; // true es moneda y false es bateria
    int battery_to_follow = 0;

    if (!chargers_pos_array.empty()) {
        float distance_to_battery;
        int i = 0;

        for (const auto &charger_pos : chargers_pos_array) {
            float x_battery = charger_pos[0];
            float y_battery = charger_pos[1];

            
            distance_to_battery = this->compute_euclidean_distance(x_battery, pos_x, y_battery, pos_y);

            // La logica de decisión
            if (distance_to_battery < 3 && battery < 50 && distance_to_battery > 0.1) {
                to_coin_or_to_battery = false;
                RCLCPP_INFO(this->get_logger(), "Al cargador");
                battery_to_follow = i;
                break;
            }
            i++;
        }
    }

    return {to_coin_or_to_battery, battery_to_follow}; 
}

/////////////////////////////////////////////////////////////////////////////
//////////POR AHORA FUNCIONA BIEN////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
void Warrior::perform_movement(bool MOVE_TO_GOAL, bool OBSTACLE_FOUND, float close_enough_avoiding, float below_avoiding, bool go_back, float angle)
{
    auto result = this->choosing_coin_or_battery();
    bool to_coin_or_to_battery = result.first; //true es moneda y false es bateria
    int battery_to_follow = result.second;
    
    std::vector<float> datos = this->euclidean_distance_and_angle_to_coin(to_coin_or_to_battery, battery_to_follow); //Para calcular la velocidad si esta cerca o lejos
    if (!datos.empty()){
        float closest_distance_to_skill = datos[0];
        float below = datos[1];
        float close_enough = datos[2];
        float angle_for_coin_charger = datos[3];

        rosgame_msgs::msg::RosgameTwist movement;


        // Reglas de movimiento referente a la distancia con las monedas
        if(MOVE_TO_GOAL){
            movement.vel.linear.x = 0.2;
            RCLCPP_INFO(this->get_logger(),"Goin' for the Gs homie");
            if (below == 1){
                if(closest_distance_to_skill < 1 && fabs(angle_for_coin_charger) > 45*2*M_PI/360){
                    movement.vel.linear.x = 0;
                }
                movement.vel.angular.z = this->PID_for_aiming(angle_for_coin_charger)/6;
                RCLCPP_INFO(this->get_logger(), "Ajustando para ir arriba");
            }else if(below == 0){
                if(closest_distance_to_skill < 1.5 && fabs(angle_for_coin_charger) > 45*2*M_PI/360){
                    movement.vel.linear.x = 0;
                }
                movement.vel.angular.z = this->PID_for_aiming(angle_for_coin_charger)/6;
                RCLCPP_INFO(this->get_logger(), "Ajustando para ir hacia abajo");
            }
            else if(close_enough == 1){
                RCLCPP_INFO(this->get_logger(), "Suficientemente cercano");
                movement.vel.linear.x = 0.8;
                if(closest_distance_to_skill < 2){
                    movement.vel.linear.x = 0.5 - ((4-2*closest_distance_to_skill)/10);
                }
                movement.vel.angular.z = this->PID_for_aiming(angle_for_coin_charger);
            }


            RCLCPP_INFO(this->get_logger(), "Close enough: %f", close_enough);
            RCLCPP_INFO(this->get_logger(), "Below: %f", below);
            RCLCPP_INFO(this->get_logger(), "Bateria: %f", battery);
        }
        else if(OBSTACLE_FOUND){
            if(go_back){
                movement.vel.linear.x = -0.5;
                movement.vel.angular.z = this->PID_for_aiming(angle)/3;
            }
            else{
                movement.vel.linear.x = 0.3;
                RCLCPP_INFO(this->get_logger(), "Should be avoiding");
                if (below_avoiding == 1){
                    movement.vel.angular.z = this->PID_for_aiming(angle)/5;
                    RCLCPP_INFO(this->get_logger(), "Ajustando para ir arriba(obst)");
                }else if(below_avoiding == 0){
                    movement.vel.angular.z = this->PID_for_aiming(angle)/5;
                    RCLCPP_INFO(this->get_logger(), "Ajustando para ir hacia abajo(obst)");
                }

                if(close_enough_avoiding == 1){
                    RCLCPP_INFO(this->get_logger(), "Suficientemente cercano(obst)");
                    movement.vel.linear.x = 0.4;
                    movement.vel.angular.z = this->PID_for_aiming(angle)*1.4;
                }
            }
        }


        // Publicar
        movement.code = code;
        this->pub1_->publish(movement);
    }

}



float Warrior::encontrarCercanoNoObstaculo(const std::vector<int>& obstacles, int search_index_best, const sensor_msgs::msg::LaserScan::SharedPtr msg, bool right_wall, bool left_wall, bool front_wall){

    double angle_min = msg->angle_min;
    std::unordered_set<int> obstacleSet(obstacles.begin(), obstacles.end());
    double angle_increment = msg->angle_increment;
    int search_index = (684/2 + search_index_best)/2;

    int punto_libre_cercano = -1;
    size_t delta = 1;
    size_t array_size = msg->ranges.size();

    while (delta < array_size) { // Límite de seguridad
        size_t numUp = search_index + delta;
        size_t numDown = search_index - delta;

        // El 86 es solo para mirar maximo a los lados, nos dan igual los muros de detras

        if (numUp < array_size - 86 && obstacleSet.find(numUp) == obstacleSet.end()) {
            punto_libre_cercano = numUp;
            break;
        }

        if (numDown >= 86 && obstacleSet.find(numDown) == obstacleSet.end()) {
            punto_libre_cercano = numDown;
            break;
        }

        delta++;
    }

    if (punto_libre_cercano == -1) {
        RCLCPP_INFO(this->get_logger(), "No hay camino libre.");
        search_index = -404;
        return search_index; //no se encontró camino libre (todo bloqueado)
        //AQUI DEBERIAMOS TIRAR PARA ATRAS
    }

    //Encontrar los Límites del Hueco que contiene a PuntoInicialLibre

    size_t limite_derecho = punto_libre_cercano;
    while (limite_derecho > 86 && obstacleSet.find(limite_derecho - 1) == obstacleSet.end()) {
        limite_derecho--;
    }
    
    // Límite izquierdo (Buscando el obstáculo y el borde del sensor hacia arriba)
    size_t limite_izquierdo = punto_libre_cercano;
    // Mientras no sea el último índice Y el índice siguiente esté libre
    while (limite_izquierdo < array_size - 86 && obstacleSet.find(limite_izquierdo + 1) == obstacleSet.end()) {
        limite_izquierdo++;
    }

    // 4. Calcular el Centro del Hueco y devolverlo
    int centro_del_hueco = (limite_izquierdo + limite_derecho) / 2;
    float angulo_a_seguir = centro_del_hueco*angle_increment+angle_min;

    RCLCPP_INFO(this->get_logger(),"Por el centro hasta que se diga lo contrario");

    float angle_wall;
    if(limite_izquierdo == msg->ranges.size()-86 or (right_wall && front_wall)){
        RCLCPP_INFO(this->get_logger(),"Siguiendo el muro");
        RCLCPP_INFO(this->get_logger(),"Muro derecho");
        float ultimo_punto_ocupado_rango = msg->ranges[limite_derecho-12];
        float penultimo_punto_ocupado_rango = msg->ranges[limite_derecho-25];
        float ultimo_punto_ocupado_angulo = (limite_derecho-12)*angle_increment - 86*angle_increment;
        float penultimo_punto_ocupado_angulo = (limite_derecho-25)*angle_increment - 86*angle_increment;

        //lo convierto a coordenadas locales del robot
        float y_ultimo = ultimo_punto_ocupado_rango * cos(ultimo_punto_ocupado_angulo);
        float y_penultimo = penultimo_punto_ocupado_rango * cos(penultimo_punto_ocupado_angulo);
        float x_ultimo = ultimo_punto_ocupado_rango * sin(ultimo_punto_ocupado_angulo);
        float x_penultimo = penultimo_punto_ocupado_rango * sin(penultimo_punto_ocupado_angulo);

        
        float m_num = (x_ultimo-x_penultimo);
        float m_den = (y_ultimo-y_penultimo);
        angle_wall = atan(m_num/m_den);

        RCLCPP_INFO(this->get_logger(), "Muro derecho, angulo calculado: %f", angle_wall*360/(2*M_PI));

        if(angle_wall<0){
            angulo_a_seguir = angle_wall + M_PI/2 + 20*2*M_PI/360;
        }else{
            angulo_a_seguir = angle_wall - M_PI/2 + 20*2*M_PI/360;
        }
        

    }else if(limite_derecho <= 86 or (left_wall && front_wall)){
        RCLCPP_INFO(this->get_logger(),"Siguiendo el muro");
        RCLCPP_INFO(this->get_logger(),"Muro izquierdo");
        float ultimo_punto_ocupado_rango = msg->ranges[limite_izquierdo+12];
        float penultimo_punto_ocupado_rango = msg->ranges[limite_izquierdo+25];
        float ultimo_punto_ocupado_angulo = ((limite_izquierdo+12)*angle_increment - 86*angle_increment);
        float penultimo_punto_ocupado_angulo = ((limite_izquierdo+25)*angle_increment - 86*angle_increment);

        float y_ultimo = ultimo_punto_ocupado_rango * cos(ultimo_punto_ocupado_angulo);
        float y_penultimo = penultimo_punto_ocupado_rango * cos(penultimo_punto_ocupado_angulo);
        float x_ultimo = ultimo_punto_ocupado_rango * sin(ultimo_punto_ocupado_angulo);
        float x_penultimo = penultimo_punto_ocupado_rango * sin(penultimo_punto_ocupado_angulo);

        //lo convierto a coordenadas locales del robot
        float m_den = (y_ultimo-y_penultimo);
        float m_num = (x_ultimo-x_penultimo);

        angle_wall = atan(m_num/m_den);
        RCLCPP_INFO(this->get_logger(), "Muro izquierdo, angulo calculado: %f", angle_wall*360/(2*M_PI));
        if(angle_wall>0){
            angulo_a_seguir = angle_wall - M_PI/2 - 20*2*M_PI/360;
        }else{
            angulo_a_seguir = angle_wall + M_PI/2 - 20*2*M_PI/360;
        }
    }

    return angulo_a_seguir;
}


void Warrior::erasing_accidentally_taken_coins(){
    if(!trajectory.empty() && !skills_pos_array.empty() && skills_pos_array.size() != trajectory.size()){
        const float COIN_MATCH_THRESHOLD = 0.1f; //Por el error de los float
        std::vector<std::vector<float>> new_trajectory;

        for (const auto &planned_coin : trajectory) 
        {
            bool coin_still_exists = false;


            for (const auto &actual_coin : skills_pos_array) 
            {
                float distance = this->compute_euclidean_distance(planned_coin[0], actual_coin[0], planned_coin[1], actual_coin[1]);

                if (distance < COIN_MATCH_THRESHOLD) {
                    coin_still_exists = true;
                    break;
                }
            }
            
            //si la moneda existe en el inventario, la incluimos en el nuevo plan.
            if (coin_still_exists) {
                new_trajectory.push_back(planned_coin);
            } 
            // Si no existe, fue recogida y la omitimos (la eliminamos del plan).
        }

        // Reemplazamos el plan antiguo con el nuevo plan limpio.
        if (trajectory.size() != new_trajectory.size()) {
            RCLCPP_WARN(this->get_logger(), "Sincronización: Se eliminaron %zu monedas del plan de ruta.", 
                        trajectory.size() - new_trajectory.size());
        }
        trajectory = move(new_trajectory); // Para pasar de un sitio a otro
    }
}


void Warrior::calculating_if_better_trajectory(){
    if(trajectory.size() > 1){
        int best_candidate_index = 0;
        for(size_t i = 1; i < trajectory.size(); i++){
            float new_x = trajectory[i][0];
            float new_y = trajectory[i][1];

            float current_x = trajectory[0][0];
            float current_y = trajectory[0][1];

            float new_star_euclidean_distance = this->compute_euclidean_distance(new_x, pos_x, new_y, pos_y);
            float current_star_euclidean_distance = this->compute_euclidean_distance(current_x, pos_x, current_y, pos_y);


            if(new_star_euclidean_distance < current_star_euclidean_distance - 1){
                best_candidate_index = i;
            }
        }
        if(best_candidate_index != 0){
            swap(trajectory[0], trajectory[best_candidate_index]);
            RCLCPP_INFO(this->get_logger(), "Trayectoria reordenada");
        }
    }
}


void Warrior::process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{

    int array_size = msg->ranges.size();
    double angle_increment = msg->angle_increment;


    //Flags para el programa de movimiento
    
    double front_distance = msg->range_max;
    int start_sweep;
    int end_sweep;
    std::vector<int> obstacles;
    float angle_to_move;

    float close_enough = 0;
    float below = 0;

    bool go_back = false;

    //Aqui busco el angulo de la moneda con respecto al robot para saber si esa parte esta ocupada
    if(!trajectory.empty()){

        this->erasing_accidentally_taken_coins();

        if(trajectory.empty()){
            return;
        }

        this->calculating_if_better_trajectory();

        float desired_x = trajectory[0][0]; //Aqui debo poner la moneda o la bateria
        float desired_y = trajectory[0][1];
        float angle_to_objective = atan2(desired_y - pos_y, desired_x - pos_x);
        float angle_diff = angle_to_objective - gamma;
        float computing_real_angle_diff = atan2(sin(angle_diff), cos(angle_diff));

        float euclidean_distance_coin = sqrt((desired_x-pos_x)*(desired_x-pos_x) + (desired_y-pos_y)*(desired_y-pos_y));
        bool obstacle_detected_in_path = false;

        // Este es el indice del laser que se supone que debe detectar el laser para saber si hay obstaculo en el camino
        int search_index = array_size/2 + computing_real_angle_diff/angle_increment;
        if(search_index-20 < 0){
            start_sweep = 0;
            RCLCPP_INFO(this->get_logger(), "Fuera del rango del laser");
        }else{
            start_sweep = search_index-20;
        }

        if(search_index + 20 > array_size){
            end_sweep = array_size - 1;
            RCLCPP_INFO(this->get_logger(), "Fuera del rango del laser");
        }else{
            end_sweep = search_index + 20;
        }
        int izquierda = 0;
        int centro = 0;
        int derecha = 0;

        for(int i = start_sweep; i < end_sweep; i++){ 
            if(msg->ranges[i] < euclidean_distance_coin && msg->ranges[i] < 2){
                obstacle_detected_in_path = true;
                break; // Detener en cuanto se encuentra el primero
            }
        }

        bool front_wall = false;
        bool left_wall = false;
        bool right_wall = false;

        
        //Mirar a los lados y al centro
        // miraba antes 30 grados (171), ahora miro 45 grado (214)
        for(int k = 86; k < 257; k++){
            if(msg->ranges[k]<0.5){
                derecha++;
                if(derecha>10){
                    obstacle_detected_in_path = true;
                    right_wall = true;
                }
                
            }


            if(msg->ranges[k+array_size/2 - 86 - 86] < 0.5){
                centro++;
                if(front_distance > msg->ranges[k+214]){
                    front_distance = msg->ranges[k+214];
                }
                if(centro>20){
                    obstacle_detected_in_path = true;
                    front_wall = true;
                }
                
            }

            // El barrido es de 128 y como no me importan los obstaculos superados, pues el barrido es de -90 a 90
            if(msg->ranges[k + array_size - 1 - 86 - 171]<0.5){
                izquierda++;
                if(izquierda>10){
                    obstacle_detected_in_path = true;
                    left_wall = true;
                }
            }
        }

        /////////////////////////////////////////////////////////////////
        OBSTACLE_FOUND = obstacle_detected_in_path;
        MOVE_TO_GOAL = !obstacle_detected_in_path;
        

        if(OBSTACLE_FOUND){
            const int MIN_OBSTACLE_POINTS = 10; 

            //Identificar y agrupar obstaculos potenciales
            std::vector<int> current_cluster;

            for(int j = 0; j < array_size; j++){
                
                bool is_obstacle_point = (msg->ranges[j] < euclidean_distance_coin && msg->ranges[j] < 1.1);

                if(is_obstacle_point){
                    current_cluster.push_back(j);
                } else {
                    if(current_cluster.size() >= MIN_OBSTACLE_POINTS){
                        obstacles.insert(obstacles.end(), current_cluster.begin(), current_cluster.end());
                    }
                    current_cluster.clear();
                }
            }

            if(current_cluster.size() >= MIN_OBSTACLE_POINTS){
                obstacles.insert(obstacles.end(), current_cluster.begin(), current_cluster.end());
            }

           
            angle_to_move = encontrarCercanoNoObstaculo(obstacles, search_index, msg, right_wall, left_wall, front_wall);

            RCLCPP_INFO(this->get_logger(), "Angulo_elegido: %f", angle_to_move*360/(2*M_PI));

            
            if(angle_to_move == -404 or front_distance < 0.45){
                go_back = true;
            }
            else if(fabs(angle_to_move) < 0.2){ //Si el valor absoluto de la diferencia de angulo esta entre -0.2 radianes y 0.2 radianes
                                                        //Enviamos close enough y se empieza a calcular el PD para que se mueva el robot y no se pare
                                                        // para reajustar la direccion
                close_enough = 1;
                below = 2;
            }else if(angle_to_move > 0){
                below = 1;
                close_enough = 0;
            }else{
                below = 0;
                close_enough = 0;
            }

            if(trajectory.size() == 1){
                recalcular = true;
            }

        }

    }
    else if((recalcular or trajectory.empty()) && trajectory_calculated){
        trajectory = this->trajectory_computation();
        recalcular = false;
        RCLCPP_INFO(this->get_logger(), "Skills Positions Array:");
        for (const auto &skill_pos : trajectory)
        {   RCLCPP_INFO(this->get_logger(), "[X] = '%f', [Y] = '%f'", skill_pos[0], skill_pos[1]);  }
    }
    
    this->perform_movement(MOVE_TO_GOAL, OBSTACLE_FOUND, close_enough, below, go_back, angle_to_move);
}






