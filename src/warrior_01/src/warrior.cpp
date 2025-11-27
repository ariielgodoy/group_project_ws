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
                sub1_ = create_subscription<sensor_msgs::msg::LaserScan>( "/" + code + "/laser_scan", 10, std::bind(&Warrior::process_laser_info, this, std::placeholders::_1));
                sub2_ = create_subscription<std_msgs::msg::String>( "/" + code + "/scene_info", 10, std::bind(&Warrior::process_scene_info, this, std::placeholders::_1));
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

    if(!trajectory_calculated && !skills_pos_array.empty() && battery > 70){
        trajectory = this->trajectory_computation();
        trajectory_calculated = true;
        
        for (const auto &coins_pos : trajectory)
        {   RCLCPP_INFO(this->get_logger(), "[X] = '%f', [Y] = '%f'", coins_pos[0], coins_pos[1]);  }
    }


    //RCL{CPP_INFO(this->get_logger(), "Msg: '%s'", msg->data.c_str());
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

std::vector<std::vector<float>> Warrior::trajectory_computation()
{
    // Nearest-neighbour algorithm for the path calculation
    float angle_coins;
    int j;
    int position_closest_distance;
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
    float Kd = 0.1; // 0.1 ha ido bien tmb
    float DELTA_T = 0.1;

    float angle_diff = (angle-gamma);
    float current_error = atan2(sinf(angle_diff), cosf(angle_diff));


    float P = Kp * current_error;

    float derivative = (current_error - previous_error) / DELTA_T;
    //float D = Kd * derivative;

    //previous_error = current_error;

    return P;//+D;
}


std::vector<float> Warrior::euclidean_distance_and_angle_to_coin(){

    if(trajectory.empty()) return {};

    std::vector<float> datos;
    float fixing_direction = 0;
    float close_enough;
    float below;
    float x_coin = trajectory[0][0];
    float y_coin = trajectory[0][1];

    float euclidean_distance_coins = compute_euclidean_distance(x_coin, pos_x, y_coin, pos_y);
    float x_for_the_angle_coins = x_coin - pos_x;
    float y_for_the_angle_coins = y_coin - pos_y;

    float angle_coins = atan2(y_for_the_angle_coins, x_for_the_angle_coins);


    float angle = angle_coins;
    

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ////// En vez de hacerlo con los if, lo he cambiado para calcularlo con atan2, que funciona mejor//
    ////// Ya gira en la direccion mas corta para dirigirse a la moneda////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////


    float angle_diff = angle - gamma;
    float computing_real_angle_diff = atan2(sin(angle_diff), cos(angle_diff));

    if(fabs(computing_real_angle_diff) < 0.14){ //Si el valor absoluto de la diferencia de angulo esta entre -0.14 radianes y 0.14 radianes
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

    if(close_enough == 1){
        fixing_direction = this->PID_for_aiming(angle);
    }else{
        previous_error = 0;
    }

    if(euclidean_distance_coins < 0.5){
        trajectory.erase(trajectory.begin());
    }

    datos.push_back(euclidean_distance_coins);
    datos.push_back(below);
    datos.push_back(close_enough);
    datos.push_back(fixing_direction);
    return datos;
}


/////////////////////////////////////////////////////////////////////////////
//////////POR AHORA FUNCIONA BIEN////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
void Warrior::perform_movement(bool MOVE_TO_GOAL, bool OBSTACLE_FOUND, float close_enough_avoiding, float below_avoiding, float fixing_direction_avoiding, float front_distance)
{
    std::vector<float> datos = this->euclidean_distance_and_angle_to_coin(); //Para calcular la velocidad si esta cerca o lejos
    if (!datos.empty()){
        float closest_distance_to_skill = datos[0];
        float below = datos[1];
        float close_enough = datos[2];
        float fixing_direction = datos[3];

        rosgame_msgs::msg::RosgameTwist movement;

        /////////////////////////////////////////////////////////////////////////////
        ///Se repite el patron de movimiento, por lo que hace falta un subprograma///
        /////////////////////////////////////////////////////////////////////////////

        // Reglas de movimiento referente a la distancia con las monedas
        if(MOVE_TO_GOAL){
            movement.vel.linear.x = 0.1;
            RCLCPP_INFO(this->get_logger(),"Goin' for the Gs homie");
            if (below == 1){
                if(closest_distance_to_skill < 2){
                    movement.vel.linear.x = 0;
                }
                movement.vel.angular.z = 0.2;
                RCLCPP_INFO(this->get_logger(), "Ajustando para ir arriba");
            }else if(below == 0){
                if(closest_distance_to_skill < 2){
                    movement.vel.linear.x = 0;
                }
                movement.vel.angular.z = -0.2;
                RCLCPP_INFO(this->get_logger(), "Ajustando para ir hacia abajo");
            }

            if(close_enough == 1){
                RCLCPP_INFO(this->get_logger(), "Suficientemente cercano");
                movement.vel.linear.x = 0.5;
                movement.vel.angular.z = fixing_direction;
            }

            RCLCPP_INFO(this->get_logger(), "Close enough: %f", close_enough);
            RCLCPP_INFO(this->get_logger(), "Below: %f", below);
            RCLCPP_INFO(this->get_logger(), "Bateria: %f", battery);
        }
        else if(OBSTACLE_FOUND){
            movement.vel.linear.x = 0.1;
            RCLCPP_INFO(this->get_logger(), "Should be avoiding");
            if (below_avoiding == 1){
                movement.vel.angular.z = 0.2;
                RCLCPP_INFO(this->get_logger(), "Ajustando para ir arriba nigga");
            }else if(below_avoiding == 0){
                movement.vel.angular.z = -0.2;
                RCLCPP_INFO(this->get_logger(), "Ajustando para ir hacia abajo nigga");
            }

            if(close_enough_avoiding == 1){
                RCLCPP_INFO(this->get_logger(), "Suficientemente cercano nigga");
                movement.vel.linear.x = 0.2;
                movement.vel.angular.z = fixing_direction_avoiding;
            }

        }


        // Publicar
        movement.code = code;
        this->pub1_->publish(movement);
    }

}



int Warrior::encontrarCercanoNoObstaculo(const std::vector<int>& obstacles, int search_index, const sensor_msgs::msg::LaserScan::SharedPtr msg){

    std::unordered_set<int> obstacleSet(obstacles.begin(), obstacles.end());
    double angle_increment = msg->angle_increment;
    search_index = 684/2;
    // 1. Verificar si el propio search_index está libre (máxima atracción)
    //if (obstacleSet.find(search_index) == obstacleSet.end()) {
    //    return search_index;
    //}

    // 2. Buscar el Primer Punto Libre (PuntoInicialLibre)
    int punto_libre_cercano = -1;
    int delta = 1;
    int array_size = msg->ranges.size();

    while (delta < array_size) { // Límite de seguridad
        int numUp = search_index + delta;
        int numDown = search_index - delta;

        // Opción 1: Hacia arriba
        if (numUp < array_size && obstacleSet.find(numUp) == obstacleSet.end()) {
            punto_libre_cercano = numUp;
            break;
        }

        // Opción 2: Hacia abajo
        if (numDown >= 0 && obstacleSet.find(numDown) == obstacleSet.end()) {
            punto_libre_cercano = numDown;
            break;
        }

        delta++;
    }

    if (punto_libre_cercano == -1) {
        return search_index; // No se encontró camino libre (todo bloqueado)
    }

    // 3. Encontrar los Límites del Hueco que contiene a PuntoInicialLibre

    // Límite derecho (Buscando el obstáculo y el borde del sensor hacia abajo)
    int limite_derecho = punto_libre_cercano;
    // Mientras no sea el índice 0 Y el índice anterior esté libre
    while (limite_derecho > 0 && obstacleSet.find(limite_derecho - 1) == obstacleSet.end()) {
        limite_derecho--;
    }
    
    // Límite izquierdo (Buscando el obstáculo y el borde del sensor hacia arriba)
    int limite_izquierdo = punto_libre_cercano;
    // Mientras no sea el último índice Y el índice siguiente esté libre
    while (limite_izquierdo < array_size - 1 && obstacleSet.find(limite_izquierdo + 1) == obstacleSet.end()) {
        limite_izquierdo++;
    }

    // 4. Calcular el Centro del Hueco y devolverlo
    int centro_del_hueco = (limite_izquierdo + limite_derecho) / 2;
    int angulo_a_seguir = centro_del_hueco*angle_increment+angle_min;

    float angle_to_follow;
    int index_to_follow = centro_del_hueco;
    if(limite_izquierdo == msg->ranges.size()-1 or limite_derecho == 0 or limite_izquierdo-limite_derecho < 90){
        //////////////////////////////////////////////////////////////
        //AQUI SE PODRIA PONER UNA FUNCION PARA NO REPETIR EL CODIGO//
        //////////////////////////////////////////////////////////////
        float angle_wall;
        RCLCPP_INFO(this->get_logger(),"Siguiendo el muro");
        if(limite_izquierdo == msg->ranges.size()-1){
            float ultimo_punto_ocupado_rango = msg->ranges[punto_libre_cercano-2];
            float penultimo_punto_ocupado_rango = msg->ranges[punto_libre_cercano-30];
            float ultimo_punto_ocupado_angulo = -(punto_libre_cercano-2)*angle_increment + M_PI/2;
            float penultimo_punto_ocupado_angulo = -(punto_libre_cercano-30)*angle_increment + M_PI/2;

            //lo convierto a coordenadas locales del robot
            float y_ultimo = ultimo_punto_ocupado_rango * sin(ultimo_punto_ocupado_angulo);
            float y_penultimo = penultimo_punto_ocupado_rango * sin(penultimo_punto_ocupado_angulo);
            float x_ultimo = ultimo_punto_ocupado_rango * cos(ultimo_punto_ocupado_angulo);
            float x_penultimo = penultimo_punto_ocupado_rango * cos(penultimo_punto_ocupado_angulo);

            
            float m_num = (x_ultimo-x_penultimo);
            float m_den = (y_ultimo-y_penultimo);
            if(m_num < 0){
                angle_wall = -angle_wall;
            }
            else if(m_den< 0){
                m_den = -m_den;
            }
            angle_wall = atan(m_num/m_den);


            angle_to_follow = angle_wall;

            

        }else if(limite_derecho == 0){
            float ultimo_punto_ocupado_rango = msg->ranges[punto_libre_cercano+2];
            float penultimo_punto_ocupado_rango = msg->ranges[punto_libre_cercano+30];
            float ultimo_punto_ocupado_angulo = (punto_libre_cercano+2)*angle_increment - M_PI/2;
            float penultimo_punto_ocupado_angulo = (punto_libre_cercano+30)*angle_increment - M_PI/2;

            float y_ultimo = ultimo_punto_ocupado_rango * sin(ultimo_punto_ocupado_angulo);
            float y_penultimo = penultimo_punto_ocupado_rango * sin(penultimo_punto_ocupado_angulo);
            float x_ultimo = ultimo_punto_ocupado_rango * cos(ultimo_punto_ocupado_angulo);
            float x_penultimo = penultimo_punto_ocupado_rango * cos(penultimo_punto_ocupado_angulo);

            //lo convierto a coordenadas locales del robot
            float m_den = (y_ultimo-y_penultimo);
            float m_num = (x_ultimo-x_penultimo);

            if(m_num < 0){
                angle_wall = -angle_wall;
            }
            else if(m_den > 0){
                m_den = -m_den;
            }
            angle_wall = atan(m_num/m_den);

            angle_to_follow = angle_wall;
        }
        
        if (angle_to_follow > M_PI) {
            angle_to_follow -= 2 * M_PI;
        } else if (angle_to_follow <= -M_PI) {
            angle_to_follow += 2 * M_PI;
        }
        angulo_a_seguir = angle_to_follow;
    }
    return angulo_a_seguir;
}

///////////////////////////////////////////////////////////////////////
//ESTO SE TIENE QUE MEJORAR, ES UN POCO CHURRO COMO LO HICE LA VERDAD//
/////////////////////////////////////////////////////////////////////////////////////
//AHORA ESTA QUITADO PORQUE QUERIA PROBAR COMO TIRABA SOLO CON LA NAVEGACION GLOBAL//
/////////////////////////////////////////////////////////////////////////////////////
void Warrior::process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{

    int array_size = msg->ranges.size();
    //double angle_min = msg->angle_min; //= 
    double angle_max = msg->angle_max; //= 
    double angle_min = msg->angle_min;
    double angle_increment = msg->angle_increment;


    //Flags para el programa de movimiento
    
    double front_distance = msg->range_max;
    int start_sweep;
    int end_sweep;
    std::vector<int> obstacles;
    int indice_libre;
    float angle_to_move;

    float close_enough = 0;
    float below = 0;
    float fixing_direction = 0;



    //Aqui busco el angulo de la moneda con respecto al robot para saber si esa parte esta ocupada
    if(!trajectory.empty()){
        float desired_x = trajectory[0][0];
        float desired_y = trajectory[0][1];

        float angle_to_objective = atan2(desired_y - pos_y, desired_x - pos_x);
        float angle_diff = angle_to_objective - gamma;
        float computing_real_angle_diff = atan2(sin(angle_diff), cos(angle_diff));

        float euclidean_distance_coin = sqrt((desired_x-pos_x)*(desired_x-pos_x) + (desired_y-pos_y)*(desired_y-pos_y));
        bool obstacle_detected_in_path = false;
        int search_index = array_size/2 + computing_real_angle_diff/angle_increment;
        if(search_index-20 < 0){
            start_sweep = 0;
            RCLCPP_INFO(this->get_logger(), "Fuera del rango del laser");
            obstacle_detected_in_path = true;
        }else{
            start_sweep = search_index-20;
        }

        if(search_index + 20 > array_size){
            end_sweep = array_size - 1;
            RCLCPP_INFO(this->get_logger(), "Fuera del rango del laser");
            obstacle_detected_in_path = true;
        }else{
            end_sweep = search_index + 20;
        }
        int izquierda = 0;
        int centro = 0;
        int derecha = 0;
        for(int k = 86; k <171; k++){
            if(msg->ranges[k]<0.7){
                derecha++;
                if(derecha>10){
                    obstacle_detected_in_path = true;
                }
                
            }


            if(msg->ranges[k+214] < 0.7){
                centro++;
                if(centro>10){
                    obstacle_detected_in_path = true;
                }
            }

            if(msg->ranges[k+427]<0.7){
                izquierda++;
                if(izquierda>10){
                    obstacle_detected_in_path = true;
                }
            }
        }
        ////////////////////////////////////////////////////////////////////////////
        
        
        // 2. Buscar obstáculos en el barrido central
        for(int i = start_sweep; i < end_sweep; i++){ 
            if(msg->ranges[i] < euclidean_distance_coin && msg->ranges[i] < 2){
                obstacle_detected_in_path = true;
                break; // Detener en cuanto se encuentra el primero
            }
        }
        /////////////////////////////////////////////////////////////////
        OBSTACLE_FOUND = obstacle_detected_in_path;
        MOVE_TO_GOAL = !obstacle_detected_in_path;
        

        if(OBSTACLE_FOUND){
            // Añade esta constante al inicio de process_laser_info o como una constante de clase
            const int MIN_OBSTACLE_POINTS = 10; 

            // --- Bloque de Relleno de Obstáculos Modificado ---

            // Paso 1: Identificar y agrupar obstáculos potenciales
            std::vector<int> current_cluster;
            //int array_size = msg->ranges.size(); // Asegúrate de usar el array_size correcto

            for(int j = 0; j < array_size; j++){
                
                // Condición: ¿Es un punto de obstáculo potencial?
                bool is_obstacle_point = (msg->ranges[j] < euclidean_distance_coin && msg->ranges[j] < msg->range_max);

                if(is_obstacle_point){
                    // Si es un obstáculo potencial, añadirlo al clúster actual
                    current_cluster.push_back(j);
                } else {
                    // Si no es un obstáculo (fin del clúster o ruido):
                    
                    // Paso 2: Verificar el tamaño del clúster anterior
                    if(current_cluster.size() >= MIN_OBSTACLE_POINTS){
                        // Si es suficientemente grande (ej. >= 10), es un obstáculo real.
                        // Añadir todos los índices del clúster a la lista final de obstáculos.
                        obstacles.insert(obstacles.end(), current_cluster.begin(), current_cluster.end());
                    }
                    
                    // Paso 3: Reiniciar el clúster para la próxima búsqueda
                    current_cluster.clear();
                }
            }

            // Paso 4: Manejar el último clúster (si el obstáculo llega al final del array)
            if(current_cluster.size() >= MIN_OBSTACLE_POINTS){
                obstacles.insert(obstacles.end(), current_cluster.begin(), current_cluster.end());
            }

            // --- Resto de process_laser_info continúa... ---
            if(!obstacles.empty()){
                for(int h = 0; h < obstacles.size(); h++){
                    RCLCPP_INFO(this->get_logger(), "%d", obstacles[h]);
                }
            }
           
            indice_libre = encontrarCercanoNoObstaculo(obstacles, search_index, msg);



            angle_to_move = indice_libre * angle_increment + angle_min;
            RCLCPP_INFO(this->get_logger(), "Angulo_elegido: %f", angle_to_move*360/(2*M_PI));


            /////////////////////////////////////////////////////////////////////////////////////////
            ////Codigo repetido, cambiar por subprograma/////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////


            float angle_difference = angle_to_move;
            float computing_real_angle_difference = atan2(sin(angle_difference), cos(angle_difference));

            if(fabs(computing_real_angle_difference) < 0.2){ //Si el valor absoluto de la diferencia de angulo esta entre -0.14 radianes y 0.14 radianes
                                                        //Enviamos close enough y se empieza a calcular el PD para que se mueva el robot y no se pare
                                                        // para reajustar la direccion
                close_enough = 1;
                below = 2;
            }else if(computing_real_angle_difference > 0){
                below = 1;
                close_enough = 0;
            }else{
                below = 0;
                close_enough = 0;
            }

            if(close_enough == 1){
                fixing_direction = this->PID_for_aiming(angle_difference);
            }

        }



    }
    RCLCPP_INFO(this->get_logger(), "close_enough: %f, below: %f", close_enough, below);
    this->perform_movement(MOVE_TO_GOAL, OBSTACLE_FOUND, close_enough, below, fixing_direction, front_distance);
}






