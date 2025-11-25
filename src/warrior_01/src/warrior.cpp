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

                float euclidean_distance_coins = sqrt((x-simulated_x_position)*(x-simulated_x_position) + (y-simulated_y_position)*(y-simulated_y_position));
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
    float D = Kd * derivative;

    previous_error = current_error;

    return P+D;
}



std::vector<float> Warrior::euclidean_distance_and_angle_to_coin(){

    if(trajectory.empty()) return {};

    std::vector<float> datos;
    float fixing_direction = 0;
    float close_enough;
    float below;
    float x_coin = trajectory[0][0];
    float y_coin = trajectory[0][1];

    float euclidean_distance_coins = sqrt((x_coin-pos_x)*(x_coin-pos_x) + (y_coin-pos_y)*(y_coin-pos_y));
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
void Warrior::perform_movement(bool MOVE_TO_GOAL, bool OBSTACLE_FOUND, bool close_enough_avoiding, bool below_avoiding, float fixing_direction_avoiding, float front_distance)
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
            RCLCPP_INFO(this->get_logger(),"Goin' for the Gs homie");
            if (below == 1){
                movement.vel.angular.z = 0.2;
                RCLCPP_INFO(this->get_logger(), "Ajustando para ir arriba");
            }else if(below == 0){
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
            RCLCPP_INFO(this->get_logger(), "Should be avoiding");
            if (below_avoiding == 1){
                movement.vel.linear.x = 0.05;
                movement.vel.angular.z = 0.2;
                if(front_distance < 0.8){
                    movement.vel.linear.x = -0.05;
                    movement.vel.angular.z = 0.2;
                }
                RCLCPP_INFO(this->get_logger(), "Ajustando para ir arriba nigga");
            }else if(below_avoiding == 0){
                movement.vel.linear.x = 0.05;
                movement.vel.angular.z = -0.2;
                if(front_distance < 0.8){
                    movement.vel.linear.x = -0.05;
                    movement.vel.angular.z = -0.2;
                }
                RCLCPP_INFO(this->get_logger(), "Ajustando para ir hacia abajo nigga");
            }

            if(close_enough_avoiding == 1){
                RCLCPP_INFO(this->get_logger(), "Suficientemente cercano nigga");
                movement.vel.linear.x = 0.2;
                movement.vel.angular.z = fixing_direction_avoiding;//fixing_direction_avoiding;
            }

        }


        // Publicar
        movement.code = code;
        this->pub1_->publish(movement);
    }

}



int Warrior::encontrarCercanoNoObstaculo(const std::vector<int>& obstacles, int search_index){
    std::unordered_set<int> obstacleSet(obstacles.begin(), obstacles.end());

    int delta = 1;
    while (true) {
        // Opción 1: Hacia arriba
        int numUp = search_index + delta;
        // El método .find() en std::unordered_set devuelve .end() si no encuentra el elemento.
        if (obstacleSet.find(numUp) == obstacleSet.end()) {
            return numUp;
        }

        // Opción 2: Hacia abajo
        int numDown = search_index - delta;
        
        if (numDown >= 0 && obstacleSet.find(numDown) == obstacleSet.end()) {
            return numDown;
        }

        delta++;
    }
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

    /*int positions_to_look_sideways = (angle_max-M_PI/2)/angle_increment;
    int left_laser = array_size - 1 - positions_to_look_sideways;
    int right_laser = positions_to_look_sideways;
    int front_laser = array_size/2;
    int sweep = 228;//0.4/angle_increment; //~=22 grados pasados a
    //posiciones de array, ademas uso int para truncar el calculo

    //Parametros para los bucles
    double left_sweep_start = left_laser - sweep;
    double left_sweep_end = left_laser + sweep;

    double front_sweep_start = front_laser - sweep;
    double front_sweep_end = front_laser + sweep;

    double right_sweep_start = right_laser - sweep;
    double right_sweep_end = right_laser + sweep;*/

    //Flags para el programa de movimiento
    
    double front_distance = msg->range_max;
    int start_sweep;
    int end_sweep;
    std::vector<int> obstacles;
    int indice_libre;
    float angle_to_move;

    bool close_enough;
    bool below;
    bool fixing_direction;



    //Aqui busco el angulo de la moneda con respecto al robot para saber si esa parte esta ocupada
    if(!trajectory.empty()){
        float desired_x = trajectory[0][0];
        float desired_y = trajectory[0][1];

        float angle_to_objective = atan2(desired_y - pos_y, desired_x - pos_x);
        float angle_diff = angle_to_objective - gamma;
        float computing_real_angle_diff = atan2(sin(angle_diff), cos(angle_diff));

        float euclidean_distance_coin = sqrt((desired_x-pos_x)*(desired_x-pos_x) + (desired_y-pos_y)*(desired_y-pos_y));

        int search_index = array_size/2 + computing_real_angle_diff/angle_increment;
        if(search_index-12 < 0){
            start_sweep = 0;
        }else{
            start_sweep = search_index-12;
        }

        if(search_index + 12 > array_size){
            end_sweep = array_size;
        }else{
            end_sweep = search_index + 12;
        }

        if(msg->ranges[search_index] > msg->range_max){
            MOVE_TO_GOAL = true;
            OBSTACLE_FOUND = false;
        }
        for(int i = start_sweep; i < end_sweep; i++){ //Deberia hacer un subprograma para calcular los calculos intermedios
            if(msg->ranges[i] < euclidean_distance_coin && msg->ranges[i] < msg->range_max){
                MOVE_TO_GOAL = false;
                OBSTACLE_FOUND = true;
            }
        }

        if(OBSTACLE_FOUND){
            for(int j = 0; j < array_size; j++){
                if(msg->ranges[j] < euclidean_distance_coin && msg->ranges[j] < msg->range_max){
                    obstacles.push_back(j);
                }

                if(j > array_size/2-10 && j < array_size/2 + 10){
                    if(front_distance > msg->ranges[j]){
                        front_distance = msg->ranges[j];
                    }
                }
            }
            
            indice_libre = encontrarCercanoNoObstaculo(obstacles, search_index);


            ///////////////////////////////////////////////////////////////////////////////////////////////////////
            //Esta parte puede que ayude en el subprograma cuando solo haya una pared, pero ahora buscar el centro
            /*if(msg->ranges[indice_libre - 1]>msg->ranges[indice_libre + 1]){
                float y = -16*msg->ranges[indice_libre + 1] + 100;
                indice_libre = indice_libre + y;
            }else{
                float y = -16*msg->ranges[indice_libre - 1] + 100;
                indice_libre = indice_libre - y;
            }*/

            angle_to_move = indice_libre * angle_increment - angle_min;


            /////////////////////////////////////////////////////////////////////////////////////////
            ////Codigo repetido, cambiar por subprograma/////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////


            float angle_difference = angle_to_move - gamma;
            float computing_real_angle_difference = atan2(sin(angle_difference), cos(angle_difference));

            if(fabs(computing_real_angle_difference) < 0.4){ //Si el valor absoluto de la diferencia de angulo esta entre -0.14 radianes y 0.14 radianes
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
            }else{
                previous_error = 0;
            }

        }



    }
    
    this->perform_movement(MOVE_TO_GOAL, OBSTACLE_FOUND, close_enough, below, fixing_direction, front_distance);
}






