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
    //RCLCPP_INFO(this->get_logger(), "Msg: '%s'", msg->data.c_str());
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

std::vector<float> Warrior::euclidean_distance_and_angle_to_coins()
{
    float closest_distance;
    float angle = 0;
    float closest_distance_coins = 500;
    float angle_coins;
    float closest_distance_battery = 500;
    float angle_battery;
    float below = 0;
    float close_enough = 0;
    std::vector<float> datos;
    //skills_pos_array; En este array estaria bien saber si esta ordenado o no y como nos vienen esas posiciones
    //Es decir, si vienen en formato de distancia o en formato de x e y

    //Al tener la distancia euclidea y el angulo, ya podriamos saber si el robot esta realmente alineado con la recta que
    // une el robot con la moneda y podriamos computar la velocidad
    std::vector<float> distances_coins;

    if (!skills_pos_array.empty()) {
        for (const auto &skill_pos : skills_pos_array)  
        {
            float x = skill_pos[0];
            float y = skill_pos[1];

            float euclidean_distance_coins = sqrt((x-pos_x)*(x-pos_x) + (y-pos_y)*(y-pos_y));
            distances_coins.push_back(euclidean_distance_coins); //No hace falta usar el contador para acceder a las posiciones
            //Y no va a haber el problema del overflow que habia antes
        }
    

        for(int j = 0; j < distances_coins.size(); j++){
            if(closest_distance_coins>distances_coins[j])
            {
                closest_distance_coins = distances_coins[j];
                float x_for_the_angle_coins = skills_pos_array[j][0] - pos_x;
                float y_for_the_angle_coins = skills_pos_array[j][1] - pos_y;

                angle_coins = atan2(y_for_the_angle_coins, x_for_the_angle_coins);
            }
        }

        //Bateria
        std::vector<float> distances_battery;

        for (const auto &skill_pos_battery : chargers_pos_array)
        {
            float x = skill_pos_battery[0];
            float y = skill_pos_battery[1];

            float euclidean_distance_battery = sqrt((x-pos_x)*(x-pos_x) + (y-pos_y)*(y-pos_y));
            distances_battery.push_back(euclidean_distance_battery);
        }

        for(int i = 0; i < distances_battery.size(); i++){
            if(closest_distance_battery>distances_battery[i])
            {
                closest_distance_battery = distances_battery[i];
                float x_for_the_angle_battery = chargers_pos_array[i][0] - pos_x;
                float y_for_the_angle_battery = chargers_pos_array[i][1] - pos_y;

                angle_battery = atan2(y_for_the_angle_battery, x_for_the_angle_battery);
            }
        }
        closest_distance=closest_distance_coins;
        angle=angle_coins;
        if(battery<30){
            if(closest_distance_battery<closest_distance_coins){
                closest_distance=closest_distance_battery;
                angle=angle_battery;
                RCLCPP_INFO(this->get_logger(), "Go to a battery");
            }
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Go to a coin");
        }

        //Comparison with robots angle

        if(gamma < angle - 0.12){
            below = 1;
            close_enough = 0;
        }else if (gamma > angle + 0.12){//0.09
            below = 0;
            close_enough = 0;
        }else{
            close_enough = 1;
        }


        if((gamma > M_PI/2 && angle < M_PI/2) or (angle > M_PI/2 && gamma < M_PI/2)){
            if(gamma < angle - 0.12){
                below = 0;
                close_enough = 0;
            }else if (gamma > angle + 0.12){//0.09
                below = 1;
                close_enough = 0;
            }else{
                close_enough = 1;
            }
        }
        
    }
    RCLCPP_INFO(this->get_logger(), "Gamma: %f, Angle: %f", gamma, angle);
    datos.push_back(closest_distance);
    datos.push_back(below);
    datos.push_back(close_enough);
    return datos; // Below = 1 es que esta por debajo de la direccion y Below = 0 por encima
}

void Warrior::perform_movement(float front_distance, bool obstacle_front, bool obstacle_left, bool obstacle_right)
{
    std::vector<float> datos = this->euclidean_distance_and_angle_to_coins(); //Para calcular la velocidad si esta cerca o lejos
    float closest_distance_to_skill = datos[0];
    float below = datos[1];
    float close_enough = datos[2];

    rosgame_msgs::msg::RosgameTwist movement;
    
    // Valor por defecto
    //RCLCPP_INFO(this->get_logger(), "%f", closest_distance_to_skill);
    //movement.vel.linear.x = 0.2;
    

    
    // Reglas de movimiento referente a la distancia con las monedas

    if (below == 1){
        movement.vel.angular.z = 0.2;
        RCLCPP_INFO(this->get_logger(), "Ajustando para ir arriba");
    }else if(below == 0){
        movement.vel.angular.z = -0.2;
        RCLCPP_INFO(this->get_logger(), "Ajustando para ir hacia abajo");
    }

    if(close_enough == 1){
        movement.vel.angular.z = 0;
        RCLCPP_INFO(this->get_logger(), "Suficientemente cercano");
        movement.vel.linear.x = 0.5;
        /*if(closest_distance_to_skill>2 && closest_distance_to_skill<3){
            movement.vel.linear.x = 0.3;
        }else if (closest_distance_to_skill>0.8){
            movement.vel.linear.x = 0.2;
        }else{
            movement.vel.linear.x = 0.15;
        }*/
    }

    RCLCPP_INFO(this->get_logger(), "Close enough: %f", close_enough);
    RCLCPP_INFO(this->get_logger(), "Below: %f", below);
    RCLCPP_INFO(this->get_logger(), "Bateria: %f", battery);


    // Reglas de giro referente a los obstaculos
    if (obstacle_front && obstacle_right) {
        movement.vel.linear.x = 0.1;
        movement.vel.angular.z = 0.4;
        RCLCPP_INFO(this->get_logger(), "A la izquierda");
    }
    else if (obstacle_front && obstacle_left) {
        movement.vel.linear.x = 0.1;
        movement.vel.angular.z = -0.4;
        RCLCPP_INFO(this->get_logger(), "A la derecha");
    }
    else if (obstacle_front) {
        movement.vel.linear.x = 0;
        movement.vel.angular.z = -0.4;
        RCLCPP_INFO(this->get_logger(), "Atras");
    }


    

    // Publicar
    movement.code = code;
    this->pub1_->publish(movement);

}


void Warrior::process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    //Reutilizacion de programa de procesamiento de laser de la navegacion reactiva

    int array_size = msg->ranges.size();
    //double angle_min = msg->angle_min; //= 
    double angle_max = msg->angle_max; //= 
    double angle_increment = msg->angle_increment;

    int positions_to_look_sideways = (angle_max-M_PI/2)/angle_increment;
    int left_laser = array_size - 1 - positions_to_look_sideways;
    int right_laser = positions_to_look_sideways;
    int front_laser = array_size/2;
    int sweep = 0.4/angle_increment; //~=22 grados pasados a
    //posiciones de array, ademas uso int para truncar el calculo

    //Parametros para los bucles
    double left_sweep_start = left_laser - sweep;
    double left_sweep_end = left_laser + sweep;

    double front_sweep_start = front_laser - sweep;
    double front_sweep_end = front_laser + sweep;

    double right_sweep_start = right_laser - sweep;
    double right_sweep_end = right_laser + sweep;

    //Flags para el programa de movimiento
    bool obstacle_front = false;
    bool obstacle_left = false;
    bool obstacle_right = false;
    double front_distance = msg->range_max;


    //Aqui busco los obstaculos por la izquierda
    for(int i = left_sweep_start; i < left_sweep_end; i++){
        if(msg->ranges[i] < 1.5){
            obstacle_left = true;
            //RCLCPP_INFO(this->get_logger(), "IZQUIERDA");
        }
    }

    //Aqui busco los obstaculos de delante
    for(int j = front_sweep_start; j < front_sweep_end; j++){
        if(msg->ranges[j] < 1){
            obstacle_front = true;
            if(front_distance > msg->ranges[j]){
                front_distance = msg->ranges[j];
            }
        }
    }

    //Aqui busco los obstaculos de la derecha
    for(int k = right_sweep_start; k < right_sweep_end; k++){
        if(msg->ranges[k] < 1.5){
            obstacle_right = true;
            //RCLCPP_INFO(this->get_logger(), "DERECHA");
        }
    }

    this->perform_movement(front_distance, obstacle_front, obstacle_left, obstacle_right);

}





