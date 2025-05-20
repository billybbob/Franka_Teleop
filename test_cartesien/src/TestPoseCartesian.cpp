/*
* File: ros2_Pose_Haption.cpp
* Author: Jerome Perret (original), Modified with impedance logic for ROS2
* Copyright: Haption SA, 2021
* Note: Permission granted to copy, modify, redistribute, provided the above copyright is mentioned
*/

#include <iostream>
#include <chrono>
#include <thread>
#include <array>
#include <vector>
#include <cmath>

// Include the RaptorAPI
#include "RaptorAPI.hpp"

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

// Structure pour stocker les positions cartésiennes
struct CartesianPose {
    double x, y, z;         // Position
    double qx, qy, qz, qw;  // Quaternion pour orientation
};

float RAD2DEG(float a)
{
    return float((a / M_PI) * 180.0f);
}
float DEG2RAD(float a)
{
    return float((a * M_PI) / 180.0f);
}

class CartesianImpedanceHaption : public rclcpp::Node
{
public:
    CartesianImpedanceHaption(const std::string& device_param_file, const std::string& protocol_params, float weight = 0.0F) 
    : Node("Pose_Haption"), weight_(weight), inSpeedMode_(false), buttonPressDebounce_(0),
      in_pose_mode_(true), controller_pose_received_(false), robot_pose_received_(false),
      nbr_chgmt_mode_(1.0f), prev_middle_button_state_(false), scale_factor_(1.0f),
      prev_left_button_state_(false), prev_right_button_state_(false)
    {
        // Create publishers
        pose_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/valeur_effecteur", 10);
        force_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/valeur_force", 10);
        mode_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/Mode_Pose_Vitesse", 10);

        // Abonnement au topic force_guide
        force_guide_subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/force_guide", 10,
            std::bind(&CartesianImpedanceHaption::force_guide_callback, this, std::placeholders::_1)
        );
        
        // Nouvel abonnement au topic force_joystick
        force_joystick_subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/force_joystick", 10,
            std::bind(&CartesianImpedanceHaption::force_joystick_callback, this, std::placeholders::_1)
        );
        
        // Initialize the connection to the device
        raptorHandle_.DefineSpecificFields(protocol_params.c_str());
        HAPTION::ErrorCode error = raptorHandle_.Init(device_param_file.c_str());
        if (error != HAPTION::ErrorCode::E_NOERROR)
        {
            std::string dllVersion{""};
            if (raptorHandle_.GetParameterValue("RaptorAPI", "DeviceDLLVersion", dllVersion) == HAPTION::ErrorCode::E_NOERROR)
            {
                RCLCPP_INFO(this->get_logger(), "Device DLL version is: %s", dllVersion.c_str());
            }
            RCLCPP_ERROR(this->get_logger(), "HAPTION::RaptorAPI::Init() failed with error %d", (int)error);
            throw std::runtime_error("Failed to initialize Raptor device");
        }
        
        // Display successful connection
        std::string robotName{""};
        (void)raptorHandle_.GetFullName(robotName);
        RCLCPP_INFO(this->get_logger(), "Successfully connected to device %s", robotName.c_str());
        
        std::string dllVersion{""};
        if (raptorHandle_.GetParameterValue("RaptorAPI", "DeviceDLLVersion", dllVersion) == HAPTION::ErrorCode::E_NOERROR)
        {
            RCLCPP_INFO(this->get_logger(), "Device DLL version is: %s", dllVersion.c_str());
        }
        
        HAPTION::CalibrationStatus status;
        raptorHandle_.GetCalibrationStatus(status);
        if (status != HAPTION::CalibrationStatus::C_CALIBRATED)
        {
            RCLCPP_ERROR(this->get_logger(), "Please calibrate the device first!");
            raptorHandle_.Close();
            throw std::runtime_error("Device not calibrated");
        }
        
        // Authorize force-feedback
        error = raptorHandle_.ActivateForceFeedback(true);
        if (error != HAPTION::ErrorCode::E_NOERROR)
        {
            RCLCPP_ERROR(this->get_logger(), "HAPTION::RaptorAPI::ActivateForceFeedback() failed with error %d", (int)error);
            (void)raptorHandle_.Close();
            throw std::runtime_error("Failed to activate force feedback");
        }
        
        // Switch to cartesian mode initially (we start with nbr_chgmt_mode_ = 1 which is position mode)
        error = raptorHandle_.StartCartesianPositionMode();
        if (error != HAPTION::ErrorCode::E_NOERROR)
        {
            RCLCPP_ERROR(this->get_logger(), "HAPTION::RaptorAPI::StartCartesianPositionMode() failed with error %d", (int)error);
            (void)raptorHandle_.Close();
            throw std::runtime_error("Failed to start cartesian position mode");
        }
        
        // Set cartesian gains to zero
        raptorHandle_.ChangeCartesianGains(0.0F, 0.0F, 0.0F, 0.0F);
        
        // Create timer for periodic execution
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), 
            std::bind(&CartesianImpedanceHaption::timer_callback, this));
        
        // Initialize force feedback values
        externalForce_ = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
        joystickForce_ = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
        
        // Initialize count
        count_ = 0;
        
        // Initialize last step time
        lastStep_ = std::chrono::system_clock::now();
        
        // Définir l'état du mode en fonction de nbr_chgmt_mode_ (impair = position, pair = vitesse)
        updateModeBasedOnCounter();
        
        // Publish initial mode status
        publishModeStatus(true);
        
        RCLCPP_INFO(this->get_logger(), "System initialized in Position Mode with scale factor: %f", scale_factor_);
    }
    
    ~CartesianImpedanceHaption()
    {
        // Close down
        (void)raptorHandle_.Close();
        RCLCPP_INFO(this->get_logger(), "Closing device connection");
    }

private:
    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr force_guide_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr force_joystick_subscription;
    
    // Variables pour stocker les positions
    CartesianPose robot_pose_;        // Position de l'effecteur du robot
    CartesianPose controller_pose_;   // Position du contrôleur

    float nbr_chgmt_mode_;              // Compteur de changements de mode
    float scale_factor_;                // Facteur d'échelle contrôlé par les boutons gauche/droite
    bool prev_middle_button_state_;     // État précédent du bouton du milieu
    bool prev_left_button_state_;       // État précédent du bouton gauche
    bool prev_right_button_state_;      // État précédent du bouton droit

    // Flags pour suivre l'état des données
    bool in_pose_mode_;               // Mode position actif
    bool controller_pose_received_;   // Position du contrôleur reçue
    bool robot_pose_received_;        // Position du robot reçue
    
    // Variables pour stocker les forces externes reçues
    HAPTION::CartesianVector externalForce_;     // Forces du topic /force_guide
    HAPTION::CartesianVector joystickForce_;     // Forces du topic /force_joystick

    // Met à jour le mode en fonction de la valeur de nbr_chgmt_mode_
    void updateModeBasedOnCounter()
    {
        // Si nbr_chgmt_mode_ est impair, on est en mode position
        // Si nbr_chgmt_mode_ est pair, on est en mode vitesse
        bool shouldBeInSpeedMode = (static_cast<int>(nbr_chgmt_mode_) % 2 == 0);
        
        if (shouldBeInSpeedMode != inSpeedMode_) {
            if (shouldBeInSpeedMode) {
                switchToSpeedMode();
            } else {
                switchToPositionMode();
            }
        }
    }

    void publishModeStatus(bool forcePublish = false)
    {
        // Create message to publish the mode
        auto mode_message = std_msgs::msg::Float32MultiArray();
        
        // Set up dimensions for mode message
        mode_message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        mode_message.layout.dim[0].size = 4;  // Maintenant on a 4 valeurs au lieu de 3
        mode_message.layout.dim[0].stride = 1;
        mode_message.layout.dim[0].label = "mode";
        
        // Fill mode data - [0] pour position mode status, [1] pour speed mode status,
        // [2] pour nb changements de mode, [3] pour facteur d'échelle
        mode_message.data.clear();
        mode_message.data.push_back(inSpeedMode_ ? 0.0f : 1.0f);  // Position mode status
        mode_message.data.push_back(inSpeedMode_ ? 1.0f : 0.0f);  // Speed mode status
        mode_message.data.push_back(nbr_chgmt_mode_);             // Nombre de changements de mode
        mode_message.data.push_back(scale_factor_);               // Facteur d'échelle
        
        // Publish mode message
        mode_publisher_->publish(mode_message);
        
        if(forcePublish) {
            RCLCPP_INFO(this->get_logger(), "Current mode: %s, Mode changes: %f, Scale factor: %f", 
                   inSpeedMode_ ? "Speed Mode" : "Position Mode", nbr_chgmt_mode_, scale_factor_);
        }
    }
    
    void switchToSpeedMode()
    {
        if(!inSpeedMode_) {
            HAPTION::CartesianVector manipSpeed;
            HAPTION::ErrorCode error = raptorHandle_.SetCartesianSpeed(manipSpeed);
            if (error != HAPTION::ErrorCode::E_NOERROR) {
                RCLCPP_ERROR(this->get_logger(), "Failed to switch to Speed Mode with error %d", (int)error);
                return;
            }
            inSpeedMode_ = true;
            in_pose_mode_ = false;
            RCLCPP_INFO(this->get_logger(), "Switched to Speed Mode");
            publishModeStatus(true);
        }
    }
    
    void switchToPositionMode()
    {
        if(inSpeedMode_) {
            HAPTION::ErrorCode error = raptorHandle_.StartCartesianPositionMode();
            if (error != HAPTION::ErrorCode::E_NOERROR) {
                RCLCPP_ERROR(this->get_logger(), "Failed to switch to Position Mode with error %d", (int)error);
                return;
            }
            inSpeedMode_ = false;
            in_pose_mode_ = true;
            RCLCPP_INFO(this->get_logger(), "Switched to Position Mode");
            publishModeStatus(true);
        }
    }

    // Callback pour recevoir les valeurs de forces du topic /force_guide
    void force_guide_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // Récupérer les forces depuis le message
        if (msg->data.size() >= 6) {
            externalForce_.t_x = msg->data[0];  // Force en X
            externalForce_.t_y = msg->data[1];  // Force en Y
            externalForce_.t_z = msg->data[2];  // Force en Z
            externalForce_.r_x = msg->data[3];  // Moment en X
            externalForce_.r_y = msg->data[4];  // Moment en Y
            externalForce_.r_z = msg->data[5];  // Moment en Z
            
            RCLCPP_DEBUG(this->get_logger(), "Received force_guide: X=%f, Y=%f, Z=%f, RX=%f, RY=%f, RZ=%f",
                externalForce_.t_x, externalForce_.t_y, externalForce_.t_z,
                externalForce_.r_x, externalForce_.r_y, externalForce_.r_z);
        } else {
            RCLCPP_WARN(this->get_logger(), "Received force_guide message with invalid size: %zu", msg->data.size());
        }
    }
    
    // Callback pour recevoir les valeurs de forces du topic /force_joystick
    void force_joystick_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // Récupérer les forces depuis le message
        if (msg->data.size() >= 6) {
            joystickForce_.t_x = msg->data[0];  // Force en X
            joystickForce_.t_y = msg->data[1];  // Force en Y
            joystickForce_.t_z = msg->data[2];  // Force en Z
            joystickForce_.r_x = msg->data[3];  // Moment en X
            joystickForce_.r_y = msg->data[4];  // Moment en Y
            joystickForce_.r_z = msg->data[5];  // Moment en Z
            
            RCLCPP_DEBUG(this->get_logger(), "Received force_joystick: X=%f, Y=%f, Z=%f, RX=%f, RY=%f, RZ=%f",
                joystickForce_.t_x, joystickForce_.t_y, joystickForce_.t_z,
                joystickForce_.r_x, joystickForce_.r_y, joystickForce_.r_z);
        } else {
            RCLCPP_WARN(this->get_logger(), "Received force_joystick message with invalid size: %zu", msg->data.size());
        }
    }

    void timer_callback()
    {
        // Get current time
        auto now = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - lastStep_).count();
        if (duration < 1000)
        {
            return;
        }
        lastStep_ = now;
        
        // Call HAPTION::RaptorAPI::ReadState()
        std::chrono::system_clock::time_point timeStamp;
        HAPTION::ErrorCode error = raptorHandle_.ReadState(timeStamp, 0);
        if (error != HAPTION::ErrorCode::E_NOERROR)
        {
            if (error != HAPTION::ErrorCode::E_SAFETYSTOP_POWERBUTTON)
            {
                RCLCPP_ERROR(this->get_logger(), "HAPTION::RaptorAPI::ReadState() failed with error %d", (int)error);
                rclcpp::shutdown();
                return;
            }
        }

        // Get power status
        HAPTION::PowerStatus powerStatus;
        raptorHandle_.GetPowerStatus(powerStatus);

        // Get the current pose and speed of the force-feedback device
        HAPTION::Displacement manipPose;
        HAPTION::CartesianVector manipSpeed;
        (void)raptorHandle_.GetCartesianPose(manipPose);
        (void)raptorHandle_.GetCartesianSpeed(manipSpeed);

        // Update controller pose
        controller_pose_.x = manipPose.t_x;
        controller_pose_.y = manipPose.t_y;
        controller_pose_.z = manipPose.t_z;
        controller_pose_.qx = manipPose.q_x;
        controller_pose_.qy = manipPose.q_y;
        controller_pose_.qz = manipPose.q_z;
        controller_pose_.qw = manipPose.q_w;
        controller_pose_received_ = true;

        // Get joint angles if needed
        HAPTION::JointVector angles;
        raptorHandle_.GetJointAngles(angles);

        // Handle mode switching and scale factor adjustment using buttons
        bool leftButtonState = false;
        bool rightButtonState = false;
        bool middleButton = false;
        float valeurGachette = 0.0;

        raptorHandle_.GetOperatorButton(HAPTION::OperatorButton::B_LEFT, leftButtonState);
        raptorHandle_.GetOperatorButton(HAPTION::OperatorButton::B_RIGHT, rightButtonState);
        raptorHandle_.GetOperatorButton(HAPTION::OperatorButton::B_NEXT, middleButton);
        raptorHandle_.GetTriggerValue(valeurGachette);

        // Détection de front montant pour le bouton du milieu (mode)
        if (middleButton && !prev_middle_button_state_) {
            nbr_chgmt_mode_ += 1.0f;
            RCLCPP_INFO(this->get_logger(), "Middle button pressed, mode changes count: %f", nbr_chgmt_mode_);
            
            // Mise à jour du mode en fonction de la nouvelle valeur de nbr_chgmt_mode_
            updateModeBasedOnCounter();
        }
        prev_middle_button_state_ = middleButton;
        
        // Détection de front montant pour le bouton gauche (diminuer scale_factor_)
        if (leftButtonState && !prev_left_button_state_) {
            if (scale_factor_ > 0.1f) {
                scale_factor_ -= 0.1f;
                // Assurer une précision à 1 décimale
                scale_factor_ = std::round(scale_factor_ * 10.0f) / 10.0f;
                RCLCPP_INFO(this->get_logger(), "Left button pressed, scale factor decreased to: %f", scale_factor_);
                publishModeStatus(true);
            }
        }
        prev_left_button_state_ = leftButtonState;
        
        // Détection de front montant pour le bouton droit (augmenter scale_factor_)
        if (rightButtonState && !prev_right_button_state_) {
            if (scale_factor_ < 2.0f) {
                scale_factor_ += 0.1f;
                // Assurer une précision à 1 décimale
                scale_factor_ = std::round(scale_factor_ * 10.0f) / 10.0f;
                RCLCPP_INFO(this->get_logger(), "Right button pressed, scale factor increased to: %f", scale_factor_);
                publishModeStatus(true);
            }
        }
        prev_right_button_state_ = rightButtonState;

        // Create message to publish the pose
        auto pose_message = std_msgs::msg::Float32MultiArray();
        
        // Set up dimensions for pose
        pose_message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        pose_message.layout.dim[0].size = 8;
        pose_message.layout.dim[0].stride = 1;
        pose_message.layout.dim[0].label = "pose";
        
        // Fill pose data
        pose_message.data.clear();
        pose_message.data.push_back(manipPose.t_x);
        pose_message.data.push_back(manipPose.t_y);
        pose_message.data.push_back(manipPose.t_z);
        pose_message.data.push_back(manipPose.q_x);
        pose_message.data.push_back(manipPose.q_y);
        pose_message.data.push_back(manipPose.q_z);
        pose_message.data.push_back(manipPose.q_w);
        pose_message.data.push_back(valeurGachette);
        
        // Publish pose message
        pose_publisher_->publish(pose_message);
        
        // Display the pose every 1000 cycles
        if (count_ % 1000 == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Effector pose (m and rad): %f %f %f %f %f %f %f, Scale factor: %f",
                manipPose.t_x, manipPose.t_y, manipPose.t_z, 
                manipPose.q_x, manipPose.q_y, manipPose.q_z, manipPose.q_w,
                scale_factor_);
            
            // Log les forces actuellement appliquées (combinaison des deux sources)
            RCLCPP_INFO(this->get_logger(), "Applied force: X=%f, Y=%f, Z=%f, RX=%f, RY=%f, RZ=%f",
                externalForce_.t_x + joystickForce_.t_x, 
                externalForce_.t_y + joystickForce_.t_y, 
                externalForce_.t_z + joystickForce_.t_z,
                externalForce_.r_x + joystickForce_.r_x, 
                externalForce_.r_y + joystickForce_.r_y, 
                externalForce_.r_z + joystickForce_.r_z);
            
            // Publish mode status periodically
            publishModeStatus();
        }

        // Create message to publish the force
        auto force_message = std_msgs::msg::Float32MultiArray();
        
        // Set up dimensions for force
        force_message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        force_message.layout.dim[0].size = 6;
        force_message.layout.dim[0].stride = 1;
        force_message.layout.dim[0].label = "force";
        
        // Fill force data - nous publions la somme des forces des deux sources
        force_message.data.clear();
        force_message.data.push_back(externalForce_.t_x + joystickForce_.t_x);
        force_message.data.push_back(externalForce_.t_y + joystickForce_.t_y);
        force_message.data.push_back(externalForce_.t_z + joystickForce_.t_z);
        force_message.data.push_back(externalForce_.r_x + joystickForce_.r_x);
        force_message.data.push_back(externalForce_.r_y + joystickForce_.r_y);
        force_message.data.push_back(externalForce_.r_z + joystickForce_.r_z);
        
        // Publish force message
        force_publisher_->publish(force_message);

        // Mode-specific handling
        if (inSpeedMode_) {
            // En mode vitesse, on contrôle la position directement avec les vitesses
            HAPTION::CartesianVector desiredSpeed;
            
            // Calculer les vitesses désirées à partir des déplacements du manipulateur
            // Ici, nous prenons la position actuelle comme référence et appliquons un facteur d'échelle
            desiredSpeed.t_x = manipPose.t_x;
            desiredSpeed.t_y = manipPose.t_y;
            desiredSpeed.t_z = manipPose.t_z;
            desiredSpeed.r_x = manipSpeed.r_x;
            desiredSpeed.r_y = manipSpeed.r_y;
            desiredSpeed.r_z = manipSpeed.r_z;
            
            // Envoyer les vitesses calculées au dispositif
            raptorHandle_.SetCartesianSpeed(desiredSpeed);
        } else {
            // Send the force-feedback device its own pose and speed back to avoid drag errors
            raptorHandle_.SetCartesianPose(manipPose);
            raptorHandle_.SetCartesianSpeed(manipSpeed);
        }

        // Créer un vecteur pour la somme des forces
        HAPTION::CartesianVector combinedForce;
        
        // Additionner les forces des deux sources
        combinedForce.t_x = externalForce_.t_x + joystickForce_.t_x;
        combinedForce.t_y = externalForce_.t_y + joystickForce_.t_y;
        combinedForce.t_z = externalForce_.t_z + joystickForce_.t_z;
        combinedForce.r_x = externalForce_.r_x + joystickForce_.r_x;
        combinedForce.r_y = externalForce_.r_y + joystickForce_.r_y;
        combinedForce.r_z = externalForce_.r_z + joystickForce_.r_z;
        
        // Appliquer les forces combinées, ajustées par le facteur d'échelle
        HAPTION::CartesianVector scaledForce = combinedForce;
        
        // Add the force overlay (for both modes)
        raptorHandle_.AddCartesianForceOverlay(scaledForce);
        
        // Release brakes
        (void)raptorHandle_.ChangeBrakeStatus(HAPTION::BrakeStatus::BRAKE_RELEASED);
        
        // Send setpoints
        error = raptorHandle_.SendSetpoints();
        if (error != HAPTION::ErrorCode::E_NOERROR)
        {
            RCLCPP_ERROR(this->get_logger(), "HAPTION::RaptorAPI::SendSetpoints() failed with error %d", (int)error);
            rclcpp::shutdown();
            return;
        }

        // Push B_STOP button to exit
        bool stopButton = false;
        raptorHandle_.GetOperatorButton(HAPTION::OperatorButton::B_EXTSIGNAL1 , stopButton);
        if (stopButton)
        {
            RCLCPP_INFO(this->get_logger(), "Stop button pressed, shutting down");
            rclcpp::shutdown();
            return;
        }
        
        count_++;
    }

    // ROS2 publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr force_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mode_publisher_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Haption device handle
    HAPTION::RaptorAPI raptorHandle_;
    
    // Counter for display
    size_t count_;
    
    // Weight parameter
    float weight_;
    
    // Last step time
    std::chrono::system_clock::time_point lastStep_;
    
    // Mode control
    bool inSpeedMode_;                      // Mode actuel: false = position, true = vitesse
    size_t buttonPressDebounce_;            // Compteur pour éviter les multiples déclenchements
};

int main(int argc, char **argv)
{
    float weight = 0.0F;
    if ((argc != 3) && (argc != 4))
    {
        std::cout << "Syntax: ros2_Pose_Haption <device_param_file> <protocol_params> [weight]" << std::endl;
        std::cout << "Example: ros2_Pose_Haption virtuose_6d_n76.param localip=127.0.0.1:localport=12120:remoteip=192.168.100.53:remoteport=5000 0.5" << std::endl;
        return 0;
    }
    if (argc == 4)
    {
        weight = (float)atof(argv[3]);
        if ((weight < 0.0F) || (weight > 30.0F))
        {
            weight = 0.0F;
        }
    }

    // Initialize ROS
    rclcpp::init(argc, argv);
    
    try {
        // Create node
        auto node = std::make_shared<CartesianImpedanceHaption>(argv[1], argv[2], weight);
        
        // Spin
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    // Shutdown
    rclcpp::shutdown();
    return 0;
}