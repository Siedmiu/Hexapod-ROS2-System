//
// Hexapod Hardware Interface Implementation
//
// Implementation of ros2_control SystemInterface for hexapod robot.
// Handles ESP32 communication, servo control, and contact sensor monitoring.
//

#include "hexapod_hardware/hexapod_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <unordered_map>
#include <thread>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <cstring>
#include <sstream> 

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace hexapod_hardware
{

// Default joint positions for robot startup (in degrees)
static constexpr double JOINT1_INITIAL_ANGLE_DEG = 0.0;   
static constexpr double JOINT2_INITIAL_ANGLE_DEG = 10.0;  
static constexpr double JOINT3_INITIAL_ANGLE_DEG = 80.0;  

// Convert to radians for internal use
static constexpr double JOINT1_INITIAL_ANGLE = JOINT1_INITIAL_ANGLE_DEG * M_PI / 180.0;
static constexpr double JOINT2_INITIAL_ANGLE = JOINT2_INITIAL_ANGLE_DEG * M_PI / 180.0;
static constexpr double JOINT3_INITIAL_ANGLE = JOINT3_INITIAL_ANGLE_DEG * M_PI / 180.0;

hardware_interface::CallbackReturn HexapodHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Initialize base system interface
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Extract configuration parameters from URDF
  port_name_ = info_.hardware_parameters["port_name"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

  RCLCPP_INFO(rclcpp::get_logger("HexapodHardwareInterface"), 
    "Port: %s, Baud: %d", port_name_.c_str(), baud_rate_);

  // Initialize joint state vectors
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  // Initialize serial communication state
  serial_fd_ = -1;
  serial_connected_ = false;
  stop_thread_ = false;

  // Setup joint to servo mapping
  initializeJointMapping();

  // Initialize command optimization system
  last_sent_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  command_tolerance_ = 0.01;  // Minimum change threshold in radians

  // Initialize contact sensors
  for (int i = 0; i < 6; ++i) {
    contact_sensors_[i] = false;
  }

  // Create ROS2 node for contact sensor publishing
  node_ = rclcpp::Node::make_shared("hexapod_contact_sensors");
  
  // Define contact sensor topic names for each leg
  std::string topic_names[6] = {
    "/hexapod/leg1/contact_status",
    "/hexapod/leg2/contact_status",
    "/hexapod/leg3/contact_status",
    "/hexapod/leg4/contact_status",
    "/hexapod/leg5/contact_status",
    "/hexapod/leg6/contact_status"
  };
  
  // Create publishers for each leg's contact status
  for (int i = 0; i < 6; ++i) {
    contact_publishers_[i] = node_->create_publisher<std_msgs::msg::Bool>(topic_names[i], 10);
  }

  RCLCPP_INFO(rclcpp::get_logger("HexapodHardwareInterface"), 
    "Initialized with %zu joints", info_.joints.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HexapodHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("HexapodHardwareInterface"), "Activating hardware interface");
  
  // Establish serial communication with ESP32
  if (!openSerialPort()) {
    RCLCPP_ERROR(rclcpp::get_logger("HexapodHardwareInterface"), 
      "Failed to open serial port %s", port_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("HexapodHardwareInterface"), 
    "Serial port %s opened successfully", port_name_.c_str());
  
  // Set initial joint positions based on joint type
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    std::string joint_name = info_.joints[i].name;
    
    if (joint_name.find("joint1_") != std::string::npos)
    {
      hw_positions_[i] = JOINT1_INITIAL_ANGLE;
      hw_commands_[i] = JOINT1_INITIAL_ANGLE;
    }
    else if (joint_name.find("joint2_") != std::string::npos)
    {
      hw_positions_[i] = JOINT2_INITIAL_ANGLE;
      hw_commands_[i] = JOINT2_INITIAL_ANGLE;
    }
    else if (joint_name.find("joint3_") != std::string::npos)
    {
      hw_positions_[i] = JOINT3_INITIAL_ANGLE;
      hw_commands_[i] = JOINT3_INITIAL_ANGLE;
    }
    else
    {
      hw_positions_[i] = 0.0;
      hw_commands_[i] = 0.0;
    }
  }

  // Start background UART communication thread
  stop_thread_ = false;
  uart_thread_ = std::thread(&HexapodHardwareInterface::uartThreadFunction, this);
  RCLCPP_INFO(rclcpp::get_logger("HexapodHardwareInterface"), "UART thread started");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HexapodHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("HexapodHardwareInterface"), "Deactivating hardware interface");
  
  // Stop UART thread properly
  stop_thread_ = true;
  if (uart_thread_.joinable()) {
    uart_thread_.join();
  }
  RCLCPP_INFO(rclcpp::get_logger("HexapodHardwareInterface"), "UART thread stopped");
  
  // Close serial port connection
  closeSerialPort();
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HexapodHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // Export position and velocity interfaces for each joint
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HexapodHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  // Export position command interfaces for each joint
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type HexapodHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Update position states to match commands (no feedback from servos)
  for (size_t i = 0; i < hw_positions_.size(); ++i)
  {
    hw_positions_[i] = hw_commands_[i];
    hw_velocities_[i] = 0.0;  // Velocity not measured
  }

  // Publish contact sensor states
  publishContactSensors();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HexapodHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Check if serial connection is available
  if (!serial_connected_) {
    return hardware_interface::return_type::ERROR;
  }

  // Send commands only for joints that changed significantly
  for (size_t i = 0; i < hw_commands_.size(); ++i)
  {
    if (!shouldSendCommand(i, hw_commands_[i])) {
      continue;  // Skip if change is below threshold
    }

    // Convert command to servo format and send
    std::string joint_name = info_.joints[i].name;
    int servo_degrees = convertRadiansToServoDegrees(joint_name, hw_commands_[i]);
    int servo_number = joint_to_servo_map_[joint_name];
    
    if (sendServoCommand(servo_number, servo_degrees)) {
      last_sent_commands_[i] = hw_commands_[i];  // Update tracking
    }
  }

  return hardware_interface::return_type::OK;
}

void HexapodHardwareInterface::uartThreadFunction()
{
  RCLCPP_INFO(rclcpp::get_logger("HexapodHardwareInterface"), "UART thread running");
  
  char temp_buffer[2048];
  
  // Main UART reading loop
  while (!stop_thread_) {
    if (!serial_connected_ || serial_fd_ == -1) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    
    // Read data from serial port
    ssize_t bytes = ::read(serial_fd_, temp_buffer, sizeof(temp_buffer));
    
    if (bytes > 0) {
      uart_buffer_.append(temp_buffer, bytes);
      
      // Process complete lines from buffer
      size_t pos = 0;
      while ((pos = uart_buffer_.find('\n')) != std::string::npos) {
        std::string complete_line = uart_buffer_.substr(0, pos);
        
        RCLCPP_INFO(rclcpp::get_logger("HexapodHardwareInterface"), 
                    "UART: '%s'", complete_line.c_str());
        checkContactSensors(complete_line);  // Parse sensor data
        
        uart_buffer_.erase(0, pos + 1);  // Remove processed line
      }
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));  // Rate limiting
  }
  
  RCLCPP_INFO(rclcpp::get_logger("HexapodHardwareInterface"), "UART thread finished");
}

void HexapodHardwareInterface::initializeJointMapping()
{
  // Map ROS2 joint names to ESP32 servo indices
  // Mapping follows hexapod leg arrangement and servo wiring
  joint_to_servo_map_ = {
    {"joint1_3", 0},  {"joint2_3", 1},  {"joint3_3", 2},   // Leg 3
    {"joint1_2", 3},  {"joint2_2", 4},  {"joint3_2", 5},   // Leg 2
    {"joint1_1", 6},  {"joint2_1", 7},  {"joint3_1", 8},   // Leg 1
    {"joint1_6", 9},  {"joint2_6", 10}, {"joint3_6", 11},  // Leg 6
    {"joint1_5", 12}, {"joint2_5", 13}, {"joint3_5", 14},  // Leg 5
    {"joint1_4", 15}, {"joint2_4", 16}, {"joint3_4", 17}   // Leg 4
  };

  RCLCPP_INFO(rclcpp::get_logger("HexapodHardwareInterface"), 
              "Initialized joint→servo mapping for %zu joints", joint_to_servo_map_.size());
}

int HexapodHardwareInterface::convertRadiansToServoDegrees(const std::string& joint_name, double angle_rad)
{
  // Convert radians to degrees
  double deg = angle_rad * 180.0 / M_PI;
  int servo_degrees;
  
  // Apply joint-specific conversion based on servo characteristics
  if (joint_name.find("joint1_") != std::string::npos) {
    // Coxa joints: ±45° range mapped to 0-180°
    servo_degrees = static_cast<int>((deg + 45.0) * (180.0 / 90.0));
  }
  else if (joint_name.find("joint2_") != std::string::npos) {
    // Femur joints: ±60° range mapped to 0-180°
    servo_degrees = static_cast<int>((deg + 60.0) * (180.0 / 120.0));
  }
  else if (joint_name.find("joint3_") != std::string::npos) {
    // Tibia joints: -30° to 90° range, inverted mapping
    servo_degrees = static_cast<int>(180.0 - ((deg + 30.0) * (180.0 / 120.0)));
  }
  else {
    return 90;  // Default center position
  }
  
  // Clamp to valid servo range
  return std::max(0, std::min(180, servo_degrees));
}

bool HexapodHardwareInterface::shouldSendCommand(size_t joint_index, double new_command)
{
  // Always send first command
  if (std::isnan(last_sent_commands_[joint_index])) {
    return true;
  }
  
  // Check if change exceeds threshold
  double change = std::abs(new_command - last_sent_commands_[joint_index]);
  return change >= command_tolerance_;
}

bool HexapodHardwareInterface::sendServoCommand(int servo_number, int angle_degrees)
{
  // Format command string for ESP32: "servo<num> <angle>\n"
  std::string command = "servo" + std::to_string(servo_number) + 
                       " " + std::to_string(angle_degrees) + "\n";
  
  return sendSerialData(command);
}

bool HexapodHardwareInterface::openSerialPort()
{
  // Open serial port with read/write access
  serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  
  if (serial_fd_ == -1) {
    return false;
  }

  // Configure serial port parameters
  struct termios options;
  tcgetattr(serial_fd_, &options);

  // Set baud rate
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

  // 8N1 configuration
  options.c_cflag &= ~PARENB;   // No parity
  options.c_cflag &= ~CSTOPB;   // 1 stop bit
  options.c_cflag &= ~CSIZE;    // Clear size bits
  options.c_cflag |= CS8;       // 8 data bits
  options.c_cflag |= CREAD;     // Enable receiver
  options.c_cflag |= CLOCAL;    // Local connection

  // Raw input mode
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_oflag &= ~OPOST;

  // Non-blocking read
  options.c_cc[VTIME] = 0;
  options.c_cc[VMIN] = 0;

  tcsetattr(serial_fd_, TCSANOW, &options);
  
  // Flush buffers before starting
  tcflush(serial_fd_, TCIOFLUSH);

  uart_buffer_.clear();
  serial_connected_ = true;
  
  return true;
}

void HexapodHardwareInterface::closeSerialPort()
{
  if (serial_fd_ != -1) {
    // Flush buffers before closing
    tcflush(serial_fd_, TCIOFLUSH);
    
    close(serial_fd_);
    serial_fd_ = -1;
    serial_connected_ = false;
  }
}

bool HexapodHardwareInterface::sendSerialData(const std::string& data)
{
  // Check connection before sending
  if (!serial_connected_ || serial_fd_ == -1) {
    return false;
  }

  // Write data to serial port
  ssize_t bytes_written = ::write(serial_fd_, data.c_str(), data.length());
  return bytes_written == static_cast<ssize_t>(data.length());
}

void HexapodHardwareInterface::checkContactSensors(const std::string& message)
{
  // Parse contact sensor status messages from ESP32
  if (message.find("Pin status:") != std::string::npos) {
    
    size_t pos = message.find("Pin status:") + 12;
    std::string values = message.substr(pos);
    
    std::istringstream iss(values);
    int sensor_value;
    int sensor_index = 0;
    
    // Read sensor values for all 6 legs
    while (iss >> sensor_value && sensor_index < 6) {
      contact_sensors_[sensor_index] = (sensor_value == 1);
      sensor_index++;
    }
  }
}

void HexapodHardwareInterface::publishContactSensors()
{
  // Publish contact status for each leg
  for (int i = 0; i < 6; ++i) {
    std_msgs::msg::Bool msg;
    msg.data = contact_sensors_[i];
    
    contact_publishers_[i]->publish(msg);
  }
}

}  // namespace hexapod_hardware

// Register hardware interface as plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  hexapod_hardware::HexapodHardwareInterface, hardware_interface::SystemInterface)