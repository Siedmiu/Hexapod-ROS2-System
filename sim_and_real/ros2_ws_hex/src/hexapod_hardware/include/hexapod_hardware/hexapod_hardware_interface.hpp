//
// Hexapod Hardware Interface Header
// 
// Hardware interface for hexapod robot communication with ESP32 microcontroller.
// Implements ros2_control SystemInterface for direct communication with physical robot
// via UART/USB. Handles servo control, contact sensor monitoring, and joint states.
//

#ifndef HEXAPOD_HARDWARE__HEXAPOD_HARDWARE_INTERFACE_HPP_
#define HEXAPOD_HARDWARE__HEXAPOD_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <unordered_map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace hexapod_hardware
{

// Main hardware interface class for hexapod robot control
class HexapodHardwareInterface : public hardware_interface::SystemInterface
{
public:
  // Initialize hardware interface with configuration parameters
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // Activate hardware interface and establish communication
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Deactivate hardware interface and close communication
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Export joint state interfaces for ros2_control
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Export joint command interfaces for ros2_control
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Read current joint states from hardware
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Write joint commands to hardware
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Configuration parameters
  std::string port_name_;               // Serial port device name
  int baud_rate_;                      // Serial communication baud rate
  
  // Joint state management
  std::vector<double> hw_positions_;   // Current joint positions in radians
  std::vector<double> hw_velocities_;  // Current joint velocities in rad/s
  std::vector<double> hw_commands_;    // Commanded joint positions in radians
  
  // Serial communication
  int serial_fd_;                      // File descriptor for serial port
  bool serial_connected_;              // Serial connection status flag
  
  // UART thread management
  std::thread uart_thread_;            // Background thread for UART communication
  bool stop_thread_;                   // Thread termination signal
  std::string uart_buffer_;            // Buffer for incoming UART data
  
  // Command optimization
  std::unordered_map<std::string, int> joint_to_servo_map_;  // ROS2 joints to ESP32 servo mapping
  std::vector<double> last_sent_commands_;                   // Previously sent commands
  double command_tolerance_;                                 // Minimum change threshold
  
  // Contact sensor system
  bool contact_sensors_[6];                                     // Contact sensor states for 6 legs
  rclcpp::Node::SharedPtr node_;                                // ROS2 node for sensor publishing
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact_publishers_[6];  // Contact status publishers
  
  // UART communication methods
  bool openSerialPort();               // Open and configure serial port
  void closeSerialPort();              // Close serial port connection
  bool sendSerialData(const std::string& data);  // Send data through serial port
  void uartThreadFunction();           // Background thread function for UART
  
  // Joint mapping and conversion methods
  void initializeJointMapping();       // Initialize joint to servo mapping
  int convertRadiansToServoDegrees(const std::string& joint_name, double angle_rad);  // Convert radians to servo degrees
  bool shouldSendCommand(size_t joint_index, double new_command);  // Check if command should be sent
  bool sendServoCommand(int servo_number, int angle_degrees);      // Send servo command to ESP32
  
  // Contact sensor methods
  void checkContactSensors(const std::string& message);  // Process contact sensor data
  void publishContactSensors();                          // Publish sensor states to ROS2
};

}  // namespace hexapod_hardware

#endif  // HEXAPOD_HARDWARE__HEXAPOD_HARDWARE_INTERFACE_HPP_