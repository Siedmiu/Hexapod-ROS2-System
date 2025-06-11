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
class HexapodHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parametry konfiguracji
  std::string port_name_;
  int baud_rate_;
  
  // Stany stawów
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  
  // Komunikacja szeregowa
  int serial_fd_;
  bool serial_connected_;
  
  // UART Thread
  std::thread uart_thread_;
  bool stop_thread_;
  std::string uart_buffer_;
  
  // Mapowanie i optymalizacja
  std::unordered_map<std::string, int> joint_to_servo_map_;
  std::vector<double> last_sent_commands_;
  double command_tolerance_;
  
  // Contact sensors
  bool contact_sensors_[6];
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact_publishers_[6];
  
  // Metody komunikacji UART
  bool openSerialPort();
  void closeSerialPort();
  bool sendSerialData(const std::string& data);
  void uartThreadFunction();
  
  // Metody mapowania i konwersji
  void initializeJointMapping();
  int convertRadiansToServoDegrees(const std::string& joint_name, double angle_rad);
  bool shouldSendCommand(size_t joint_index, double new_command);
  bool sendServoCommand(int servo_number, int angle_degrees);
  
  // Contact sensors
  void checkContactSensors(const std::string& message);
  void publishContactSensors();
};

}  // namespace hexapod_hardware

#endif  // HEXAPOD_HARDWARE__HEXAPOD_HARDWARE_INTERFACE_HPP_