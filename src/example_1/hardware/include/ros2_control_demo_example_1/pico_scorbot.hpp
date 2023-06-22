#ifndef SCORBOT_PICO_COMMS_HPP
#define SCORBOT_PICO_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <iostream>

#define PI 3.14159265359

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  default:
    std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
    return LibSerial::BaudRate::BAUD_57600;
  }
}

class PicoScorbot
{

public:
  PicoScorbot() = default;

  void connect(const std::string &serial_upper_device, const std::string &serial_lower_device, int32_t baud_rate, int32_t timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    serial_upper_controller_conn_.Open(serial_upper_device);
    serial_upper_controller_conn_.SetBaudRate(convert_baud_rate(baud_rate));

    serial_lower_controller_conn_.Open(serial_lower_device);
    serial_lower_controller_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_upper_controller_conn_.Close();
    serial_lower_controller_conn_.Close();
  }

  bool connected() const
  {
    return serial_upper_controller_conn_.IsOpen() && serial_lower_controller_conn_.IsOpen();
  }

  void sendDataToDevices(std::vector<double> &data)
  {
    if (data.size() != 7)
    {
      std::cout << "Error: Input data must be a vector of size 7." << std::endl;
      return;
    }

    double slide_base_position = data[0];
    double body_position = -data[1] * 180.0 / PI;
    double shoulder_position = data[2] * 180.0 / PI;
    double elbow_position = -data[3] * 180.0 / PI - shoulder_position;
    double wrist_left_position = (-data[4] + data[5]) * 180.0 / PI + elbow_position;
    double wrist_right_position = (data[4] + data[5]) * 180.0 / PI - elbow_position;
 
    std::string position_lower_controller = "p " + std::to_string(slide_base_position) + "," + std::to_string(body_position) + "," + std::to_string(shoulder_position);
    std::string position_upper_controller = "p " + std::to_string(elbow_position) + "," + std::to_string(wrist_left_position) + "," + std::to_string(wrist_right_position);

    serial_lower_controller_conn_.Write(position_lower_controller + "\n");
    serial_upper_controller_conn_.Write(position_upper_controller + "\n");
    
    if (last_extruder_pos != data[6]){
        // extruder_pos = (data[6]) * _mm_by_steps * 10;
        // std::string position_extruder = "n " + std::to_string(extruder_pos) + "," + std::to_string(_vel_extruder);
        extruder_pos = (data[6])*10;
        // std::cout << extruder_pos << std::endl;
        std::string position_extruder = "n " + std::to_string(extruder_pos) + "," + "10";
        serial_upper_controller_conn_.Write(position_extruder + "\n");
        last_extruder_pos = data[6];
    }
  }

  void configKGammaGain(float new_k_gamma){
    std::string set_new_k_gamma = "k " + std::to_string(new_k_gamma);

    serial_lower_controller_conn_.Write(set_new_k_gamma + "\n");
    serial_upper_controller_conn_.Write(set_new_k_gamma + "\n");
  }

  void ConfigExtruder(float mm_by_steps){
    _mm_by_steps = mm_by_steps;
    _vel_extruder = mm_by_steps * vel_robot;
  }

  void getDataFromDevices(std::vector<double> &data)
  {
    std::string response_lower_controller, response_upper_controller;

    std::vector<double> data_to_controllers;

    serial_lower_controller_conn_.Write("e\n");
    serial_lower_controller_conn_.ReadLine(response_lower_controller, '\n', timeout_ms_);

    serial_upper_controller_conn_.Write("e\n");
    serial_upper_controller_conn_.ReadLine(response_upper_controller, '\n', timeout_ms_);

    std::stringstream ss(response_lower_controller + "," + response_upper_controller);
    std::string token;
    while (std::getline(ss, token, ',')) {
        data_to_controllers.push_back(std::stod(token));
    }

    data[0] = data_to_controllers[0] / 100.0;
    data[1] = -data_to_controllers[1] * PI / 180.0;
    data[2] = data_to_controllers[2] * PI / 180.0;
    data[3] = -data_to_controllers[3] * PI / 180.0 - data_to_controllers[2] * PI / 180.0;
    data[4] = (-data_to_controllers[4] + data_to_controllers[5]) * PI / 180.0 + data_to_controllers[3] * PI / 180.0;
    data[5] = (data_to_controllers[4] + data_to_controllers[5]) * PI / 180.0;
    data[6] = data_to_controllers[6] * PI / 180.0;

  }

private:
  LibSerial::SerialPort serial_lower_controller_conn_, serial_upper_controller_conn_;
  std::vector<double> actual_pos_joints, actual_vel_joints;
  double extruder_pos = 0;
  double last_extruder_pos = 0;
  float _mm_by_steps;
  float _vel_extruder;
  float vel_robot = 20;
  int timeout_ms_;
};

#endif // SCORBOT_PICO_COMMS_HPP