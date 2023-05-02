#ifndef SCORBOT_PICO_COMMS_HPP
#define SCORBOT_PICO_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <iostream>

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
    std::string position_lower_controller = "p " + std::to_string((data)[0]) + "," + std::to_string((data)[1]) + "," + std::to_string((data)[2]);
    std::string position_upper_controller = "p " + std::to_string((data)[3]) + "," + std::to_string((data)[4]) + "," + std::to_string((data)[5]);
    std::string position_extruder = "n " + std::to_string((data)[6]);

    serial_lower_controller_conn_.Write(position_lower_controller + "\n");
    serial_upper_controller_conn_.Write(position_upper_controller + "\n");
    serial_upper_controller_conn_.Write(position_extruder + "\n");
  }

  void getDataFromDevices(std::vector<double> &data)
  {
    std::string response_lower_controller, response_upper_controller;

    std::vector<double> previous_data;

    serial_lower_controller_conn_.Write("e\n");
    serial_lower_controller_conn_.ReadLine(response_lower_controller, '\n', timeout_ms_);

    serial_upper_controller_conn_.Write("e\n");
    serial_upper_controller_conn_.ReadLine(response_upper_controller, '\n', timeout_ms_);

    std::stringstream ss(response_lower_controller + "," + response_upper_controller);
    std::string token;
    while (std::getline(ss, token, ',')) {
        previous_data.push_back(std::stod(token));
    }

    data = previous_data;
  }

private:
  LibSerial::SerialPort serial_lower_controller_conn_, serial_upper_controller_conn_;
  std::vector<double> actual_pos_joints, actual_vel_joints;
  int timeout_ms_;
};

#endif // SCORBOT_PICO_COMMS_HPP