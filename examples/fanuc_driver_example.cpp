#include <exception>
#include <iostream>

#include "fanuc_driver/fanuc_driver.hpp"

int main(int argc, char* argv[])
{
  if (argc != 2)
  {
    std::cerr << "Usage: fanuc_driver_example <controller-ip>\n";
    return 1;
  }

  try
  {
    Fanuc_driver fanuc(argv[1]);
    Robot_driver& driver = fanuc;
    const auto connection = fanuc.connect();
    std::cout << "RMI version: " << connection.MajorVersion << '.' << connection.MinorVersion << '\n';
    const auto status = driver.getStatus();
    std::cout << "Servo ready: " << status.servo_ready << '\n';
    const auto joints = fanuc.readJointAngles();
    std::cout << "J1: " << joints.JointAngle.J1 << " degrees\n";
    driver.disconnect();
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
