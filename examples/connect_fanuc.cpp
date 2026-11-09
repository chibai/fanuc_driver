#include <exception>
#include <iostream>

#include "robot_driver/robot_driver.hpp"

namespace
{
int reportError(const ResponseStatus& status)
{
  std::cerr << status.code() << ": " << status.message() << " (" << status.detail << ")\n";
  return 1;
}
}  // namespace

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
    if (!connection.ok())
      return reportError(connection);
    const auto status = driver.getStatus();
    if (!status.ok())
      return reportError(status.status);
    std::cout << "Servo ready: " << status.data->servo_ready << '\n';
    const auto disconnected = driver.disconnect();
    if (!disconnected.ok())
      return reportError(disconnected);
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
