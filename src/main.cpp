#include <iostream>

#include "engine/pid_controller.hpp"
#include "engine/utils.hpp"

int main() {
  std::cout << engine::get_status_message() << std::endl;

  engine::PidController pid({1.0, 0.1, 0.05}, {-10.0, 10.0, -5.0, 5.0}, 0.01);

  double output = pid.compute(10.0, 0.0);
  std::cout << "PID output: " << output << std::endl;
  return 0;
}
