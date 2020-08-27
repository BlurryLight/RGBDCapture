#include "OpenNISensor.h"
#include <memory>

int main(int argc, char **argv) {
  std::cout << "DepthScaleFactor:" << kDepthScaleFactor << std::endl;
  auto sensor = std::unique_ptr<RGBDSensor>(new OpenNISensor);
  sensor->scan();
  return 0;
}
