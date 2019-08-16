// last update: 20190815
// author: yimeng

#include "carriage/bar_actuator.h"

namespace actuator_train {

BarActuator::BarActuator(const double target,
                         bool is_complete):
Carriage("BarActuator", is_complete, target), n(0) {
  setEqualFunction("Roughly");
}

void BarActuator::init() {
  std::cout << "BarActuator: Target: " << getTarget() << std::endl;
}

void BarActuator::proc() {
  std::cout << "BarActuator running " << n++ << std::endl;
}    

} // namespace actuator_train