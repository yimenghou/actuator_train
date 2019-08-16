// last update: 20190815
// author: yimeng

#include "carriage/foo_actuator.h"

namespace actuator_train {

FooActuator::FooActuator(const double target1,
                         const double target2,
                         bool is_complete):
Carriage("FooActuator", is_complete, target1, target2), n(0) {
  setEqualFunction("Strictly");
}

void FooActuator::init() {
  std::cout << "FooActuator: Target: " << getTarget(0) << ", "<< getTarget(1) << std::endl;
}

void FooActuator::proc() {
  std::cout << "FooActuator running " << n++ << std::endl;
}

} // namespace actuator_train