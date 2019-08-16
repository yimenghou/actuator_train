// last update: 20190815
// author: yimeng

#pragma once

#include <sstream>
#include <iostream>
#include "carriage_base.h"

namespace actuator_train {

/**
 * @class FooActuator
 * @brief The actuator class that performs foo actions
 */
class FooActuator: public Carriage<double> {
public:
  FooActuator(const double target1,
              const double target2,
              bool is_complete = false);

  void init();

  void proc();
  
private:
  size_t n;
};

} // namespace actuator_train