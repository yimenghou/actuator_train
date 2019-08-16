// last update: 20190815
// author: yimeng

#pragma once

#include <sstream>
#include <iostream>
#include "carriage_base.h"

namespace actuator_train {

/**
 * @class BarActuator
 * @brief The actuator class that performs bar actions
 */
class BarActuator: public Carriage<double> {
public:
  BarActuator(const double target,
              bool is_complete = false);

  void init();

  void proc();
    
private:
  size_t n;
};

} // namespace actuator_train