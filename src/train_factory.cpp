// last update: 20190815
// author: yimeng

#include "train_factory.h"

namespace actuator_train {

Train TrainFactory::dummyTrain() {
  Train dummy_train;
  dummy_train.add<FooActuator>(-2500, 3300.1, true);
  dummy_train.add<BarActuator>(0.1, true);
  dummy_train.build();
  dummy_train.add<FooActuator>(-2500, 3300.1);
  dummy_train.build();
  dummy_train.add<BarActuator>(1.1);
  dummy_train.buildTrain();
  return dummy_train;
}

} // namespace actuator_train