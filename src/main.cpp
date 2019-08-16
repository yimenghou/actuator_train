// last update: 20190815
// author: yimeng

#include "train_factory.h"

int main(int argc, char **argv) {
  actuator_train::TrainFactory factory;
  auto* train = factory.GetTrainByName("dummy_train");
  auto ret = train->ignite();
  return 0;
}