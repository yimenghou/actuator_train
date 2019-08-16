// last update: 20190815
// author: yimeng

#pragma once

#include <unordered_map>

#include "carriage_base.h"
#include "train.h"

#include "carriage/foo_actuator.h"
#include "carriage/bar_actuator.h"

namespace actuator_train {

/**
 * @class TrainFactory
 * @brief A Train factory that contains all kinds of train implementation for different functionality
 */
class TrainFactory {
public:

  TrainFactory() {
    RegisterTrain("dummy_train", dummyTrain());
  }
  virtual ~TrainFactory() = default;

  /**
   * @brief Define a dummy train
   */
  Train dummyTrain();

  void RegisterTrain(const std::string& name, Train tr) {
    std::pair<std::string, Train> temp(name, tr);
    train_map_.insert(temp);
  }

  Train* GetTrainByName(const std::string& name) {
    if (train_map_.find(name) == train_map_.end()) {
      std::cout << "Cannot find specified target with key: " << name << std::endl;
      return nullptr;
    }
    return &train_map_[name];
  }

// private:
  std::unordered_map<std::string, Train> train_map_;
};

} // namespace actuator_train
