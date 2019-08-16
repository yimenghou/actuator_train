// last update: 20190815
// author: yimeng

#pragma once

#include <sstream>
#include <utility>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <mutex>
#include <list>
#include <thread>
#include <memory>
#include "carriage_base.h"

namespace actuator_train {

typedef Carriage<double> CarriageMember;
typedef std::list<std::shared_ptr<CarriageMember>> CarriageUnit;
typedef std::vector<CarriageUnit> CarriageTrain;

enum class IgniteResult:int {
  Fail,
  Success,
  Error
};

using ignite_result_type = std::underlying_type<IgniteResult>::type;

/**
 * @struct Train
 * @brief A train that implement the various actuator functionality
 */
class Train {
public:
  Train():
  carriage_exec_idx_(0),
  is_ignited_(false),
  is_extinguish_(false)
  {}
  virtual ~Train() = default;

  /**
   * @brief add/register member Carriages
   * @param args target Carriage
   */
  template<typename T, typename... Args> 
  inline void add(Args&&... args) {
    static_assert(std::is_base_of<Carriage<double>, T>::value, "Please use correct types");
    this->carriage_.push_back(std::make_shared<T>(std::forward<Args>(args)...));
  }

  /**
   * @brief purge carriage from this train
   * @param c target Carriage
   * @return merged train
   */
  // Train& operator>>(const CarriageMember& c) {
  //   this->carriage_.pop_back();
  //   return *this;
  // }

  /**
   * @brief merge train with target train
   * @param t target train
   * @return merged train
   */
  Train& operator+(const Train& t) {
    for(auto& t_carriage:t.getCarriage()) {
      this->carriage_.push_back(t_carriage);
    }
    for(auto& t_train:t.getTrain()) {
      this->train_.push_back(t_train);
    }
    return *this;
  }

  /**
   * @brief merge train with target train
   * @param t target train
   * @return merged train
   */
  Train& operator+=(const Train& t) {
    for(auto& t_carriage:t.getCarriage()) {
      this->carriage_.push_back(t_carriage);
    }
    for(auto& t_train:t.getTrain()) {
      this->train_.push_back(t_train);
    }
    return *this;
  }

  /**
   * @brief assign target train to this train
   * @param t target train
   * @return updated train
   */
  Train& operator=(const Train& t) {
    this->carriage_=t.getCarriage();
    this->train_=t.getTrain();
    this->carriage_exec_idx_=t.getCarriageExecIdx();
    return *this;
  }

  /**
   * @brief feed current data to target carriage in current executing stage in this train
   * @param name the carriage name 
   * @param data the input argument of the carriage
   */
  template <typename... Args>
  void feedCurrent(const std::string& name, Args&&... data) {
    if (!is_ignited_) {return;}
    auto& current_carriage_ = train_.at(carriage_exec_idx_);
    for(auto& c:current_carriage_) {
      if (c->name() == name) {
        c->setCurrent(std::forward<Args>(data)...);
      }
    }
  }

  /**
   * @brief collect the data to target carriage in current executing stage in this train
   * @param name the carriage name 
   * @param data the input argument of the carriage
   */
  template<typename T = double>
  std::vector<T> collectTarget(const std::string& name) {
    std::vector<T> temp;
    if (!is_ignited_) {return temp;}
    auto& current_carriage_ = train_.at(carriage_exec_idx_);
    for(auto& c:current_carriage_) {
      if (c->name() == name) {
        return c->getTargetVec();
      }
    }
    return temp;
  }

  /**
   * @brief ignite the train, start running
   * @param from_idx from which index that the train starts igniting, the default is from beginning(from_idx=0)
   */
  IgniteResult ignite(const size_t& from_idx = 0) {
    if (train_.empty()) {
      std::cout << FUNC_NAME << "An empty train cannot be ignited!" << std::endl;
      return IgniteResult::Error;
    }
    is_ignited_ = true;
    carriage_exec_idx_ = from_idx;
    std::cout << FUNC_NAME << "Start." << std::endl;
    while(1) {
      auto& current_carriage_ = train_.at(carriage_exec_idx_);
      if (is_extinguish_) {
        is_ignited_ = false;
        is_extinguish_ = false;
        for(auto& c:current_carriage_) {c->stop();}  
        return IgniteResult::Fail;
      }
      for(auto& c:current_carriage_) {
        if(!c->isInited()) {
          std::cout << FUNC_NAME << "-> Stage: " << carriage_exec_idx_ << std::endl;
          c->init();
          c->setInit();
          continue;
        }
        if (c->count_++%static_cast<uint16_t>(loop_rate_/c->update_freq_)!=0) {
          continue;
        }
        c->update();
      }
      const auto& stage_complete = checkStageComplete();
      if (stage_complete) {
        if (carriage_exec_idx_>=train_.size()-1) {
          is_ignited_ = false;
          return IgniteResult::Success;
        }
        carriage_exec_idx_++;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1000/loop_rate_));
    }
    return IgniteResult::Success;
  }

  /**
   * @brief Carriage size getter
   * @return The size of current carriage
   */
  inline size_t getCarriageSize() const {
    return carriage_.size();
  }

  /**
   * @brief Train size getter
   * @return The size of current train
   */
  inline size_t getTrainSize() const {
    return train_.size();
  }

  /**
   * @brief Carriage getter
   * @return Current carriage
   */
  inline CarriageUnit getCarriage() const {
    return carriage_;
  }

  /**
   * @brief Train getter
   * @return Current train
   */
  inline CarriageTrain getTrain() const {
    return train_;
  }

  /**
   * @brief The index getter of current executed carriage
   * @return The index
   */
  inline size_t getCarriageExecIdx() const {
    return carriage_exec_idx_;
  }

  /**
   * @brief Check if the train has been ignited
   * @return yes or no
   */
  inline bool isTrainIgnited() const {
    return is_ignited_;
  }

  /**
   * @brief Extinguish current train, make it exit silently
   */
  inline void extinguish() {
    if (is_ignited_) {
      is_extinguish_ = true;
      while (is_ignited_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  /**
   * @brief Build the entire train
   */
  inline void buildTrain() {
    if (!carriage_.empty()) {
      build();
    }
    observeTrain();
  }

  /**
   * @brief Build this carriage
   * @return success or not
   */
  bool build() {
    if (carriage_.empty()) {
      std::cerr << FUNC_NAME << "An empty carriage cannot be built!" << std::endl;
      return false;
    }
    train_.push_back(std::move(carriage_));
    carriage_.clear();
    return true;
  }

  /**
   * @brief Clear all member within this carriage
   */
  inline void clearCarriage() {
    carriage_.clear();
  }

  /**
   * @brief Observe all the information(carriage members and their attributes), and print them out
   */
  void observeTrain() {
    std::ostringstream oss;
    oss << "--------------------------------------" <<
    "------------------------------------------" << std::endl;
    oss << "{Train summary}" << std::endl;
    for(CarriageTrain::iterator it=train_.begin();it!=train_.end();++it) {
      oss << "[ Stage " << std::distance(train_.begin(), it) << " ]" <<  std::endl;
      for(const auto& c:*it) {
        oss << "  [Actuator]:" << c->name() 
            << ", [Goal]:" << c->goalName() 
            << ", [Criterion]:" << c->equalName()
            << ", [Target]:";
        for(size_t i=0;i<c->data().len;i++) {
          oss << c->data().getTarget(i);
          if (i<c->data().len-1) {oss << ", ";}
        }
        oss << std::endl;
      }
    }
    oss << "--------------------------------------" <<
    "------------------------------------------" << std::endl;
    std::cout << oss.str();
  }

  const uint32_t loop_rate_ = kLoopRate; // 10 Hz

private:

  /**
   * @brief Check whether current running stage has complete or not
   * @return complete or not
   */
  bool checkStageComplete() {
    bool complete = true;
    std::string not_complete_list = "";
    auto& current_carriage_ = train_.at(carriage_exec_idx_);
    for(auto& c:current_carriage_) {
      complete &= c->isComplete();
      if (!c->isComplete()) {
        not_complete_list += c->name() +", ";
      }
    }
    // if (!complete) {
    //   not_complete_list = not_complete_list.substr(0, not_complete_list.size()-1);
    //   std::cout << FUNC_NAME << "Stage: " << carriage_exec_idx_ << 
    //   ", Wait for Actuator [" << not_complete_list << "] to complete .." << std::endl;
    // } else {
    //   std::cout << FUNC_NAME << "Stage: " << carriage_exec_idx_ << ", complete" << std::endl;
    // }
    return complete;
  }

  size_t carriage_exec_idx_;
  CarriageUnit carriage_;
  CarriageTrain train_;
  bool is_ignited_, is_extinguish_;
};

} // namespace actuator_train