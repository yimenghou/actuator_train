// last update: 20190815
// author: yimeng

#pragma once

#include <cmath>
#include <sstream>
#include <utility>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <mutex>
#include <list>
#include <thread>
#include <functional>

#define FUNC_NAME " ["  << __FUNCTION__ << "] "
#define EqualCriterion(name, criterion)                       \
inline bool name##Equal(const double t1, const double t2) {   \
  return (fabs(t1-t2) <= criterion)?1:0;                      \
}                                                             \

static constexpr double kEpsilon = 0.001;
static constexpr double kEpsilonLoose = 1.0;
static constexpr int kLoopRate = 10;

EqualCriterion(Strictly, kEpsilon)
EqualCriterion(Roughly, kEpsilonLoose)

namespace actuator_train {

enum class ExecutionOutcome:int {
  SUCCESS=0,
  TIMEOUT=-1,
  FAIL=-2,
};

/**
 * @class Blob
 * @brief The class that contains the elementary storage for target value and current value
 */
template <typename T>
class Blob {
public:

  /**
   * @brief The default constructor that takes the size of the target/current value as input
   * @param l The length of the target/current T value pointer
   */
  explicit Blob(const size_t& l) {
    len = l;
    for(size_t i=0;i<len;i++) {target.push_back(T());}
    for(size_t i=0;i<len;i++) {current.push_back(T());}
  }

  /**
   * @brief The default copy assigner
   */
  Blob(const Blob& obj) {*this = obj;}

  /**
   * @brief The default destructor
   */
  virtual ~Blob() = default;

  /**
   * @brief The default constructor that takes the size of the target/current value as input
   * @param l The length of the target/current T value pointer
   */
  template<typename... Args>
  inline void setTarget(Args&&... args) {
    len = sizeof...(args);
    std::vector<T> list = {args...};
    target = std::move(list);
  }

  /**
   * @brief The default constructor that takes the size of the target/current value as input
   * @param args Current value argument list
   */
  template<typename... Args>
  inline void setCurrent(Args&&... args) {
    len = sizeof...(args);
    std::vector<T> list = {args...};
    current = std::move(list);
  }
  
  /**
   * @brief Set initial current value with default value
   */
  inline void setInitialCurrent() {
    for(auto& c:current) {c=T();}
  }

  /**
   * @brief The target value getter
   * @return target value
   */  
  inline T getTarget(size_t index) const {
    return target[index];
  }

  inline T getTarget() const {
    return target[0];
  }

  /**
   * @brief The current value getter
   * return current value
   */  
  inline T getCurrent(size_t index) const {
    return current[index];
  }

  inline T getCurrent() const {
    return current[0];
  }
  

  inline std::vector<T> getTargetVec() const {
    return target;
  }

  inline std::vector<T> getCurrentVec() const {
    return target;
  }

  size_t len;
private:
  std::vector<T> target, current;
};

/**
 * @class Carriage
 * @brief The class that contructs the base actuator
 */
template <typename T>
class Carriage {
public:

  /**
   * @brief The default constructor
   */  
  template<typename... Args>
  Carriage(const std::string& name,
           const bool& is_complete, 
           Args&&... args):
    update_freq_(kLoopRate),
    name_(name),
    data_(sizeof...(args)),
    is_complete_(is_complete)
    {
      setTarget(std::forward<Args>(args)...);
      setInitialCurrent();
      setGoalFunction("standard");
      setEqualFunction("Roughly");
      setUpdateFrequency(kLoopRate);
    }

  /**
   * @brief The default destructor
   */  
  virtual ~Carriage() = default;

  /**
   * @brief Set goal function as which the actuator is consider finished
   * @param goal_name The name of the goal
   */  
  virtual void setGoalFunction(const std::string& goal_name) {
    goal_name_ = goal_name;
    if (goal_name_=="standard") {
      checkGoal = std::bind(&Carriage::naiveCheckGoal, this);   
    } else {
      checkGoal = std::bind(&Carriage::customizedCheckGoal, this);     
    }
  }

  /**
   * @brief Set goal function as which the actuator is consider finished
   * @param goal_name The name of the goal
   */  
  virtual void setEqualFunction(const std::string& equal_name) {
    namespace ph = std::placeholders;
    equal_name_ = equal_name;
    if (equal_name_ == "Strictly") {
      checkEqual = std::bind(&StrictlyEqual, ph::_1, ph::_2);  
    } else {
      checkEqual = std::bind(&RoughlyEqual, ph::_1, ph::_2);     
    }
  }

  /**
   * @brief Set Update frequency the actuator is working
   * @param freq the frequency in Hz
   */ 
  virtual void setUpdateFrequency(const double& freq) {
    update_freq_ = freq<=kLoopRate?freq:kLoopRate;
  }

  /**
   * @brief Check the goal in a naive comarison way (conpare current and target with specific criterion)
   * @param goal_name The name of the goal
   */ 
  bool naiveCheckGoal() {
    bool ret = true;
    for(size_t i=0;i<data_.len;i++) {
      const auto& compare_result = checkEqual(data_.getTarget(i), data_.getCurrent(i));
      ret &= compare_result;
      // std::cout << std::setprecision(7) << FUNC_NAME << "Compare result: " << compare_result 
      // << ", Target[" << i << "]: " << *(data_.getTarget(i)) 
      // << ", Current[" << i << "]: " << *(data_.getCurrent(i)) << std::endl;
      }
    return ret;
  }

  /**
   * @brief Check whether the goal is finished by the is_complete_flag
   * @return complete or not
   */ 
  inline bool customizedCheckGoal() {
    return is_complete_;
  }

  /**
   * @brief set target value and forward it to its data member
   * @param args the arguments
   */ 
  template<typename... Args>
  void setTarget(Args&&... args) {
    data_.setTarget(std::forward<Args>(args)...);
  }

  /**
   * @brief set target value and forward it to its data member
   * @param args the arguments
   */ 
  template<typename... Args>
  void setCurrent(Args&&... args) {
    data_.setCurrent(std::forward<Args>(args)...);
  }

  /**
   * @brief set initial current value to data member
   */ 
  inline void setInitialCurrent() {
    data_.setInitialCurrent();
  }

  /**
   * @brief Get the value for the first current
   * @param index the index of this current value
   * @return the pointer of the current values
   */ 
  inline T getCurrent(size_t index) const {
    return data_.getCurrent(index);
  }

  /**
   * @brief Get the value for the first current
   * @return the current value
   */
  inline T getCurrent() const {
    return data_.getCurrent();
  } 

  /**
   * @brief Get the vector for current values
   * @return the vector
   */
  inline std::vector<T> getCurrentVec() const {
    return data_.getCurrentVec();
  } 

  /**
   * @brief Get the vector for target values
   * @return the vector
   */
  inline std::vector<T> getTargetVec() const {
    return data_.getTargetVec();
  } 

  /**
   * @brief Get target value with specified index
   * @param index the index
   * @return the target value
   */  
  inline T getTarget(size_t index) const {
    return data_.getTarget(index);
  } 

  /**
   * @brief Get the first target value
   * @return the target value
   */  
  inline T getTarget() const {
    return data_.getTarget();
  } 

  /**
   * @brief The initialization function
   */ 
  virtual void init() {}

  /**
   * @brief The backbone function that iterates
   */ 
  void update() {
    proc();
    is_complete_ = checkGoal();
  }

  /**
   * @brief The detail function that iterates inside Update
   */ 
  virtual void proc() {
    std::cout << FUNC_NAME << "Carriage" << std::endl;
  }

  /**
   * @brief The detail function that execute stopping flow
   */ 
  virtual void stop() {}

  /**
   * @brief Set complate
   */ 
  inline void setComplete() {
    is_complete_ = true;
  }

  /**
   * @brief Set initialzation true
   */ 
  inline void setInit() {
    is_initialized_ = true;
  }

  /**
   * @brief Check if complete
   * @return complete or not
   */ 
  inline bool isComplete() const {
    return is_complete_;
  }

  /**
   * @brief Check if this carriage envoke the initialzation function
   * @return complete or not
   */ 
  inline bool isInited() const {
    return is_initialized_;
  }

  /**
   * @brief Observe if complete
   * @return complete or not
   */ 
  inline std::string name() const {
    return name_;
  }

  /**
   * @brief Observe the goal name
   * @return the goal name
   */ 
  inline std::string goalName() const {
    return goal_name_;
  }

  /**
   * @brief Observe the equal criterion name
   * @return the equal criterion name
   */ 
  inline std::string equalName() const {
    return equal_name_;
  }

  /**
   * @brief Data getter
   * @return data blob
   */ 
  inline Blob<T> data() const {
    return data_;
  }

  double update_freq_;
  uint32_t count_;
private:
  std::string name_;
  Blob<T> data_;
  bool is_complete_;
  std::string goal_name_;
  std::string equal_name_;
  bool is_initialized_;

  std::function<bool(const double, const double)> checkEqual;
  std::function<bool()> checkGoal;
};

} // namespace actuator_train