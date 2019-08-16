cmake_minimum_required (VERSION 2.8.3)
project(actuator_train)
 
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

include_directories(
  include
)

file(GLOB_RECURSE actuator_train_srcs
  src/carriage/*.cpp
  src/train_factory.cpp
)
add_library(${PROJECT_NAME} SHARED
  ${actuator_train_srcs}
)

add_executable(${PROJECT_NAME}_example
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_example
  ${PROJECT_NAME}
)