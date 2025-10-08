cmake_minimum_required(VERSION 3.8)
project(scissor_lift_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Num.msg"
#   "srv/AddThreeInts.srv"
# )

# Install directories
install(DIRECTORY
  urdf
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()