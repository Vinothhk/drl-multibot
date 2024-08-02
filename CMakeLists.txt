cmake_minimum_required(VERSION 3.8)
project(turtlebot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
    dqn_agent/dqn_agent_1.py
    dqn_environment/dqn_environment_1.py
    dqn_gazebo/dqn_gazebo_1.py
    dqn_agent/dqn_agent_2.py
    dqn_environment/dqn_environment_2.py
    dqn_gazebo/dqn_gazebo_2.py
    dqn_agent/dqn_agent_3.py
    dqn_environment/dqn_environment_3.py
    dqn_gazebo/dqn_gazebo_3.py

    dqn_environment/env_test_1.py
    dqn_gazebo/gazebo_test_1.py
    dqn_test/dqn_test_1.py

    graphs/action1.py
    graphs/result1.py
    graphs/action2.py
    graphs/result2.py
    graphs/action3.py
    graphs/result3.py

    algorithms/maxq.py
    algorithms/cqlite.py
    algorithms/sarsa.py
    algorithms/qlearning.py
    
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    model_weights
    urdf
    rviz
    launch
    algorithms
    meshes
    models
    config
    worlds
    maps
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
