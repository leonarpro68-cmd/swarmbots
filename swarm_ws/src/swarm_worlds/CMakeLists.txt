cmake_minimum_required(VERSION 3.8)
project(swarm_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
