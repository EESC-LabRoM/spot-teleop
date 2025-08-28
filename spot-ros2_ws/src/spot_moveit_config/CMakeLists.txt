cmake_minimum_required(VERSION 3.22)
project(spot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(spot_description REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/continuos_to_spot.py 
        DESTINATION lib/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
