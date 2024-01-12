cmake_minimum_required(VERSION 3.20)
project(TemperatureManager)
# find YARP
find_package(YARP REQUIRED)
# set up our program
add_executable(${PROJECT_NAME} ./src/TemperatureManager.h ./src/TemperatureManager.cpp  ./src/main.cpp )

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/config.ini DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})

# link with YARP libraries
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})

install(TARGETS ${PROJECT} DESTINATION bin)