cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 14)

project(astar_ros)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        geometry_msgs
        nav_msgs
        tf
        message_generation
        )

add_service_files(
  FILES
  GetPlan.srv
)

generate_messages(DEPENDENCIES geometry_msgs nav_msgs)

catkin_package(
        CATKIN_DEPENDS roscpp rospy geometry_msgs nav_msgs tf message_runtime
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/astar_server.cpp src/Astar.cpp src/OccMapTransform.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})