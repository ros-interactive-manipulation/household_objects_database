cmake_minimum_required(VERSION 2.8.3)
project(household_objects_database)

set(PACKAGE_DEPS
  household_objects_database_msgs
  manipulation_msgs
  sql_database
  roscpp
  tf
  actionlib 
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPS})
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package(
  INCLUDE_DIRS include
  )

include_directories(include ply ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
  src/objects_database.cpp
  src/database_helper_classes.cpp
)

add_library(mesh_loader 
  src/mesh_loader.cpp
  src/ply.c
)

add_executable(objects_database_node 
  nodes/objects_database_node.cpp
)
target_link_libraries(objects_database_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(insert_model 
  src/insert_model.cpp
)
target_link_libraries(insert_model ${PROJECT_NAME} mesh_loader boost_filesystem boost_system ${catkin_LIBRARIES})



