cmake_minimum_required(VERSION 3.0.2)
project(ardusub_control)

## 编译配置
add_compile_options(-std=c++11)

## 查找catkin宏和库
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

## 系统依赖包
# find_package(Boost REQUIRED COMPONENTS system)

## 取消注释以下行，如果包具有setup.py。
# catkin_python_setup()

################################################
## 声明ROS消息、服务和动作 ##
################################################

## 生成消息在'msg'文件夹中
add_message_files(
  FILES
  ArduSubCommand.msg
  ArduSubStatus.msg
)

## 生成服务在'srv'文件夹中
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## 生成动作在'action'文件夹中
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## 生成添加的消息和服务与指定的依赖关系
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

################################################
## 声明ROS动态重新配置参数 ##
################################################

## 生成动态重新配置参数在'cfg'文件夹中
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin特定配置 ##
###################################
## catkin_package宏生成cmake配置文件，供其他包使用
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ardusub_control
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs message_runtime
#  DEPENDS system_lib
)

###########
## 构建 ##
###########

## 指定包含目录
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## 声明C++库
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ardusub_control.cpp
# )

## 添加cmake目标依赖关系
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 声明C++可执行文件
# add_executable(${PROJECT_NAME}_node src/ardusub_control_node.cpp)

## 重命名C++可执行文件而不加前缀
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## 添加cmake目标依赖关系
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 指定要链接到可执行文件的库
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## 安装 ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## 标记可执行文件和/或库进行安装
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## 标记cpp头文件进行安装
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## 标记其他文件进行安装(例如启动和bag文件等)
install(FILES
  launch/ardusub_control.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## 安装Python脚本
catkin_install_python(PROGRAMS
  scripts/ardusub_ros_bridge.py
  scripts/ardusub_keyboard_ros.py
  scripts/ardusub_gui_controller.py
  scripts/ardusub_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## 测试 ##
#############

## 添加gtest基础单元测试
# catkin_add_gtest(${PROJECT_NAME}-test src/test_ardusub_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## 添加文件夹的测试
# catkin_add_nosetests(test)
