cmake_minimum_required(VERSION 2.8.3)

# Project configuration
project(magic_box)
set(CMAKE_BUILD_TYPE Debug)

#set c++ standard
set(CMAKE_CXX_STANDARD 11)

#set include directories
#include_directories(${PROJECT_SOURCE_DIR})

#search for SOURCES, HEADERS, MESSAGES, SERVICES and RESOURCES
set(SRC_PREFIX src/)

file(GLOB SOURCE ${SRC_PREFIX}*.cpp)
MESSAGE(STATUS "Add MAIN-SourceFiles: " ${SOURCE})

file(GLOB HEADERS ${SRC_PREFIX}*.h)
MESSAGE(STATUS "Add MAIN-HeaderFiles: " ${HEADERS})

#create relative paths to messages for catkin
file(GLOB RAW_MESSAGES msg/*.msg)
foreach(message ${RAW_MESSAGES}) 
    file(RELATIVE_PATH REL_MESSAGES ${CMAKE_CURRENT_LIST_DIR}/msg/ ${message})
    list(APPEND MESSAGES ${REL_MESSAGES})
ENDFOREACH()
MESSAGE(STATUS "Found ROS-Messages: " ${MESSAGES})

#create relative paths to services for catkin
file(GLOB RAW_SERVICES srv/*.srv)
foreach(service ${RAW_SERVICES}) 
    file(RELATIVE_PATH REL_SERVICE ${CMAKE_CURRENT_LIST_DIR}/srv/ ${service})
    list(APPEND SERVICES ${REL_SERVICE})
ENDFOREACH()
MESSAGE(STATUS "Found ROS-Services: " ${SERVICES})

file(GLOB RAW_ACTIONS action/*.action)
foreach(action ${RAW_ACTIONS})
    file(RELATIVE_PATH REL_ACTIONS ${CMAKE_CURRENT_LIST_DIR}/action/ ${action})
    list(APPEND ACTIONS ${REL_ACTIONS})
ENDFOREACH()
MESSAGE(STATUS "Found ROS-Actions: " ${ACTIONS})


#find catkin packages
find_package(catkin REQUIRED
    message_generation
    roscpp
    rospy    
    genmsg )
    
#init python
catkin_python_setup()

#add all message files
#add_message_files(
#    FILES
#    ${MESSAGES}
#)

#add all services
add_service_files(
    FILES    ${SERVICES}
)

#add all actions
#add_action_files(FILES ${ACTIONS})

#generate messages 
generate_messages(
   DEPENDENCIES
   std_msgs   
)



#create catkin package with all needed dependencies
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES #${PROJECT_NAME}
  CATKIN_DEPENDS
    message_runtime
    rospy
  #  DEPENDS system_lib
)

#include directories
include_directories(include ${CATKIN_DEVEL_PREFIX}/include)
include_directories(include ${catkin_INCLUDE_DIRS}
)

#SET GLOBAL DEPENDECIES and Links
set(LINKLIBS)
set(GLOB_LINKLIBS ${catkin_LIBRARIES} ) 
set(SUBLIBS)
set(GLOB_DEPENDENCIES 
    ${PROJECT_NAME}_generate_messages
    ${catkin_EXPORTED_TARGETS}
)

#read all directories as submodules
FILE(GLOB children RELATIVE ${CMAKE_CURRENT_LIST_DIR}/${SRC_PREFIX} ${CMAKE_CURRENT_LIST_DIR}/${SRC_PREFIX}/*)
SET(dirlist "")
FOREACH(child ${children})
    IF(IS_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/${SRC_PREFIX}/${child})
        LIST(APPEND dirlist ${child})
    ENDIF()
ENDFOREACH()
SET(SUBDIRS ${dirlist})

#add subdirectories with the components
foreach(subdir ${SUBDIRS}) 
    MESSAGE(STATUS "ADD SubComponent: " ${subdir})
    add_subdirectory(${SRC_PREFIX}/${subdir}/)
ENDFOREACH()

#create axecutable and set dependencies
#SET(NODENAME magic_box)
#add_executable(${NODENAME} ${SRC_PREFIX}magic_box.cpp)

#add_dependencies(${NODENAME} ${GLOB_DEPENDENCIES} magic_box_actions)
#target_link_libraries(${NODENAME} ${LINKLIBS} ${GLOB_LINKLIBS} magic_box_actions)


#manage installation e.g. all include files
#install(TARGETS ${PROJECT_NAME}
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )