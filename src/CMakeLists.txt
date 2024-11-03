cmake_minimum_required(VERSION 3.0.0)

project(NDIProject VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CMakeDependentOption)
include(CmakePrintHelpers)



#Find boost
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR}/cmake )

include( FindBoost )
#Find Qt
include( FindQt )
#find eigen
get_filename_component( Eigen_DIR E:/tang_software/RobotSimulator_v4.2.1_20230726-2/thirdparty/Eigen340/share/eigen3/cmake ABSOLUTE )
find_package( Eigen3 CONFIG PATHS ${Eigen_DIR} NO_DEFAULT_PATH )
# find MFC
get_filename_component( Eigen_DIR E:/tang_software/RobotSimulator_v4.2.1_20230726-2/thirdparty/Eigen340/share/eigen3/cmake ABSOLUTE )
# find_package(MFC REQUIRED)
include_directories(${MFC_INCLUDE_DIRS})

 set(USE_MFC ON)
# if(MFC_FOUND)
   #  message( STATUS "Boost is found!")
# endif()
   # message(FATAL_ERROR "MFC is not found! Timing is not disabled! ")

add_subdirectory(NDI)
add_subdirectory(NDIClient)
add_subdirectory(Human+)
add_subdirectory(HumanWidget)
