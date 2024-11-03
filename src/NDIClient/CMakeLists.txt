#############################################################
#	CMake requirement
#############################################################
cmake_minimum_required(VERSION 3.20)

#############################################################
#	Set project name, files, dependancies
#############################################################
#	Set project name
set( TARGET_NAME "NDIClient" )

message( STATUS "\n-------------   ${TARGET_NAME}  -------------")

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fPIC")

#############################################################
# 把.cpp和.c文件都塞到SOURCE_FILES里
file(GLOB SOURCES "*.cpp" "*.c" )
# 把.h文件都塞到HEADER_FILES里
file(GLOB HEADERS "./*.h"  "*.hpp" )
file(GLOB INTERFACE_HEADERS 
    ${PROJECT_SOURCE_DIR}/include/${TARGET_NAME}/*.h
    ${PROJECT_SOURCE_DIR}/include/${TARGET_NAME}/*.hpp
    )


#############################################################
#	Create target and Set the target prorperties
#############################################################
#	Create target
add_library( ${TARGET_NAME} STATIC ${SOURCES} ${HEADERS} ${INTERFACE_HEADERS} )
add_library( ${TARGET_NAME}::${TARGET_NAME} ALIAS ${TARGET_NAME})

#  Set project include directories
target_include_directories( ${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/${TARGET_NAME}>
        
    ) 

#   Add dependancies
target_link_libraries( ${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:Eigen3::Eigen>

    PRIVATE
        $<BUILD_INTERFACE:Boost::headers>
        $<BUILD_INTERFACE:Boost::thread>
        $<BUILD_INTERFACE:Boost::filesystem>
        $<BUILD_INTERFACE:Boost::iostreams>
      
        
)


message( STATUS "-------------   ${TARGET_NAME}  -------------\n")



