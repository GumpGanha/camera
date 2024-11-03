
############################################################
#	CMake requirement
#############################################################
cmake_minimum_required( VERSION 3.20 )

#	Set project name
set( TARGET_NAME "Human+" )
message( STATUS "\n-------------   ${TARGET_NAME}  -------------")


#############################################################
#	Add the head and source files to the current project
#############################################################
file(GLOB SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/lib/*.cpp")

FILE(GLOB HEAD_FILES "./*.h" "./*.hpp")


file(GLOB INTERFACE_HEADERS 
    ${PROJECT_SOURCE_DIR}/${TARGET_NAME}/include/*.h
    ${PROJECT_SOURCE_DIR}/${TARGET_NAME}/include/*.hpp
    )

#############################################################
#	Create target and Set the target prorperties
#############################################################
#	Create target
#add_executable( ${TARGET_NAME} ${SRC_FILES} ${HEAD_FILES} ${INTERFACE_HEADERS})


#[[
set (INCLUDE_PATH   "${CMAKE_CURRENT_SOURCE_DIR}/opencv340")
include_directories(${INCLUDE_PATH}/include)     
#include_directories(${INCLUDE_PATH}/include/opencv2)     

link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/opencv340/x64/lib)

# 全路径引入
LINK_LIBRARIES("C:/Users/GumpGan/Desktop/lab_ur3/human+001/src/Human+/opencv340/x64/bin/opencv_world340d.dll")
LINK_LIBRARIES("C:/Users/GumpGan/Desktop/lab_ur3/human+001/src/Human+/opencv340/x64/bin/opencv_ffmpeg340_64.dll")


target_include_directories(${TARGET_NAME} PUBLIC
${INCLUDE_PATH}/include
)

# 链接库
target_link_libraries(${TARGET_NAME}
    optimized ${OPENCV_PATH}/x64/lib/opencv_world340.lib
    debug ${OPENCV_PATH}/x64/lib/opencv_world340d.lib
)
]]#


# 设置OpenCV库的路径
set(OPENCV_PATH "${CMAKE_CURRENT_SOURCE_DIR}/opencv340")

# 添加头文件目录
include_directories(${OPENCV_PATH}/include)
include_directories(${OPENCV_PATH}/include/opencv2)

# 指定库目录
link_directories(${OPENCV_PATH}/x64/lib)

# 把${SOURCE_FILES} ${HEADER_FILES} 等包装成一个静态库
add_library(${TARGET_NAME} STATIC ${SRC_FILES} ${HEAD_FILES} ${INTERFACE_HEADERS})
add_library(${TARGET_NAME}::${TARGET_NAME} ALIAS ${TARGET_NAME})



# 复制DLL文件到可执行文件所在的目录
add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${OPENCV_PATH}/x64/bin/opencv_world340d.dll"
    $<TARGET_FILE_DIR:${TARGET_NAME}>)

add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${OPENCV_PATH}/x64/bin/opencv_ffmpeg340_64.dll"
    $<TARGET_FILE_DIR:${TARGET_NAME}>)

add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${OPENCV_PATH}/x64/bin/MoveSenseCameraD.dll"
    $<TARGET_FILE_DIR:${TARGET_NAME}>)

#   Current Include Directories
target_include_directories( ${TARGET_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}/${TARGET_NAME}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    )

#   Add dependancies
target_link_libraries( ${TARGET_NAME}
    PUBLIC
        # $<BUILD_INTERFACE:Eigen3::Eigen>
        optimized ${OPENCV_PATH}/x64/lib/opencv_world340.lib
        debug ${OPENCV_PATH}/x64/lib/opencv_world340d.lib
        debug ${OPENCV_PATH}/x64/lib/MoveSenseCameraD.lib
    PRIVATE
        $<BUILD_INTERFACE:Boost::headers>
        $<BUILD_INTERFACE:Boost::thread>
        $<BUILD_INTERFACE:Boost::filesystem>
        $<BUILD_INTERFACE:Boost::iostreams>
      
)

    message( STATUS "-------------   ${TARGET_NAME}  -------------\n")
   

















#[[
    #############################################################
#	CMake requirement
#############################################################
cmake_minimum_required(VERSION 3.20)

#	Set project name
set(TARGET_NAME "Human+")
message(STATUS "\n-------------   ${TARGET_NAME}  -------------")


#############################################################
#	Add the head and source files to the current project
#############################################################
file(GLOB_RECURSE SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/lib/*.cpp")
#file(GLOB_RECURSE HEAD_FILES "${CMAKE_CURRENT_SOURCE_DIR}/lib/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/lib/*.hpp")
FILE(GLOB HEAD_FILES "./*.h" "./*.hpp")
# Add interface headers (if they are in a different directory)
file(GLOB INTERFACE_HEADERS 
    "${PROJECT_SOURCE_DIR}/include/${TARGET_NAME}/*.h"
    "${PROJECT_SOURCE_DIR}/include/${TARGET_NAME}/*.hpp"
)

#############################################################
#	Create target and Set the target properties
#############################################################
#	Create a static library target
add_library(${TARGET_NAME} STATIC ${SRC_FILES} ${HEAD_FILES} ${INTERFACE_HEADERS})
add_library(${TARGET_NAME}::${TARGET_NAME} ALIAS ${TARGET_NAME})
# 设置OpenCV库的路径
set(OPENCV_PATH "${CMAKE_CURRENT_SOURCE_DIR}/opencv340")

# 添加头文件目录
target_include_directories(${TARGET_NAME} PUBLIC
    ${OPENCV_PATH}/include
    ${OPENCV_PATH}/include/opencv2
    ${PROJECT_SOURCE_DIR}/include  # Include directory for the headers
)

# 指定库目录
link_directories(${OPENCV_PATH}/x64/lib)

# 链接OpenCV库
target_link_libraries(${TARGET_NAME}
    PUBLIC
        optimized ${OPENCV_PATH}/x64/lib/opencv_world340.lib
        debug ${OPENCV_PATH}/x64/lib/opencv_world340d.lib
        debug ${OPENCV_PATH}/x64/lib/MoveSenseCameraD.lib
    PRIVATE
        Boost::headers
        Boost::thread
        Boost::filesystem
        Boost::iostreams
)

# 复制DLL文件到库文件所在的目录
add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${OPENCV_PATH}/x64/bin/opencv_world340d.dll"
    $<TARGET_FILE_DIR:${TARGET_NAME}>)

add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${OPENCV_PATH}/x64/bin/opencv_ffmpeg340_64.dll"
    $<TARGET_FILE_DIR:${TARGET_NAME}>)

add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${OPENCV_PATH}/x64/bin/MoveSenseCameraD.dll"
    $<TARGET_FILE_DIR:${TARGET_NAME}>)

# Export library for use by other projects
install(TARGETS ${TARGET_NAME} EXPORT ${TARGET_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${TARGET_NAME}/
    DESTINATION include/${TARGET_NAME}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

#[[ Create and install CMake config files
include(CMakePackageConfigHelpers)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}Config.cmake"
    INSTALL_DESTINATION lib/cmake/${TARGET_NAME}
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}ConfigVersion.cmake"
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/${TARGET_NAME}
)
]]#
message(STATUS "-------------   ${TARGET_NAME}  -------------\n")