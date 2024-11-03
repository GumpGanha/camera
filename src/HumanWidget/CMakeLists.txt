# 需要cmake的最低版本
cmake_minimum_required(VERSION 3.20)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(TARGET_NAME "HumanWidget")

# 把.cpp和.c文件都塞到SOURCE_FILES里
file(GLOB SOURCE_FILES_ "*.cpp" "*.c" )

# 把.h文件都塞到HEADER_FILES里
file(GLOB HEAD_FILES "./*.h")

file(GLOB UI_FILES "./*.ui")
source_group( "Form Files" FILES ${UI_FILES} )

file(GLOB QRC_FILES "./*.qrc")
source_group( "Resource Files" FILES ${QRC_FILES} )

file( GLOB INTERFACE_HEADERS 
    ${PROJECT_SOURCE_DIR}/${TARGET_NAME}/*.h
    ${PROJECT_SOURCE_DIR}/${TARGET_NAME}/*.hpp
)
# 设置OpenCV库的路径
set(OPENCV_PATH "${PROJECT_SOURCE_DIR}/Human+/opencv340")

# 添加可执行文件
add_executable( ${TARGET_NAME} 
 ${SOURCE_FILES_} ${HEAD_FILES} ${UI_FILES} ${QRC_FILES} ${INTERFACE_HEADERS} )


#复制json文件
add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_CURRENT_SOURCE_DIR}/ConfigFiles"
    "${PROJECT_BINARY_DIR}/${TARGET_NAME}/ConfigFiles"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_CURRENT_SOURCE_DIR}/json_glasses.json"
   "${PROJECT_BINARY_DIR}/${TARGET_NAME}/json_glasses.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_CURRENT_SOURCE_DIR}/json_object.json"
    "${PROJECT_BINARY_DIR}/${TARGET_NAME}/json_object.json"
    )

# 添加头文件路径
target_include_directories( ${TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OPENCV_PATH}/include
    ${OPENCV_PATH}/include/opencv2
)

# 链接其他的库
target_link_libraries( ${TARGET_NAME}
    PRIVATE
        $<BUILD_INTERFACE:Qt5::Widgets>
        $<BUILD_INTERFACE:Qt5::Core>
        $<BUILD_INTERFACE:Boost::headers>
        $<BUILD_INTERFACE:Boost::thread>
        $<BUILD_INTERFACE:Human+::Human+>

)


