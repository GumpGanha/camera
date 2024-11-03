#[[ 需要cmake的最低版本
cmake_minimum_required(VERSION 3.20)

#message( STATUS "\n-------------- ${TARGET_NAME}  -------------")

set(TARGET_NAME "Human+")

# 把.cpp和.c文件都塞到SOURCE_FILES里
file(GLOB SOURCE_FILES_ "*.cpp" "*.c" )
# 把.cpp和.c文件都塞到HEADER_FILES里
file(GLOB HEADER_FILES "*.h" )

file(GLOB INTERFACE_HEADERS 
    ${PROJECT_SOURCE_DIR}/include/${TARGET_NAME}/*.h
    ${PROJECT_SOURCE_DIR}/include/${TARGET_NAME}/*.hpp
    )

# 把${SOURCE_FILES} ${HEADER_FILES} 等包装成一个静态库
#add_library(HumanLib STATIC ${SRC_FILES} ${HEAD_FILES} ${INTERFACE_HEADERS})
#add_library(HumanLib::HumanLib ALIAS HumanLib)

# 添加头文件路径
target_include_directories( ${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/${TARGET_NAME}>
    ) 



# 链接其他的库
target_link_libraries( ${TARGET_NAME}
    PRIVATE
        $<BUILD_INTERFACE:Boost::headers>
        $<BUILD_INTERFACE:Boost::thread>
)
]]#