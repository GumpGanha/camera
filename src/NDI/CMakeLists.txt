cmake_minimum_required(VERSION 3.10)
 
# project( NDI_MFC_Demo )
set(TARGET_NAME "NDIMFC")
 
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
SET(CMAKE_MFC_FLAG 2)

message( STATUS "\n-------------   ${TARGET_NAME}  -------------")
# find_package(MFC REQUIRED)
 
# include_directories(${MFC_INCLUDE_DIRS})
 
set(USE_MFC ON)
 
file(GLOB_RECURSE SRC_FILES
    "NDIOfficial/*.h"
    "NDIOfficial/*.cpp"
)
 
file(GLOB_RECURSE RC_FILES
    "NDIOfficial/*.rc"
    # "*.rc2"
)
 
add_executable( ${TARGET_NAME} WIN32 ${SRC_FILES} ${RC_FILES} )
 
target_link_libraries( ${TARGET_NAME} ${MFC_LIBRARIES} 
    PRIVATE
        $<BUILD_INTERFACE:Boost::headers>
        $<BUILD_INTERFACE:Boost::thread>
        $<BUILD_INTERFACE:Boost::filesystem>
        $<BUILD_INTERFACE:Boost::iostreams>
    PUBLIC
        $<BUILD_INTERFACE:NDIClient::NDIClient>
)

target_compile_definitions( ${TARGET_NAME} PRIVATE
    -DWIN32
    -D_DEBUG
    -D_WINDOWS
    # -D_UNICODE
    # -DUNICODE
    -D_AFXDLL
    # _BIND_TO_CURRENT_CRT_VERSION
    # _BIND_TO_CURRENT_MFC_VERSION
)

# target_link_options( ${PROJECT_NAME} PRIVATE
#     /ENTRY:wWinMainCRTStartup
#     # /ENTRY:mainCRTStartup
# )

message( STATUS "\n-------------   ${TARGET_NAME}  -------------")

