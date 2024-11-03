#linuxx下
# find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
# if( Qt5_FOUND )
#     message( STATUS "Qt5 with ${Qt5_VERSION} is found!" )
#     if( TARGET Qt5::Core )
#         message( STATUS "Qt5::Core is found!" )
#     endif()
#     if( TARGET Qt5::Widgets )
#         message( STATUS "Qt5::Widgets is found!" )
#     endif()
# else()
#     message( ERROR "Qt5 is NOT found!" )
# endif()

set(Qt5_DIR  E:/tang_software/common/vs_indep/Qt5155_x64/lib/cmake/Qt5)
find_package( Qt5 CONFIG COMPONENTS Core Widgets   PATHS  ${Qt5_DIR} NO_DEFAULT_PATH)
if( Qt5_FOUND )
    message( STATUS "Qt5 with ${Qt5_VERSION} is found!" )
    if( TARGET Qt5::Core )
        message( STATUS "Qt5::Core is found!" )
    endif()
    if( TARGET Qt5::Widgets )
        message( STATUS "Qt5::Widgets is found!" )
    endif()
else()
    message( ERROR "Qt5 is NOT found!" )
endif()

set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}")
