#######################################################
#                     Find Boost                      #
#           Boost should be found before pinocchio    #
#######################################################
set( CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE )

set(Boost_Components 
    locale date_time 
    filesystem timer 
    regex 
    thread 
    serialization 
    system 
    program_options 
    iostreams
    coroutine)
# linux下
# find_package(Boost REQUIRED COMPONENTS ${Boost_Components} )

set(Boost_DIR E:/tang_software/common/vs2019/boost_1_78_0/lib64-msvc-14.2/cmake/Boost-1.78.0)
# set(Boost_DIR F:/thirdparty/boost_1_80_0)
find_package(Boost REQUIRED COMPONENTS ${Boost_Components} PATHS ${Boost_DIR})


# if(Boost_FOUND)
#     message( STATUS "Boost is found!")

# else()
#     message(FATAL_ERROR "Boost is not found! Timing is not disabled! ")
# endif()
if( Boost_FOUND )
    message( STATUS "Boost version ${Boost_VERSION} is FOUND!" )
    foreach( each_component  ${Boost_Components} )
        if( TARGET Boost::${each_component} )
            set( Boost_Found_Components ${Boost_Found_Components} Boost::${each_component} )
            message( STATUS "Target Boost::${each_component} is FOUND!" )
        else()
            message( STATUS "Target Boost::${each_component} is NOT found!" )
        endif()
    endforeach()

    #   How to Using Boost
    message( STATUS "\nTips of Using Boost: " )
    message( STATUS "-- Using Boost headers: Boost::headers" )
    message( STATUS "-- Using Boost targets: ${Boost_Found_Components} " )
else()
    message(STATUS "Boost is NOT found!")
endif( Boost_FOUND )

