cmake_minimum_required(VERSION 3.5)
project(Example)

set(CMAKE_CXX_STANDARD 11)


set(CMAKE_INCLUDE_CURRENT_DIR   ON)
set(CMAKE_VERBOSE_MAKEFILE      ON)
set(CMAKE_COLOR_MAKEFILE        ON)


if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()


option(USE_CONAN "Use the Conan package manager to automatically fetch the Kortex API" ON)
option(DOWNLOAD_API "Automatically download the API if conan is not used" ON)


# 设置 CMake 策略以支持 Boost_ROOT 变量
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

# 设置 Boost 的根目录路径（根据你的实际路径）
# Please Change the directions of this library for yourself
set(Boost_ROOT "G:/BootsLibrary/boost_1_85_0/boost_1_85_0")

# 查找 Boost 库，要求找到 system 模块
find_package(Boost 1.85 REQUIRED COMPONENTS)

# 检查是否找到了 Boost 库
if (Boost_FOUND)
    # 包含 Boost 库的头文件目录
    include_directories(${Boost_INCLUDE_DIRS})


else()
    message(FATAL_ERROR "Could not find Boost libraries. Please make sure BOOST_ROOT is set correctly.")
endif()

macro(configure_msvc_runtime)

    # Default to statically-linked runtime.
    if("${MSVC_RUNTIME}" STREQUAL "")
        set(MSVC_RUNTIME "static")
    endif()

    # Set compiler options.
    set(variables
            CMAKE_C_FLAGS_DEBUG
            CMAKE_C_FLAGS_MINSIZEREL
            CMAKE_C_FLAGS_RELEASE
            CMAKE_C_FLAGS_RELWITHDEBINFO
            CMAKE_CXX_FLAGS_DEBUG
            CMAKE_CXX_FLAGS_MINSIZEREL
            CMAKE_CXX_FLAGS_RELEASE
            CMAKE_CXX_FLAGS_RELWITHDEBINFO
    )

    if(${MSVC_RUNTIME} STREQUAL "static")
        message(STATUS
                "MSVC -> forcing use of statically-linked runtime."
        )
        foreach(variable ${variables})
            if(${variable} MATCHES "/MD")
                string(REGEX REPLACE "/MD" "/MT" ${variable} "${${variable}}")
            endif()
        endforeach()

    else()
        message(STATUS
                "MSVC -> forcing use of dynamically-linked runtime."
        )
        foreach(variable ${variables})
            if(${variable} MATCHES "/MT")
                string(REGEX REPLACE "/MT" "/MD" ${variable} "${${variable}}")
            endif()
        endforeach()

    endif()

endmacro()


include_directories(${PROJECT_SOURCE_DIR}/thirdParty/cxxopts/)

# Please Change the directions of this library for yourself
include_directories(G:\\eigin\\eigen-3.4.0\\eigen-3.4.0)

if(MSVC)
    configure_msvc_runtime()
else()
    add_compile_options(-Wall)
    add_compile_options(-Wno-reorder)

endif()

if(UNIX)
    add_definitions(-D_OS_UNIX)
elseif(WIN32)
    add_definitions(-D_OS_WINDOWS -DNOMINMAX)
    if(MSVC)
        add_compile_options(/bigobj)
    endif()
endif()


# Setup Kortex Api Path
if(NOT KORTEX_SUB_DIR)
    set(KORTEX_SUB_DIR "")
else()
    set(KORTEX_SUB_DIR "${KORTEX_SUB_DIR}/")
endif()

set(KORTEX_DIR "${PROJECT_SOURCE_DIR}/kortex_api/${KORTEX_SUB_DIR}")

if(CMAKE_BUILD_TYPE EQUAL "Debug")
    set(KORTEX_LIB_SUBDIR "debug")
else()
    set(KORTEX_LIB_SUBDIR "release")
endif()

# Download the API
if(DOWNLOAD_API)
    if(UNIX)
        execute_process(COMMAND ./download_kortex_api.sh ${KORTEX_SUB_DIR}
                WORKING_DIRECTORY ../scripts
                RESULT_VARIABLE DOWNLOAD_API_RESULT
                OUTPUT_VARIABLE DOWNLOAD_API_OUTPUT)
        if(NOT DOWNLOAD_API_RESULT EQUAL 0)
            message("Kortex API was not downloaded prior to running CMake.")
            message(FATAL_ERROR ${DOWNLOAD_API_OUTPUT})
        endif()
    elseif(WIN32)
        execute_process(COMMAND ./download_kortex_api.bat ${KORTEX_SUB_DIR}
                WORKING_DIRECTORY ../scripts
                RESULT_VARIABLE DOWNLOAD_API_RESULT
                OUTPUT_VARIABLE DOWNLOAD_API_OUTPUT)
        if(NOT DOWNLOAD_API_RESULT EQUAL 0)
            message("Kortex API was not downloaded prior to running CMake.")
            message(FATAL_ERROR ${DOWNLOAD_API_OUTPUT})
        endif()
    endif()
endif()
if(UNIX)
    link_libraries(${KORTEX_DIR}lib/${KORTEX_LIB_SUBDIR}/libKortexApiCpp.a)
elseif(WIN32)
    link_libraries(${KORTEX_DIR}lib/${KORTEX_LIB_SUBDIR}/KortexApiCpp.lib)
endif()


# Add Include Directories
include_directories(${KORTEX_DIR}include)
include_directories(${KORTEX_DIR}include/client)
include_directories(${KORTEX_DIR}include/common)
include_directories(${KORTEX_DIR}include/messages)
include_directories(${KORTEX_DIR}include/client_stubs)



# link other libs
if(UNIX)
    link_libraries(pthread)
elseif(WIN32)
    link_libraries(winMM ws2_32)
else()
    MESSAGE(FATAL_ERROR "Unknown os! Not supported yet")
endif()



add_executable(Example
        utilities.cpp
        Low-level-control/main.cpp
        Low-level-control/Jacobian.cpp
        Low-level-control/Trajectory.cpp
        Low-level-control/ForwardKinematic.cpp
        Low-level-control/Constants.cpp
        Low-level-control/InverseKinematic.cpp
        Low-level-control/SerialReader.cpp
)

add_executable(test
        utilities.cpp
        Low-level-control/test.cpp
        Low-level-control/Jacobian.cpp
        Low-level-control/Trajectory.cpp
        Low-level-control/ForwardKinematic.cpp
        Low-level-control/Constants.cpp
        Low-level-control/InverseKinematic.cpp
        Low-level-control/SerialReader.cpp
)

add_executable(main_FC
        utilities.cpp
        Low-level-control/main_FC.cpp
        Low-level-control/Jacobian.cpp
        Low-level-control/Trajectory.cpp
        Low-level-control/ForwardKinematic.cpp
        Low-level-control/Constants.cpp
        Low-level-control/InverseKinematic.cpp
        Low-level-control/SerialReader.cpp
)

add_executable(main_FC_Velocity
        utilities.cpp
        Low-level-control/main_FC_Velocity.cpp
        Low-level-control/Jacobian.cpp
        Low-level-control/Trajectory.cpp
        Low-level-control/ForwardKinematic.cpp
        Low-level-control/Constants.cpp
        Low-level-control/InverseKinematic.cpp
        Low-level-control/SerialReader.cpp
)

target_link_libraries(Example ${Boost_LIBRARIES})