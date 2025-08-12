cmake_minimum_required(VERSION 3.10)

cmake_policy(SET CMP0072 NEW)

project(piper_dual_arm_teleop)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_CLIENT "Build real hardware piper arm controller client" ON)
option(BUILD_SERVER "Build MuJoCo GUI simulator server" ON)
option(BUILD_TESTS  "Build tests" OFF)
option(BUILD_TOOL   "Build piper manipulator manager tool" ON)

add_definitions(-DPROJECT_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

if ( BUILD_SERVER )
    set(OpenGL_GL_PREFERENCE "GLVND")
    find_package(OpenGL REQUIRED)
    find_package(glfw3 REQUIRED)
    add_executable(server
        src/server.cpp
        third_party/mujoco-3.3.2-linux-x86_64/simulate/glfw_adapter.cc
        third_party/mujoco-3.3.2-linux-x86_64/simulate/glfw_dispatch.cc
        third_party/mujoco-3.3.2-linux-x86_64/simulate/platform_ui_adapter.cc
        third_party/mujoco-3.3.2-linux-x86_64/simulate/simulate.cc
        third_party/lodepng/lodepng.cpp
        third_party/lodepng/lodepng_fuzzer.cpp
        third_party/lodepng/lodepng_util.cpp
    )
    target_include_directories(server PRIVATE
        include
        third_party/spdlog-1.15.3/include
        third_party/asio-1.12.2/include
        third_party/eigen-3.4.0
        third_party/mujoco-3.3.2-linux-x86_64/simulate
        third_party/mujoco-3.3.2-linux-x86_64/include
        third_party/lodepng
        third_party/inter-process-comm/itc
        third_party/inter-process-comm
    )
    target_link_directories(server PRIVATE
        third_party/mujoco-3.3.2-linux-x86_64/lib
    )
    target_link_libraries(server PRIVATE
        OpenGL::GL
        glfw
        libmujoco.so
        pthread
    )
endif()

if ( BUILD_CLIENT )
    add_executable(client src/client.cpp)
    target_include_directories(client PRIVATE
        include
        third_party/spdlog-1.15.3/include
        third_party/asio-1.12.2/include
        third_party/eigen-3.4.0
        third_party/inter-process-comm/itc
        third_party/inter-process-comm
    )
endif()

if ( BUILD_TESTS )
    add_executable(test_mujoco_backend
        test/test_mujoco_backend.cpp
    )
    target_include_directories(test_mujoco_backend PRIVATE
        include
        third_party/eigen-3.4.0
        third_party/mujoco-3.3.2-linux-x86_64/include
    )
    target_link_directories(test_mujoco_backend PRIVATE
        third_party/mujoco-3.3.2-linux-x86_64/lib
    )
    target_link_libraries(test_mujoco_backend PRIVATE
        libmujoco.so
        pthread
    )
endif()

if ( BUILD_TOOL )
    add_executable(pipermanager 
        src/pipermanager.cpp
    )
    target_include_directories(pipermanager PRIVATE
        include
        third_party/spdlog-1.15.3/include
        third_party/eigen-3.4.0
    )
endif()