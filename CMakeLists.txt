cmake_minimum_required(VERSION 3.8)

cmake_policy(SET CMP0072 NEW)

project(pathfinder_control)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(
    -DPROJECT_PATH="${CMAKE_CURRENT_SOURCE_DIR}"
)

add_executable(control
    src/arm_interface.cpp
    src/arm_control.cpp
    src/main.cpp
    src/log.cpp
)
target_include_directories(control PRIVATE
    include
    third_party/asio-1.34.2/include
    third_party/eigen-3.4.0
    third_party/spdlog-1.15.3/include
)
target_link_libraries(control
    pthread
)

if(BUILD_TESTING)
    set(OpenGL_GL_PREFERENCE "GLVND")
    find_package(OpenGL REQUIRED)
    find_package(glfw3 REQUIRED)
    add_executable(sim
        src/arm_control.cpp
        src/sim.cpp
        src/log.cpp
        third_party/mujoco-3.3.2-linux-x86_64/simulate/glfw_adapter.cc
        third_party/mujoco-3.3.2-linux-x86_64/simulate/glfw_dispatch.cc
        third_party/mujoco-3.3.2-linux-x86_64/simulate/platform_ui_adapter.cc
        third_party/mujoco-3.3.2-linux-x86_64/simulate/simulate.cc
        third_party/lodepng/lodepng.cpp
        third_party/lodepng/lodepng_fuzzer.cpp
        third_party/lodepng/lodepng_util.cpp
    )
    target_include_directories(sim PRIVATE
        include
        third_party/spdlog-1.15.3/include
        third_party/asio-1.34.2/include
        third_party/eigen-3.4.0
        third_party/mujoco-3.3.2-linux-x86_64/simulate
        third_party/mujoco-3.3.2-linux-x86_64/include
        third_party/lodepng
    )
    target_link_directories(sim PRIVATE
        third_party/mujoco-3.3.2-linux-x86_64/lib
    )
    target_link_libraries(sim PRIVATE
        OpenGL::GL
        glfw
        libmujoco.so
        pthread
    )
endif()