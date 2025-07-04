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

add_executable(deploy
    src/arm_interface.cpp
    src/arm_controller.cpp
    src/arm_model.cpp
    src/arm_planner.cpp
    src/deploy.cpp
)
target_include_directories(deploy PRIVATE
    include
    third_party/asio-1.34.2/include
    third_party/eigen-3.4.0
    third_party/spdlog-1.15.3/include
    third_party/inter-process-comm/itc
    third_party/inter-process-comm
)
target_link_libraries(deploy
    pthread
)

if(BUILD_TESTING)
    set(OpenGL_GL_PREFERENCE "GLVND")
    find_package(OpenGL REQUIRED)
    find_package(glfw3 REQUIRED)
    add_executable(simulate
        src/arm_interface.cpp
        src/arm_controller.cpp
        src/arm_model.cpp
        src/arm_planner.cpp
        src/sim.cpp
        third_party/mujoco-3.3.2-linux-x86_64/simulate/glfw_adapter.cc
        third_party/mujoco-3.3.2-linux-x86_64/simulate/glfw_dispatch.cc
        third_party/mujoco-3.3.2-linux-x86_64/simulate/platform_ui_adapter.cc
        third_party/mujoco-3.3.2-linux-x86_64/simulate/simulate.cc
        third_party/lodepng/lodepng.cpp
        third_party/lodepng/lodepng_fuzzer.cpp
        third_party/lodepng/lodepng_util.cpp
    )
    target_include_directories(simulate PRIVATE
        include
        third_party/spdlog-1.15.3/include
        third_party/asio-1.34.2/include
        third_party/eigen-3.4.0
        third_party/mujoco-3.3.2-linux-x86_64/simulate
        third_party/mujoco-3.3.2-linux-x86_64/include
        third_party/lodepng
        third_party/inter-process-comm/itc
        third_party/inter-process-comm
    )
    target_link_directories(simulate PRIVATE
        third_party/mujoco-3.3.2-linux-x86_64/lib
    )
    target_link_libraries(simulate PRIVATE
        OpenGL::GL
        glfw
        libmujoco.so
        pthread
    )
endif()