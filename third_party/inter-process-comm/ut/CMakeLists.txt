
include_directories(${CMAKE_SOURCE_DIR}
                    ${CMAKE_SOURCE_DIR}/msgs)

file(GLOB TEST_SOURCES "*.cpp")


foreach(test_src ${TEST_SOURCES})
    get_filename_component(test_name ${test_src} NAME_WE)

    add_executable(${test_name} ${test_src})
endforeach()