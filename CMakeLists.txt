cmake_minimum_required(VERSION 3.20)

project(rm3d_camera_sim LANGUAGES CXX C)

add_library(imgui STATIC
    third_party/imgui/imgui.cpp
    third_party/imgui/imgui_draw.cpp
    third_party/imgui/imgui_tables.cpp
    third_party/imgui/imgui_widgets.cpp
    third_party/imgui/backends/imgui_impl_win32.cpp
    third_party/imgui/backends/imgui_impl_opengl2.cpp
)
target_include_directories(imgui PUBLIC
    third_party/imgui
    third_party/imgui/backends
)
target_compile_features(imgui PUBLIC cxx_std_17)
if (MSVC)
    target_compile_options(imgui PRIVATE /W0)
endif()

add_executable(rm_camera_sim
    main.cpp
    control_panel.cpp
    control_panel.h
    simulator.cpp
    simulator.h
)

target_compile_features(rm_camera_sim PRIVATE cxx_std_17)
target_link_libraries(rm_camera_sim PRIVATE imgui)

# Ninja + a localized MSVC on some machines fails to parse /showIncludes, so
# make the important internal header dependencies explicit.
set_source_files_properties(main.cpp PROPERTIES OBJECT_DEPENDS
    "${CMAKE_SOURCE_DIR}/simulator.h;${CMAKE_SOURCE_DIR}/control_panel.h")
set_source_files_properties(control_panel.cpp PROPERTIES OBJECT_DEPENDS
    "${CMAKE_SOURCE_DIR}/simulator.h;${CMAKE_SOURCE_DIR}/control_panel.h")
set_source_files_properties(simulator.cpp PROPERTIES OBJECT_DEPENDS
    "${CMAKE_SOURCE_DIR}/simulator.h")

if (MSVC)
    target_compile_definitions(rm_camera_sim PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_options(rm_camera_sim PRIVATE
        /W4
        /permissive-
        /utf-8
        /MP
    )
else()
    target_compile_options(rm_camera_sim PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(rm_camera_sim PRIVATE
    opengl32
    gdi32
    user32
)

add_custom_command(TARGET rm_camera_sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/assets"
            "$<TARGET_FILE_DIR:rm_camera_sim>/assets"
    COMMENT "Copying simulator assets"
)
set_property(TARGET rm_camera_sim APPEND PROPERTY LINK_DEPENDS
    "${CMAKE_SOURCE_DIR}/assets/field.rfield"
    "${CMAKE_SOURCE_DIR}/assets/car.rmesh")

set_target_properties(rm_camera_sim PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
)
