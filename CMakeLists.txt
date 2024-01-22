cmake_minimum_required(VERSION 3.10)
project(ros_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DOpenGL_GL_PREFERENCE=GLVND)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenGL REQUIRED)
# if SDL2 not found print message
find_package(SDL2) # "sudo apt-get install libsdl2-dev" to install on Ubuntu
if(NOT SDL2_FOUND)
    message(STATUS "SDL2 not found, using apt-get to install")
    execute_process(COMMAND sudo apt-get install libsdl2-dev)
endif()
find_package(SDL2 REQUIRED)

set(GLAD_SOURCES "lib/glad/src/glad.c")
set(GLAD_HEADERS "lib/glad/include")
add_library(glad ${GLAD_SOURCES} ${GLAD_HEADERS})
target_include_directories(glad PUBLIC 
    ${GLAD_HEADERS}
    ${SDL2_INCLUDE_DIRS}
)

file(GLOB IMGUI_SOURCES 
    "lib/imgui/*.cpp"
    "lib/imgui/backends/imgui_impl_opengl3.cpp"
    "lib/imgui/backends/imgui_impl_sdl2.cpp"
)
add_library(imgui ${IMGUI_SOURCES})
target_include_directories(imgui PUBLIC "lib/imgui")
target_link_libraries(imgui glad ${SDL2_LIBRARIES})

file(GLOB SOURCES "src/*.cpp")
list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/example.cpp")

add_library(ros_gui ${SOURCES})
target_include_directories(ros_gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ros_gui PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(ros_gui
  imgui
  glad
  ${OPENGL_gl_LIBRARY}
  ${SDL2_LIBRARIES}
  ${CMAKE_DL_LIBS}
)
ament_target_dependencies(ros_gui 
  rclcpp
  std_msgs
  sensor_msgs
  image_transport
)

install(TARGETS ros_gui
  DESTINATION lib/ros_gui)

install(DIRECTORY config/
  DESTINATION share/ros_gui/config)

find_package(ros_gui REQUIRED)

add_executable(ros_gui_example "src/example.cpp")
target_link_libraries(ros_gui_example
    ros_gui
)

target_include_directories(ros_gui_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ros_gui_example PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(ros_gui_example
    rclcpp
    std_msgs
    ros_gui
)

install(TARGETS ros_gui_example
  DESTINATION lib/ros_gui)

ament_package()