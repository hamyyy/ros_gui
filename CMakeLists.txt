cmake_minimum_required(VERSION 3.10)

project(ros_gui)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  image_transport
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ros_gui
 CATKIN_DEPENDS roscpp rospy image_transport
#  DEPENDS system_lib
)

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

include_directories(${SDL2_INCLUDE_DIRS})

add_library(glad ${GLAD_SOURCES} ${GLAD_HEADERS})
target_include_directories(glad PUBLIC ${GLAD_HEADERS})

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

add_library(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC "include" "lib" ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
    imgui
    glad
    ${OPENGL_gl_LIBRARY}
    ${SDL2_LIBRARIES}
    ${CMAKE_DL_LIBS}
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_example "src/example.cpp")
target_link_libraries(${PROJECT_NAME}_example
    ${PROJECT_NAME}
)