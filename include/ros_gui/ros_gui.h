#pragma once

#include <glad/glad.h>
#include <SDL.h>
#include <imgui/imgui.h>
#include <imgui/backends/imgui_impl_sdl2.h>
#include <imgui/backends/imgui_impl_opengl3.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

namespace gui
{
    class App
    {
    public:
        struct Spec
        {
            int window_width = 1280,
                window_height = 720;
            const char *window_title = "ROS GUI";
            const char *package_name = nullptr;
            const char *library_name = "ros_gui";
        } spec;

        App(Spec spec, int argc, char *argv[]);
        ~App();
        void run(void (*setupCallback)(void) = nullptr, void (*loopCallback)(void) = nullptr);

    private:
        SDL_Window *window;
        SDL_GLContext gl_context;
        const char *glsl_version = "#version 150";
        int initSDL();
        int initGL();
        void initImGui();
        void loop();

        void (*setupCallback)(void);
        void (*loopCallback)(void);

        std::string ini_filename;
    };

    class Image
    {
    public:
        Image(const char *img_topic);
        ~Image();

    private:
        GLuint texture;

        ros::NodeHandle* nh;
        image_transport::Subscriber sub;
        void callback(const sensor_msgs::ImageConstPtr &msg);
    };

} // namespace ros
