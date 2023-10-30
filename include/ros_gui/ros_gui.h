#pragma once

#include <glad/glad.h>
#include <SDL.h>
#include <imgui/imgui.h>
#include <imgui/backends/imgui_impl_sdl2.h>
#include <imgui/backends/imgui_impl_opengl3.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <vector>
#include <string>

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
        Image(const std::string img_topic);
        ~Image();

        void *getTexture()
        {
            if (texture == 0)
                return (void *)nullptr;
            return (void *)(intptr_t)texture;
        }

        enum class Flags
        {
            None = 0,
            FlipVertically = 1 << 0,
            FlipHorizontally = 1 << 1,

            Rotate90 = 1 << 2,
            Rotate180 = 1 << 3,
            Rotate270 = 1 << 4,
        };

        void draw(Flags flags = Flags::None, bool autoresize = true, ImVec2 size = ImVec2(0, 0));

        void updateData();
        void setData(std::vector<uint8_t, std::allocator<uint8_t>> data);

        int width;
        int height;
        std::vector<uint8_t, std::allocator<uint8_t>> data;

    private:
        GLuint texture;

        ros::NodeHandle *nh;
        image_transport::Subscriber sub;
        void callback(const sensor_msgs::ImageConstPtr &msg);

        bool imageReady = false;
        std::string topicName;

        ros::Time lastUpdate;
        ros::Duration allowedSilenceTime = ros::Duration(3.0);
    };

} // namespace ros
