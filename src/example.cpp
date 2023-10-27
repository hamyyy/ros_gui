#include <ros_gui/ros_gui.h>

#include <ros/ros.h>

void setup()
{
    // code only runs once
}

void loop()
{
    // code runs every frame

    ImGui::Begin("Hello, world!");
    ImGui::Text("This is some useful text.");
    ImGui::End();
}

int main(int argc, char *argv[])
{
    gui::App::Spec spec;
    spec.window_title = "ROS GUI";
    spec.package_name = "ros_gui";

    gui::App *app = new gui::App(spec, argc, argv);
    app->run(setup, loop);
    delete app;

    return 0;
}