#include <ros_gui/ros_gui.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>

namespace gui
{
    App::App(Spec s, int argc, char *argv[]) : spec(s)
    {
        if (spec.package_name == nullptr)
        {
            throw std::runtime_error("Package name is not specified.\n\n\nPlease specify the package name in the App::Spec struct.\n\n");
        }

        if (ros::package::getPath(spec.package_name) == "")
        {
            throw std::runtime_error("\n\nPackage \"" + std::string(spec.package_name) + "\" does not exist.\nPlease specify the correct package name in the App::Spec struct.\n\n");
        }

        ini_filename = ros::package::getPath(spec.package_name) + "/config/imgui.ini";
    }

    App::~App()
    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();

        SDL_GL_DeleteContext(gl_context);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }

    void App::run(void (*setupCB)(void), void (*loopCB)(void))
    {
        setupCallback = setupCB;
        loopCallback = loopCB;
        if (initSDL() != 0)
        {
            std::cerr << "[ERROR] Failed to initialize SDL" << std::endl;
            return;
        }
        else
        {
            std::cout << "[INFO] SDL initialized" << std::endl;
        }

        if (initGL() != 0)
        {
            std::cerr << "[ERROR] Failed to initialize OpenGL" << std::endl;
            return;
        }
        else
        {
            std::cout << "[INFO] OpenGL initialized" << std::endl;
        }

        initImGui();
        loop();
    }

    int App::initSDL()
    {
        if (SDL_Init(SDL_INIT_VIDEO) != 0)
        {
            printf("[ERROR] %s\n", SDL_GetError());
            return -1;
        }

        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
        SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
        SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

        SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

        SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
        window = SDL_CreateWindow(
            spec.window_title,
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            spec.window_width,
            spec.window_height,
            window_flags);
        // limit to which minimum size user can resize the window
        SDL_SetWindowMinimumSize(window, 500, 300);

        gl_context = SDL_GL_CreateContext(window);
        if (gl_context == NULL)
        {
            std::cerr << "[ERROR] Failed to create a GL context: "
                      << SDL_GetError() << std::endl;
            return -1;
        }
        SDL_GL_MakeCurrent(window, gl_context);

        // enable VSync
        SDL_GL_SetSwapInterval(1);

        return 0;
    }

    int App::initGL()
    {
        if (!gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress))
        {
            std::cerr << "[ERROR] Couldn't initialize GLAD" << std::endl;
            return -1;
        }
        else
        {
            std::cout << "[INFO] GLAD initialized" << std::endl;
        }

        std::cout << "[INFO] OpenGL renderer: "
                  << glGetString(GL_RENDERER)
                  << std::endl;

        std::cout << "[INFO] OpenGL from GLAD: "
                  << GLVersion.major
                  << "."
                  << GLVersion.minor
                  << std::endl;

        glViewport(0, 0, spec.window_width, spec.window_height);

        return 0;
    }

    void App::initImGui()
    {
        // setup Dear ImGui context
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO &io = ImGui::GetIO();
        (void)io;

        io.IniFilename = ini_filename.c_str();
        // auto tff = ros::package::getPath(spec.package_name) + "/config/fonts/Verdana.ttf";
        auto tff = ros::package::getPath(spec.package_name) + "/config/fonts/DroidSans.ttf";
        io.Fonts->AddFontFromFileTTF(tff.c_str(), 18.0f);
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
        io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

        ImGui::StyleColorsDark();

        ImGuiStyle &style = ImGui::GetStyle();
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            style.WindowRounding = 0.0f;
            style.Colors[ImGuiCol_WindowBg].w = 1.0f;
        }

        // setup platform/renderer bindings
        ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
        ImGui_ImplOpenGL3_Init(glsl_version);

        ImVec4 background = ImVec4(35 / 255.0f, 35 / 255.0f, 35 / 255.0f, 1.00f);
        glClearColor(background.x, background.y, background.z, background.w);
    }

    void App::loop()
    {
        if (setupCallback)
        {
            setupCallback();
        }

        bool running = true;
        while (running)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

            SDL_Event event;
            while (SDL_PollEvent(&event))
            {
                ImGui_ImplSDL2_ProcessEvent(&event);
                switch (event.type)
                {
                case SDL_QUIT:
                    running = false;
                    break;

                case SDL_WINDOWEVENT:
                    switch (event.window.event)
                    {
                    case SDL_WINDOWEVENT_RESIZED:
                        spec.window_width = event.window.data1;
                        spec.window_height = event.window.data2;
                        glViewport(0, 0, spec.window_width, spec.window_height);
                        break;
                    }
                    break;

                case SDL_KEYDOWN:
                    switch (event.key.keysym.sym)
                    {
                    case SDLK_ESCAPE:
                        running = false;
                        break;
                    }
                    break;
                }
            }

            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplSDL2_NewFrame(window);
            ImGui::NewFrame();

            ImGui::DockSpaceOverViewport();

            if (loopCallback)
            {
                loopCallback();
            }

            // rendering
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            auto &io = ImGui::GetIO();
            if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
            {
                SDL_Window *backup_current_window = SDL_GL_GetCurrentWindow();
                SDL_GLContext backup_current_context = SDL_GL_GetCurrentContext();
                ImGui::UpdatePlatformWindows();
                ImGui::RenderPlatformWindowsDefault();
                SDL_GL_MakeCurrent(backup_current_window, backup_current_context);
            }

            SDL_GL_SwapWindow(window);
        }
    }

    Image::~Image()
    {
        delete nh;
    }

    Image::Image(const char *img_topic)
    {
        nh = new ros::NodeHandle();
        image_transport::ImageTransport it(*nh);
        sub = it.subscribe(img_topic, 1, &Image::callback, this);
    }

    void Image::callback(const sensor_msgs::ImageConstPtr &msg)
    {
        // std::cout << "Image received" << std::endl;
        // std::cout << "width: " << msg->width << std::endl;
        // std::cout << "height: " << msg->height << std::endl;
        // std::cout << "encoding: " << msg->encoding << std::endl;
        // std::cout << "step: " << msg->step << std::endl;
        // std::cout << "data size: " << msg->data.size() << std::endl;
        // std::cout << "is_bigendian: " << msg->is_bigendian << std::endl;
        // std::cout << "======================" << std::endl;

        if (msg->encoding == "rgb8")
        {
            if (texture == 0)
            {
                glGenTextures(1, &texture);
                glBindTexture(GL_TEXTURE_2D, texture);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, msg->width, msg->height, 0, GL_RGB, GL_UNSIGNED_BYTE, msg->data.data());
                glGenerateMipmap(GL_TEXTURE_2D);
                glBindTexture(GL_TEXTURE_2D, 0);
            }
            else
            {
                glBindTexture(GL_TEXTURE_2D, texture);
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, msg->width, msg->height, GL_RGB, GL_UNSIGNED_BYTE, msg->data.data());
                glBindTexture(GL_TEXTURE_2D, 0);
            }
        }
        else if (msg->encoding == "bgr8")
        {
            if (texture == 0)
            {
                glGenTextures(1, &texture);
                glBindTexture(GL_TEXTURE_2D, texture);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, msg->width, msg->height, 0, GL_BGR, GL_UNSIGNED_BYTE, msg->data.data());
                glGenerateMipmap(GL_TEXTURE_2D);
                glBindTexture(GL_TEXTURE_2D, 0);
            }
            else
            {
                glBindTexture(GL_TEXTURE_2D, texture);
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, msg->width, msg->height, GL_BGR, GL_UNSIGNED_BYTE, msg->data.data());
                glBindTexture(GL_TEXTURE_2D, 0);
            }
        }
    }
} // namespace ros
