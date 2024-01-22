#include <ros_gui/ros_gui.h>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <string>

namespace gui
{
    App::App(Spec s, [[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) : spec(s)
    {
        if (spec.package_name == nullptr)
        {
            throw std::runtime_error("Package name is not specified.\n\n\nPlease specify the package name in the App::Spec struct.\n\n");
        }

        if (ament_index_cpp::get_package_share_directory(spec.package_name) == "")
        {
            throw std::runtime_error("\n\nPackage \"" + std::string(spec.package_name) + "\" does not exist.\nPlease specify the correct package name in the App::Spec struct.\n\n");
        }

        ini_filename = ament_index_cpp::get_package_share_directory(spec.package_name) + "/config/imgui.ini";
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
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3); // try 3 if 4 doesn't work
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

        SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
        window = SDL_CreateWindow(
            spec.window_title,
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            spec.window_width,
            spec.window_height,
            window_flags);

        if (window == NULL)
        {
            std::cerr << "[ERROR] Failed to create a window: "
                      << SDL_GetError() << std::endl;
            return -1;
        }

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
        // auto tff = ament_index_cpp::get_package_share_directory(spec.library_name) + "/config/fonts/Verdana.ttf";
        auto tff = ament_index_cpp::get_package_share_directory(spec.library_name) + "/config/fonts/DroidSans.ttf";
        io.Fonts->AddFontFromFileTTF(tff.c_str(), 18.0f);
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
        io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

        io.ConfigDockingTransparentPayload = true;
        io.ConfigInputTextEnterKeepActive = true;

        ImGui::StyleColorsDark();

        ImGuiStyle &style = ImGui::GetStyle();
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            style.WindowRounding = 0.0f;
            style.Colors[ImGuiCol_WindowBg].w = 1.0f;
        }

        // setup platform/renderer bindings
        if (!ImGui_ImplSDL2_InitForOpenGL(window, gl_context))
        {
            std::cerr << "[ERROR] Failed to initialize ImGui SDL2" << std::endl;
            return;
        }
        if (!ImGui_ImplOpenGL3_Init(glsl_version))
        {
            std::cerr << "[ERROR] Failed to initialize ImGui OpenGL3" << std::endl;
            return;
        }

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

    Image::Image([[maybe_unused]] const std::string img_topic) : width(0), height(0), texture(0)
    {
        // topicName = img_topic;
        // rclcpp::NodeOptions options;
        // nh = std::make_shared<rclcpp::Node>("image_node", options);
        // image_transport::ImageTransport it(nh);
        // sub = it.subscribe(img_topic, 1, std::bind(&Image::callback, this, std::placeholders::_1));

        // width = 640;
        // height = 480;
        // data.resize(width * height * 3);

        // // for (int i = 0; i < width * height * 3; i++)
        // // {
        // //     data[i] = rand() % 255;
        // // }

        // glGenTextures(1, &texture);
        // glBindTexture(GL_TEXTURE_2D, texture);
        // glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data.data());
        // glGenerateMipmap(GL_TEXTURE_2D);
    }

    Image::~Image()
    {
        nh.reset();
    }

    void Image::updateData()
    {
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data.data());
    }

    void Image::setData(std::vector<uint8_t, std::allocator<uint8_t>> d)
    {
        if ((uint32_t)d.size() != width * height * 3)
        {
            std::cerr << "[ERROR] Image data size does not match" << std::endl;
            return;
        }

        data = d;
        updateData();
    }

    void Image::draw(Flags flags, bool autoresize, ImVec2 size)
    {
        if (texture == 0)
            return;

        if (size.x == 0 || size.y == 0)
        {
            size.x = width;
            size.y = height;
        }

        if (autoresize)
        {
            auto avail = ImGui::GetContentRegionAvail();
            auto ratio = (float)width / (float)height;

            size = ImVec2(avail.x, avail.x / ratio);
            if (size.y > avail.y)
            {
                size = ImVec2(avail.y * ratio, avail.y);
            }
        }

        ImVec2 uv0 = ImVec2(0, 0);
        ImVec2 uv1 = ImVec2(1, 1);

        if ((int)flags & (int)Flags::FlipVertically)
        {
            uv0.y = 1;
            uv1.y = 0;
        }

        if ((int)flags & (int)Flags::FlipHorizontally)
        {
            uv0.x = 1;
            uv1.x = 0;
        }

        if ((int)flags & (int)Flags::Rotate90)
        {
            uv0 = ImVec2(1, 0);
            uv1 = ImVec2(0, 1);
        }
        else if ((int)flags & (int)Flags::Rotate180)
        {
            uv0 = ImVec2(1, 1);
            uv1 = ImVec2(0, 0);
        }
        else if ((int)flags & (int)Flags::Rotate270)
        {
            uv0 = ImVec2(0, 1);
            uv1 = ImVec2(1, 0);
        }

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        std::string title = "Image##" + topicName;
        ImGui::BeginChild(title.c_str(), size, true, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::PopStyleVar();

        ImGui::Image((void *)(intptr_t)texture, size, uv0, uv1);

        if (rclcpp::Clock().now() - lastUpdate > allowedSilenceTime)
        {
            imageReady = false;
        }

        if (!imageReady)
        {
            std::string text = "Waiting for " + topicName;

            ImVec2 pos;
            auto cursor = ImGui::GetCursorPos();
            auto textSize = ImGui::CalcTextSize(text.c_str());
            auto style = ImGui::GetStyle();

            pos.x = cursor.x + (size.x - textSize.x) * 0.5 - style.FramePadding.x;
            pos.y = cursor.y - (size.y - textSize.y) * 0.5 - style.FramePadding.y;

            ImGui::SetCursorPosX(pos.x);
            ImGui::SetCursorPosY(pos.y);

            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 255));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0, 0, 0, 255));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0, 0, 0, 255));
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 0);
            ImGui::Button(text.c_str());
            ImGui::PopStyleVar();
            ImGui::PopStyleColor(3);

            ImGui::SetCursorPos(cursor);
        }

        ImGui::EndChild();
    }

    void Image::callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // std::cout << "Image received" << std::endl;
        // std::cout << "width: " << msg->width << std::endl;
        // std::cout << "height: " << msg->height << std::endl;
        // std::cout << "encoding: " << msg->encoding << std::endl;
        // std::cout << "step: " << msg->step << std::endl;
        // std::cout << "data size: " << msg->data.size() << std::endl;
        // std::cout << "is_bigendian: " << msg->is_bigendian << std::endl;
        // std::cout << "======================" << std::endl;

        bool resize = false;

        if (width != msg->width || height != msg->height || !imageReady)
        {
            width = msg->width;
            height = msg->height;
            data.resize(width * height * 3);
            resize = true;
        }

        if (msg->encoding == "rgb8")
        {
            if (resize)
            {
                glDeleteTextures(1, &texture);
                glGenTextures(1, &texture);
                glBindTexture(GL_TEXTURE_2D, texture);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, msg->width, msg->height, 0, GL_RGB, GL_UNSIGNED_BYTE, msg->data.data());
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            }
            else
            {
                glBindTexture(GL_TEXTURE_2D, texture);
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, msg->width, msg->height, GL_RGB, GL_UNSIGNED_BYTE, msg->data.data());
            }
        }
        else if (msg->encoding == "bgr8")
        {
            if (resize)
            {
                glDeleteTextures(1, &texture);
                glGenTextures(1, &texture);
                glBindTexture(GL_TEXTURE_2D, texture);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, msg->width, msg->height, 0, GL_BGR, GL_UNSIGNED_BYTE, msg->data.data());
                glGenerateMipmap(GL_TEXTURE_2D);
            }
            else
            {
                glBindTexture(GL_TEXTURE_2D, texture);
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, msg->width, msg->height, GL_BGR, GL_UNSIGNED_BYTE, msg->data.data());
            }
        }
        else if (msg->encoding == "mono8")
        {
            if (resize)
            {
                glDeleteTextures(1, &texture);
                glGenTextures(1, &texture);
                glBindTexture(GL_TEXTURE_2D, texture);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, msg->width, msg->height, 0, GL_RED, GL_UNSIGNED_BYTE, msg->data.data());
                glGenerateMipmap(GL_TEXTURE_2D);
            }
            else
            {
                glBindTexture(GL_TEXTURE_2D, texture);
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, msg->width, msg->height, GL_RED, GL_UNSIGNED_BYTE, msg->data.data());
            }
        }
        else
        {
            std::cerr << "[ERROR] Unsupported image encoding: " << msg->encoding << std::endl;
            return;
        }

        imageReady = true;
        lastUpdate = rclcpp::Clock().now();
    }
} // namespace ros
