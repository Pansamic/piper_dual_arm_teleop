/**
 * @file mujoco_frontend.hpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <thread>
#include <mutex>
#include <atomic>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <glfw_adapter.h>
#include <simulate.h>
#include <array_safety.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

class MujocoFrontend
{
public:
    MujocoFrontend() = delete;
    explicit MujocoFrontend(mjModel* model, mjData* data) : mujoco_model_(model), mujoco_data_(data){}
    ~MujocoFrontend() = default;
    void start()
    {
        this->render_thread_ = std::thread([this](){
            // init GLFW
            if (!glfwInit())
            {
                mju_error("Could not initialize GLFW");
            }

            // create window, make OpenGL context current, request v-sync
            GLFWwindow* window = glfwCreateWindow(1200, 900, "piper dual arm", NULL, NULL);
            this->window_ = window;
            glfwMakeContextCurrent(window);
            glfwSwapInterval(1);

            // initialize visualization data structures
            mjv_defaultCamera(&cam);
            mjv_defaultOption(&opt);
            mjv_defaultScene(&scn);
            mjr_defaultContext(&con);

            // create scene and context
            mjv_makeScene(this->mujoco_model_, &scn, 2000);
            mjr_makeContext(this->mujoco_model_, &con, mjFONTSCALE_150);

            // install GLFW mouse and keyboard callbacks
            glfwSetKeyCallback(window, keyboard);
            glfwSetCursorPosCallback(window, mouse_move);
            glfwSetMouseButtonCallback(window, mouse_button);
            glfwSetScrollCallback(window, scroll);

            glfwSetWindowUserPointer(window, this);

            // run main loop, target real-time simulation and 60 fps rendering
            while (!glfwWindowShouldClose(window))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));

                // get framebuffer viewport
                mjrRect viewport = {0, 0, 0, 0};
                glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

                // update scene and render
                mjv_updateScene(this->mujoco_model_, this->mujoco_data_, &opt, NULL, &cam, mjCAT_ALL, &scn);
                mjr_render(viewport, &scn, &con);

                // swap OpenGL buffers (blocking call due to v-sync)
                glfwSwapBuffers(window);

                // process pending GUI events, call GLFW callbacks
                glfwPollEvents();
            }
        });
    }
    void stop()
    {
        if (this->render_thread_.joinable())
        {
            // Signal the window to close
            if (this->window_)
            {
                glfwSetWindowShouldClose(this->window_, GLFW_TRUE);
            }
            this->render_thread_.join();
        }
    }
private:
    mjModel* mujoco_model_;
    mjData* mujoco_data_;
    mjvCamera cam;                      // abstract camera
    mjvOption opt;                      // visualization options
    mjvScene scn;                       // abstract scene
    mjrContext con;                     // custom GPU context

    GLFWwindow* window_;

    // mouse interaction
    bool button_left = false;
    bool button_middle = false;
    bool button_right =  false;
    double lastx = 0;
    double lasty = 0;

    std::thread render_thread_;

    // static keyboard callback
    static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
    {
        MujocoFrontend* frontend = static_cast<MujocoFrontend*>(glfwGetWindowUserPointer(window));
        if (frontend)
        {
            frontend->keyboard_impl(window, key, scancode, act, mods);
        }
    }

    // static mouse button callback
    static void mouse_button(GLFWwindow* window, int button, int act, int mods)
    {
        MujocoFrontend* frontend = static_cast<MujocoFrontend*>(glfwGetWindowUserPointer(window));
        if (frontend)
        {
            frontend->mouse_button_impl(window, button, act, mods);
        }
    }

    // static mouse move callback
    static void mouse_move(GLFWwindow* window, double xpos, double ypos)
    {
        MujocoFrontend* frontend = static_cast<MujocoFrontend*>(glfwGetWindowUserPointer(window));
        if (frontend)
        {
            frontend->mouse_move_impl(window, xpos, ypos);
        }
    }

    // static scroll callback
    static void scroll(GLFWwindow* window, double xoffset, double yoffset)
    {
        MujocoFrontend* frontend = static_cast<MujocoFrontend*>(glfwGetWindowUserPointer(window));
        if (frontend)
        {
            frontend->scroll_impl(window, xoffset, yoffset);
        }
    }

    // actual implementation methods
    void keyboard_impl(GLFWwindow* window, int key, int scancode, int act, int mods)
    {
        // backspace: reset simulation
        if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE)
        {
            mj_resetData(mujoco_model_, mujoco_data_);
            mj_forward(mujoco_model_, mujoco_data_);
        }
    }

    void mouse_button_impl(GLFWwindow* window, int button, int act, int mods)
    {
        // update button state
        if (button == GLFW_MOUSE_BUTTON_LEFT)
        {
            button_left = (act == GLFW_PRESS);
        }
        else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
        {
            button_middle = (act == GLFW_PRESS);
        }
        else if (button == GLFW_MOUSE_BUTTON_RIGHT)
        {
            button_right = (act == GLFW_PRESS);
        }

        // update mouse position
        glfwGetCursorPos(window, &lastx, &lasty);
    }

    void mouse_move_impl(GLFWwindow* window, double xpos, double ypos)
    {
        // no buttons down: nothing to do
        if (!button_left && !button_middle && !button_right)
        {
            return;
        }

        // compute mouse displacement, save
        double dx = xpos - lastx;
        double dy = ypos - lasty;
        lastx = xpos;
        lasty = ypos;

        // get current window size
        int width, height;
        glfwGetWindowSize(window, &width, &height);

        // get shift key state
        bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                          glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

        // determine action based on mouse button
        mjtMouse action;
        if (button_right)
        {
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        }
        else if (button_left)
        {
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        }
        else
        {
            action = mjMOUSE_ZOOM;
        }

        // move camera
        mjv_moveCamera(mujoco_model_, action, dx/height, dy/height, &scn, &cam);
    }

    void scroll_impl(GLFWwindow* window, double xoffset, double yoffset)
    {
        // emulate vertical mouse motion = 5% of window height
        mjv_moveCamera(mujoco_model_, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
    }
};


// class MujocoFrontend
// {
// public:
//     MujocoFrontend(mjModel* m, mjData* d) : mujoco_model_(m), mujoco_data_(d){};
//     ~MujocoFrontend() = default;

//     void start()
//     {
//         this->render_thread_ = std::thread([this]()
//         {
//             mjvCamera cam;
//             mjv_defaultCamera(&cam);

//             mjvOption opt;
//             mjv_defaultOption(&opt);

//             mjvPerturb pert;
//             mjv_defaultPerturb(&pert);

//             {
//                 std::lock_guard<std::mutex> lock(this->sim_mutex_);
//                 this->sim_ = std::make_unique<mujoco::Simulate>(
//                     std::make_unique<mujoco::GlfwAdapter>(),
//                     &cam, &opt, &pert, false
//                 );
//                 this->sim_ready_ = true;
//             }

//             this->sim_->Load(this->mujoco_model_, this->mujoco_data_, PROJECT_PATH"/assets/mujoco_model/piper_dual_arm_torque_full.xml");

//             this->sim_ready_cv_.notify_one();

//             this->sim_->RenderLoop();
//         });
//     }
//     void stop()
//     {
//         this->sim_->exitrequest.store(1);
//         this->render_thread_.join();
//     }
// private:
//     std::mutex sim_mutex_;
//     std::condition_variable sim_ready_cv_;
//     bool sim_ready_ = false;

//     mjModel* mujoco_model_; // MuJoCo model
//     mjData* mujoco_data_; // MuJoCo data

//     std::unique_ptr<mujoco::Simulate> sim_;
//     std::thread render_thread_;
// };