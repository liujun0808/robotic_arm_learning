#pragma once
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <string>
#include <memory>
#include "Eigen/Dense"

class UIctr{
public:
    GLFWwindow *window;
    // keyboard
    struct ButtonState {
        bool key_w{false};
        bool key_s{false};
        bool key_a{false};
        bool key_d{false};
        bool key_h{false};
        bool key_j{false};
        bool key_space{false};
    } buttonRead;

    // mouse interaction
    bool button_left{false};
    bool button_middle{false};
    bool button_right{false};
    std::vector<Eigen::Vector3d> link_trajectory;

    bool runSim{true};
    bool isContinuous{true};
    double lastx{0};
    double lasty{0};
    mjModel* mj_model;
    mjData* mj_data;

    UIctr(mjModel *modelIn, mjData *dataIn);
    void iniGLFW();
    void createWindow(const char * windowTitle, bool saveVideo);
    void updateScene();

    // keyboard callback
    void Keyboard(int key, int scancode, int act, int mods);
    // mouse button callback
    void Mouse_button(int button, int act, int mods);
    // mouse move callback
    void Mouse_move(double xpos, double ypos);
    // scroll callback
    void Scroll(double xoffset, double yoffset);

    ButtonState getButtonState();


    void Close();

    void enableTracking();

    // 轨迹存储结构
    struct TrajectoryPoint {
        mjtNum pos[3];
        mjtNum time;
    };
    // 全局轨迹存储
    std::vector<TrajectoryPoint> left_foot_trajectory;
    const int MAX_TRAJECTORY_POINTS = 1000; // 最大轨迹点数
    void record_trajectory(const mjModel*m,const mjData* d,const std::string &body);
    void render_trajectory(mjvScene* scn);
private:
    unsigned char* image_rgb_;
    float* image_depth_;

    FILE* file;

    int width{1200};
    int height{800};
    bool save_video{false};

    bool isTrack{false};
    // UI handler
    mjvCamera cam;                      // abstract camera
    mjvOption opt;                      // visualization options
    mjvScene scn;                       // abstract scene
    mjrContext con;                     // custom GPU context
};
