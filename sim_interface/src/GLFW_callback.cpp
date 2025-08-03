#include "GLFW_callback.h"


UIctr::UIctr(mjModel *modelIn, mjData *dataIn) {
    mj_model=modelIn;
    mj_data=dataIn;
    cam=mjvCamera();
    opt=mjvOption();
    scn=mjvScene();
    con=mjrContext();
}


static void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Scroll(xoffset, yoffset);
}
static void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Mouse_move(xpos, ypos);
}
static void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Mouse_button(button, act, mods);
}

static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Keyboard(key, scancode, act, mods);
}

static void window_close_callback(GLFWwindow* window)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Close();
}

void UIctr::iniGLFW() {
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");
//    char **tmp;
//    glutInit(0,tmp);
    //glutDisplayFunc(UIctr::displaySimTime);
}

// create window, make OpenGL context current, request v-sync, adjust view, bond callbacks, etc.
void UIctr::createWindow(const char* windowTitle, bool saveVideo) {
    window=glfwCreateWindow(width, height, windowTitle, NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    mjv_defaultCamera(&cam);
    // Set up mujoco visualization objects
    // adjust view point
    double arr_view[] = {150, -16, 3, 0, 0.000000, 1.00000}; //view the right side
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    if (isTrack) {
        cam.lookat[2] += 0.8;
        cam.type = mjCAMERA_TRACKING;
        cam.trackbodyid = 1;
    }
    else
        cam.type = mjCAMERA_FREE;

    //mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(mj_model, &scn, 20000);                // space for 2000 objects
    mjr_makeContext(mj_model, &con, mjFONTSCALE_150);   // model-specific context
    mjv_moveCamera(mj_model, mjMOUSE_ROTATE_H, 0.0, 0.0, &scn, &cam);

    // install GLFW mouse and keyboard callbacks
    glfwSetWindowUserPointer(window, this);
    glfwSetWindowCloseCallback(window, window_close_callback);
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    save_video=saveVideo;
    if (save_video)
    {
        image_rgb_ = (unsigned char*)malloc(3*width*height*sizeof(unsigned char));
        image_depth_ = (float*)malloc(sizeof(float)*width*height);

        // create output rgb file
        file = fopen("../record/rgbRec.out", "wb");
        if( !file )
            mju_error("Could not open rgbfile for writing");
    }
}

void UIctr::record_trajectory_site(const mjModel*m ,const mjData* d,const std::string &site) {
    int site_id = mj_name2id(m, mjOBJ_SITE, site.c_str());
    if (site_id != -1) {
        TrajectoryPoint point;
        mju_copy3(point.pos, d->site_xpos + 3*site_id);
        point.time = d->time;
        
        ee_tool0_trajectory.push_back(point);
        
        // 限制轨迹长度
        if (ee_tool0_trajectory.size() > MAX_TRAJECTORY_POINTS) {
            ee_tool0_trajectory.erase(ee_tool0_trajectory.begin());
        }
    }
}

void UIctr::record_trajectory(const mjModel*m ,const mjData* d,const std::string &body) {
    int link_id = mj_name2id(m, mjOBJ_BODY, body.c_str());
    if (link_id != -1) {
        TrajectoryPoint point;
        mju_copy3(point.pos, d->xpos + 3*link_id);
        point.time = d->time;
        
        left_foot_trajectory.push_back(point);
        
        // 限制轨迹长度
        if (left_foot_trajectory.size() > MAX_TRAJECTORY_POINTS) {
            left_foot_trajectory.erase(left_foot_trajectory.begin());
        }
    }
}
// 渲染轨迹
void UIctr::render_trajectory(mjvScene* scn) {
    if (ee_tool0_trajectory.size() < 2) return;
    
    // 轨迹颜色 (黄色)
    const float rgba[4] = {0, 1.0, 1.0, 1};
    
    // 绘制轨迹线
    // for (size_t i = 1; i < ee_tool0_trajectory.size(); i++) {
    //     // 检查场景是否还能容纳更多几何体
    //     if (scn->ngeom >= scn->maxgeom) break;
        
    //     // 获取当前几何体指针
    //     mjvGeom* geom = &scn->geoms[scn->ngeom];
        
    //     // 初始化线段几何体
    //     TrajectoryPoint point = ee_tool0_trajectory[i];
    //     mjtNum size[3] = {0.0, 0.0 , 0.005};  // 前两个参数无用？ 第三个参数定义线条的长度
    //     mjv_initGeom(geom, mjGEOM_LINE, size, point.pos ,NULL,rgba);
        
    //     // 增加几何体计数
    //     scn->ngeom++;
    // }
    
    // 绘制轨迹点（可选）
    for (const auto& point : ee_tool0_trajectory) {
        if (scn->ngeom >= scn->maxgeom) break;
        
        mjvGeom* geom = &scn->geoms[scn->ngeom];
        mjtNum size[3] = {0.002, 0.0 , 0.0};  // 球体半径
        mjv_initGeom(geom, mjGEOM_SPHERE, size,  point.pos, NULL,rgba);
        scn->ngeom++;
    }
}

void UIctr::updateScene() {
    if (!isContinuous)
        runSim= false;

    buttonRead.key_w=false;
    buttonRead.key_a=false;
    buttonRead.key_s=false;
    buttonRead.key_d=false;
    buttonRead.key_space=false;
    buttonRead.key_h=false;
    buttonRead.key_j=false;

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwMakeContextCurrent(window);
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

//        UIctr::opt.frame = mjFRAME_WORLD; //mjFRAME_BODY
//        UIctr::opt.flags[mjVIS_COM]  = 1 ; //mjVIS_JOINT;
//        UIctr::opt.flags[mjVIS_JOINT]  = 1 ;

    // update scene and render
    mjv_updateScene(mj_model, mj_data, &opt, NULL, &cam, mjCAT_ALL, &scn);
    if (!ee_tool0_trajectory.empty())
    {
        render_trajectory(&scn);
    }
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjr_render(viewport, &scn, &con);
    std::string timeStr = "Simulation Time: " + std::to_string(mj_data->time);
    char buffer[100];
    std::sprintf(buffer, "Time: %.3f", mj_data->time);

    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, buffer, NULL, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);
    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();

    if (save_video)
    {
        mjr_readPixels(image_rgb_, image_depth_, viewport, &con);
        fwrite(image_rgb_, sizeof(unsigned char), 3*width*height, file);
    }
}



// keyboard callback
void UIctr::Keyboard(int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(mj_model, mj_data);
        mj_forward(mj_model, mj_data);
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_1)
    {
        runSim=!runSim;
        isContinuous= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_2)
    {
        runSim= true;
        isContinuous= false;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_W){
        buttonRead.key_w= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_A){
        buttonRead.key_a= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_S){
        buttonRead.key_s= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_D){
        buttonRead.key_d= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_H){
        buttonRead.key_h= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_J){
        buttonRead.key_j= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_SPACE){
        buttonRead.key_space= true;
    }
}

// mouse button callback
void UIctr::Mouse_button(int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void UIctr::Mouse_move(double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(mj_model, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void UIctr::Scroll(double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(mj_model, mjMOUSE_ZOOM, 0, 0.05*yoffset, &scn, &cam);
}

void UIctr::Close() {
    // Free mujoco objects
    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);


    glfwTerminate();
}

void UIctr::enableTracking() {
    isTrack=true;
}

UIctr::ButtonState UIctr::getButtonState() {
    ButtonState tmp=buttonRead;
    buttonRead.key_w= false;
    buttonRead.key_a= false;
    buttonRead.key_s= false;
    buttonRead.key_d= false;
    buttonRead.key_h= false;
    buttonRead.key_j= false;
    buttonRead.key_space= false;
    return tmp;
}