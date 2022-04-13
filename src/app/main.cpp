#include "application.h"
#include <imgui.h>

#include <iostream>
#include <math.h>
#include <deque>
#include <chrono>

#include "../boids/boids.h"


class TestApp : public Application
{
#define COLOR_OUT    nvgRGBA(220,50,50,255)
#define COLOR_IN     nvgRGBA(50,50,220,255)
#define COLOR_SOLVED nvgRGBA(50,220,50,255)


public:

    TestApp(int w, int h, const char * title) : Application(title, w, h) {
        
        ImGui::StyleColorsClassic();
        
        const char* name = IMGUI_FONT_FOLDER"/Cousine-Regular.ttf";
        nvgCreateFont(vg, "sans", name);
        
    }

    void process() override {
        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::microseconds>(now-lastFrame).count() >= 10./60. * 1.e6)
        {
            if(keyDown[GLFW_KEY_R])
                boids.initializePositions(currentMethod); // Everytime method has been changed, press R to re-initialization
            if(keyDown[GLFW_KEY_SPACE])
                boids.pause();
            if(keyDown[GLFW_KEY_ESCAPE])
                exit(0);
            lastFrame = now;
        }
    }
    //TODO : add something in GUI
    void drawImGui() override {

        using namespace ImGui;

        const char* names[] = {"FreeFall", "Separation", "Alignment", "Cohesion", "Leading", "Circle"};
        const char* update_names[] = {"Explicit EUler", "Symplectic Euler", "Explicit Midpoint"};
        Begin("Menu");
        Combo("Boids Behavior", (int*)&currentMethod, names, 6);
        Combo("Update Method", (int*)&updateRule, update_names, 3);
        SliderFloat("Step Size", &boids.h, 0.01, 1);
        Checkbox("Obstacle", &boids.obs_flag);
        if(boids.obs_flag){
            SliderFloat("Obstacle Size", &boids.obs_radius, 0.01, 0.5);
            SliderFloat("Foresight distance (avoid collision)", &boids.avoid_distance, 0.0, 0.5);
        }
        if(currentMethod == COHESION){
            SliderFloat("Cohesion Range", &boids.cohesion_r, 0.0, 1.0);
        }
        if(currentMethod == ALIGNMENT){
            SliderFloat("Cohesion Range", &boids.cohesion_r, 0.0, 1.0);
            SliderFloat("Alignment Range", &boids.alignment_r, 0.0, 1.0);
        }
        if(currentMethod == SEPARATION){
            SliderFloat("Cohesion Range", &boids.cohesion_r, 0.0, 1.0);
            SliderFloat("Alignment Range", &boids.alignment_r, 0.0, 1.0);
            SliderFloat("Separation Range", &boids.separation_r, 0.0, 1.0);
        } 
        if(currentMethod == LEADER){
            SliderFloat("Cohesion Range", &boids.cohesion_r, 0.0, 1.0);
            SliderFloat("Alignment Range", &boids.alignment_r, 0.0, 2.0);
            SliderFloat("Separation Range", &boids.separation_r, 0.0, 1.0);
            SliderFloat("Follow distance", &boids.sight, 0.0, 1.0);
        }    
        // SliderFloat("Obs", (float*)&boids.obs_radius, 0.0f, 1.0f);
        End();



    }

    void drawNanoVG() override {

        
        boids.updateBehavior(currentMethod, updateRule);
        
        VectorXT boids_pos = boids.getPositions();
        auto shift_01_to_screen = [](TV pos_01, T scale, T width, T height)
        {
            return TV(0.5 * width + scale * pos_01[0] * width, 0.5 * height + scale * pos_01[1] * height);
        };
        auto shift_screen_to_01 = [](TV pos_01, T scale, T width, T height)
        {
            return TV((pos_01[0] - 0.5 * width)/width/scale, (pos_01[1] - 0.5 * height) / height / scale);
        };
        T scale = 0.5;
        if(boids.obs_flag){
            TV target_pos = boids.getLeaderPos();
            TV obs_pos = boids.getObsPos();

            for(int i = 0; i < boids.getParticleNumber(); i++)
            {
                TV pos = boids_pos.segment<2>(i * 2);
                nvgBeginPath(vg);
        
                // just map position from 01 simulation space to scree space
                // feel free to make changes
                // the only thing that matters is you have pos computed correctly from your simulation
                
                TV screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
                TV screen_obs_pos = shift_01_to_screen(TV(obs_pos[0], obs_pos[1]), scale, width, height);
                nvgCircle(vg, screen_pos[0], screen_pos[1], 2.f);
                nvgFillColor(vg, COLOR_OUT);
                nvgFill(vg);

        
            }
            TV screen_obs_pos = shift_01_to_screen(TV(obs_pos[0], obs_pos[1]), scale, width, height);                //Obstacle
            nvgBeginPath(vg);
            nvgCircle(vg, screen_obs_pos[0], screen_obs_pos[1], boids.obs_radius*width*scale);
            nvgFillColor(vg, COLOR_IN);
            nvgFill(vg);

            if(currentMethod == LEADER){
                TV mouse_screen_pos = TV(mouseState.lastMouseX, mouseState.lastMouseY);
                nvgBeginPath(vg);
                nvgCircle(vg, mouse_screen_pos[0], mouse_screen_pos[1], 5.f);
                nvgFillColor(vg, COLOR_SOLVED);
                nvgFill(vg); 
                boids.leader_pos = shift_screen_to_01(mouse_screen_pos, scale, width, height);
            }
            // // Origin
            // nvgBeginPath(vg);
            // nvgCircle(vg, 0.5*width, 0.5*height, 5.f);
            // nvgFillColor(vg, COLOR_SOLVED);
            // nvgFill(vg);  

        }
        else{

            for(int i = 0; i < boids.getParticleNumber(); i++)
            {
                TV pos = boids_pos.segment<2>(i * 2);
                nvgBeginPath(vg);
        
                // just map position from 01 simulation space to scree space
                // feel free to make changes
                // the only thing that matters is you have pos computed correctly from your simulation
                TV screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
                nvgCircle(vg, screen_pos[0], screen_pos[1], 2.f);
                nvgFillColor(vg, COLOR_OUT);
                nvgFill(vg);            
            }

            // // Origin
            // nvgBeginPath(vg);
            // nvgCircle(vg, 0.5*width, 0.5*height,5.f);
            // nvgFillColor(vg, COLOR_SOLVED);
            // nvgFill(vg);  

            if(currentMethod == LEADER){
                TV mouse_screen_pos = TV(mouseState.lastMouseX, mouseState.lastMouseY);
                TV mouse_screen_vel = TV(mouseState.mouseMoveX, mouseState.mouseMoveY);
                nvgBeginPath(vg);
                nvgCircle(vg, mouse_screen_pos[0], mouse_screen_pos[1], 5.f);
                nvgFillColor(vg, COLOR_SOLVED);
                nvgFill(vg); 
                boids.leader_pos = shift_screen_to_01(mouse_screen_pos, scale, width, height);
                boids.leader_vel = shift_screen_to_01(mouse_screen_vel, scale, width, height);
            }

        }

        // for(int i = 0; i < boids.getParticleNumber(); i++)
        // {
        //     TV pos = boids_pos.segment<2>(i * 2);
        //     nvgBeginPath(vg);
    
        //     // just map position from 01 simulation space to scree space
        //     // feel free to make changes
        //     // the only thing that matters is you have pos computed correctly from your simulation
        //     T scale = 0.3;
        //     TV screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
        //     nvgCircle(vg, screen_pos[0], screen_pos[1], 2.f);
        //     nvgCircle(vg, 0.5  * width, 0.5 * height, 5.f);
        //     nvgFillColor(vg, COLOR_OUT);
        //     nvgFill(vg);

        //     //Origin
        //     nvgBeginPath(vg);
        //     nvgCircle(vg, 0.5  * width, 0.5 * height, 50.f);
        //     nvgFillColor(vg, COLOR_IN);
        //     nvgFill(vg);            

        // }

    }

protected:
    void mouseButtonPressed(int button, int mods) override {

    }

    void mouseButtonReleased(int button, int mods) override {
        
    }

private:
    int loadFonts(NVGcontext* vg)
    {
        int font;
        font = nvgCreateFont(vg, "sans", "../example/Roboto-Regular.ttf");
        if (font == -1) {
            printf("Could not add font regular.\n");
            return -1;
        }
        font = nvgCreateFont(vg, "sans-bold", "../example/Roboto-Bold.ttf");
        if (font == -1) {
            printf("Could not add font bold.\n");
            return -1;
        }
        return 0;
    }

private:

    MethodTypes currentMethod = FREEFALL;
    // UpdateRule updateRule = EXPLICIT_EULER;
    UpdateRule updateRule = SYMPLECTIC_EULER;
    // UpdateRule updateRule = EXPLICIT_MIDPOINT;
    Boids boids = Boids(40,currentMethod);
    std::chrono::high_resolution_clock::time_point lastFrame;
};

int main(int, char**)
{
    int width = 960;
    int height = 720;
    std::cout << " main " << std::endl;
    TestApp app(width, height, "Assignment 3 Boids");
    app.run();

    return 0;
}
