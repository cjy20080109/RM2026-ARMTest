#pragma once

#include <windows.h>

namespace sim {
class Simulator;
}

// Right-hand Dear ImGui sidebar for arm, camera, and light controls.
class ControlPanel {
public:
    ControlPanel() = default;
    ~ControlPanel();

    ControlPanel(const ControlPanel&) = delete;
    ControlPanel& operator=(const ControlPanel&) = delete;

    bool initialize(HWND hwnd);
    void shutdown();

    // Draws the sidebar. Call after the main 3D view has been rendered and
    // before SwapBuffers.
    void render(sim::Simulator& simulator, int windowWidth, int windowHeight);

    bool ready() const {
        return initialized_;
    }

    bool usingChineseFont() const {
        return chineseFont_;
    }

private:
    void applyStyle();
    bool initializeFonts();

    bool initialized_ = false;
    bool chineseFont_ = false;
    int activeTab_ = 0;
};

