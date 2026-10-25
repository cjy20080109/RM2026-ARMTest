// =============================================================================
//  RM 3D camera simulator - Win32 / OpenGL host and main loop
//
//  Build:
//      cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
//      cmake --build build
//
//  Controls:
//      W/S : move along the robot's +X / -X (mecanum, heading unchanged)
//      A/D : strafe along the robot's +Y / -Y
//      Q/E : yaw counter-clockwise / clockwise
//      R   : reset the robot to the field origin
//      LMB : click a camera pane to query that pixel's ray depth
//
//  The loop below is the integration point:
//      capture 5 cameras -> process 5 frames -> draw 6 views.
// =============================================================================

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <GL/gl.h>

#include "control_panel.h"
#include "simulator.h"

#include "imgui.h"
#include "imgui_impl_win32.h"

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd,
                                                              UINT msg,
                                                              WPARAM wParam,
                                                              LPARAM lParam);

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr int kInitialWidth = 1760;
constexpr int kInitialHeight = 990;
constexpr int kMinWidth = 1000;
constexpr int kMinHeight = 650;

void showError(const char* message) {
    std::fprintf(stderr, "%s\n", message);
    MessageBoxA(nullptr, message, "RM 3D Simulator - error", MB_OK | MB_ICONERROR);
}

// =============================================================================
//  USER CODE: camera pipeline
//
//  Interface locations:
//    - userProcessCamera(): below; called for camera indices 0..4
//    - userUpdateArm(): below; called before servo integration
//    - Simulator::UserSwitches: written by the sidebar "接口" tab
//
//  Framework:  captureCamera -> userProcessCamera -> show (automatic)
//
//  Return the image that should be displayed. You normally modify `img` in
//  place and `return img`. The framework wires the returned image into the
//  five-camera display, so there is no explicit show call in user code.
//
//  Camera indices:
//      0..3 = car cameras (+X, -X, +Y, -Y)
//      4    = gripper camera
//
//  Requirements:
//      - return a CameraFrame& that stays alive until renderWindow() returns
//      - keep img.width / img.height if you want the normal 4:3 pane
//      - `switches` is read-only; set it from the sidebar "接口" tab
//
//  `img.rgba` is RGBA8, top-left origin. `img.depth` is raw OpenGL depth.
// =============================================================================
sim::CameraFrame& userProcessCamera(int cameraIndex,
                                    sim::CameraFrame& img,
                                    const sim::UserSwitches& switches) {
    (void)cameraIndex;

    if (switches.grayscale || switches.invert) {
        for (std::size_t i = 0; i + 3 < img.rgba.size(); i += 4) {
            const std::uint8_t r = img.rgba[i + 0];
            const std::uint8_t g = img.rgba[i + 1];
            const std::uint8_t b = img.rgba[i + 2];
            if (switches.grayscale) {
                const std::uint8_t y = static_cast<std::uint8_t>(
                    (static_cast<int>(r) * 77 + static_cast<int>(g) * 150 +
                     static_cast<int>(b) * 29) >> 8);
                img.rgba[i + 0] = y;
                img.rgba[i + 1] = y;
                img.rgba[i + 2] = y;
            }
            if (switches.invert) {
                img.rgba[i + 0] = static_cast<std::uint8_t>(255 - img.rgba[i + 0]);
                img.rgba[i + 1] = static_cast<std::uint8_t>(255 - img.rgba[i + 1]);
                img.rgba[i + 2] = static_cast<std::uint8_t>(255 - img.rgba[i + 2]);
            }
        }
    }

    if (switches.cameraOverlay) {
        const int cx = img.width / 2;
        const int cy = img.height / 2;
        for (int d = -24; d <= 24; ++d) {
            const int x = cx + d;
            const int y = cy + d;
            for (int t = -1; t <= 1; ++t) {
                if (x >= 0 && x < img.width && cy + t >= 0 && cy + t < img.height) {
                    const std::size_t i =
                        (static_cast<std::size_t>(cy + t) * img.width + x) * 4;
                    img.rgba[i + 0] = 40;
                    img.rgba[i + 1] = 255;
                    img.rgba[i + 2] = 170;
                }
                if (cx + t >= 0 && cx + t < img.width && y >= 0 && y < img.height) {
                    const std::size_t i =
                        (static_cast<std::size_t>(y) * img.width + cx + t) * 4;
                    img.rgba[i + 0] = 40;
                    img.rgba[i + 1] = 255;
                    img.rgba[i + 2] = 170;
                }
            }
        }
    }

    return img;
}

// =============================================================================
//  USER CODE: arm motor targets
//
//  Called every frame BEFORE the servo simulation. Set the four joint targets
//  here; the framework handles acceleration/deceleration, max speed, and max
//  acceleration. Joint order:
//      0 = base yaw, 1 = shoulder, 2 = elbow, 3 = wrist
//
//  Leave this function empty to control the arm from the sidebar instead.
// =============================================================================
void userUpdateArm(sim::ArmControl& arm, const sim::UserSwitches& switches) {
    if (switches.armDemo) {
        arm.setTargetsDegrees(0.0f, 55.0f, -70.0f, 20.0f);
    }

    // Examples for direct API control:
    //   arm.setTargetsDegrees(0.0f, 55.0f, -70.0f, 20.0f);
    //   arm.setTargetDegrees(1, 80.0f);
    //   arm.setGripperOpening(0.12f);  // 0 .. 0.14 m
}

// =============================================================================
//  Win32 window + OpenGL context
// =============================================================================
class SimWindow {
public:
    SimWindow() = default;
    ~SimWindow() {
        destroy();
    }

    SimWindow(const SimWindow&) = delete;
    SimWindow& operator=(const SimWindow&) = delete;

    bool create(const char* title, int width, int height) {
        SetProcessDPIAware();

        WNDCLASSEXA wc{};
        wc.cbSize = sizeof(wc);
        wc.style = CS_OWNDC | CS_HREDRAW | CS_VREDRAW;
        wc.lpfnWndProc = &SimWindow::wndProc;
        wc.hInstance = GetModuleHandleA(nullptr);
        wc.hCursor = LoadCursorA(nullptr, IDC_ARROW);
        wc.hbrBackground = nullptr;
        wc.lpszClassName = "RM3DCameraSimulatorWindow";
        if (!RegisterClassExA(&wc)) {
            showError("RegisterClassEx failed.");
            return false;
        }

        RECT work{};
        SystemParametersInfoA(SPI_GETWORKAREA, 0, &work, 0);
        const int workW = work.right - work.left;
        const int workH = work.bottom - work.top;
        width = std::max(kMinWidth, std::min(width, workW - 40));
        height = std::max(kMinHeight, std::min(height, workH - 40));

        RECT rect{0, 0, width, height};
        AdjustWindowRectEx(&rect, WS_OVERLAPPEDWINDOW, FALSE, 0);
        const int actualW = rect.right - rect.left;
        const int actualH = rect.bottom - rect.top;
        const int x = work.left + (workW - actualW) / 2;
        const int y = work.top + (workH - actualH) / 2;

        hwnd_ = CreateWindowExA(0,
                                wc.lpszClassName,
                                title,
                                WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN | WS_CLIPSIBLINGS,
                                x,
                                y,
                                actualW,
                                actualH,
                                nullptr,
                                nullptr,
                                wc.hInstance,
                                this);
        if (hwnd_ == nullptr) {
            showError("CreateWindowEx failed.");
            return false;
        }

        if (!createOpenGl()) {
            return false;
        }

        RECT client{};
        GetClientRect(hwnd_, &client);
        width_ = client.right - client.left;
        height_ = client.bottom - client.top;
        running_ = true;

        ShowWindow(hwnd_, SW_SHOW);
        UpdateWindow(hwnd_);
        return true;
    }

    bool pumpMessages() {
        MSG msg{};
        while (PeekMessageA(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                running_ = false;
                return false;
            }
            TranslateMessage(&msg);
            DispatchMessageA(&msg);
        }
        return running_;
    }

    void present() const {
        if (hdc_ != nullptr && hglrc_ != nullptr) {
            SwapBuffers(hdc_);
        }
    }

    bool keyDown(int virtualKey) const {
        if (virtualKey < 0 || virtualKey >= 256) {
            return false;
        }
        return keys_[static_cast<std::size_t>(virtualKey)];
    }

    bool consumeMouseClick(int& x, int& y) {
        if (!mouseClicked_) {
            return false;
        }
        x = mouseX_;
        y = mouseY_;
        mouseClicked_ = false;
        return true;
    }

    void requestClose() {
        running_ = false;
    }

    int clientWidth() {
        RECT rect{};
        GetClientRect(hwnd_, &rect);
        return std::max(1, static_cast<int>(rect.right - rect.left));
    }

    int clientHeight() {
        RECT rect{};
        GetClientRect(hwnd_, &rect);
        return std::max(1, static_cast<int>(rect.bottom - rect.top));
    }

    bool minimized() const {
        return hwnd_ != nullptr && IsIconic(hwnd_) != FALSE;
    }

    HWND nativeHandle() const {
        return hwnd_;
    }

private:
    static LRESULT CALLBACK wndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
        SimWindow* self = reinterpret_cast<SimWindow*>(GetWindowLongPtrA(hwnd, GWLP_USERDATA));
        if (msg == WM_NCCREATE) {
            auto* create = reinterpret_cast<CREATESTRUCTA*>(lParam);
            self = static_cast<SimWindow*>(create->lpCreateParams);
            if (self != nullptr) {
                self->hwnd_ = hwnd;
                SetWindowLongPtrA(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(self));
            }
        }
        if (self == nullptr) {
            return DefWindowProcA(hwnd, msg, wParam, lParam);
        }

        // Let Dear ImGui consume mouse/keyboard events for the sidebar. Our
        // own handling below still runs for movement keys and depth clicks.
        if (ImGui::GetCurrentContext() != nullptr) {
            ImGui_ImplWin32_WndProcHandler(hwnd, msg, wParam, lParam);
        }

        switch (msg) {
            case WM_CLOSE:
                self->running_ = false;
                return 0;
            case WM_DESTROY:
                self->running_ = false;
                PostQuitMessage(0);
                return 0;
            case WM_ERASEBKGND:
                return 1;
            case WM_PAINT: {
                PAINTSTRUCT ps{};
                BeginPaint(hwnd, &ps);
                EndPaint(hwnd, &ps);
                return 0;
            }
            case WM_SIZE:
                if (wParam != SIZE_MINIMIZED) {
                    self->width_ = LOWORD(lParam);
                    self->height_ = HIWORD(lParam);
                }
                return 0;
            case WM_GETMINMAXINFO: {
                auto* info = reinterpret_cast<MINMAXINFO*>(lParam);
                info->ptMinTrackSize.x = kMinWidth;
                info->ptMinTrackSize.y = kMinHeight;
                return 0;
            }
            case WM_KEYDOWN:
                if (wParam < 256) {
                    self->keys_[wParam] = true;
                }
                if (wParam == VK_ESCAPE) {
                    self->running_ = false;
                }
                return 0;
            case WM_KEYUP:
                if (wParam < 256) {
                    self->keys_[wParam] = false;
                }
                return 0;
            case WM_SYSKEYUP:
                if (wParam < 256) {
                    self->keys_[wParam] = false;
                }
                return 0;
            case WM_KILLFOCUS:
                self->keys_.fill(false);
                return 0;
            case WM_LBUTTONDOWN:
                self->mouseX_ = static_cast<short>(LOWORD(lParam));
                self->mouseY_ = static_cast<short>(HIWORD(lParam));
                self->mouseClicked_ = true;
                SetFocus(hwnd);
                return 0;
            case WM_ACTIVATEAPP:
                if (wParam == FALSE) {
                    self->keys_.fill(false);
                }
                return 0;
            default:
                break;
        }
        return DefWindowProcA(hwnd, msg, wParam, lParam);
    }

    bool createOpenGl() {
        hdc_ = GetDC(hwnd_);
        if (hdc_ == nullptr) {
            showError("GetDC failed.");
            return false;
        }

        PIXELFORMATDESCRIPTOR pfd{};
        pfd.nSize = sizeof(pfd);
        pfd.nVersion = 1;
        pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
        pfd.iPixelType = PFD_TYPE_RGBA;
        pfd.cColorBits = 32;
        pfd.cDepthBits = 24;
        pfd.cStencilBits = 8;
        pfd.iLayerType = PFD_MAIN_PLANE;

        const int format = ChoosePixelFormat(hdc_, &pfd);
        if (format == 0) {
            showError("ChoosePixelFormat failed.");
            return false;
        }
        if (!SetPixelFormat(hdc_, format, &pfd)) {
            showError("SetPixelFormat failed.");
            return false;
        }

        hglrc_ = wglCreateContext(hdc_);
        if (hglrc_ == nullptr) {
            showError("wglCreateContext failed.");
            return false;
        }
        if (!wglMakeCurrent(hdc_, hglrc_)) {
            showError("wglMakeCurrent failed.");
            return false;
        }

        using SwapIntervalProc = BOOL(WINAPI*)(int);
        auto swapInterval = reinterpret_cast<SwapIntervalProc>(
            wglGetProcAddress("wglSwapIntervalEXT"));
        if (swapInterval != nullptr) {
            vsyncEnabled_ = swapInterval(1) != FALSE;
        }
        return true;
    }

    void destroy() {
        if (hglrc_ != nullptr) {
            wglMakeCurrent(nullptr, nullptr);
            wglDeleteContext(hglrc_);
            hglrc_ = nullptr;
        }
        if (hdc_ != nullptr && hwnd_ != nullptr) {
            ReleaseDC(hwnd_, hdc_);
            hdc_ = nullptr;
        }
        if (hwnd_ != nullptr) {
            DestroyWindow(hwnd_);
            hwnd_ = nullptr;
        }
        running_ = false;
    }

    HWND hwnd_ = nullptr;
    HDC hdc_ = nullptr;
    HGLRC hglrc_ = nullptr;
    bool running_ = false;
    bool vsyncEnabled_ = false;
    int width_ = kInitialWidth;
    int height_ = kInitialHeight;

    std::array<bool, 256> keys_{};
    bool mouseClicked_ = false;
    int mouseX_ = 0;
    int mouseY_ = 0;
};

// =============================================================================
//  Screenshot helper (PPM P6, useful for headless verification)
// =============================================================================
bool saveScreenshotPpm(const std::string& path, int width, int height) {
    if (width <= 0 || height <= 0) {
        return false;
    }
    std::vector<std::uint8_t> rgba(static_cast<std::size_t>(width) * height * 4);
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());

    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return false;
    }
    out << "P6\n" << width << " " << height << "\n255\n";
    for (int y = height - 1; y >= 0; --y) {
        const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
        for (int x = 0; x < width; ++x) {
            const std::uint8_t* p = rgba.data() + (row + static_cast<std::size_t>(x)) * 4;
            char rgb[3] = {static_cast<char>(p[0]),
                           static_cast<char>(p[1]),
                           static_cast<char>(p[2])};
            out.write(rgb, 3);
        }
    }
    return static_cast<bool>(out);
}

void printUsage() {
    std::printf(
        "RM 3D camera simulator\n"
        "Usage: rm_camera_sim [--screenshot <file.ppm>] [--self-test] "
        "[--size <width>x<height>] [--click <x>,<y>] [--pose <x>,<y>[,yaw]]\n"
        "  --screenshot  render one frame, save it, and exit\n"
        "  --self-test   print camera poses and ray-depth samples, then exit\n"
        "  --size        initial client size, e.g. 1760x990 (default)\n"
        "  --click       simulate one left-click in client coordinates (for screenshots)\n"
        "  --pose        place the car on the terrain at field (x,y) and optional yaw\n");
}

void runMotionSelfTest(sim::Simulator& simulator) {
    simulator.reset();
    const sim::CarState startPose = simulator.car();
    sim::InputState input;
    input.forward = 1.0f;
    for (int i = 0; i < 10; ++i) {
        simulator.update(input, 0.05f);
    }
    const sim::CarState afterForward = simulator.car();
    const sim::CameraPose forwardPose = simulator.cameraPose(0);

    input = {};
    input.strafeLeft = 1.0f;
    for (int i = 0; i < 10; ++i) {
        simulator.update(input, 0.05f);
    }
    const sim::CarState afterStrafe = simulator.car();

    input = {};
    input.yawRate = 1.0f;
    for (int i = 0; i < 10; ++i) {
        simulator.update(input, 0.05f);
    }
    const sim::CarState afterYaw = simulator.car();
    const sim::CameraPose rotatedPose = simulator.cameraPose(0);

    const bool movedForward = afterForward.x > startPose.x + 1.0f &&
                              std::fabs(afterForward.y - startPose.y) < 0.01f;
    const bool headingHeld = std::fabs(afterForward.yaw) < 1.0e-5f;
    const bool strafed = afterStrafe.y > afterForward.y + 1.0f &&
                         std::fabs(afterStrafe.x - afterForward.x) < 0.01f;
    const bool yawed = afterYaw.yaw > 1.0f;
    const bool cameraTurned = rotatedPose.forward.x < forwardPose.forward.x - 0.3f &&
                              rotatedPose.forward.y > 0.7f;

    std::printf("[self-test] motion: forward=(%s) heading-held=(%s) strafe=(%s) "
                "yaw=(%s) camera-rotated=(%s)\n",
                movedForward ? "PASS" : "FAIL",
                headingHeld ? "PASS" : "FAIL",
                strafed ? "PASS" : "FAIL",
                yawed ? "PASS" : "FAIL",
                cameraTurned ? "PASS" : "FAIL");
    std::printf("[self-test]   after W: x=%.3f y=%.3f yaw=%.3f\n",
                afterForward.x,
                afterForward.y,
                afterForward.yaw);
    std::printf("[self-test]   after A: x=%.3f y=%.3f yaw=%.3f\n",
                afterStrafe.x,
                afterStrafe.y,
                afterStrafe.yaw);
    std::printf("[self-test]   after Q: x=%.3f y=%.3f yaw=%.3f  cam0 fwd=(%+.3f, %+.3f)\n",
                afterYaw.x,
                afterYaw.y,
                afterYaw.yaw,
                rotatedPose.forward.x,
                rotatedPose.forward.y);
    simulator.reset();
}

void runLightSelfTest(sim::Simulator& simulator) {
    simulator.resetLightRig();
    const bool defaults = simulator.lightCount() == 4;

    simulator.setLightCount(0);
    const bool zero = simulator.lightCount() == 0;
    simulator.setLightCount(100);
    const bool clampedMax = simulator.lightCount() == sim::kMaxLights;

    simulator.resetLightRig();
    simulator.setLightPosition(0, {1.25f, -2.5f, 6.75f});
    const sim::Vec3 position = simulator.lightSettings(0).position;
    const bool moved = std::fabs(position.x - 1.25f) < 1.0e-5f &&
                       std::fabs(position.y + 2.5f) < 1.0e-5f &&
                       std::fabs(position.z - 6.75f) < 1.0e-5f;

    simulator.setLightIntensity(0, 1.25f);
    const bool intensity = std::fabs(simulator.lightSettings(0).intensity - 1.25f) < 1.0e-5f;
    simulator.resetLightRig();

    std::printf("[self-test] lights: defaults=(%s) zero=(%s) clamp8=(%s) position=(%s) "
                "intensity=(%s)\n",
                defaults ? "PASS" : "FAIL",
                zero ? "PASS" : "FAIL",
                clampedMax ? "PASS" : "FAIL",
                moved ? "PASS" : "FAIL",
                intensity ? "PASS" : "FAIL");
}

void runArmSelfTest(sim::Simulator& simulator) {
    sim::ArmControl control(simulator);
    simulator.resetArmPose();
    simulator.setServoLimits(180.0f, 720.0f);
    control.setTargetDegrees(1, 140.0f);

    const float dt = 1.0f / 120.0f;
    float previousVelocity = simulator.armVelocityDegrees(1);
    float maxVelocity = 0.0f;
    float maxAcceleration = 0.0f;
    for (int step = 0; step < 600; ++step) {
        simulator.updateArm(dt);
        const float velocity = simulator.armVelocityDegrees(1);
        maxVelocity = std::max(maxVelocity, std::fabs(velocity));
        if (step > 0) {
            maxAcceleration = std::max(maxAcceleration,
                                       std::fabs(velocity - previousVelocity) / dt);
        }
        previousVelocity = velocity;
    }

    const float shoulderAfterMove = simulator.armCurrentDegrees(1);
    const bool reached = std::fabs(shoulderAfterMove - 140.0f) < 0.25f;
    const bool speedLimited = maxVelocity <= 180.5f;
    const bool accelLimited = maxAcceleration <= 720.5f;
    const bool accelerated = maxVelocity > 30.0f && maxAcceleration > 30.0f;

    control.setTargetDegrees(0, -360.0f);
    const bool clampedJoint = std::fabs(simulator.armTargetDegrees(0) + 180.0f) < 0.25f;

    simulator.setArmLinkLengths(0.42f, 0.36f);
    const sim::ArmConfig& cfg = simulator.armConfig();
    const bool lengths = std::fabs(cfg.upperArmLength - 0.42f) < 1.0e-5f &&
                         std::fabs(cfg.lowerArmLength - 0.36f) < 1.0e-5f;

    simulator.setGripperOpening(0.30f);
    const bool gripperClamped =
        std::fabs(simulator.armConfig().gripperOpening - sim::kGripperMaxOpening) < 1.0e-5f;

    const float oldHeight = simulator.cameraMountHeight();
    simulator.setCameraMountHeight(0.75f);
    const bool cameraHeight =
        std::fabs(simulator.cameraPose(0).position.z - (simulator.car().z + 0.75f)) < 1.0e-5f;
    simulator.setCameraMountHeight(oldHeight);

    simulator.setArmLinkLengths(0.32f, 0.28f);
    simulator.setArmBasePosition({0.0f, 0.0f, 0.22f});
    simulator.resetArmPose();

    std::printf("[self-test] arm: reached=(%s) max-speed=(%s) max-accel=(%s) "
                "accelerated=(%s) joint-limit=(%s) lengths=(%s) gripper14cm=(%s) "
                "camera-height=(%s)\n",
                reached ? "PASS" : "FAIL",
                speedLimited ? "PASS" : "FAIL",
                accelLimited ? "PASS" : "FAIL",
                accelerated ? "PASS" : "FAIL",
                clampedJoint ? "PASS" : "FAIL",
                lengths ? "PASS" : "FAIL",
                gripperClamped ? "PASS" : "FAIL",
                cameraHeight ? "PASS" : "FAIL");
    std::printf("[self-test]   arm peak speed=%.1f deg/s, peak accel=%.1f deg/s^2, "
                "shoulder after move=%.2f deg\n",
                maxVelocity,
                maxAcceleration,
                shoulderAfterMove);
}

void runCarPhysicsSelfTest(sim::Simulator& simulator) {
    const bool modelLoaded = simulator.carModelLoaded();

    sim::UserSwitches testSwitches;
    testSwitches.cameraOverlay = true;
    testSwitches.grayscale = true;
    simulator.setUserSwitches(testSwitches);
    const sim::UserSwitches readBack = simulator.userSwitches();
    const bool switchState = readBack.cameraOverlay && readBack.grayscale &&
                             !readBack.invert && !readBack.armDemo;
    simulator.setUserSwitches({});

    // Broad ramp near the east wooden platform.
    simulator.setCarPose(3.5f, -0.5f, 0.0f);
    const float slopeRollDeg = simulator.car().roll * (180.0f / 3.1415926f);
    const float slopePitchDeg = simulator.car().pitch * (180.0f / 3.1415926f);
    const bool tilted = std::fabs(slopeRollDeg) > 4.0f || std::fabs(slopePitchDeg) > 2.0f;

    // Drop test on open floor.
    float ground = 0.0f;
    simulator.setCarPose(0.0f, -1.3f, 0.0f);
    const bool groundKnown = simulator.groundHeightAt(0.0f, -1.3f, ground);
    simulator.dropCarFromHeight(0.55f);
    const float droppedFrom = simulator.car().z;
    float lowest = droppedFrom;
    bool sawAirborne = simulator.car().airborne;
    sim::InputState idle{};
    for (int i = 0; i < 240; ++i) {
        simulator.update(idle, 1.0f / 120.0f);
        sawAirborne = sawAirborne || simulator.car().airborne;
        lowest = std::min(lowest, simulator.car().z);
    }
    const bool gravity = groundKnown && sawAirborne &&
                         lowest < droppedFrom - 0.25f &&
                         !simulator.car().airborne &&
                         std::fabs(simulator.car().z - ground) < 0.06f;
    const float settledZ = simulator.car().z;

    // Drive straight at the central platform; the steep ridge must stop the
    // chassis instead of teleporting it onto the high surface.
    simulator.setCarPose(0.0f, -1.0f, 3.1415926f * 0.5f);
    sim::InputState forward{};
    forward.forward = 1.0f;
    for (int i = 0; i < 120; ++i) {
        simulator.update(forward, 1.0f / 120.0f);
    }
    const bool collision = simulator.car().z < 0.30f && simulator.car().y < -0.15f;

    // Drive through the north bridge/tunnel under the three gray ores.
    simulator.setCarPose(0.0f, 1.45f, 3.1415926f * 0.5f);
    for (int i = 0; i < 180; ++i) {
        simulator.update(forward, 1.0f / 120.0f);
    }
    const bool tunnel = simulator.car().y > 1.95f && simulator.car().z < 0.20f;
    const float tunnelY = simulator.car().y;
    const float tunnelZ = simulator.car().z;

    // Blue arch with a single ore on top, on the raised east platform.
    simulator.setCarPose(4.90f, -1.40f, -3.1415926f * 0.5f);
    for (int i = 0; i < 180; ++i) {
        simulator.update(forward, 1.0f / 120.0f);
    }
    const bool blueArch = simulator.car().y < -1.90f && simulator.car().z < 0.35f;
    const float blueArchY = simulator.car().y;
    const float blueArchZ = simulator.car().z;

    simulator.reset();

    std::printf("[self-test] car: model=(%s) tilt=(%s) gravity=(%s) collision=(%s) "
                "tunnel=(%s) blue-arch=(%s) switches=(%s)\n",
                modelLoaded ? "PASS" : "FAIL",
                tilted ? "PASS" : "FAIL",
                gravity ? "PASS" : "FAIL",
                collision ? "PASS" : "FAIL",
                tunnel ? "PASS" : "FAIL",
                blueArch ? "PASS" : "FAIL",
                switchState ? "PASS" : "FAIL");
    const sim::SimConfig& carConfig = simulator.config();
    std::printf("[self-test]   slope roll=%+.1f deg pitch=%+.1f deg; "
                "drop %.2f -> %.2f m, ground=%.2f m\n",
                slopeRollDeg,
                slopePitchDeg,
                droppedFrom,
                settledZ,
                ground);
    std::printf("[self-test]   body %.2f x %.2f m, 4 bumper boxes "
                "z=%.2f..%.2f m, wheelbase %.2f m, track %.2f m\n",
                carConfig.carLength,
                carConfig.carWidth,
                carConfig.carBumperBottom,
                carConfig.carBumperBottom + carConfig.carBumperHeight,
                carConfig.carWheelBase,
                carConfig.carTrackWidth);
    std::printf("[self-test]   tunnels ended at centre y=%.2f z=%.2f, "
                "blue-arch y=%.2f z=%.2f\n",
                tunnelY,
                tunnelZ,
                blueArchY,
                blueArchZ);
}

void runDepthSelfTest(const sim::Simulator& simulator) {
    const sim::CameraIntrinsics& intr = simulator.cameraIntrinsics(0);
    std::printf("\n[self-test] camera intrinsics: %dx%d  fx=%.2f fy=%.2f cx=%.2f cy=%.2f\n",
                intr.width,
                intr.height,
                intr.fx,
                intr.fy,
                intr.cx,
                intr.cy);

    const float sampleV[] = {0.15f, 0.50f, 0.75f, 0.90f};
    const char* sampleName[] = {"sky", "horizon", "floor", "near-floor"};
    for (int cam = 0; cam < sim::kCameraCount; ++cam) {
        const sim::CameraPose& pose = simulator.cameraPose(cam);
        std::printf("[self-test] cam %d pos=(%.3f, %.3f, %.3f) "
                    "forward=(%+.3f, %+.3f, %+.3f)\n",
                    cam,
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.forward.x,
                    pose.forward.y,
                    pose.forward.z);
        for (int s = 0; s < 4; ++s) {
            const float u = static_cast<float>(intr.width) * 0.5f;
            const float v = static_cast<float>(intr.height) * sampleV[s];
            float depth = 0.0f;
            sim::Vec3 world;
            if (simulator.getRayDepth(cam, u, v, depth) &&
                simulator.getPixelWorld(cam, u, v, world)) {
                std::printf("  %-10s pixel(%7.2f,%7.2f) depth=%8.4f m  "
                            "world=(%+.3f, %+.3f, %+.3f)\n",
                            sampleName[s],
                            u,
                            v,
                            depth,
                            world.x,
                            world.y,
                            world.z);
            } else {
                std::printf("  %-10s pixel(%7.2f,%7.2f) no hit\n",
                            sampleName[s],
                            u,
                            v);
            }
        }
    }
    std::printf("[self-test] done\n\n");
    std::fflush(stdout);
}

}  // namespace

int main(int argc, char** argv) {
    std::string screenshotPath;
    bool selfTest = false;
    int initialWidth = kInitialWidth;
    int initialHeight = kInitialHeight;
    int syntheticClickX = -1;
    int syntheticClickY = -1;
    float poseX = 0.0f;
    float poseY = 0.0f;
    float poseYawDeg = 0.0f;
    bool hasPose = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        }
        if (arg == "--screenshot") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "--screenshot requires a path\n");
                return 2;
            }
            screenshotPath = argv[++i];
            continue;
        }
        if (arg == "--self-test") {
            selfTest = true;
            continue;
        }
        if (arg == "--size") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "--size requires <width>x<height>\n");
                return 2;
            }
            const std::string sizeArg = argv[++i];
            int parsedW = 0;
            int parsedH = 0;
            if (std::sscanf(sizeArg.c_str(), "%dx%d", &parsedW, &parsedH) != 2 ||
                parsedW <= 0 || parsedH <= 0) {
                std::fprintf(stderr, "Invalid --size: %s\n", sizeArg.c_str());
                return 2;
            }
            initialWidth = parsedW;
            initialHeight = parsedH;
            continue;
        }
        if (arg == "--pose") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "--pose requires <x>,<y>[,yaw_deg]\n");
                return 2;
            }
            const std::string poseArg = argv[++i];
            if (std::sscanf(poseArg.c_str(), "%f,%f,%f", &poseX, &poseY, &poseYawDeg) < 2) {
                std::fprintf(stderr, "Invalid --pose: %s\n", poseArg.c_str());
                return 2;
            }
            hasPose = true;
            continue;
        }
        if (arg == "--click") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "--click requires <x>,<y>\n");
                return 2;
            }
            const std::string clickArg = argv[++i];
            int parsedX = 0;
            int parsedY = 0;
            if (std::sscanf(clickArg.c_str(), "%d,%d", &parsedX, &parsedY) != 2 ||
                parsedX < 0 || parsedY < 0) {
                std::fprintf(stderr, "Invalid --click: %s\n", clickArg.c_str());
                return 2;
            }
            syntheticClickX = parsedX;
            syntheticClickY = parsedY;
            continue;
        }
        std::fprintf(stderr, "Unknown argument: %s\n", arg.c_str());
        printUsage();
        return 2;
    }

    std::printf(
        "RM 3D camera simulator\n"
        "  WASD: move (robot frame, mecanum strafe)\n"
        "  Q/E : rotate CCW/CW\n"
        "  R   : reset to spawn\n"
        "  LMB : query ray depth in a camera pane\n"
        "  Sidebar: Arm | Camera | Lights\n");

    SimWindow window;
    if (!window.create("RM 3D Camera Simulator", initialWidth, initialHeight)) {
        return 1;
    }

    sim::Simulator simulator;
    simulator.initializeGraphics();

    ControlPanel controlPanel;
    if (!controlPanel.initialize(window.nativeHandle())) {
        showError("Could not initialize the Dear ImGui control panel.");
        return 1;
    }

    if (hasPose) {
        simulator.setCarPose(poseX, poseY, poseYawDeg * 3.1415926f / 180.0f);
    }

    sim::ArmControl armControl(simulator);
    bool resetHeldLastFrame = false;
    bool screenshotSaved = false;

    using Clock = std::chrono::steady_clock;
    auto previousTime = Clock::now();
    float smoothedFps = 60.0f;

    while (window.pumpMessages()) {
        if (window.minimized()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
            continue;
        }

        const auto frameTime = Clock::now();
        const float dt = std::min(0.05f,
                                  std::chrono::duration<float>(frameTime - previousTime).count());
        previousTime = frameTime;

        const bool resetHeld = window.keyDown('R');
        if (resetHeld && !resetHeldLastFrame) {
            simulator.reset();
        }
        resetHeldLastFrame = resetHeld;

        sim::InputState input;
        input.forward = (window.keyDown('W') ? 1.0f : 0.0f) -
                        (window.keyDown('S') ? 1.0f : 0.0f);
        input.strafeLeft = (window.keyDown('A') ? 1.0f : 0.0f) -
                           (window.keyDown('D') ? 1.0f : 0.0f);
        input.yawRate = (window.keyDown('Q') ? 1.0f : 0.0f) -
                        (window.keyDown('E') ? 1.0f : 0.0f);
        input.boost = window.keyDown(VK_SHIFT);
        simulator.update(input, dt);

        // User arm targets -> servo simulation (acceleration/speed limited).
        const sim::UserSwitches uiSwitches = simulator.userSwitches();
        userUpdateArm(armControl, uiSwitches);
        simulator.updateArm(dt);

        // ---------------------------------------------------------------------
        //  Frame loop: img -> userProcessCamera -> show (handled internally).
        // ---------------------------------------------------------------------
        for (int cameraIndex = 0; cameraIndex < sim::kCameraCount; ++cameraIndex) {
            simulator.captureCamera(cameraIndex);
            sim::CameraFrame& shown = userProcessCamera(cameraIndex,
                                                        simulator.cameraFrame(cameraIndex),
                                                        uiSwitches);
            simulator.setCameraDisplayFrame(cameraIndex, &shown);
        }

        int mouseX = 0;
        int mouseY = 0;
        if (window.consumeMouseClick(mouseX, mouseY)) {
            const int w = window.clientWidth();
            const int h = window.clientHeight();
            const sim::DepthProbe probe =
                simulator.handleMouseClick(w, h, mouseX, mouseY);
            if (probe.cameraIndex >= 0) {
                // You can also query any pixel directly:
                //   float depth = simulator.getRayDepth(0, px, py);
                //   sim::Vec3 world;
                //   simulator.getPixelWorld(0, px, py, world);
                (void)probe;
            }
        }

        const int width = window.clientWidth();
        const int height = window.clientHeight();

        if (syntheticClickX >= 0 && syntheticClickY >= 0) {
            simulator.handleMouseClick(width, height, syntheticClickX, syntheticClickY);
            syntheticClickX = -1;
            syntheticClickY = -1;
        }

        const float instantFps = dt > 0.0f ? 1.0f / dt : 0.0f;
        smoothedFps += (instantFps - smoothedFps) * 0.08f;
        simulator.setFps(smoothedFps);

        simulator.renderWindow(width, height);
        controlPanel.render(simulator, width, height);

        if (!screenshotPath.empty() && !screenshotSaved) {
            screenshotSaved = saveScreenshotPpm(screenshotPath, width, height);
            if (!screenshotSaved) {
                std::fprintf(stderr, "Could not write screenshot: %s\n", screenshotPath.c_str());
                return 1;
            }
            std::printf("Saved screenshot: %s (%dx%d)\n", screenshotPath.c_str(), width, height);
            if (selfTest) {
                runDepthSelfTest(simulator);
                runMotionSelfTest(simulator);
                runCarPhysicsSelfTest(simulator);
                runArmSelfTest(simulator);
                runLightSelfTest(simulator);
            }
            window.requestClose();
        } else if (selfTest) {
            runDepthSelfTest(simulator);
            runMotionSelfTest(simulator);
            runCarPhysicsSelfTest(simulator);
            runArmSelfTest(simulator);
            runLightSelfTest(simulator);
            window.requestClose();
        }

        window.present();

        // If the driver did not provide wglSwapIntervalEXT, avoid a busy loop.
        // SwapBuffers normally throttles us to the monitor refresh rate.
        // (Sleep(1) is also harmless on systems with adaptive sync.)
        if (dt < 0.004f) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    return 0;
}
