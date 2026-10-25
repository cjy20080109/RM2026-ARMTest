#define NOMINMAX
#define WIN32_LEAN_AND_MEAN

#include "control_panel.h"

#include "simulator.h"

#include "imgui.h"
#include "imgui_impl_opengl2.h"
#include "imgui_impl_win32.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

constexpr ImVec4 kAccent(0.20f, 0.85f, 0.95f, 1.0f);
constexpr ImVec4 kMuted(0.58f, 0.66f, 0.75f, 1.0f);
constexpr ImVec4 kText(0.90f, 0.94f, 0.98f, 1.0f);

bool fileExists(const char* path) {
    const DWORD attributes = GetFileAttributesA(path);
    return attributes != INVALID_FILE_ATTRIBUTES &&
           (attributes & FILE_ATTRIBUTE_DIRECTORY) == 0;
}

}  // namespace

ControlPanel::~ControlPanel() {
    shutdown();
}

bool ControlPanel::initialize(HWND hwnd) {
    if (initialized_) {
        return true;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.LogFilename = nullptr;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    applyStyle();
    initializeFonts();

    if (!ImGui_ImplWin32_Init(hwnd)) {
        ImGui::DestroyContext();
        return false;
    }
    if (!ImGui_ImplOpenGL2_Init()) {
        ImGui_ImplWin32_Shutdown();
        ImGui::DestroyContext();
        return false;
    }

    initialized_ = true;
    return true;
}

void ControlPanel::shutdown() {
    if (!initialized_) {
        return;
    }
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    initialized_ = false;
}

bool ControlPanel::initializeFonts() {
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->Clear();
    chineseFont_ = false;

    // Use a Windows system font so the sidebar can render Chinese labels
    // without shipping a font file or touching any environment variable.
    const char* chineseFonts[] = {
        "C:\\Windows\\Fonts\\msyh.ttc",
        "C:\\Windows\\Fonts\\simhei.ttf",
        "C:\\Windows\\Fonts\\simsun.ttc",
    };
    for (const char* path : chineseFonts) {
        if (!fileExists(path)) {
            continue;
        }
        ImFontConfig cfg;
        cfg.OversampleH = 2;
        cfg.OversampleV = 2;
        cfg.PixelSnapH = true;
        if (io.Fonts->AddFontFromFileTTF(path,
                                         18.0f,
                                         &cfg,
                                         io.Fonts->GetGlyphRangesChineseSimplifiedCommon()) !=
            nullptr) {
            chineseFont_ = true;
            return true;
        }
        io.Fonts->Clear();
    }

    if (fileExists("C:\\Windows\\Fonts\\segoeui.ttf")) {
        if (io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\segoeui.ttf", 17.0f) != nullptr) {
            return true;
        }
        io.Fonts->Clear();
    }

    io.Fonts->AddFontDefault();
    return true;
}

void ControlPanel::applyStyle() {
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();

    style.WindowRounding = 0.0f;
    style.ChildRounding = 4.0f;
    style.FrameRounding = 4.0f;
    style.PopupRounding = 4.0f;
    style.ScrollbarRounding = 4.0f;
    style.GrabRounding = 4.0f;
    style.WindowBorderSize = 0.0f;
    style.ChildBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.WindowPadding = ImVec2(14.0f, 12.0f);
    style.FramePadding = ImVec2(7.0f, 4.0f);
    style.ItemSpacing = ImVec2(8.0f, 6.0f);
    style.ItemInnerSpacing = ImVec2(6.0f, 4.0f);
    style.ScrollbarSize = 12.0f;
    style.WindowMenuButtonPosition = ImGuiDir_None;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_Text] = kText;
    colors[ImGuiCol_TextDisabled] = kMuted;
    colors[ImGuiCol_WindowBg] = ImVec4(0.024f, 0.034f, 0.047f, 1.0f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.035f, 0.047f, 0.062f, 1.0f);
    colors[ImGuiCol_PopupBg] = ImVec4(0.035f, 0.047f, 0.062f, 0.98f);
    colors[ImGuiCol_Border] = ImVec4(0.12f, 0.17f, 0.22f, 1.0f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.07f, 0.10f, 0.13f, 1.0f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.10f, 0.14f, 0.18f, 1.0f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.12f, 0.17f, 0.21f, 1.0f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.024f, 0.034f, 0.047f, 1.0f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.024f, 0.034f, 0.047f, 1.0f);
    colors[ImGuiCol_Button] = ImVec4(0.08f, 0.13f, 0.16f, 1.0f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.12f, 0.24f, 0.27f, 1.0f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.15f, 0.32f, 0.35f, 1.0f);
    colors[ImGuiCol_Header] = ImVec4(0.08f, 0.12f, 0.16f, 1.0f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.11f, 0.18f, 0.21f, 1.0f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.12f, 0.21f, 0.24f, 1.0f);
    colors[ImGuiCol_SliderGrab] = ImVec4(0.20f, 0.78f, 0.88f, 1.0f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.35f, 0.95f, 1.0f, 1.0f);
    colors[ImGuiCol_CheckMark] = kAccent;
    colors[ImGuiCol_Separator] = ImVec4(0.11f, 0.16f, 0.21f, 1.0f);
    colors[ImGuiCol_SeparatorHovered] = kAccent;
    colors[ImGuiCol_SeparatorActive] = kAccent;
    colors[ImGuiCol_ScrollbarBg] = ImVec4(0.02f, 0.03f, 0.04f, 0.7f);
    colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.16f, 0.24f, 0.28f, 1.0f);
    colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.22f, 0.36f, 0.40f, 1.0f);
    colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.25f, 0.46f, 0.50f, 1.0f);
}

void ControlPanel::render(sim::Simulator& simulator, int windowWidth, int windowHeight) {
    if (!initialized_ || windowWidth <= 0 || windowHeight <= 0) {
        return;
    }

    const int sidebarWidth = sim::sidebarWidthForWindow(windowWidth);
    const int headerHeight = sim::headerHeightForWindow(windowHeight);
    if (sidebarWidth <= 0 || windowHeight <= headerHeight) {
        return;
    }

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();

    const auto tr = [this](const char* zh, const char* en) {
        return chineseFont_ ? zh : en;
    };

    ImGui::SetNextWindowPos(ImVec2(static_cast<float>(windowWidth - sidebarWidth),
                                  static_cast<float>(headerHeight)));
    ImGui::SetNextWindowSize(ImVec2(static_cast<float>(sidebarWidth),
                                   static_cast<float>(windowHeight - headerHeight)));
    ImGui::SetNextWindowBgAlpha(1.0f);

    const ImGuiWindowFlags windowFlags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;

    ImGui::Begin("##control_sidebar", nullptr, windowFlags);

    ImDrawList* drawList = ImGui::GetWindowDrawList();
    const ImVec2 windowPos = ImGui::GetWindowPos();
    const ImVec2 windowSize = ImGui::GetWindowSize();
    drawList->AddRectFilled(windowPos,
                            ImVec2(windowPos.x + 2.0f, windowPos.y + windowSize.y),
                            ImGui::GetColorU32(kAccent));

    ImGui::PushStyleColor(ImGuiCol_Text, kAccent);
    ImGui::TextUnformatted(tr("控制台", "CONTROL"));
    ImGui::PopStyleColor();
    ImGui::TextDisabled(tr("机械臂 · 相机 · 灯光 · 接口", "Arm - Camera - Lights - API"));
    ImGui::Spacing();

    const float valueReserve = 60.0f;
    auto sliderRow = [&](const char* label,
                         const char* id,
                         float& value,
                         float lo,
                         float hi,
                         const char* valueFormat,
                         float labelWidth) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(label);
        ImGui::SameLine(labelWidth);
        ImGui::SetNextItemWidth(std::max(70.0f,
                                         ImGui::GetContentRegionAvail().x - valueReserve));
        const bool changed = ImGui::SliderFloat(id,
                                                &value,
                                                lo,
                                                hi,
                                                "",
                                                ImGuiSliderFlags_AlwaysClamp);
        ImGui::SameLine();
        ImGui::TextColored(kAccent, valueFormat, value);
        return changed;
    };

    // Custom segmented tabs: they stay readable and clickable with the dark
    // theme, unlike relying on ImGui tab colors for this small panel.
    const char* tabLabels[] = {
        tr("机械臂", "ARM"),
        tr("相机", "CAMERA"),
        tr("灯光", "LIGHTS"),
        tr("接口", "API"),
    };
    const float tabGap = 4.0f;
    const float tabWidth = std::max(54.0f,
                                    (ImGui::GetContentRegionAvail().x - tabGap * 3.0f) / 4.0f);
    int requestedTab = activeTab_;
    for (int tab = 0; tab < 4; ++tab) {
        if (tab > 0) {
            ImGui::SameLine(0.0f, tabGap);
        }
        const bool active = activeTab_ == tab;
        ImGui::PushStyleColor(ImGuiCol_Button,
                              active ? ImVec4(0.10f, 0.38f, 0.42f, 1.0f)
                                     : ImVec4(0.06f, 0.09f, 0.12f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                              ImVec4(0.13f, 0.30f, 0.33f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.16f, 0.45f, 0.50f, 1.0f));
        if (ImGui::Button(tabLabels[tab], ImVec2(tabWidth, 27.0f))) {
            requestedTab = tab;
        }
        ImGui::PopStyleColor(3);
    }
    if (requestedTab != activeTab_) {
        activeTab_ = requestedTab;
        ImGui::SetScrollY(0.0f);
    }
    ImGui::Separator();
    ImGui::Spacing();

    if (activeTab_ == 0) {
        const sim::ArmConfig& arm = simulator.armConfig();

        ImGui::TextDisabled(tr("底座相对小车位置 (m)", "BASE OFFSET (m)"));
        float baseX = arm.basePosition.x;
        float baseY = arm.basePosition.y;
        float baseZ = arm.basePosition.z;
        bool baseChanged = false;
        baseChanged |= sliderRow("X", "##base_x", baseX, -0.28f, 0.28f, "%.3f", 22.0f);
        baseChanged |= sliderRow("Y", "##base_y", baseY, -0.22f, 0.22f, "%.3f", 22.0f);
        baseChanged |= sliderRow("Z", "##base_z", baseZ, 0.12f, 0.65f, "%.3f", 22.0f);
        if (baseChanged) {
            simulator.setArmBasePosition({baseX, baseY, baseZ});
        }

        ImGui::Spacing();
        ImGui::TextDisabled(tr("臂长 (m)", "ARM LENGTHS (m)"));
        float upper = arm.upperArmLength;
        float lower = arm.lowerArmLength;
        bool linksChanged = false;
        linksChanged |= sliderRow(tr("大臂", "Upper"), "##upper", upper, 0.08f, 0.80f, "%.2f", 54.0f);
        linksChanged |= sliderRow(tr("小臂", "Lower"), "##lower", lower, 0.08f, 0.80f, "%.2f", 54.0f);
        if (linksChanged) {
            simulator.setArmLinkLengths(upper, lower);
        }

        ImGui::Spacing();
        ImGui::TextDisabled(tr("夹爪 (cm)", "GRIPPER (cm)"));
        float openingCm = arm.gripperOpening * 100.0f;
        if (sliderRow(tr("张开", "Open"), "##gripper", openingCm, 0.0f, 14.0f, "%.1f", 44.0f)) {
            simulator.setGripperOpening(openingCm * 0.01f);
        }
        ImGui::TextDisabled(tr("最大张开 14.0 cm", "Max opening 14.0 cm"));

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextDisabled(tr("关节目标角度 (deg)", "JOINT TARGETS (deg)"));
        const char* jointNamesZh[] = {"底座", "肩部", "小臂", "手部"};
        const char* jointNamesEn[] = {"Base", "Shoulder", "Elbow", "Wrist"};
        for (int i = 0; i < sim::kArmJointCount; ++i) {
            float target = simulator.armTargetDegrees(i);
            char id[32];
            std::snprintf(id, sizeof(id), "##joint_target_%d", i);
            if (sliderRow(tr(jointNamesZh[i], jointNamesEn[i]),
                          id,
                          target,
                          arm.jointMinDeg[static_cast<std::size_t>(i)],
                          arm.jointMaxDeg[static_cast<std::size_t>(i)],
                          "%+.1f",
                          54.0f)) {
                simulator.setArmTargetDegrees(i, target);
            }
        }
        ImGui::TextDisabled(tr("实际  %+.1f  %+.1f  %+.1f  %+.1f",
                               "Actual  %+.1f  %+.1f  %+.1f  %+.1f"),
                            simulator.armCurrentDegrees(0),
                            simulator.armCurrentDegrees(1),
                            simulator.armCurrentDegrees(2),
                            simulator.armCurrentDegrees(3));

        ImGui::Spacing();
        ImGui::TextDisabled(tr("电机响应 (deg/s, deg/s²)",
                               "SERVO RESPONSE (deg/s, deg/s^2)"));
        float maxSpeed = arm.servoMaxSpeedDeg;
        float maxAccel = arm.servoMaxAccelDeg;
        const bool speedChanged = sliderRow(tr("限速", "Speed"),
                                            "##servo_speed",
                                            maxSpeed,
                                            10.0f,
                                            720.0f,
                                            "%.0f",
                                            54.0f);
        const bool accelChanged = sliderRow(tr("加速度", "Accel"),
                                            "##servo_accel",
                                            maxAccel,
                                            20.0f,
                                            3600.0f,
                                            "%.0f",
                                            54.0f);
        if (speedChanged || accelChanged) {
            simulator.setServoLimits(maxSpeed, maxAccel);
        }

        if (ImGui::Button(tr("恢复初始姿态", "Reset arm pose"), ImVec2(-1.0f, 26.0f))) {
            simulator.resetArmPose();
        }
        ImGui::TextWrapped(tr("主循环每帧调用 main.cpp 的 userUpdateArm()，可在其中写电机目标。",
                              "userUpdateArm() in main.cpp is called every frame for API control."));
    } else if (activeTab_ == 1) {
        float height = simulator.cameraMountHeight();
        ImGui::TextDisabled(tr("相机安装高度（相对小车原点）", "CAMERA HEIGHT (ROBOT FRAME)"));
        if (sliderRow(tr("高度", "Height"), "##camera_height", height, 0.05f, 1.50f, "%.2f", 44.0f)) {
            simulator.setCameraMountHeight(height);
        }
        ImGui::TextDisabled(tr("单位: m", "unit: m"));

        const sim::SimConfig& config = simulator.config();
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextDisabled(tr("相机参数", "CAMERA RIG"));
        ImGui::Text(tr("垂直 FOV: %.0f°", "Vertical FOV: %.0f deg"), config.cameraFovY);
        ImGui::Text(tr("图像: %d x %d", "Image: %d x %d"),
                    config.cameraWidth,
                    config.cameraHeight);
        ImGui::Text(tr("偏置: X %.2f m / Y %.2f m",
                       "Offsets: X %.2f m / Y %.2f m"),
                    config.cameraOffsetX,
                    config.cameraOffsetY);

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextWrapped(tr("画面处理函数 userProcessCamera() 位于 main.cpp，返回值由框架自动接到显示。",
                              "userProcessCamera() in main.cpp returns the image shown by the "
                              "framework."));
        ImGui::TextWrapped(tr("CAM 4 是夹爪相机，安装高度由机械臂姿态自动决定。",
                              "CAM 4 is the gripper camera; its pose follows the arm automatically."));
        ImGui::TextWrapped(tr("左键点击任意相机画面可查询该像素的射线深度。",
                              "Left-click any camera pane to query that pixel's ray depth."));
    } else if (activeTab_ == 2) {
        int lightCount = simulator.lightCount();
        ImGui::TextUnformatted(tr("灯光数量", "LIGHT COUNT"));
        ImGui::SameLine();
        ImGui::TextColored(kAccent, "%d / %d", lightCount, sim::kMaxLights);

        const float rowRight = ImGui::GetWindowWidth() -
                               ImGui::GetStyle().WindowPadding.x - 64.0f;
        ImGui::SameLine(rowRight);
        if (ImGui::Button("-##remove_light", ImVec2(28.0f, 24.0f))) {
            simulator.setLightCount(lightCount - 1);
            lightCount = simulator.lightCount();
        }
        ImGui::SameLine(0.0f, 4.0f);
        if (ImGui::Button("+##add_light", ImVec2(28.0f, 24.0f))) {
            simulator.setLightCount(lightCount + 1);
            lightCount = simulator.lightCount();
        }

        if (ImGui::Button(tr("重置灯光位置", "Reset light rig"), ImVec2(-1.0f, 24.0f))) {
            simulator.resetLightRig();
            lightCount = simulator.lightCount();
        }

        ImGui::Spacing();
        ImGui::Separator();

        if (lightCount == 0) {
            ImGui::TextWrapped(tr("当前没有点光源，场景只使用环境光。点击“+”添加灯光。",
                                  "No point lights. Ambient light only. Use \"+\" to add."));
        } else {
            ImGui::TextDisabled(tr("俯视图金色圆环 = 灯光 XY 位置",
                                   "Gold rings in top view = light XY"));
        }

        const sim::SimConfig& config = simulator.config();
        const float xRange = std::max(7.0f, config.fieldHalfX * 1.35f);
        const float yRange = std::max(4.0f, config.fieldHalfY * 1.35f);

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 3.0f));
        for (int i = 0; i < lightCount; ++i) {
            const sim::LightSettings settings = simulator.lightSettings(i);
            ImGui::PushID(i);

            const ImVec4 lightColor(settings.color.x, settings.color.y, settings.color.z, 1.0f);
            char header[64];
            std::snprintf(header,
                          sizeof(header),
                          "%s %d###light_header",
                          tr("灯光", "LIGHT"),
                          i + 1);
            ImGui::PushStyleColor(ImGuiCol_Text, lightColor);
            const bool open = ImGui::CollapsingHeader(
                header,
                ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_SpanAvailWidth);
            ImGui::PopStyleColor();

            if (open) {
                float position[3] = {settings.position.x,
                                     settings.position.y,
                                     settings.position.z};
                bool changed = false;

                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6.0f, 2.0f));
                changed |= sliderRow("X", "##pos_x", position[0], -xRange, xRange, "%+.2f", 22.0f);
                changed |= sliderRow("Y", "##pos_y", position[1], -yRange, yRange, "%+.2f", 22.0f);
                changed |= sliderRow("Z", "##pos_z", position[2], 0.05f, 12.0f, "%.2f", 22.0f);
                if (changed) {
                    simulator.setLightPosition(i,
                                               {position[0], position[1], position[2]});
                }

                float intensity = settings.intensity;
                if (sliderRow(tr("强度", "Int"), "##intensity", intensity, 0.0f, 2.0f, "%.2f", 44.0f)) {
                    simulator.setLightIntensity(i, intensity);
                }
                ImGui::PopStyleVar();
            }
            ImGui::PopID();
        }
        ImGui::PopStyleVar();

        ImGui::Spacing();
        ImGui::TextWrapped(tr("拖动滑块移动灯光；灯光会影响俯视图和五路相机画面。",
                              "Drag sliders to move lights. Lighting affects the top view and "
                              "all five camera images."));
    } else {
        sim::UserSwitches switches = simulator.userSwitches();
        bool changed = false;
        ImGui::TextDisabled(tr("传给 main.cpp 用户函数的开关", "STATE PASSED TO USER HOOKS"));
        changed |= ImGui::Checkbox(tr("相机叠加层", "Camera overlay"),
                                   &switches.cameraOverlay);
        changed |= ImGui::Checkbox(tr("灰度处理", "Grayscale"), &switches.grayscale);
        changed |= ImGui::Checkbox(tr("反色处理", "Invert"), &switches.invert);
        changed |= ImGui::Checkbox(tr("机械臂演示", "Arm demo"), &switches.armDemo);
        if (changed) {
            simulator.setUserSwitches(switches);
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextWrapped(
            tr("这些状态每帧以 const sim::UserSwitches& 传给 main.cpp 的 "
               "userProcessCamera() 和 userUpdateArm()。",
               "These flags are passed every frame as const sim::UserSwitches& to "
               "userProcessCamera() and userUpdateArm() in main.cpp."));
        ImGui::TextWrapped(
            tr("相机索引 4 为夹爪相机；接口返回值由框架直接显示。",
               "Camera index 4 is the gripper camera; the framework displays the "
               "returned frame automatically."));
        const sim::UserSwitches& live = simulator.userSwitches();
        ImGui::TextDisabled(tr("当前: overlay=%d gray=%d invert=%d armDemo=%d",
                               "Live: overlay=%d gray=%d invert=%d armDemo=%d"),
                            live.cameraOverlay ? 1 : 0,
                            live.grayscale ? 1 : 0,
                            live.invert ? 1 : 0,
                            live.armDemo ? 1 : 0);
    }

    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
}
