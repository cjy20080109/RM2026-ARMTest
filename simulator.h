#pragma once

// =============================================================================
//  RM 3D camera simulator - public interface
//
//  Coordinate system (right handed, metres):
//      +X : field length / robot front at yaw = 0
//      +Y : robot left / field width
//      +Z : up
//      yaw: rotation around +Z, positive = counter-clockwise (Q)
//
//  The robot starts at the field origin. Four cameras are mounted at the local
//  positions (+ox,0), (-ox,0), (0,+oy), (0,-oy), all looking outwards, i.e.
//  along the vector from the robot origin to the camera position.
//
//  Camera images use a top-left origin, RGBA8. Camera depth buffers store the
//  raw OpenGL NDC depth (0..1, 1 = no geometry hit). The easy-to-use ray depth
//  API is getRayDepth()/getPixelWorld() below.
// =============================================================================

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace sim {

constexpr int kCarCameraCount = 4;
constexpr int kCameraCount = kCarCameraCount + 1;  // + gripper camera
constexpr int kMaxLights = 8;
constexpr int kArmJointCount = 4;
constexpr float kGripperMaxOpening = 0.14f;  // metres

// Layout helpers shared with the sidebar UI.
int headerHeightForWindow(int windowHeight);
int sidebarWidthForWindow(int windowWidth);

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

Vec3 operator+(const Vec3& a, const Vec3& b);
Vec3 operator-(const Vec3& a, const Vec3& b);
Vec3 operator*(const Vec3& v, float s);
float dot(const Vec3& a, const Vec3& b);
Vec3 cross(const Vec3& a, const Vec3& b);
float length(const Vec3& v);
Vec3 normalize(const Vec3& v);

struct CameraPose {
    Vec3 position;
    Vec3 forward;   // outward optical axis, unit length
    Vec3 right;     // image +x direction, unit length
    Vec3 up;        // image -y (screen up) direction, unit length
};

struct CameraIntrinsics {
    int width = 0;
    int height = 0;
    float fx = 0.0f;  // pixels
    float fy = 0.0f;  // pixels (same as fx)
    float cx = 0.0f;
    float cy = 0.0f;
};

struct Ray {
    Vec3 origin;
    Vec3 direction;  // normalized
};

struct CameraFrame {
    int width = 0;
    int height = 0;

    // Top-left origin, row-major, RGBA8. This is the buffer the image
    // processor may modify before it is displayed.
    std::vector<std::uint8_t> rgba;

    // Raw OpenGL depth buffer, top-left origin, row-major. 0 = near plane,
    // 1 = background. You normally want getRayDepth() instead of this.
    std::vector<float> depth;
};

struct CarState {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;       // wheel contact plane (car-local Z=0) above field
    float yaw = 0.0f;     // radians, positive CCW
    float pitch = 0.0f;   // radians, positive nose up
    float roll = 0.0f;    // radians, positive left side up
    float verticalVelocity = 0.0f;
    bool airborne = false;
};

// Input is expressed in the robot body frame so the mecanum wheels can strafe
// without changing the heading.
struct InputState {
    float forward = 0.0f;     // W = +1, S = -1 (robot +X)
    float strafeLeft = 0.0f;  // A = +1, D = -1 (robot +Y)
    float yawRate = 0.0f;     // Q = +1 (CCW), E = -1 (CW)
    bool boost = false;       // Shift
};

// Sidebar switches exported to the user hooks. They are read-only inside the
// callbacks; the sidebar (or Simulator::setUserSwitches) owns the state.
struct UserSwitches {
    bool cameraOverlay = false;  // draw a frame/crosshair in processed images
    bool grayscale = false;
    bool invert = false;
    bool armDemo = false;        // example flag for userUpdateArm
};

struct LightSettings {
    Vec3 position{-3.5f, -1.2f, 4.5f};
    Vec3 color{1.0f, 0.94f, 0.84f};
    float intensity = 0.62f;
};

// Joint order: 0 = base yaw, 1 = shoulder elevation, 2 = elbow, 3 = wrist.
struct ArmConfig {
    Vec3 basePosition{0.0f, 0.0f, 0.22f};  // car-local metres
    float upperArmLength = 0.32f;
    float lowerArmLength = 0.28f;
    float wristLength = 0.09f;
    float gripperOpening = 0.08f;  // metres, clamped to [0, kGripperMaxOpening]

    // Servo response limits. Angles below are in degrees for UI convenience.
    float servoMaxSpeedDeg = 180.0f;
    float servoMaxAccelDeg = 720.0f;

    // Supported target range per joint (degrees).
    std::array<float, kArmJointCount> jointMinDeg{-180.0f, -80.0f, -150.0f, -120.0f};
    std::array<float, kArmJointCount> jointMaxDeg{180.0f, 150.0f, 150.0f, 120.0f};

    // Initial target pose (arm raised, clear of the front camera).
    std::array<float, kArmJointCount> jointTargetDeg{0.0f, 60.0f, -30.0f, 20.0f};
};

struct SimConfig {
    // Field size inferred from the supplied 26校内赛场地.STEP (12 m x 5 m).
    float fieldHalfX = 6.0f;
    float fieldHalfY = 2.5f;
    float wallHeight = 0.5f;

    // Camera mounts, in the robot body frame.
    float cameraOffsetX = 0.34f;
    float cameraOffsetY = 0.34f;
    float cameraMountHeight = 0.32f;  // optical centre above the robot origin, metres

    // Car dimensions used by the terrain contact solver.
    float carLength = 0.58f;       // 58 cm outer bumper length
    float carWidth = 0.58f;        // 58 cm outer bumper width
    float carBodyHeight = 0.18f;   // clearance used for tunnels/overhangs
    float carWheelBase = 0.44f;    // front-to-rear axle distance
    float carTrackWidth = 0.52f;   // left-to-right wheel distance
    float carWheelRadius = 0.076f;
    float gravity = 9.81f;
    float maxClimbStep = 0.11f;    // vertical step that blocks the chassis
    float carBumperThickness = 0.035f;
    float carBumperBottom = 0.10f;  // above the 0.076 m wheel axle
    float carBumperHeight = 0.08f;  // 0.10 .. 0.18 m side collision boxes

    // The restored central platform occupies the exact field origin. Start on
    // the open centre line just in front of it; findSpawn() searches farther
    // out only if this patch is occupied.
    float spawnX = 0.0f;
    float spawnY = -1.30f;

    float cameraFovY = 70.0f;  // vertical FOV; horizontal is ~86 deg at 4:3
    int cameraWidth = 960;
    int cameraHeight = 720;
    float cameraNear = 0.03f;
    float cameraFar = 80.0f;

    float moveSpeed = 2.2f;       // m/s
    float boostMultiplier = 1.8f;
    float rotateSpeed = 2.4f;     // rad/s

    // A few collidable field props make the four camera feeds and the depth
    // query more interesting. Set to false for an empty field.
    bool spawnDemoProps = true;
};

struct DepthProbe {
    int cameraIndex = -1;
    float imageX = 0.0f;
    float imageY = 0.0f;
    float depth = std::numeric_limits<float>::infinity();
    Vec3 world;
    bool hit = false;
};

class Simulator {
public:
    explicit Simulator(const SimConfig& config = SimConfig{});
    ~Simulator();

    Simulator(const Simulator&) = delete;
    Simulator& operator=(const Simulator&) = delete;

    // Call once after a current OpenGL context exists.
    void initializeGraphics();

    void reset();
    void update(const InputState& input, float dt);

    // Renders camera i into the back buffer and reads back RGBA + depth.
    // Call for i = 0..3 from the main loop, then let your processor modify
    // cameraFrame(i), then call renderWindow().
    void captureCamera(int cameraIndex);

    const CameraFrame& cameraFrame(int cameraIndex) const;
    CameraFrame& cameraFrame(int cameraIndex);

    // Framework plumbing for the "img -> user function -> show" pipeline.
    // After user code returns an image, point the display at it; the next
    // renderWindow() will show that frame. The frame must stay alive until
    // renderWindow() returns. Pass nullptr to use the internal capture buffer.
    void setCameraDisplayFrame(int cameraIndex, const CameraFrame* frame);

    // Adjustable camera rig.
    float cameraMountHeight() const;
    void setCameraMountHeight(float height);

    // ---------------------------------------------------------------------
    //  Ray / depth interface
    // ---------------------------------------------------------------------

    // Continuous image coordinates with (0,0) at the top-left corner of the
    // camera image. The returned distance is the Euclidean distance along the
    // normalized ray from the camera optical centre to the first scene hit.
    // Misses / sky return +infinity.
    float getRayDepth(int cameraIndex, float imageX, float imageY) const;

    // Same query, but returns false on an out-of-range pixel or sky hit.
    bool getRayDepth(int cameraIndex,
                     float imageX,
                     float imageY,
                     float& outDepth) const;

    // Camera ray for a pixel of the last captured frame. Useful for
    // projecting detections into 3D. Uses capturedCameraPose(), matching
    // getRayDepth()/getPixelWorld().
    Ray getCameraRay(int cameraIndex, float imageX, float imageY) const;

    // Pixel ray + depth -> world-space hit point. Returns false on a miss.
    bool getPixelWorld(int cameraIndex,
                       float imageX,
                       float imageY,
                       Vec3& outWorld) const;

    // Convenience for the currently displayed probe (or invalid if none).
    const DepthProbe& depthProbe() const;

    // Click inside one of the four camera panes to probe that pixel. The
    // result is also drawn in the window and returned for console logging.
    DepthProbe handleMouseClick(int windowWidth,
                                int windowHeight,
                                int mouseX,
                                int mouseY);

    // ---------------------------------------------------------------------
    //  State / display
    // ---------------------------------------------------------------------
    const CarState& car() const;
    const CameraPose& cameraPose(int cameraIndex) const;

    // Terrain queries and scripted placement. setCarPose() snaps the chassis
    // to the terrain at the requested XY and resets vertical motion.
    bool groundHeightAt(float x, float y, float& outHeight) const;
    void setCarPose(float x, float y, float yaw = 0.0f);
    void dropCarFromHeight(float heightAboveGround);
    bool carModelLoaded() const;

    const UserSwitches& userSwitches() const;
    void setUserSwitches(const UserSwitches& switches);

    // ---------------------------------------------------------------------
    //  Editable light rig (used by the sidebar)
    // ---------------------------------------------------------------------
    int lightCount() const;
    void setLightCount(int count);  // clamped to [0, kMaxLights]

    const LightSettings& lightSettings(int index) const;
    void setLightSettings(int index, const LightSettings& settings);
    void setLightPosition(int index, const Vec3& position);
    void setLightIntensity(int index, float intensity);

    // Restores the default multi-light rig.
    void resetLightRig();

    // ---------------------------------------------------------------------
    //  4-DOF arm / servo interface
    // ---------------------------------------------------------------------
    const ArmConfig& armConfig() const;

    // Base position is in the car body frame. Lengths/opening are metres.
    void setArmBasePosition(const Vec3& basePosition);
    void setArmLinkLengths(float upperArmLength, float lowerArmLength);
    void setGripperOpening(float openingMeters);
    void setServoLimits(float maxSpeedDeg, float maxAccelDeg);

    // Sets a motor target. The actual joint follows with acceleration and
    // speed limits; it does not snap instantly.
    void setArmTargetDegrees(int joint, float degrees);
    void setArmTargetRadians(int joint, float radians);
    void setArmTargetsDegrees(const std::array<float, kArmJointCount>& degrees);
    void setArmTargetsRadians(const std::array<float, kArmJointCount>& radians);

    float armTargetDegrees(int joint) const;
    float armCurrentDegrees(int joint) const;
    float armVelocityDegrees(int joint) const;

    // Advances the servo simulation. Call once per frame after user arm code.
    void updateArm(float dt);

    // Restores the default four-joint pose and stops all joints.
    void resetArmPose();

    // Pose used for the most recent captureCamera(cameraIndex) call. The ray
    // depth / pixel-to-world queries use this pose, so they stay consistent
    // with the stored frame even after the robot moves.
    const CameraPose& capturedCameraPose(int cameraIndex) const;

    const CameraIntrinsics& cameraIntrinsics(int cameraIndex) const;
    const SimConfig& config() const;

    void setFps(float fps);

    // Draws the whole five-view window. Requires a current OpenGL context.
    void renderWindow(int windowWidth, int windowHeight);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Small user-facing arm handle. Keep it independent from the rendering and
// servo internals: set targets, read feedback, and let the framework animate.
class ArmControl {
public:
    explicit ArmControl(Simulator& simulator) : simulator_(&simulator) {}

    void setTargetDegrees(int joint, float degrees) {
        simulator_->setArmTargetDegrees(joint, degrees);
    }
    void setTargetRadians(int joint, float radians) {
        simulator_->setArmTargetRadians(joint, radians);
    }
    void setTargetsDegrees(float baseYawDeg,
                           float shoulderDeg,
                           float elbowDeg,
                           float wristDeg) {
        simulator_->setArmTargetsDegrees({baseYawDeg, shoulderDeg, elbowDeg, wristDeg});
    }
    void setTargetsRadians(float baseYawRad,
                           float shoulderRad,
                           float elbowRad,
                           float wristRad) {
        simulator_->setArmTargetsRadians({baseYawRad, shoulderRad, elbowRad, wristRad});
    }
    void setGripperOpening(float openingMeters) {
        simulator_->setGripperOpening(openingMeters);
    }

    float targetDegrees(int joint) const {
        return simulator_->armTargetDegrees(joint);
    }
    float currentDegrees(int joint) const {
        return simulator_->armCurrentDegrees(joint);
    }
    const ArmConfig& config() const {
        return simulator_->armConfig();
    }

private:
    Simulator* simulator_;
};

}  // namespace sim
