#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <GL/gl.h>

#include "simulator.h"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace sim {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;

float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

int clampi(int v, int lo, int hi) {
    return std::max(lo, std::min(hi, v));
}

float radians(float degrees) {
    return degrees * (kPi / 180.0f);
}

Vec3 rotateZ(const Vec3& v, float angle) {
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    return {v.x * c - v.y * s, v.x * s + v.y * c, v.z};
}

// Car-local -> world: yaw around +Z, pitch around the lateral axis (positive
// nose up), roll around the longitudinal axis (positive left side up).
Vec3 rotateCarLocal(const Vec3& v, float yaw, float pitch, float roll) {
    const float cr = std::cos(roll);
    const float sr = std::sin(roll);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);
    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);

    const float x1 = v.x;
    const float y1 = v.y * cr - v.z * sr;
    const float z1 = v.y * sr + v.z * cr;

    const float x2 = x1 * cp - z1 * sp;
    const float y2 = y1;
    const float z2 = x1 * sp + z1 * cp;

    return {x2 * cy - y2 * sy, x2 * sy + y2 * cy, z2};
}

struct Color {
    float r = 1.0f;
    float g = 1.0f;
    float b = 1.0f;
    float a = 1.0f;
};

struct Rect {
    int x = 0;
    int y = 0;  // top-left coordinate
    int w = 0;
    int h = 0;

    bool contains(int px, int py) const {
        return px >= x && py >= y && px < x + w && py < y + h;
    }
};

Rect fitAspect(float aspect, int outerW, int outerH) {
    if (outerW <= 0 || outerH <= 0 || aspect <= 0.0f) {
        return {};
    }
    int w = outerW;
    int h = static_cast<int>(std::lround(static_cast<float>(w) / aspect));
    if (h > outerH) {
        h = outerH;
        w = static_cast<int>(std::lround(static_cast<float>(h) * aspect));
    }
    return {0, 0, std::max(1, w), std::max(1, h)};
}

struct Layout {
    int windowWidth = 0;
    int windowHeight = 0;
    int headerHeight = 60;
    int sidebarWidth = 0;
    Rect topPane;
    Rect robotInset;
    Rect sidebar;
    std::array<Rect, kCameraCount> cameraPanes{};
    std::array<Rect, kCameraCount> cameraImages{};
};

Layout makeLayout(int width, int height, float cameraAspect, float fieldAspect) {
    Layout l;
    l.windowWidth = width;
    l.windowHeight = height;
    l.headerHeight = headerHeightForWindow(height);
    l.sidebarWidth = sidebarWidthForWindow(width);

    const int margin = 8;
    const int gap = 8;
    const int contentH = std::max(1, height - l.headerHeight);
    const int contentW = std::max(1, width - l.sidebarWidth);

    // Keep the top pane close to the field's aspect ratio. A side control bar
    // eats into the content width, so the camera row gets a fixed practical
    // minimum instead of always taking the same share as before.
    float topHf = static_cast<float>(contentW - 2 * margin) /
                  std::max(2.25f, fieldAspect + 0.10f);
    const int maxTopH = contentH - 3 * gap - 230;
    topHf = std::min(topHf, static_cast<float>(std::max(120, maxTopH)));
    topHf = std::max(topHf, 120.0f);
    int topH = static_cast<int>(std::lround(topHf));
    topH = std::min(topH, std::max(1, contentH - 3 * gap - 80));

    l.topPane = {margin, l.headerHeight + gap, contentW - 2 * margin, topH};
    l.sidebar = {width - l.sidebarWidth,
                 l.headerHeight,
                 l.sidebarWidth,
                 height - l.headerHeight};

    int insetW = clampi(static_cast<int>(static_cast<float>(l.topPane.w) * 0.24f), 200, 360);
    int insetH = static_cast<int>(static_cast<float>(insetW) * 0.76f);
    insetW = std::min(insetW, std::max(120, l.topPane.w - 24));
    insetH = std::min(insetH, std::max(90, l.topPane.h - 24));
    l.robotInset = {l.topPane.x + l.topPane.w - insetW - 12,
                    l.topPane.y + l.topPane.h - insetH - 12,
                    insetW,
                    insetH};

    const int camY = l.topPane.y + l.topPane.h + gap;
    const int camH = std::max(50, height - camY - margin);
    const int cellW = std::max(40,
                               (contentW - 2 * margin - (kCameraCount - 1) * gap) /
                                   kCameraCount);

    for (int i = 0; i < kCameraCount; ++i) {
        Rect pane{margin + i * (cellW + gap), camY, cellW, camH};
        l.cameraPanes[i] = pane;

        Rect fitted = fitAspect(cameraAspect, pane.w - 12, pane.h - 12);
        fitted.x = pane.x + (pane.w - fitted.w) / 2;
        fitted.y = pane.y + (pane.h - fitted.h) / 2;
        l.cameraImages[i] = fitted;
    }

    return l;
}

struct Mat4 {
    float m[16]{};  // column-major, directly usable by glLoadMatrixf
};

Mat4 perspectiveMatrix(float fovY, float aspect, float zNear, float zFar) {
    Mat4 mat{};
    const float f = 1.0f / std::tan(fovY * 0.5f);
    mat.m[0] = f / aspect;
    mat.m[5] = f;
    mat.m[10] = (zFar + zNear) / (zNear - zFar);
    mat.m[11] = -1.0f;
    mat.m[14] = (2.0f * zFar * zNear) / (zNear - zFar);
    return mat;
}

Mat4 lookAtMatrix(const Vec3& eye, const Vec3& center, const Vec3& up) {
    const Vec3 f = normalize(center - eye);
    const Vec3 s = normalize(cross(f, up));
    const Vec3 u = cross(s, f);

    Mat4 mat{};
    mat.m[0] = s.x;
    mat.m[1] = u.x;
    mat.m[2] = -f.x;
    mat.m[3] = 0.0f;
    mat.m[4] = s.y;
    mat.m[5] = u.y;
    mat.m[6] = -f.y;
    mat.m[7] = 0.0f;
    mat.m[8] = s.z;
    mat.m[9] = u.z;
    mat.m[10] = -f.z;
    mat.m[11] = 0.0f;
    mat.m[12] = -dot(s, eye);
    mat.m[13] = -dot(u, eye);
    mat.m[14] = dot(f, eye);
    mat.m[15] = 1.0f;
    return mat;
}

Mat4 carRotationMatrix(const CarState& car) {
    const Vec3 x = rotateCarLocal({1.0f, 0.0f, 0.0f}, car.yaw, car.pitch, car.roll);
    const Vec3 y = rotateCarLocal({0.0f, 1.0f, 0.0f}, car.yaw, car.pitch, car.roll);
    const Vec3 z = rotateCarLocal({0.0f, 0.0f, 1.0f}, car.yaw, car.pitch, car.roll);
    Mat4 mat{};
    mat.m[0] = x.x;
    mat.m[1] = x.y;
    mat.m[2] = x.z;
    mat.m[4] = y.x;
    mat.m[5] = y.y;
    mat.m[6] = y.z;
    mat.m[8] = z.x;
    mat.m[9] = z.y;
    mat.m[10] = z.z;
    mat.m[15] = 1.0f;
    return mat;
}

void setColor(const Color& c, float alphaScale = 1.0f) {
    glColor4f(c.r, c.g, c.b, c.a * alphaScale);
}

// -----------------------------------------------------------------------------
//  Immediate-mode geometry helpers
// -----------------------------------------------------------------------------

void drawBox(const Vec3& center, const Vec3& size, const Color& color) {
    const float hx = size.x * 0.5f;
    const float hy = size.y * 0.5f;
    const float hz = size.z * 0.5f;
    const float x0 = center.x - hx;
    const float x1 = center.x + hx;
    const float y0 = center.y - hy;
    const float y1 = center.y + hy;
    const float z0 = center.z - hz;
    const float z1 = center.z + hz;

    setColor(color);
    glBegin(GL_QUADS);

    glNormal3f(1.0f, 0.0f, 0.0f);
    glVertex3f(x1, y0, z0);
    glVertex3f(x1, y1, z0);
    glVertex3f(x1, y1, z1);
    glVertex3f(x1, y0, z1);

    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(x0, y1, z0);
    glVertex3f(x0, y0, z0);
    glVertex3f(x0, y0, z1);
    glVertex3f(x0, y1, z1);

    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(x1, y1, z0);
    glVertex3f(x0, y1, z0);
    glVertex3f(x0, y1, z1);
    glVertex3f(x1, y1, z1);

    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertex3f(x0, y0, z0);
    glVertex3f(x1, y0, z0);
    glVertex3f(x1, y0, z1);
    glVertex3f(x0, y0, z1);

    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(x0, y0, z1);
    glVertex3f(x1, y0, z1);
    glVertex3f(x1, y1, z1);
    glVertex3f(x0, y1, z1);

    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(x0, y1, z0);
    glVertex3f(x1, y1, z0);
    glVertex3f(x1, y0, z0);
    glVertex3f(x0, y0, z0);

    glEnd();
}

void drawCylinderBetween(const Vec3& a,
                         const Vec3& b,
                         float radius,
                         const Color& color,
                         int segments = 16) {
    Vec3 axis = b - a;
    const float len = length(axis);
    if (len <= 1.0e-7f) {
        return;
    }
    axis = axis * (1.0f / len);
    Vec3 reference = std::fabs(axis.z) < 0.9f ? Vec3{0.0f, 0.0f, 1.0f}
                                               : Vec3{1.0f, 0.0f, 0.0f};
    const Vec3 u = normalize(cross(axis, reference));
    const Vec3 v = cross(axis, u);

    setColor(color);
    glBegin(GL_QUADS);
    for (int i = 0; i < segments; ++i) {
        const float a0 = kTwoPi * static_cast<float>(i) / static_cast<float>(segments);
        const float a1 = kTwoPi * static_cast<float>(i + 1) / static_cast<float>(segments);
        const Vec3 n0 = u * std::cos(a0) + v * std::sin(a0);
        const Vec3 n1 = u * std::cos(a1) + v * std::sin(a1);
        glNormal3f(n0.x, n0.y, n0.z);
        glVertex3f(a.x + n0.x * radius, a.y + n0.y * radius, a.z + n0.z * radius);
        glVertex3f(b.x + n0.x * radius, b.y + n0.y * radius, b.z + n0.z * radius);
        glNormal3f(n1.x, n1.y, n1.z);
        glVertex3f(b.x + n1.x * radius, b.y + n1.y * radius, b.z + n1.z * radius);
        glVertex3f(a.x + n1.x * radius, a.y + n1.y * radius, a.z + n1.z * radius);
    }
    glEnd();

    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(-axis.x, -axis.y, -axis.z);
    glVertex3f(a.x, a.y, a.z);
    for (int i = 0; i <= segments; ++i) {
        const float angle = kTwoPi * static_cast<float>(i) / static_cast<float>(segments);
        const Vec3 n = u * std::cos(angle) + v * std::sin(angle);
        glVertex3f(a.x + n.x * radius, a.y + n.y * radius, a.z + n.z * radius);
    }
    glEnd();

    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(axis.x, axis.y, axis.z);
    glVertex3f(b.x, b.y, b.z);
    for (int i = segments; i >= 0; --i) {
        const float angle = kTwoPi * static_cast<float>(i) / static_cast<float>(segments);
        const Vec3 n = u * std::cos(angle) + v * std::sin(angle);
        glVertex3f(b.x + n.x * radius, b.y + n.y * radius, b.z + n.z * radius);
    }
    glEnd();
}

void drawOrientedBox(const Vec3& center,
                     const Vec3& axisX,
                     const Vec3& axisY,
                     const Vec3& axisZ,
                     const Vec3& size,
                     const Color& color) {
    const GLfloat matrix[16] = {
        axisX.x, axisX.y, axisX.z, 0.0f,
        axisY.x, axisY.y, axisY.z, 0.0f,
        axisZ.x, axisZ.y, axisZ.z, 0.0f,
        center.x, center.y, center.z, 1.0f,
    };
    glPushMatrix();
    glMultMatrixf(matrix);
    drawBox({0.0f, 0.0f, 0.0f}, size, color);
    glPopMatrix();
}

void drawFloorTile(float x0, float y0, float x1, float y1, const Color& color, float z = 0.0f) {
    setColor(color);
    glBegin(GL_QUADS);
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(x0, y0, z);
    glVertex3f(x1, y0, z);
    glVertex3f(x1, y1, z);
    glVertex3f(x0, y1, z);
    glEnd();
}

void drawTopDisk(const Vec3& center, float radius, const Color& color, int segments = 32) {
    setColor(color);
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(center.x, center.y, center.z);
    for (int i = 0; i <= segments; ++i) {
        const float a = kTwoPi * static_cast<float>(i) / static_cast<float>(segments);
        glVertex3f(center.x + std::cos(a) * radius,
                   center.y + std::sin(a) * radius,
                   center.z);
    }
    glEnd();
}

// -----------------------------------------------------------------------------
//  Tiny OpenGL bitmap-text renderer (Consolas, ASCII)
// -----------------------------------------------------------------------------

struct GlFont {
    HFONT handle = nullptr;
    GLuint listBase = 0;
    int pixelHeight = 0;
    int cellWidth = 0;
    int ascent = 0;
};

class TextRenderer {
public:
    ~TextRenderer() {
        shutdown();
    }

    bool initialize(HDC dc) {
        shutdown();
        if (dc == nullptr) {
            return false;
        }
        hdc_ = dc;
        ok_ = createFont(dc, 12, fontSmall_) &&
              createFont(dc, 15, fontBody_) &&
              createFont(dc, 21, fontTitle_);
        return ok_;
    }

    void shutdown() {
        if (fontSmall_.listBase != 0) {
            glDeleteLists(fontSmall_.listBase, 128);
            fontSmall_.listBase = 0;
        }
        if (fontBody_.listBase != 0) {
            glDeleteLists(fontBody_.listBase, 128);
            fontBody_.listBase = 0;
        }
        if (fontTitle_.listBase != 0) {
            glDeleteLists(fontTitle_.listBase, 128);
            fontTitle_.listBase = 0;
        }
        for (GlFont* f : {&fontSmall_, &fontBody_, &fontTitle_}) {
            if (f->handle != nullptr) {
                DeleteObject(f->handle);
                f->handle = nullptr;
            }
        }
        ok_ = false;
    }

    bool ready() const {
        return ok_;
    }

    const GlFont& small() const {
        return fontSmall_;
    }
    const GlFont& body() const {
        return fontBody_;
    }
    const GlFont& title() const {
        return fontTitle_;
    }

    int textWidth(const GlFont& font, const std::string& text) const {
        return static_cast<int>(text.size()) * font.cellWidth;
    }

    void draw(const GlFont& font,
              const std::string& text,
              int x,
              int baselineFromTop,
              int windowHeight,
              const Color& color,
              bool shadow = false) const {
        if (!ok_ || font.listBase == 0 || text.empty()) {
            return;
        }

        const int glY = windowHeight - baselineFromTop;
        if (x < 0 || x >= 100000 || glY < 0 || glY > windowHeight) {
            return;
        }

        glDisable(GL_TEXTURE_2D);
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);

        if (shadow) {
            glColor4f(0.0f, 0.0f, 0.0f, 0.78f);
            glRasterPos2i(x + 1, glY - 1);
            glListBase(font.listBase);
            glCallLists(static_cast<GLsizei>(text.size()), GL_UNSIGNED_BYTE, text.data());
        }

        glColor4f(color.r, color.g, color.b, color.a);
        glRasterPos2i(x, glY);
        glListBase(font.listBase);
        glCallLists(static_cast<GLsizei>(text.size()), GL_UNSIGNED_BYTE, text.data());
    }

    void drawRight(const GlFont& font,
                   const std::string& text,
                   int rightX,
                   int baselineFromTop,
                   int windowHeight,
                   const Color& color,
                   bool shadow = false) const {
        draw(font,
             text,
             rightX - textWidth(font, text),
             baselineFromTop,
             windowHeight,
             color,
             shadow);
    }

private:
    bool createFont(HDC dc, int pixelHeight, GlFont& out) {
        out.listBase = glGenLists(128);
        if (out.listBase == 0) {
            return false;
        }

        out.pixelHeight = pixelHeight;
        out.handle = CreateFontA(-pixelHeight,
                                 0,
                                 0,
                                 0,
                                 FW_NORMAL,
                                 FALSE,
                                 FALSE,
                                 FALSE,
                                 DEFAULT_CHARSET,
                                 OUT_DEFAULT_PRECIS,
                                 CLIP_DEFAULT_PRECIS,
                                 ANTIALIASED_QUALITY,
                                 FIXED_PITCH | FF_MODERN,
                                 "Consolas");
        if (out.handle == nullptr) {
            return false;
        }

        HGDIOBJ oldFont = SelectObject(dc, out.handle);
        const BOOL ok = wglUseFontBitmapsA(dc, 0, 128, out.listBase);
        TEXTMETRICA tm{};
        GetTextMetricsA(dc, &tm);
        SIZE ext{};
        GetTextExtentPoint32A(dc, "M", 1, &ext);
        SelectObject(dc, oldFont);

        out.cellWidth = std::max(1, static_cast<int>(ext.cx));
        out.ascent = static_cast<int>(tm.tmAscent);
        if (out.ascent <= 0) {
            out.ascent = std::max(1, pixelHeight - 3);
        }
        return ok != FALSE;
    }

    HDC hdc_ = nullptr;
    bool ok_ = false;
    GlFont fontSmall_;
    GlFont fontBody_;
    GlFont fontTitle_;
};

const Color kUiText{0.90f, 0.94f, 0.98f, 1.0f};
const Color kUiMuted{0.64f, 0.72f, 0.80f, 1.0f};
const Color kUiAccent{0.20f, 0.85f, 0.95f, 1.0f};
const Color kPanelBg{0.035f, 0.045f, 0.060f, 1.0f};
const Color kHeaderBg{0.022f, 0.030f, 0.044f, 1.0f};

struct Obstacle {
    Vec3 center;
    Vec3 halfSize;
    Color color;
    bool collide = true;
};

struct CameraMount {
    Vec3 localOffset;
    const char* name;
    Color color;
};

// -----------------------------------------------------------------------------
//  Baked field mesh generated from 26校内赛场地.STEP with OpenCASCADE.
//  File format: see tools/bake_field.py
// -----------------------------------------------------------------------------

#pragma pack(push, 1)
struct FieldVertex {
    float px, py, pz;
    float nx, ny, nz;
    std::uint8_t r, g, b, a;
};
#pragma pack(pop)

static_assert(sizeof(FieldVertex) == 28, "unexpected FieldVertex packing");

bool findAssetNamed(const char* assetName, std::string& outPath) {
    std::array<std::string, 5> candidates;
    char exePath[MAX_PATH]{};
    DWORD length = GetModuleFileNameA(nullptr, exePath, MAX_PATH);
    std::string exeDir;
    if (length > 0 && length < MAX_PATH) {
        exeDir = exePath;
        const std::size_t slash = exeDir.find_last_of("\\/");
        if (slash != std::string::npos) {
            exeDir.resize(slash);
        }
        candidates[2] = exeDir + "\\assets\\" + assetName;
        candidates[3] = exeDir + "\\..\\assets\\" + assetName;
    }
    candidates[0] = std::string("assets\\") + assetName;
    candidates[1] = std::string("..\\assets\\") + assetName;
    candidates[4] = std::string("..\\..\\assets\\") + assetName;

    for (const std::string& candidate : candidates) {
        std::ifstream test(candidate, std::ios::binary);
        if (test.good()) {
            outPath = candidate;
            return true;
        }
    }
    return false;
}

struct FieldMesh {
    std::vector<FieldVertex> vertices;
    std::vector<std::uint16_t> indices;
    Vec3 minBound;
    Vec3 maxBound;
    bool loaded = false;

    bool load() {
        std::string path;
        if (!findAssetNamed("field.rfield", path)) {
            return false;
        }
        std::ifstream in(path, std::ios::binary);
        if (!in) {
            return false;
        }

        char magic[8]{};
        std::uint32_t version = 0;
        std::uint32_t vertexCount = 0;
        std::uint32_t indexCount = 0;
        float bounds[6]{};
        std::uint32_t reserved = 0;
        in.read(magic, sizeof(magic));
        in.read(reinterpret_cast<char*>(&version), sizeof(version));
        in.read(reinterpret_cast<char*>(&vertexCount), sizeof(vertexCount));
        in.read(reinterpret_cast<char*>(&indexCount), sizeof(indexCount));
        in.read(reinterpret_cast<char*>(bounds), sizeof(bounds));
        in.read(reinterpret_cast<char*>(&reserved), sizeof(reserved));
        if (!in || std::memcmp(magic, "RMFIELD1", 8) != 0 || version != 1) {
            return false;
        }
        if (vertexCount == 0 || vertexCount > 65535 ||
            indexCount == 0 || indexCount > 4000000 ||
            indexCount % 3 != 0) {
            return false;
        }

        vertices.resize(vertexCount);
        std::vector<std::uint32_t> indices32(indexCount);
        in.read(reinterpret_cast<char*>(vertices.data()),
                static_cast<std::streamsize>(vertices.size() * sizeof(FieldVertex)));
        in.read(reinterpret_cast<char*>(indices32.data()),
                static_cast<std::streamsize>(indices32.size() * sizeof(std::uint32_t)));
        if (!in) {
            vertices.clear();
            return false;
        }
        indices.resize(indices32.size());
        for (std::size_t i = 0; i < indices32.size(); ++i) {
            if (indices32[i] >= vertexCount) {
                vertices.clear();
                indices.clear();
                return false;
            }
            indices[i] = static_cast<std::uint16_t>(indices32[i]);
        }

        minBound = {bounds[0], bounds[1], bounds[2]};
        maxBound = {bounds[3], bounds[4], bounds[5]};
        loaded = true;
        return true;
    }

    void draw() const {
        if (!loaded || vertices.empty() || indices.empty()) {
            return;
        }
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        glVertexPointer(3,
                        GL_FLOAT,
                        static_cast<GLsizei>(sizeof(FieldVertex)),
                        &vertices[0].px);
        glNormalPointer(GL_FLOAT,
                        static_cast<GLsizei>(sizeof(FieldVertex)),
                        &vertices[0].nx);
        glColorPointer(4,
                       GL_UNSIGNED_BYTE,
                       static_cast<GLsizei>(sizeof(FieldVertex)),
                       &vertices[0].r);
        glDrawElements(GL_TRIANGLES,
                       static_cast<GLsizei>(indices.size()),
                       GL_UNSIGNED_SHORT,
                       indices.data());
        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
    }
};

// -----------------------------------------------------------------------------
//  Baked car mesh generated from the local SLDPRT assembly by tools/bake_car.py
// -----------------------------------------------------------------------------

struct CarMesh {
    std::vector<std::vector<FieldVertex>> batches;
    std::vector<std::vector<std::uint16_t>> batchIndices;
    Vec3 minBound;
    Vec3 maxBound;
    bool loaded = false;

    bool load() {
        std::string path;
        if (!findAssetNamed("car.rmesh", path)) {
            return false;
        }
        std::ifstream in(path, std::ios::binary);
        if (!in) {
            return false;
        }

        char magic[8]{};
        std::uint32_t version = 0;
        std::uint32_t batchCount = 0;
        float bounds[6]{};
        std::uint32_t reserved = 0;
        in.read(magic, sizeof(magic));
        in.read(reinterpret_cast<char*>(&version), sizeof(version));
        in.read(reinterpret_cast<char*>(&batchCount), sizeof(batchCount));
        in.read(reinterpret_cast<char*>(bounds), sizeof(bounds));
        in.read(reinterpret_cast<char*>(&reserved), sizeof(reserved));
        if (!in || std::memcmp(magic, "RMCAR001", 8) != 0 || version != 1 ||
            batchCount == 0 || batchCount > 64) {
            return false;
        }

        batches.resize(batchCount);
        batchIndices.resize(batchCount);
        for (std::uint32_t b = 0; b < batchCount; ++b) {
            std::uint32_t vertexCount = 0;
            std::uint32_t indexCount = 0;
            in.read(reinterpret_cast<char*>(&vertexCount), sizeof(vertexCount));
            in.read(reinterpret_cast<char*>(&indexCount), sizeof(indexCount));
            if (!in || vertexCount == 0 || vertexCount > 65535 ||
                indexCount == 0 || indexCount % 3 != 0 ||
                indexCount > 2000000) {
                return false;
            }
            std::vector<FieldVertex>& vertices = batches[b];
            std::vector<std::uint16_t>& indices = batchIndices[b];
            vertices.resize(vertexCount);
            indices.resize(indexCount);
            in.read(reinterpret_cast<char*>(vertices.data()),
                    static_cast<std::streamsize>(vertices.size() * sizeof(FieldVertex)));
            in.read(reinterpret_cast<char*>(indices.data()),
                    static_cast<std::streamsize>(indices.size() * sizeof(std::uint16_t)));
            if (!in) {
                return false;
            }
            for (std::uint16_t index : indices) {
                if (index >= vertexCount) {
                    return false;
                }
            }
        }

        minBound = {bounds[0], bounds[1], bounds[2]};
        maxBound = {bounds[3], bounds[4], bounds[5]};
        loaded = true;
        return true;
    }

    void draw() const {
        if (!loaded) {
            return;
        }
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        for (std::size_t b = 0; b < batches.size(); ++b) {
            const std::vector<FieldVertex>& vertices = batches[b];
            const std::vector<std::uint16_t>& indices = batchIndices[b];
            if (vertices.empty() || indices.empty()) {
                continue;
            }
            glVertexPointer(3,
                            GL_FLOAT,
                            static_cast<GLsizei>(sizeof(FieldVertex)),
                            &vertices[0].px);
            glNormalPointer(GL_FLOAT,
                            static_cast<GLsizei>(sizeof(FieldVertex)),
                            &vertices[0].nx);
            glColorPointer(4,
                           GL_UNSIGNED_BYTE,
                           static_cast<GLsizei>(sizeof(FieldVertex)),
                           &vertices[0].r);
            glDrawElements(GL_TRIANGLES,
                           static_cast<GLsizei>(indices.size()),
                           GL_UNSIGNED_SHORT,
                           indices.data());
        }
        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
    }
};

// -----------------------------------------------------------------------------
//  Height-field captured from the loaded field mesh. Used for wheel support,
//  slope alignment, gravity and chassis collision.
// -----------------------------------------------------------------------------

struct TerrainField {
    float minX = 0.0f;
    float minY = 0.0f;
    float cell = 0.02f;
    int nx = 0;
    int ny = 0;
    std::vector<float> heights;
    bool ready = false;

    bool build(const FieldMesh& mesh) {
        if (!mesh.loaded || mesh.vertices.empty() || mesh.indices.empty()) {
            return false;
        }
        minX = mesh.minBound.x;
        minY = mesh.minBound.y;
        const float maxX = mesh.maxBound.x;
        const float maxY = mesh.maxBound.y;
        cell = 0.02f;
        nx = static_cast<int>(std::ceil((maxX - minX) / cell)) + 1;
        ny = static_cast<int>(std::ceil((maxY - minY) / cell)) + 1;
        if (nx <= 1 || ny <= 1 || nx > 2000 || ny > 2000) {
            return false;
        }
        heights.assign(static_cast<std::size_t>(nx) * ny, -1000.0f);

        for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            const FieldVertex& va = mesh.vertices[mesh.indices[t + 0]];
            const FieldVertex& vb = mesh.vertices[mesh.indices[t + 1]];
            const FieldVertex& vc = mesh.vertices[mesh.indices[t + 2]];
            const Vec3 a{va.px, va.py, va.pz};
            const Vec3 b{vb.px, vb.py, vb.pz};
            const Vec3 c{vc.px, vc.py, vc.pz};
            const Vec3 n = cross(b - a, c - a);
            const float nlen = length(n);
            if (nlen < 1.0e-8f || std::fabs(n.z / nlen) < 0.18f) {
                continue;
            }

            const float x0 = std::min(a.x, std::min(b.x, c.x));
            const float x1 = std::max(a.x, std::max(b.x, c.x));
            const float y0 = std::min(a.y, std::min(b.y, c.y));
            const float y1 = std::max(a.y, std::max(b.y, c.y));
            const int ix0 = clampi(static_cast<int>(std::floor((x0 - minX) / cell)), 0, nx - 1);
            const int ix1 = clampi(static_cast<int>(std::ceil((x1 - minX) / cell)), 0, nx - 1);
            const int iy0 = clampi(static_cast<int>(std::floor((y0 - minY) / cell)), 0, ny - 1);
            const int iy1 = clampi(static_cast<int>(std::ceil((y1 - minY) / cell)), 0, ny - 1);

            const float v0x = b.x - a.x;
            const float v0y = b.y - a.y;
            const float v1x = c.x - a.x;
            const float v1y = c.y - a.y;
            const float den = v0x * v1y - v0y * v1x;
            if (std::fabs(den) < 1.0e-10f) {
                continue;
            }
            for (int iy = iy0; iy <= iy1; ++iy) {
                const float py = minY + (static_cast<float>(iy) + 0.5f) * cell;
                for (int ix = ix0; ix <= ix1; ++ix) {
                    const float px = minX + (static_cast<float>(ix) + 0.5f) * cell;
                    const float rx = px - a.x;
                    const float ry = py - a.y;
                    const float u = (rx * v1y - ry * v1x) / den;
                    const float v = (v0x * ry - v0y * rx) / den;
                    if (u < -1.0e-4f || v < -1.0e-4f || u + v > 1.0001f) {
                        continue;
                    }
                    const float z = a.z + u * (b.z - a.z) + v * (c.z - a.z);
                    float& dst = heights[static_cast<std::size_t>(iy) * nx + ix];
                    if (z > dst) {
                        dst = z;
                    }
                }
            }
        }
        ready = true;
        return true;
    }

    bool sample(float x, float y, float& outHeight) const {
        if (!ready || x < minX || y < minY) {
            return false;
        }
        const int ix = static_cast<int>(std::floor((x - minX) / cell));
        const int iy = static_cast<int>(std::floor((y - minY) / cell));
        if (ix < 0 || iy < 0 || ix >= nx || iy >= ny) {
            return false;
        }
        const float h = heights[static_cast<std::size_t>(iy) * nx + ix];
        outHeight = (h < -500.0f) ? 0.0f : h;
        return true;
    }

};

}  // namespace

// =============================================================================
//  Implementation
// =============================================================================

struct Simulator::Impl {
    SimConfig config;
    CarState car;
    CameraIntrinsics intrinsics;
    std::array<CameraPose, kCameraCount> poses;
    std::array<CameraPose, kCameraCount> capturePoses;
    std::array<CameraFrame, kCameraCount> frames;
    std::array<CameraMount, kCameraCount> mounts;
    UserSwitches userSwitches;
    std::vector<Obstacle> obstacles;
    FieldMesh fieldMesh;
    CarMesh carMesh;
    TerrainField terrain;

    int lightCount = 4;
    std::array<LightSettings, kMaxLights> lights{};

    ArmConfig armConfig;
    struct ServoJoint {
        float currentRad = 0.0f;
        float targetRad = 0.0f;
        float velocityRad = 0.0f;
    };
    std::array<ServoJoint, kArmJointCount> armJoints{};

    struct BumperBox {
        Vec3 center;  // car-local
        Vec3 half;
    };
    std::array<BumperBox, 4> bumperBoxes{};

    std::array<const CameraFrame*, kCameraCount> displayFrames{};

    GLuint textures[kCameraCount]{};
    TextRenderer text;
    bool graphicsReady = false;

    std::vector<std::uint8_t> readColor;
    std::vector<float> readDepth;

    DepthProbe probe;
    float fps = 0.0f;
    float lastMeasuredSpeed = 0.0f;

    explicit Impl(const SimConfig& cfg) : config(cfg) {
        intrinsics.width = config.cameraWidth;
        intrinsics.height = config.cameraHeight;
        // Square pixels: cameraFovY is the vertical FOV used by glFrustum, so
        // fx and fy are identical. Horizontal FOV follows from the aspect ratio.
        const float focal = (static_cast<float>(config.cameraHeight) * 0.5f) /
                            std::tan(radians(config.cameraFovY) * 0.5f);
        intrinsics.fx = focal;
        intrinsics.fy = focal;
        intrinsics.cx = static_cast<float>(config.cameraWidth) * 0.5f;
        intrinsics.cy = static_cast<float>(config.cameraHeight) * 0.5f;

        mounts = {{
            {{config.cameraOffsetX, 0.0f, 0.0f}, "CAM 0  +X", {1.00f, 0.42f, 0.30f, 1.0f}},
            {{-config.cameraOffsetX, 0.0f, 0.0f}, "CAM 1  -X", {0.25f, 0.78f, 1.00f, 1.0f}},
            {{0.0f, config.cameraOffsetY, 0.0f}, "CAM 2  +Y", {0.45f, 0.95f, 0.45f, 1.0f}},
            {{0.0f, -config.cameraOffsetY, 0.0f}, "CAM 3  -Y", {0.92f, 0.62f, 1.00f, 1.0f}},
            {{0.0f, 0.0f, 0.0f}, "CAM 4  GRIP", {0.95f, 0.78f, 0.22f, 1.0f}},
        }};

        const int cameraW = config.cameraWidth;
        const int cameraH = config.cameraHeight;
        readColor.resize(static_cast<std::size_t>(cameraW) * cameraH * 4);
        readDepth.resize(static_cast<std::size_t>(cameraW) * cameraH);
        for (CameraFrame& f : frames) {
            f.width = cameraW;
            f.height = cameraH;
            f.rgba.resize(static_cast<std::size_t>(cameraW) * cameraH * 4, 0);
            f.depth.assign(static_cast<std::size_t>(cameraW) * cameraH, 1.0f);
        }

        fieldMesh.load();
        if (fieldMesh.loaded) {
            std::printf("[sim] field mesh loaded (%zu vertices, %zu triangles)\n",
                        fieldMesh.vertices.size(),
                        fieldMesh.indices.size() / 3);
            if (terrain.build(fieldMesh)) {
                std::printf("[sim] terrain collider built (%dx%d, %.0f mm cells)\n",
                            terrain.nx, terrain.ny, terrain.cell * 1000.0f);
            }
        } else {
            std::printf("[sim] assets/field.rfield not found, using procedural fallback field\n");
        }

        carMesh.load();
        if (carMesh.loaded) {
            std::size_t tris = 0;
            for (const auto& indices : carMesh.batchIndices) {
                tris += indices.size() / 3;
            }
            std::printf("[sim] local car model loaded (%zu triangles, %zu batches)\n",
                        tris, carMesh.batches.size());
        } else {
            std::printf("[sim] assets/car.rmesh not found, using procedural car fallback\n");
        }

        // The real STEP field already has plenty of props; only use the demo
        // pylons for the procedural fallback scene.
        if (config.spawnDemoProps && !fieldMesh.loaded) {
            obstacles = {
                {{3.6f, 1.55f, 0.36f}, {0.12f, 0.12f, 0.36f}, {0.98f, 0.52f, 0.16f, 1.0f}, true},
                {{3.6f, -1.55f, 0.36f}, {0.12f, 0.12f, 0.36f}, {0.20f, 0.48f, 0.95f, 1.0f}, true},
                {{-3.6f, 1.55f, 0.36f}, {0.12f, 0.12f, 0.36f}, {0.85f, 0.20f, 0.20f, 1.0f}, true},
                {{-3.6f, -1.55f, 0.36f}, {0.12f, 0.12f, 0.36f}, {0.82f, 0.82f, 0.86f, 1.0f}, true},
            };
        }

        buildBumperBoxes();
        resetLightRig();
        resetArmPose();
        reset();
    }

    void buildBumperBoxes() {
        const float hl = config.carLength * 0.5f;
        const float hw = config.carWidth * 0.5f;
        const float t = clampf(config.carBumperThickness, 0.01f, 0.10f);
        const float bh = config.carBumperHeight * 0.5f;
        const float bz = config.carBumperBottom + bh;
        bumperBoxes[0] = {{hl - t * 0.5f, 0.0f, bz}, {t * 0.5f, hw, bh}};
        bumperBoxes[1] = {{-hl + t * 0.5f, 0.0f, bz}, {t * 0.5f, hw, bh}};
        bumperBoxes[2] = {{0.0f, hw - t * 0.5f, bz}, {hl, t * 0.5f, bh}};
        bumperBoxes[3] = {{0.0f, -hw + t * 0.5f, bz}, {hl, t * 0.5f, bh}};
    }

    static float degToRad(float degrees) {
        return radians(degrees);
    }

    static float radToDeg(float radiansValue) {
        return radiansValue * (180.0f / kPi);
    }

    void resetArmPose() {
        armConfig.gripperOpening = 0.08f;
        for (int i = 0; i < kArmJointCount; ++i) {
            const float start = degToRad(armConfig.jointTargetDeg[static_cast<std::size_t>(i)]);
            armJoints[static_cast<std::size_t>(i)].currentRad = start;
            armJoints[static_cast<std::size_t>(i)].targetRad = start;
            armJoints[static_cast<std::size_t>(i)].velocityRad = 0.0f;
        }
        updateCameraPoses();
    }

    void setArmTargetRadians(int joint, float targetRadians) {
        if (joint < 0 || joint >= kArmJointCount || !std::isfinite(targetRadians)) {
            return;
        }
        const std::size_t index = static_cast<std::size_t>(joint);
        const float minRad = degToRad(armConfig.jointMinDeg[index]);
        const float maxRad = degToRad(armConfig.jointMaxDeg[index]);
        const float clamped = clampf(targetRadians, minRad, maxRad);
        armJoints[index].targetRad = clamped;
    }

    void updateArm(float dt) {
        dt = clampf(dt, 0.0f, 0.05f);
        if (dt <= 0.0f) {
            return;
        }
        const float maxSpeed = std::max(1.0e-4f, degToRad(armConfig.servoMaxSpeedDeg));
        const float maxAccel = std::max(1.0e-4f, degToRad(armConfig.servoMaxAccelDeg));

        for (ServoJoint& joint : armJoints) {
            float error = joint.targetRad - joint.currentRad;
            if (std::fabs(error) < 1.0e-6f && std::fabs(joint.velocityRad) < 1.0e-5f) {
                joint.currentRad = joint.targetRad;
                joint.velocityRad = 0.0f;
                continue;
            }

            // Servo velocity command: proportional to the remaining angle error.
            // The velocity command is then rate-limited to max acceleration and
            // saturated to max speed. This produces visible acceleration and
            // braking phases without a hard snap at the target.
            constexpr float kPositionGain = 6.0f;  // 1/s
            float desiredVelocity = clampf(error * kPositionGain, -maxSpeed, maxSpeed);
            float dv = clampf(desiredVelocity - joint.velocityRad,
                              -maxAccel * dt,
                              maxAccel * dt);
            joint.velocityRad = clampf(joint.velocityRad + dv, -maxSpeed, maxSpeed);
            joint.currentRad += joint.velocityRad * dt;

            error = joint.targetRad - joint.currentRad;
            if (std::fabs(error) < 0.001f && std::fabs(joint.velocityRad) < 0.02f) {
                joint.currentRad = joint.targetRad;
                joint.velocityRad = 0.0f;
            }
        }
        updateCameraPoses();
    }

    void resetLightRig() {
        lightCount = 4;
        lights[0] = {{-3.7f, -1.6f, 4.8f}, {1.0f, 0.94f, 0.84f}, 0.62f};
        lights[1] = {{3.7f, -1.6f, 4.8f}, {0.92f, 0.96f, 1.0f}, 0.62f};
        lights[2] = {{-3.7f, 1.6f, 5.4f}, {1.0f, 0.94f, 0.84f}, 0.58f};
        lights[3] = {{3.7f, 1.6f, 5.4f}, {0.92f, 0.96f, 1.0f}, 0.58f};
        for (int i = 4; i < kMaxLights; ++i) {
            const float angle = kTwoPi * static_cast<float>(i) /
                                static_cast<float>(kMaxLights);
            lights[i] = {{std::cos(angle) * 3.0f,
                          std::sin(angle) * 1.5f,
                          5.0f + static_cast<float>(i % 3)},
                         {1.0f, 0.94f, 0.84f},
                         0.55f};
        }
    }

    ~Impl() {
        if (graphicsReady) {
            glDeleteTextures(kCameraCount, textures);
            text.shutdown();
        }
    }

    void updateCameraPoses() {
        for (int i = 0; i < kCarCameraCount; ++i) {
            const Vec3& local = mounts[i].localOffset;
            const Vec3 localPos{local.x, local.y, config.cameraMountHeight};
            const Vec3 localForward = normalize(Vec3{local.x, local.y, 0.0f});
            const Vec3 worldOffset = rotateCarLocal(localPos, car.yaw, car.pitch, car.roll);
            const Vec3 forward = normalize(rotateCarLocal(localForward, car.yaw, car.pitch, car.roll));
            const Vec3 up = normalize(rotateCarLocal({0.0f, 0.0f, 1.0f}, car.yaw, car.pitch, car.roll));
            CameraPose& p = poses[i];
            p.position = {car.x + worldOffset.x, car.y + worldOffset.y, car.z + worldOffset.z};
            p.forward = forward;
            p.up = up;
            p.right = normalize(cross(forward, up));
        }

        // Gripper camera: mounted above the gripper housing, looking along the
        // gripper's local Z axis. It follows every arm joint automatically.
        const ArmPoseLocal armPose = armPoseLocal();
        const Vec3 forwardLocal = normalize(armPose.gripperDirection);
        const Vec3 upLocal = normalize(cross(forwardLocal, armPose.side));
        const Vec3 camLocal = armPose.palm + forwardLocal * 0.045f + upLocal * 0.045f;
        const Vec3 worldOffset = rotateCarLocal(camLocal, car.yaw, car.pitch, car.roll);
        const Vec3 forward = normalize(rotateCarLocal(forwardLocal, car.yaw, car.pitch, car.roll));
        const Vec3 up = normalize(rotateCarLocal(upLocal, car.yaw, car.pitch, car.roll));
        CameraPose& grip = poses[kCarCameraCount];
        grip.position = {car.x + worldOffset.x, car.y + worldOffset.y, car.z + worldOffset.z};
        grip.forward = forward;
        grip.up = up;
        grip.right = normalize(cross(forward, up));
    }

    void reset() {
        car = {};
        float sx = config.spawnX;
        float sy = config.spawnY;
        findSpawn(sx, sy);
        car.x = sx;
        car.y = sy;
        car.yaw = 0.0f;
        snapCarToTerrain();
        probe = {};
        lastMeasuredSpeed = 0.0f;
        capturePoses = poses;
        for (int i = 0; i < kCameraCount; ++i) {
            displayFrames[static_cast<std::size_t>(i)] = &frames[static_cast<std::size_t>(i)];
        }
    }

    void setCameraMountHeight(float height) {
        config.cameraMountHeight = clampf(height, 0.05f, 2.0f);
        updateCameraPoses();
    }

    float sampleGround(float x, float y) const {
        float h = 0.0f;
        if (terrain.sample(x, y, h)) {
            return h;
        }
        if (std::fabs(x) <= config.fieldHalfX && std::fabs(y) <= config.fieldHalfY) {
            return 0.0f;
        }
        return -1000.0f;
    }

    bool inTunnelZone(float x, float y, float carZ) const {
        // The two 800 mm-wide bridges on the long edges have a clear opening
        // under the top beam. The raw height field only sees the roof, so keep
        // their footprint on the current (possibly raised) floor while the
        // chassis passes under the beam.
        if (carZ < -0.02f || carZ > config.carBodyHeight + 0.17f) {
            return false;
        }
        const float ay = std::fabs(y);
        const bool centreArch = std::fabs(x) <= 0.40f && ay >= 1.75f && ay <= 2.35f;
        const bool blueArch = x >= 4.55f && x <= 5.25f && y >= -2.45f && y <= -1.90f;
        return centreArch || blueArch;
    }

    float sampleGroundForCar(float x, float y, float carZ) const {
        if (inTunnelZone(x, y, carZ)) {
            return std::max(0.005f, carZ);
        }
        return sampleGround(x, y);
    }

    void sampleWheelGround(float heights[4]) const {
        const float hx = config.carWheelBase * 0.5f;
        const float hy = config.carTrackWidth * 0.5f;
        const Vec3 local[4] = {
            {hx, hy, 0.0f}, {hx, -hy, 0.0f}, {-hx, hy, 0.0f}, {-hx, -hy, 0.0f}};
        for (int i = 0; i < 4; ++i) {
            const Vec3 w = rotateZ(local[i], car.yaw);
            heights[i] = sampleGroundForCar(car.x + w.x, car.y + w.y, car.z);
        }
    }

    void terrainPose(float& outZ, float& outPitch, float& outRoll, float& outSpread) const {
        if (!terrain.ready) {
            outZ = 0.0f;
            outPitch = 0.0f;
            outRoll = 0.0f;
            outSpread = 0.0f;
            return;
        }
        float h[4]{};
        sampleWheelGround(h);
        const float front = (h[0] + h[1]) * 0.5f;
        const float rear = (h[2] + h[3]) * 0.5f;
        const float left = (h[0] + h[2]) * 0.5f;
        const float right = (h[1] + h[3]) * 0.5f;
        outZ = (front + rear) * 0.5f;
        outPitch = std::atan2(front - rear, config.carWheelBase);
        outRoll = std::atan2(left - right, config.carTrackWidth);
        outSpread = std::max(std::max(h[0], h[1]), std::max(h[2], h[3])) -
                    std::min(std::min(h[0], h[1]), std::min(h[2], h[3]));
    }

    void snapCarToTerrain() {
        float z = 0.0f;
        float pitch = 0.0f;
        float roll = 0.0f;
        float spread = 0.0f;
        terrainPose(z, pitch, roll, spread);
        car.z = z;
        car.pitch = pitch;
        car.roll = roll;
        car.verticalVelocity = 0.0f;
        car.airborne = false;
        updateCameraPoses();
    }

    bool clearSpawn(float x, float y) const {
        const float hx = config.carWheelBase * 0.5f;
        const float hy = config.carTrackWidth * 0.5f;
        const Vec3 local[4] = {
            {hx, hy, 0.0f}, {hx, -hy, 0.0f}, {-hx, hy, 0.0f}, {-hx, -hy, 0.0f}};
        float lo = 1.0e9f;
        float hi = -1.0e9f;
        for (int i = 0; i < 4; ++i) {
            const Vec3 w = rotateZ(local[i], 0.0f);
            const float h = sampleGround(x + w.x, y + w.y);
            if (h < -100.0f) {
                return false;
            }
            lo = std::min(lo, h);
            hi = std::max(hi, h);
        }
        const float center = sampleGround(x, y);
        if (center < -100.0f || center >= 0.06f || hi >= 0.06f || (hi - lo) >= 0.035f) {
            return false;
        }
        // Camera mounts must also start outside tall geometry, otherwise a
        // spawn beside the central platform can put one camera inside it.
        const Vec3 cameraLocal[kCarCameraCount] = {
            {config.cameraOffsetX, 0.0f, 0.0f},
            {-config.cameraOffsetX, 0.0f, 0.0f},
            {0.0f, config.cameraOffsetY, 0.0f},
            {0.0f, -config.cameraOffsetY, 0.0f},
        };
        for (const Vec3& c : cameraLocal) {
            const float h = sampleGround(x + c.x, y + c.y);
            if (h > config.cameraMountHeight - 0.04f) {
                return false;
            }
        }
        for (const BumperBox& b : bumperBoxes) {
            for (int sx = -1; sx <= 1; sx += 2) {
                for (int sy = -1; sy <= 1; sy += 2) {
                    const Vec3 p{b.center.x + static_cast<float>(sx) * b.half.x,
                                 b.center.y + static_cast<float>(sy) * b.half.y,
                                 0.0f};
                    const float h = sampleGround(x + p.x, y + p.y);
                    if (h > 0.06f) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    void findSpawn(float& sx, float& sy) const {
        if (clearSpawn(config.spawnX, config.spawnY)) {
            sx = config.spawnX;
            sy = config.spawnY;
            return;
        }
        // Search outward from the requested spawn. The restored central
        // platform sits exactly at the field origin, so the nearest free floor
        // is normally a short distance along -Y or +Y.
        for (float radius = 0.3f; radius <= 7.0f; radius += 0.2f) {
            constexpr int kRays = 24;
            for (int i = 0; i < kRays; ++i) {
                // Prefer the clear floor just in front of the central platform.
                const float a = -kPi * 0.5f +
                                kTwoPi * static_cast<float>(i) / static_cast<float>(kRays);
                const float x = config.spawnX + std::cos(a) * radius;
                const float y = config.spawnY + std::sin(a) * radius;
                if (std::fabs(x) > config.fieldHalfX - 0.5f ||
                    std::fabs(y) > config.fieldHalfY - 0.5f) {
                    continue;
                }
                if (clearSpawn(x, y)) {
                    sx = x;
                    sy = y;
                    return;
                }
            }
        }
        sx = config.spawnX;
        sy = config.spawnY;
    }

    bool wouldStepBlock(float x, float y) const {
        const float hx = config.carWheelBase * 0.5f;
        const float hy = config.carTrackWidth * 0.5f;
        const Vec3 local[5] = {
            {0.0f, 0.0f, 0.0f},
            {hx, hy, 0.0f}, {hx, -hy, 0.0f}, {-hx, hy, 0.0f}, {-hx, -hy, 0.0f}};
        for (const Vec3& l : local) {
            const Vec3 w = rotateZ(l, car.yaw);
            const float h = sampleGroundForCar(x + w.x, y + w.y, car.z);
            if (h < -100.0f) {
                return true;
            }
            if (h > car.z + config.maxClimbStep) {
                return true;
            }
        }
        // Four bumper boxes are sampled on their centres and outer corners so
        // the chassis cannot overlap a step or wall between the wheels.
        for (const BumperBox& b : bumperBoxes) {
            for (int ix = -1; ix <= 1; ++ix) {
                for (int iy = -1; iy <= 1; ++iy) {
                    const Vec3 p{b.center.x + static_cast<float>(ix) * b.half.x,
                                 b.center.y + static_cast<float>(iy) * b.half.y,
                                 0.0f};
                    const Vec3 w = rotateZ(p, car.yaw);
                    const float h = sampleGroundForCar(x + w.x, y + w.y, car.z);
                    if (h < -100.0f) {
                        return true;
                    }
                    if (h > car.z + config.maxClimbStep) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void moveCar(float dx, float dy) {
        const float dist = std::sqrt(dx * dx + dy * dy);
        const int steps = std::max(1, static_cast<int>(std::ceil(dist / 0.045f)));
        const float sx = dx / static_cast<float>(steps);
        const float sy = dy / static_cast<float>(steps);
        for (int i = 0; i < steps; ++i) {
            if (!wouldStepBlock(car.x + sx, car.y + sy)) {
                car.x += sx;
                car.y += sy;
                continue;
            }
            // Slide along whichever axis remains clear.
            if (!wouldStepBlock(car.x + sx, car.y)) {
                car.x += sx;
            }
            if (!wouldStepBlock(car.x, car.y + sy)) {
                car.y += sy;
            }
        }
    }

    void updateVertical(float dt) {
        float groundZ = 0.0f;
        float groundPitch = 0.0f;
        float groundRoll = 0.0f;
        float spread = 0.0f;
        terrainPose(groundZ, groundPitch, groundRoll, spread);

        if (!terrain.ready) {
            car.z = 0.0f;
            car.pitch = 0.0f;
            car.roll = 0.0f;
            car.verticalVelocity = 0.0f;
            car.airborne = false;
            return;
        }

        constexpr float kContactSlop = 0.05f;
        constexpr float kMaxAirborneSpread = 0.22f;
        if (!car.airborne &&
            (car.z > groundZ + kContactSlop || spread > kMaxAirborneSpread)) {
            car.airborne = true;
            car.verticalVelocity = 0.0f;
        }

        if (car.airborne) {
            car.verticalVelocity -= config.gravity * dt;
            car.verticalVelocity = std::max(car.verticalVelocity, -12.0f);
            car.z += car.verticalVelocity * dt;
            if (car.z <= groundZ) {
                car.z = groundZ;
                car.pitch = groundPitch;
                car.roll = groundRoll;
                car.verticalVelocity = 0.0f;
                car.airborne = false;
            }
        } else {
            car.z = groundZ;
            car.verticalVelocity = 0.0f;
            const float blend = 1.0f - std::exp(-12.0f * dt);
            car.pitch += (groundPitch - car.pitch) * blend;
            car.roll += (groundRoll - car.roll) * blend;
            if (spread > kMaxAirborneSpread) {
                car.airborne = true;
            }
        }
    }

    void resolveCollisions() {
        for (const Obstacle& o : obstacles) {
            if (!o.collide) {
                continue;
            }
            const float minX = o.center.x - o.halfSize.x;
            const float maxX = o.center.x + o.halfSize.x;
            const float minY = o.center.y - o.halfSize.y;
            const float maxY = o.center.y + o.halfSize.y;
            const float r = config.carWidth * 0.5f;
            if (car.x <= minX - r || car.x >= maxX + r ||
                car.y <= minY - r || car.y >= maxY + r) {
                continue;
            }

            const float pushNegX = std::fabs(car.x - (minX - r));
            const float pushPosX = std::fabs((maxX + r) - car.x);
            const float pushNegY = std::fabs(car.y - (minY - r));
            const float pushPosY = std::fabs((maxY + r) - car.y);
            const float minPush = std::min(std::min(pushNegX, pushPosX),
                                           std::min(pushNegY, pushPosY));
            if (minPush == pushNegX) {
                car.x = minX - r;
            } else if (minPush == pushPosX) {
                car.x = maxX + r;
            } else if (minPush == pushNegY) {
                car.y = minY - r;
            } else {
                car.y = maxY + r;
            }
        }

        // Keep the camera mounts inside the walls as well as the chassis.
        const float cameraReach = std::max(config.cameraOffsetX, config.cameraOffsetY);
        const float margin = cameraReach + 0.08f;
        car.x = clampf(car.x, -config.fieldHalfX + margin, config.fieldHalfX - margin);
        car.y = clampf(car.y, -config.fieldHalfY + margin, config.fieldHalfY - margin);
    }

    void update(const InputState& input, float dt) {
        dt = clampf(dt, 0.0f, 0.05f);

        Vec3 bodyMove{input.forward, input.strafeLeft, 0.0f};
        const float inputLen = std::sqrt(bodyMove.x * bodyMove.x + bodyMove.y * bodyMove.y);
        if (inputLen > 1.0f) {
            bodyMove.x /= inputLen;
            bodyMove.y /= inputLen;
        }

        const Vec3 worldMove = rotateZ(bodyMove, car.yaw);
        const float speed = config.moveSpeed * (input.boost ? config.boostMultiplier : 1.0f);
        const float oldX = car.x;
        const float oldY = car.y;
        moveCar(worldMove.x * speed * dt, worldMove.y * speed * dt);
        car.yaw += input.yawRate * config.rotateSpeed * dt;
        car.yaw = std::remainder(car.yaw, kTwoPi);

        resolveCollisions();
        updateVertical(dt);
        updateCameraPoses();

        const float dx = car.x - oldX;
        const float dy = car.y - oldY;
        lastMeasuredSpeed = std::sqrt(dx * dx + dy * dy) / std::max(1.0e-5f, dt);
    }

    Ray rayForPose(const CameraPose& p, float imageX, float imageY) const {
        const float xCam = (imageX - intrinsics.cx) / intrinsics.fx;
        const float yCam = -(imageY - intrinsics.cy) / intrinsics.fy;
        Vec3 dir = p.forward + p.right * xCam + p.up * yCam;
        return {p.position, normalize(dir)};
    }

    bool sampleRayDepth(int cameraIndex,
                        float imageX,
                        float imageY,
                        float& outDepth,
                        Ray* outRay = nullptr) const {
        if (cameraIndex < 0 || cameraIndex >= kCameraCount) {
            return false;
        }
        const CameraFrame& frame = frames[cameraIndex];
        if (frame.width <= 0 || frame.height <= 0 || frame.depth.empty()) {
            return false;
        }
        if (!(imageX >= 0.0f && imageY >= 0.0f) ||
            imageX >= static_cast<float>(frame.width) ||
            imageY >= static_cast<float>(frame.height)) {
            return false;
        }

        // Snap to the pixel centre so depth and unprojection use the exact
        // same sample.
        const int ix = clampi(static_cast<int>(std::floor(imageX)), 0, frame.width - 1);
        const int iy = clampi(static_cast<int>(std::floor(imageY)), 0, frame.height - 1);
        const float ndcDepth = frame.depth[static_cast<std::size_t>(iy) * frame.width + ix];
        if (!(ndcDepth < 1.0f)) {
            return false;
        }

        const float ndcZ = 2.0f * ndcDepth - 1.0f;
        const float nearZ = config.cameraNear;
        const float farZ = config.cameraFar;
        const float eyeZ = (2.0f * nearZ * farZ) /
                           ((farZ - nearZ) * ndcZ - (farZ + nearZ));
        if (!(eyeZ < 0.0f)) {
            return false;
        }

        // Use the pose that was active when this frame was captured. This
        // keeps the depth query valid even when the robot has moved since.
        const CameraPose& capturedPose = capturePoses[cameraIndex];
        const Ray ray = rayForPose(capturedPose,
                                  static_cast<float>(ix) + 0.5f,
                                  static_cast<float>(iy) + 0.5f);
        const float forwardComponent = dot(ray.direction, capturedPose.forward);
        if (forwardComponent <= 1.0e-5f) {
            return false;
        }

        const float distance = (-eyeZ) / forwardComponent;
        if (!(distance > 0.0f) || !std::isfinite(distance)) {
            return false;
        }

        outDepth = distance;
        if (outRay != nullptr) {
            *outRay = ray;
        }
        return true;
    }

    // -------------------------------------------------------------------------
    //  Scene rendering
    // -------------------------------------------------------------------------

    void enableSceneLighting() {
        const GLfloat globalAmbient[4] = {0.24f, 0.26f, 0.30f, 1.0f};
        const GLfloat black[4] = {0.0f, 0.0f, 0.0f, 1.0f};

        glEnable(GL_LIGHTING);
        glEnable(GL_COLOR_MATERIAL);
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globalAmbient);
        glShadeModel(GL_SMOOTH);

        for (int i = 0; i < kMaxLights; ++i) {
            glDisable(GL_LIGHT0 + i);
        }

        for (int i = 0; i < lightCount; ++i) {
            const LightSettings& light = lights[i];
            const GLenum id = GL_LIGHT0 + i;
            const GLfloat ambient[4] = {light.color.x * 0.05f,
                                        light.color.y * 0.05f,
                                        light.color.z * 0.05f,
                                        1.0f};
            const GLfloat diffuse[4] = {light.color.x * light.intensity,
                                        light.color.y * light.intensity,
                                        light.color.z * light.intensity,
                                        1.0f};
            const GLfloat position[4] = {light.position.x,
                                         light.position.y,
                                         light.position.z,
                                         1.0f};
            glEnable(id);
            glLightfv(id, GL_AMBIENT, ambient);
            glLightfv(id, GL_DIFFUSE, diffuse);
            glLightfv(id, GL_SPECULAR, black);
            glLightfv(id, GL_POSITION, position);
            glLightf(id, GL_CONSTANT_ATTENUATION, 1.0f);
            glLightf(id, GL_LINEAR_ATTENUATION, 0.018f);
            glLightf(id, GL_QUADRATIC_ATTENUATION, 0.0025f);
        }
    }

    void drawCameraBackground() {
        glDisable(GL_LIGHTING);
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_BLEND);
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Clear sky to a hazy horizon. The background is intentionally left
        // at depth = 1.0 so getRayDepth() reports a miss for sky pixels.
        glBegin(GL_QUADS);
        glColor3f(0.17f, 0.27f, 0.42f);
        glVertex2f(0.0f, 1.0f);
        glVertex2f(1.0f, 1.0f);
        glColor3f(0.64f, 0.71f, 0.76f);
        glVertex2f(1.0f, 0.0f);
        glVertex2f(0.0f, 0.0f);
        glEnd();

        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
    }

    void drawFieldMarkings() {
        const float fx = config.fieldHalfX;
        const float fy = config.fieldHalfY;
        const float bx = fx - 0.18f;
        const float by = fy - 0.18f;

        glDisable(GL_LIGHTING);
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glLineWidth(2.0f);

        glColor3f(0.72f, 0.78f, 0.82f);
        glBegin(GL_LINES);
        // Outer boundary.
        glVertex3f(-bx, -by, 0.010f);
        glVertex3f(bx, -by, 0.010f);
        glVertex3f(bx, -by, 0.010f);
        glVertex3f(bx, by, 0.010f);
        glVertex3f(bx, by, 0.010f);
        glVertex3f(-bx, by, 0.010f);
        glVertex3f(-bx, by, 0.010f);
        glVertex3f(-bx, -by, 0.010f);

        // Half-court line and subtle thirds.
        glVertex3f(0.0f, -by, 0.010f);
        glVertex3f(0.0f, by, 0.010f);
        glVertex3f(-3.0f, -by, 0.010f);
        glVertex3f(-3.0f, by, 0.010f);
        glVertex3f(3.0f, -by, 0.010f);
        glVertex3f(3.0f, by, 0.010f);

        // Centre cross.
        glVertex3f(-0.45f, 0.0f, 0.011f);
        glVertex3f(0.45f, 0.0f, 0.011f);
        glVertex3f(0.0f, -0.45f, 0.011f);
        glVertex3f(0.0f, 0.45f, 0.011f);
        glEnd();

        glLineWidth(2.4f);
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 96; ++i) {
            const float a = kTwoPi * static_cast<float>(i) / 96.0f;
            glVertex3f(std::cos(a) * 1.15f, std::sin(a) * 1.15f, 0.011f);
        }
        glEnd();

        // Team-coloured end zones.
        glColor3f(0.82f, 0.20f, 0.18f);
        glBegin(GL_LINE_LOOP);
        glVertex3f(-bx, -1.45f, 0.010f);
        glVertex3f(-fx + 2.1f, -1.45f, 0.010f);
        glVertex3f(-fx + 2.1f, 1.45f, 0.010f);
        glVertex3f(-bx, 1.45f, 0.010f);
        glEnd();

        glColor3f(0.20f, 0.42f, 0.95f);
        glBegin(GL_LINE_LOOP);
        glVertex3f(fx - 2.1f, -1.45f, 0.010f);
        glVertex3f(bx, -1.45f, 0.010f);
        glVertex3f(bx, 1.45f, 0.010f);
        glVertex3f(fx - 2.1f, 1.45f, 0.010f);
        glEnd();

        glLineWidth(1.0f);
    }

    void drawWalls() {
        const float fx = config.fieldHalfX;
        const float fy = config.fieldHalfY;
        const float h = config.wallHeight;
        const float t = 0.10f;

        enableSceneLighting();

        const Color wallColor{0.22f, 0.28f, 0.36f, 1.0f};
        const Color railColor{0.56f, 0.63f, 0.70f, 1.0f};
        const Color kickColor{0.085f, 0.105f, 0.13f, 1.0f};

        drawBox({0.0f, fy + t * 0.5f, h * 0.5f}, {2.0f * fx + 2.0f * t, t, h}, wallColor);
        drawBox({0.0f, -fy - t * 0.5f, h * 0.5f}, {2.0f * fx + 2.0f * t, t, h}, wallColor);
        drawBox({fx + t * 0.5f, 0.0f, h * 0.5f}, {t, 2.0f * fy, h}, wallColor);
        drawBox({-fx - t * 0.5f, 0.0f, h * 0.5f}, {t, 2.0f * fy, h}, wallColor);

        drawBox({0.0f, fy + t * 0.5f, h + 0.025f}, {2.0f * fx + 2.0f * t, t + 0.05f, 0.05f}, railColor);
        drawBox({0.0f, -fy - t * 0.5f, h + 0.025f}, {2.0f * fx + 2.0f * t, t + 0.05f, 0.05f}, railColor);
        drawBox({fx + t * 0.5f, 0.0f, h + 0.025f}, {t + 0.05f, 2.0f * fy, 0.05f}, railColor);
        drawBox({-fx - t * 0.5f, 0.0f, h + 0.025f}, {t + 0.05f, 2.0f * fy, 0.05f}, railColor);

        drawBox({0.0f, fy - 0.015f, 0.075f}, {2.0f * fx, 0.04f, 0.15f}, kickColor);
        drawBox({0.0f, -fy + 0.015f, 0.075f}, {2.0f * fx, 0.04f, 0.15f}, kickColor);
        drawBox({fx - 0.015f, 0.0f, 0.075f}, {0.04f, 2.0f * fy, 0.15f}, kickColor);
        drawBox({-fx + 0.015f, 0.0f, 0.075f}, {0.04f, 2.0f * fy, 0.15f}, kickColor);

        // Vertical posts along the long walls and team accent strips.
        const Color postColor{0.30f, 0.36f, 0.43f, 1.0f};
        for (float x : {-6.0f, -3.0f, 3.0f, 6.0f}) {
            drawBox({x, fy - 0.012f, h * 0.52f}, {0.055f, 0.03f, h * 0.9f}, postColor);
            drawBox({x, -fy + 0.012f, h * 0.52f}, {0.055f, 0.03f, h * 0.9f}, postColor);
        }
        // Team accents sit away from the centre so the wall targets stay clear.
        for (float y : {-1.75f, 1.75f}) {
            drawBox({fx - 0.012f, y, h * 0.48f}, {0.03f, 0.14f, h * 0.72f}, {0.14f, 0.36f, 0.92f, 1.0f});
            drawBox({-fx + 0.012f, y, h * 0.48f}, {0.03f, 0.14f, h * 0.72f}, {0.86f, 0.16f, 0.16f, 1.0f});
        }
    }

    void drawObstacles() {
        if (obstacles.empty()) {
            return;
        }
        enableSceneLighting();
        const Color baseColor{0.10f, 0.12f, 0.14f, 1.0f};
        const Color capColor{0.88f, 0.90f, 0.92f, 1.0f};
        for (const Obstacle& o : obstacles) {
            drawBox({o.center.x, o.center.y, 0.018f}, {0.30f, 0.30f, 0.036f}, baseColor);
            drawBox({o.center.x, o.center.y, 0.12f}, {0.26f, 0.26f, 0.20f}, baseColor);
            drawBox({o.center.x, o.center.y, 0.40f}, {0.225f, 0.225f, 0.34f}, o.color);
            drawBox({o.center.x, o.center.y, 0.575f}, {0.245f, 0.245f, 0.055f}, capColor);
            drawBox({o.center.x, o.center.y, 0.71f}, {0.28f, 0.28f, 0.045f}, o.color);
        }
    }

    void drawArena() {
        const float fx = config.fieldHalfX;
        const float fy = config.fieldHalfY;

        if (fieldMesh.loaded) {
            glEnable(GL_DEPTH_TEST);
            glDepthMask(GL_TRUE);
            glDepthFunc(GL_LEQUAL);
            glDisable(GL_CULL_FACE);
            glDisable(GL_BLEND);
            enableSceneLighting();
            fieldMesh.draw();

            // The CAD start-zone pads sit within 1 mm of the floor and can
            // z-fight in the fixed-function pipeline. Redraw their visible tops
            // slightly above the floor so the red/blue team areas stay legible.
            glDisable(GL_LIGHTING);
            glDisable(GL_BLEND);
            glEnable(GL_DEPTH_TEST);
            glDepthMask(GL_TRUE);
            glBegin(GL_QUADS);
            glNormal3f(0.0f, 0.0f, 1.0f);
            glColor3f(0.62f, 0.055f, 0.12f);
            glVertex3f(-5.98f, -2.46f, 0.012f);
            glVertex3f(-5.02f, -2.46f, 0.012f);
            glVertex3f(-5.02f, -0.52f, 0.012f);
            glVertex3f(-5.98f, -0.52f, 0.012f);
            glColor3f(0.035f, 0.22f, 0.72f);
            glVertex3f(5.02f, 0.52f, 0.012f);
            glVertex3f(5.98f, 0.52f, 0.012f);
            glVertex3f(5.98f, 2.46f, 0.012f);
            glVertex3f(5.02f, 2.46f, 0.012f);
            glEnd();

            return;
        }

        glDisable(GL_LIGHTING);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LEQUAL);
        glDisable(GL_CULL_FACE);
        glDisable(GL_BLEND);

        // Floor tiles.
        constexpr int nx = 18;
        constexpr int ny = 5;
        const float tileW = (2.0f * fx) / static_cast<float>(nx);
        const float tileH = (2.0f * fy) / static_cast<float>(ny);
        for (int ix = 0; ix < nx; ++ix) {
            for (int iy = 0; iy < ny; ++iy) {
                const float x0 = -fx + static_cast<float>(ix) * tileW;
                const float y0 = -fy + static_cast<float>(iy) * tileH;
                const bool alt = ((ix + iy) & 1) != 0;
                const Color tile = alt ? Color{0.145f, 0.215f, 0.230f, 1.0f}
                                       : Color{0.120f, 0.180f, 0.200f, 1.0f};
                drawFloorTile(x0, y0, x0 + tileW, y0 + tileH, tile);
            }
        }

        // Soft team-colour end zones.
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        glBegin(GL_QUADS);
        glColor4f(0.85f, 0.12f, 0.10f, 0.10f);
        glVertex3f(-fx, -fy, 0.004f);
        glVertex3f(-fx + 2.1f, -fy, 0.004f);
        glVertex3f(-fx + 2.1f, fy, 0.004f);
        glVertex3f(-fx, fy, 0.004f);
        glColor4f(0.10f, 0.24f, 0.90f, 0.10f);
        glVertex3f(fx - 2.1f, -fy, 0.004f);
        glVertex3f(fx, -fy, 0.004f);
        glVertex3f(fx, fy, 0.004f);
        glVertex3f(fx - 2.1f, fy, 0.004f);
        glEnd();
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);

        drawFieldMarkings();
        drawWalls();
        drawObstacles();
        glDisable(GL_LIGHTING);
        glDisable(GL_BLEND);
        glLineWidth(1.0f);
    }

    void drawFovWedges() {
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glDisable(GL_CULL_FACE);

        const float halfFov = std::atan((static_cast<float>(intrinsics.width) * 0.5f) / intrinsics.fx);
        const float maxWedgeLength = 3.4f;
        const float boundX = config.fieldHalfX - 0.03f;
        const float boundY = config.fieldHalfY - 0.03f;

        auto clippedLength = [&](const Vec3& origin, const Vec3& d) {
            float dist = maxWedgeLength;
            constexpr float kEpsilon = 1.0e-6f;
            if (d.x > kEpsilon) {
                dist = std::min(dist, (boundX - origin.x) / d.x);
            } else if (d.x < -kEpsilon) {
                dist = std::min(dist, (-boundX - origin.x) / d.x);
            }
            if (d.y > kEpsilon) {
                dist = std::min(dist, (boundY - origin.y) / d.y);
            } else if (d.y < -kEpsilon) {
                dist = std::min(dist, (-boundY - origin.y) / d.y);
            }
            return std::max(0.0f, std::min(dist, maxWedgeLength));
        };

        for (int i = 0; i < kCarCameraCount; ++i) {
            const Vec3& f = poses[i].forward;
            const Vec3 origin{poses[i].position.x, poses[i].position.y, 0.026f};
            const Color& c = mounts[i].color;
            glColor4f(c.r, c.g, c.b, 0.115f);
            glBegin(GL_TRIANGLE_FAN);
            glVertex3f(origin.x, origin.y, origin.z);
            for (int s = 0; s <= 16; ++s) {
                const float a = -halfFov + (2.0f * halfFov) * static_cast<float>(s) / 16.0f;
                const Vec3 d = rotateZ(f, a);
                const float dist = clippedLength(origin, d);
                glVertex3f(origin.x + d.x * dist,
                           origin.y + d.y * dist,
                           origin.z);
            }
            glEnd();

            const Vec3 left = rotateZ(f, -halfFov);
            const Vec3 right = rotateZ(f, halfFov);
            const float leftLen = clippedLength(origin, left);
            const float rightLen = clippedLength(origin, right);
            glColor4f(c.r, c.g, c.b, 0.45f);
            glLineWidth(1.2f);
            glBegin(GL_LINE_STRIP);
            glVertex3f(origin.x, origin.y, origin.z + 0.001f);
            glVertex3f(origin.x + left.x * leftLen,
                       origin.y + left.y * leftLen,
                       origin.z + 0.001f);
            glVertex3f(origin.x + right.x * rightLen,
                       origin.y + right.y * rightLen,
                       origin.z + 0.001f);
            glVertex3f(origin.x, origin.y, origin.z + 0.001f);
            glEnd();
        }

        glLineWidth(1.0f);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }

    struct ArmPoseLocal {
        Vec3 base;
        Vec3 shoulder;
        Vec3 elbow;
        Vec3 wrist;
        Vec3 palm;
        Vec3 forward;
        Vec3 side;
        Vec3 upperDirection;
        Vec3 lowerDirection;
        Vec3 gripperDirection;
    };

    // Forward kinematics in the car body frame. The arm plane rotates around
    // Z with the base joint; shoulder/elbow/wrist are pitch joints in that
    // plane. Angles are measured from +X toward +Z.
    ArmPoseLocal armPoseLocal() const {
        ArmPoseLocal pose;
        const float baseYaw = armJoints[0].currentRad;
        const float shoulder = armJoints[1].currentRad;
        const float elbow = armJoints[2].currentRad;
        const float wrist = armJoints[3].currentRad;

        pose.forward = {std::cos(baseYaw), std::sin(baseYaw), 0.0f};
        pose.side = {-std::sin(baseYaw), std::cos(baseYaw), 0.0f};
        pose.base = armConfig.basePosition;
        pose.shoulder = pose.base + Vec3{0.0f, 0.0f, 0.085f};

        const float a1 = shoulder;
        const float a2 = shoulder + elbow;
        const float a3 = shoulder + elbow + wrist;
        pose.upperDirection = pose.forward * std::cos(a1) + Vec3{0.0f, 0.0f, std::sin(a1)};
        pose.lowerDirection = pose.forward * std::cos(a2) + Vec3{0.0f, 0.0f, std::sin(a2)};
        pose.gripperDirection = pose.forward * std::cos(a3) + Vec3{0.0f, 0.0f, std::sin(a3)};

        pose.elbow = pose.shoulder + pose.upperDirection * armConfig.upperArmLength;
        pose.wrist = pose.elbow + pose.lowerDirection * armConfig.lowerArmLength;
        pose.palm = pose.wrist + pose.gripperDirection * armConfig.wristLength;
        return pose;
    }

    void drawArm3D() {
        const ArmPoseLocal pose = armPoseLocal();
        const Color dark{0.10f, 0.12f, 0.13f, 1.0f};
        const Color linkColor{0.86f, 0.46f, 0.10f, 1.0f};
        const Color jointColor{0.18f, 0.66f, 0.82f, 1.0f};
        const Color metal{0.62f, 0.66f, 0.68f, 1.0f};
        const Color gripperColor{0.26f, 0.29f, 0.32f, 1.0f};
        const Color jawColor{0.82f, 0.85f, 0.87f, 1.0f};

        drawCylinderBetween(pose.base + Vec3{0.0f, 0.0f, 0.005f},
                            pose.shoulder,
                            0.045f,
                            dark,
                            18);
        drawCylinderBetween(pose.base + Vec3{0.0f, 0.0f, 0.035f},
                            pose.base + Vec3{0.0f, 0.0f, 0.050f},
                            0.072f,
                            jointColor,
                            20);

        drawCylinderBetween(pose.shoulder - pose.side * 0.045f,
                            pose.shoulder + pose.side * 0.045f,
                            0.035f,
                            jointColor,
                            18);
        drawCylinderBetween(pose.shoulder, pose.elbow, 0.026f, linkColor, 16);
        drawCylinderBetween(pose.elbow - pose.side * 0.040f,
                            pose.elbow + pose.side * 0.040f,
                            0.032f,
                            jointColor,
                            18);
        drawCylinderBetween(pose.elbow, pose.wrist, 0.023f, linkColor, 16);
        drawCylinderBetween(pose.wrist - pose.side * 0.034f,
                            pose.wrist + pose.side * 0.034f,
                            0.028f,
                            jointColor,
                            18);
        drawCylinderBetween(pose.wrist, pose.palm, 0.016f, metal, 14);

        Vec3 localZ = pose.gripperDirection;
        Vec3 localY = pose.side;
        Vec3 localX = normalize(cross(localY, localZ));
        if (length(localX) < 0.5f) {
            localX = {1.0f, 0.0f, 0.0f};
        }

        drawOrientedBox(pose.palm + localZ * 0.025f,
                        localX,
                        localY,
                        localZ,
                        {0.070f, 0.060f, 0.050f},
                        gripperColor);

        // Gripper camera body and lens, matching the pose at camera index 4.
        const Vec3 camUp = normalize(cross(localZ, localY));
        const Vec3 camCenter = pose.palm + localZ * 0.015f + camUp * 0.045f;
        drawOrientedBox(camCenter,
                        localX,
                        localY,
                        localZ,
                        {0.030f, 0.025f, 0.040f},
                        {0.055f, 0.060f, 0.070f, 1.0f});
        drawCylinderBetween(camCenter + localZ * 0.020f,
                            camCenter + localZ * 0.040f,
                            0.010f,
                            {0.18f, 0.66f, 0.82f, 1.0f},
                            12);

        const float opening = clampf(armConfig.gripperOpening, 0.0f, kGripperMaxOpening);
        const float jawOffset = opening * 0.5f + 0.007f;
        for (int sign = -1; sign <= 1; sign += 2) {
            const Vec3 jawCenter = pose.palm + localZ * 0.080f +
                                   localY * (static_cast<float>(sign) * jawOffset);
            drawOrientedBox(jawCenter,
                            localX,
                            localY,
                            localZ,
                            {0.050f, 0.013f, 0.080f},
                            jawColor);
        }
    }

    void drawRobot3D(bool topView) {
        glDisable(GL_CULL_FACE);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LEQUAL);
        glDisable(GL_BLEND);

        if (topView) {
            glDisable(GL_LIGHTING);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);
            float shadowZ = 0.004f;
            float ground = 0.0f;
            if (terrain.sample(car.x, car.y, ground)) {
                shadowZ = ground + 0.004f;
            }
            drawTopDisk({car.x, car.y, shadowZ}, 0.38f, {0.0f, 0.0f, 0.0f, 0.30f}, 32);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
        }

        enableSceneLighting();
        glPushMatrix();
        glTranslatef(car.x, car.y, car.z);
        const Mat4 rotation = carRotationMatrix(car);
        glMultMatrixf(rotation.m);

        if (carMesh.loaded) {
            carMesh.draw();
            // Small arm pedestal bridging the local chassis top to the arm base.
            drawBox({0.0f, 0.0f, 0.193f}, {0.16f, 0.16f, 0.056f},
                    {0.15f, 0.17f, 0.19f, 1.0f});

            // High-visibility hub caps on the four local mecanum wheels.
            const Color hub{0.66f, 0.70f, 0.74f, 1.0f};
            for (int sx = -1; sx <= 1; sx += 2) {
                for (int sy = -1; sy <= 1; sy += 2) {
                    const float x = static_cast<float>(sx) * 0.21f;
                    const float y = static_cast<float>(sy) * 0.26f;
                    drawCylinderBetween({x, y, 0.076f},
                                        {x, y + static_cast<float>(sy) * 0.032f, 0.076f},
                                        0.030f,
                                        hub,
                                        16);
                }
            }
            // Forward direction marker so the robot heading is still obvious
            // in the top view despite the detailed local CAD mesh.
            glDisable(GL_LIGHTING);
            setColor({1.0f, 0.80f, 0.14f, 1.0f});
            glBegin(GL_TRIANGLES);
            glVertex3f(0.255f, 0.0f, 0.170f);
            glVertex3f(0.165f, 0.055f, 0.170f);
            glVertex3f(0.165f, -0.055f, 0.170f);
            glEnd();
            glEnable(GL_LIGHTING);
        } else {
            const Color wheelColor{0.035f, 0.040f, 0.045f, 1.0f};
            const Color hubColor{0.32f, 0.35f, 0.38f, 1.0f};
            const Color chassisColor{0.075f, 0.095f, 0.085f, 1.0f};
            const Color boardColor{0.12f, 0.32f, 0.22f, 1.0f};
            const Color batteryColor{0.16f, 0.20f, 0.30f, 1.0f};
            const Color metal{0.48f, 0.52f, 0.55f, 1.0f};

            for (int sx = -1; sx <= 1; sx += 2) {
                for (int sy = -1; sy <= 1; sy += 2) {
                    const float x = static_cast<float>(sx) * 0.175f;
                    const float y = static_cast<float>(sy) * 0.205f;
                    drawCylinderBetween({x, y - 0.030f, 0.075f},
                                        {x, y + 0.030f, 0.075f},
                                        0.062f,
                                        wheelColor,
                                        20);
                    const float hubY = y + static_cast<float>(sy) * 0.033f;
                    drawCylinderBetween({x, hubY, 0.075f},
                                        {x, hubY + static_cast<float>(sy) * 0.012f, 0.075f},
                                        0.030f,
                                        hubColor,
                                        16);
                }
            }
            drawBox({0.0f, 0.0f, 0.105f}, {0.44f, 0.34f, 0.055f}, chassisColor);
            drawBox({0.0f, 0.0f, 0.105f}, {0.44f, 0.34f, 0.055f}, chassisColor);
            drawBox({-0.06f, -0.02f, 0.230f}, {0.18f, 0.13f, 0.055f}, batteryColor);
            drawBox({0.10f, -0.07f, 0.232f}, {0.12f, 0.09f, 0.018f}, boardColor);
            drawBox({0.0f, 0.145f, 0.135f}, {0.34f, 0.020f, 0.045f}, metal);
            drawBox({0.0f, -0.145f, 0.135f}, {0.34f, 0.020f, 0.045f}, metal);
        }

        drawArm3D();

        glPopMatrix();
        glDisable(GL_LIGHTING);
    }

    void drawRobotInset(const Rect& inset, int windowHeight) {
        if (inset.w <= 20 || inset.h <= 20) {
            return;
        }
        const int glY = windowHeight - inset.y - inset.h;
        glViewport(inset.x, glY, inset.w, inset.h);
        glEnable(GL_SCISSOR_TEST);
        glScissor(inset.x, glY, inset.w, inset.h);
        glClearColor(0.014f, 0.021f, 0.030f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        const ArmPoseLocal armPose = armPoseLocal();
        Vec3 bmin{-0.34f, -0.34f, 0.0f};
        Vec3 bmax{0.34f, 0.34f, 0.28f};
        const Vec3 armPoints[] = {armPose.base,
                                  armPose.shoulder,
                                  armPose.elbow,
                                  armPose.wrist,
                                  armPose.palm};
        for (const Vec3& p : armPoints) {
            bmin.x = std::min(bmin.x, p.x);
            bmin.y = std::min(bmin.y, p.y);
            bmin.z = std::min(bmin.z, p.z);
            bmax.x = std::max(bmax.x, p.x);
            bmax.y = std::max(bmax.y, p.y);
            bmax.z = std::max(bmax.z, p.z);
        }
        const float jawReach = 0.12f;
        bmin = bmin - Vec3{jawReach, jawReach, jawReach};
        bmax = bmax + Vec3{jawReach, jawReach, jawReach};
        bmin.z = std::max(0.0f, bmin.z);

        const Vec3 centerLocal{(bmin.x + bmax.x) * 0.5f,
                               (bmin.y + bmax.y) * 0.5f,
                               (bmin.z + bmax.z) * 0.5f};
        const Vec3 halfExtents{(bmax.x - bmin.x) * 0.5f,
                               (bmax.y - bmin.y) * 0.5f,
                               (bmax.z - bmin.z) * 0.5f};
        const float radius = std::max(0.50f, length(halfExtents) + 0.06f);
        const Vec3 centerOffset =
            rotateCarLocal(centerLocal, car.yaw, car.pitch, car.roll);
        const Vec3 centerWorld = {car.x + centerOffset.x,
                                  car.y + centerOffset.y,
                                  car.z + centerOffset.z};
        const float fovY = 48.0f;
        const float distance = clampf(radius / std::sin(radians(fovY * 0.5f)) * 1.05f,
                                      0.9f,
                                      6.0f);
        const Vec3 viewDirLocal = normalize(Vec3{1.15f, -1.50f, 0.32f});
        const Vec3 viewDirWorld =
            rotateCarLocal(viewDirLocal, car.yaw, car.pitch, car.roll);
        const Vec3 eye = centerWorld + viewDirWorld * distance;

        const float aspect = static_cast<float>(inset.w) / static_cast<float>(inset.h);
        const Mat4 projection = perspectiveMatrix(radians(fovY), aspect, 0.03f, 12.0f);
        const Mat4 view = lookAtMatrix(eye, centerWorld, {0.0f, 0.0f, 1.0f});
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(projection.m);
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(view.m);

        glDisable(GL_LIGHTING);
        glDisable(GL_BLEND);
        glDisable(GL_CULL_FACE);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LEQUAL);

        // Small showcase floor. Follow the car vertically so the preview also
        // reads correctly when the robot is climbing or falling.
        const float floorX = car.x;
        const float floorY = car.y;
        const float floorZ = car.z;
        drawFloorTile(floorX - 1.20f, floorY - 1.00f, floorX + 1.20f, floorY + 1.00f,
                      {0.095f, 0.125f, 0.145f, 1.0f}, floorZ);
        glColor4f(0.23f, 0.29f, 0.33f, 1.0f);
        glLineWidth(1.0f);
        glBegin(GL_LINES);
        for (int g = -5; g <= 5; ++g) {
            const float x = static_cast<float>(g) * 0.20f;
            glVertex3f(floorX + x, floorY - 1.00f, floorZ + 0.004f);
            glVertex3f(floorX + x, floorY + 1.00f, floorZ + 0.004f);
            const float y = static_cast<float>(g) * 0.20f;
            glVertex3f(floorX - 1.20f, floorY + y, floorZ + 0.004f);
            glVertex3f(floorX + 1.20f, floorY + y, floorZ + 0.004f);
        }
        glEnd();

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        drawTopDisk({car.x, car.y, floorZ + 0.006f}, 0.38f, {0.0f, 0.0f, 0.0f, 0.28f}, 32);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);

        drawRobot3D(false);

        glDisable(GL_DEPTH_TEST);
        glDisable(GL_LIGHTING);
        glDisable(GL_SCISSOR_TEST);
    }

    void captureCameraFrame(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= kCameraCount) {
            return;
        }
        const CameraPose& pose = poses[cameraIndex];
        capturePoses[cameraIndex] = pose;
        CameraFrame& frame = frames[cameraIndex];
        displayFrames[cameraIndex] = &frame;
        const int w = frame.width;
        const int h = frame.height;
        if (w <= 0 || h <= 0) {
            return;
        }

        glViewport(0, 0, w, h);
        glEnable(GL_SCISSOR_TEST);
        glScissor(0, 0, w, h);
        glClearColor(0.55f, 0.64f, 0.72f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        drawCameraBackground();
        glClear(GL_DEPTH_BUFFER_BIT);

        const Mat4 projection = perspectiveMatrix(radians(config.cameraFovY),
                                                  static_cast<float>(w) / static_cast<float>(h),
                                                  config.cameraNear,
                                                  config.cameraFar);
        const Mat4 view = lookAtMatrix(pose.position, pose.position + pose.forward, pose.up);
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(projection.m);
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(view.m);

        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LEQUAL);
        glDisable(GL_CULL_FACE);
        glDisable(GL_BLEND);
        drawArena();
        drawRobot3D(false);
        glDisable(GL_LIGHTING);

        glReadBuffer(GL_BACK);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, readColor.data());
        glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, readDepth.data());

        const std::size_t rowBytes = static_cast<std::size_t>(w) * 4;
        for (int y = 0; y < h; ++y) {
            const std::size_t dstRow = static_cast<std::size_t>(h - 1 - y);
            std::memcpy(frame.rgba.data() + dstRow * rowBytes,
                        readColor.data() + static_cast<std::size_t>(y) * rowBytes,
                        rowBytes);
        }
        for (int y = 0; y < h; ++y) {
            const std::size_t dstRow = static_cast<std::size_t>(h - 1 - y);
            std::memcpy(frame.depth.data() + dstRow * static_cast<std::size_t>(w),
                        readDepth.data() + static_cast<std::size_t>(y) * static_cast<std::size_t>(w),
                        static_cast<std::size_t>(w) * sizeof(float));
        }

        glDisable(GL_SCISSOR_TEST);
    }

    // -------------------------------------------------------------------------
    //  Window / UI rendering
    // -------------------------------------------------------------------------

    void beginUi(int width, int height) {
        glViewport(0, 0, width, height);
        glDisable(GL_SCISSOR_TEST);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_LIGHTING);
        glDisable(GL_CULL_FACE);
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_BLEND);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0.0, static_cast<double>(width), 0.0, static_cast<double>(height), -1.0, 1.0);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
    }

    void fillRect(const Rect& r, int windowHeight, const Color& color) {
        if (r.w <= 0 || r.h <= 0) {
            return;
        }
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        if (color.a < 0.999f) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        } else {
            glDisable(GL_BLEND);
        }
        glColor4f(color.r, color.g, color.b, color.a);
        const float y0 = static_cast<float>(windowHeight - r.y - r.h);
        const float y1 = static_cast<float>(windowHeight - r.y);
        glBegin(GL_QUADS);
        glVertex2f(static_cast<float>(r.x), y0);
        glVertex2f(static_cast<float>(r.x + r.w), y0);
        glVertex2f(static_cast<float>(r.x + r.w), y1);
        glVertex2f(static_cast<float>(r.x), y1);
        glEnd();
        glDisable(GL_BLEND);
    }

    void strokeRect(const Rect& r, int windowHeight, const Color& color, float lineWidth = 1.0f) {
        if (r.w <= 0 || r.h <= 0) {
            return;
        }
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(color.r, color.g, color.b, color.a);
        glLineWidth(lineWidth);
        const float x0 = static_cast<float>(r.x) + 0.5f;
        const float x1 = static_cast<float>(r.x + r.w) - 0.5f;
        const float y0 = static_cast<float>(windowHeight - r.y - r.h) + 0.5f;
        const float y1 = static_cast<float>(windowHeight - r.y) - 0.5f;
        glBegin(GL_LINE_LOOP);
        glVertex2f(x0, y0);
        glVertex2f(x1, y0);
        glVertex2f(x1, y1);
        glVertex2f(x0, y1);
        glEnd();
        glLineWidth(1.0f);
        glDisable(GL_BLEND);
    }

    const CameraFrame& displayFrame(int cameraIndex) const {
        if (cameraIndex < 0 || cameraIndex >= kCameraCount) {
            return frames[0];
        }
        const CameraFrame* frame = displayFrames[static_cast<std::size_t>(cameraIndex)];
        return frame != nullptr ? *frame : frames[static_cast<std::size_t>(cameraIndex)];
    }

    void uploadCameraTexture(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= kCameraCount) {
            return;
        }
        const CameraFrame& frame = displayFrame(cameraIndex);
        if (frame.width <= 0 || frame.height <= 0 || frame.rgba.empty()) {
            return;
        }
        if (textures[cameraIndex] == 0) {
            glGenTextures(1, &textures[cameraIndex]);
        }
        glBindTexture(GL_TEXTURE_2D, textures[cameraIndex]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D,
                     0,
                     GL_RGBA,
                     frame.width,
                     frame.height,
                     0,
                     GL_RGBA,
                     GL_UNSIGNED_BYTE,
                     frame.rgba.data());
    }

    void drawCameraImage(const Rect& image, int windowHeight, int cameraIndex) {
        const CameraFrame& frame = displayFrame(cameraIndex);
        if (frame.rgba.empty()) {
            fillRect(image, windowHeight, {0.0f, 0.0f, 0.0f, 1.0f});
            return;
        }

        uploadCameraTexture(cameraIndex);
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        glEnable(GL_TEXTURE_2D);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glBindTexture(GL_TEXTURE_2D, textures[cameraIndex]);
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

        const float x0 = static_cast<float>(image.x);
        const float x1 = static_cast<float>(image.x + image.w);
        const float y0 = static_cast<float>(windowHeight - image.y - image.h);
        const float y1 = static_cast<float>(windowHeight - image.y);

        // The frame data is stored top-row first. OpenGL treats the first
        // uploaded row as v=0, so v=0 is mapped to the top of the screen quad.
        glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 0.0f);
        glVertex2f(x0, y1);
        glTexCoord2f(1.0f, 0.0f);
        glVertex2f(x1, y1);
        glTexCoord2f(1.0f, 1.0f);
        glVertex2f(x1, y0);
        glTexCoord2f(0.0f, 1.0f);
        glVertex2f(x0, y0);
        glEnd();
        glDisable(GL_TEXTURE_2D);
    }

    void drawTopPane(const Layout& layout) {
        const Rect& pane = layout.topPane;
        if (pane.w <= 0 || pane.h <= 0) {
            return;
        }

        const int glY = layout.windowHeight - pane.y - pane.h;
        glViewport(pane.x, glY, pane.w, pane.h);
        glEnable(GL_SCISSOR_TEST);
        glScissor(pane.x, glY, pane.w, pane.h);
        glClearColor(0.012f, 0.018f, 0.026f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        const float fx = config.fieldHalfX;
        const float fy = config.fieldHalfY;
        const float baseW = 2.0f * fx + 0.65f;
        const float baseH = 2.0f * fy + 0.65f;
        const float paneAspect = static_cast<float>(pane.w) / static_cast<float>(pane.h);
        const float baseAspect = baseW / baseH;
        float halfW = baseW * 0.5f;
        float halfH = baseH * 0.5f;
        if (paneAspect > baseAspect) {
            halfW = halfH * paneAspect;
        } else {
            halfH = halfW / paneAspect;
        }

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-halfW, halfW, -halfH, halfH, -10.0, 10.0);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glDisable(GL_TEXTURE_2D);
        glDisable(GL_CULL_FACE);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_TRUE);

        drawArena();
        drawFovWedges();
        drawRobot3D(true);
        drawRobotInset(layout.robotInset, layout.windowHeight);

        glDisable(GL_DEPTH_TEST);
        glDisable(GL_LIGHTING);
        glDisable(GL_SCISSOR_TEST);
    }

    void drawTopPaneLabel(const Layout& layout) {
        const Rect& pane = layout.topPane;
        const int top = pane.y + 8;
        const int baseTop = top + 8;
        char labelBuffer[160];
        std::snprintf(labelBuffer,
                      sizeof(labelBuffer),
                      "TOP VIEW   |   +X right   +Y up   |   %.1f x %.1f m",
                      config.fieldHalfX * 2.0f,
                      config.fieldHalfY * 2.0f);
        const std::string label = labelBuffer;
        const int width = text.textWidth(text.small(), label) + 24;
        fillRect({pane.x + 8, top, width, 24},
                 layout.windowHeight,
                 {0.0f, 0.0f, 0.0f, 0.60f});
        fillRect({pane.x + 8, top, 4, 24}, layout.windowHeight, kUiAccent);
        text.draw(text.small(),
                  label,
                  pane.x + 20,
                  baseTop + text.small().ascent,
                  layout.windowHeight,
                  kUiText,
                  true);

        const Rect& inset = layout.robotInset;
        strokeRect(inset,
                   layout.windowHeight,
                   {kUiAccent.r, kUiAccent.g, kUiAccent.b, 0.85f},
                   1.5f);
        fillRect({inset.x, inset.y, inset.w, 22},
                 layout.windowHeight,
                 {0.0f, 0.0f, 0.0f, 0.62f});
        fillRect({inset.x, inset.y, 3, 22}, layout.windowHeight, kUiAccent);
        text.draw(text.small(),
                  car.airborne ? "ROBOT / ARM  AIR" : "ROBOT / ARM",
                  inset.x + 10,
                  inset.y + 5 + text.small().ascent,
                  layout.windowHeight,
                  kUiText,
                  true);
    }

    void drawProbeOverlay(const Layout& layout, int cameraIndex, const Rect& image) {
        if (probe.cameraIndex != cameraIndex || displayFrame(cameraIndex).rgba.empty()) {
            return;
        }
        const CameraFrame& frame = displayFrame(cameraIndex);
        if (frame.width <= 0 || frame.height <= 0 || image.w <= 0 || image.h <= 0) {
            return;
        }

        const float u = clampf(probe.imageX, 0.0f, static_cast<float>(frame.width - 1));
        const float v = clampf(probe.imageY, 0.0f, static_cast<float>(frame.height - 1));
        const int px = image.x + static_cast<int>(u / static_cast<float>(frame.width) *
                                                  static_cast<float>(image.w));
        const int py = image.y + static_cast<int>(v / static_cast<float>(frame.height) *
                                                  static_cast<float>(image.h));

        const Color color = probe.hit ? Color{0.35f, 1.0f, 0.40f, 1.0f}
                                      : Color{1.0f, 0.38f, 0.30f, 1.0f};
        const int arm = 10;
        const int glY = layout.windowHeight - py;

        glDisable(GL_TEXTURE_2D);
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(0.0f, 0.0f, 0.0f, 0.75f);
        glLineWidth(3.0f);
        glBegin(GL_LINES);
        glVertex2f(static_cast<float>(px - arm), static_cast<float>(glY));
        glVertex2f(static_cast<float>(px + arm), static_cast<float>(glY));
        glVertex2f(static_cast<float>(px), static_cast<float>(glY - arm));
        glVertex2f(static_cast<float>(px), static_cast<float>(glY + arm));
        glEnd();

        glColor4f(color.r, color.g, color.b, color.a);
        glLineWidth(1.5f);
        glBegin(GL_LINES);
        glVertex2f(static_cast<float>(px - arm), static_cast<float>(glY));
        glVertex2f(static_cast<float>(px + arm), static_cast<float>(glY));
        glVertex2f(static_cast<float>(px), static_cast<float>(glY - arm));
        glVertex2f(static_cast<float>(px), static_cast<float>(glY + arm));
        glEnd();
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 28; ++i) {
            const float a = kTwoPi * static_cast<float>(i) / 28.0f;
            glVertex2f(static_cast<float>(px) + std::cos(a) * 7.0f,
                       static_cast<float>(glY) + std::sin(a) * 7.0f);
        }
        glEnd();
        glLineWidth(1.0f);
        glDisable(GL_BLEND);

        char line1[128];
        char line2[160];
        if (probe.hit) {
            std::snprintf(line1,
                          sizeof(line1),
                          "u %.1f  v %.1f   depth %.3f m",
                          probe.imageX,
                          probe.imageY,
                          probe.depth);
            std::snprintf(line2,
                          sizeof(line2),
                          "world (%.2f, %.2f, %.2f) m",
                          probe.world.x,
                          probe.world.y,
                          probe.world.z);
        } else {
            std::snprintf(line1,
                          sizeof(line1),
                          "u %.1f  v %.1f   no hit",
                          probe.imageX,
                          probe.imageY);
            std::snprintf(line2, sizeof(line2), "sky / ray returned infinity");
        }

        const int infoW = image.w - 12;
        const int infoH = 39;
        const int infoTop = image.y + image.h - infoH - 6;
        fillRect({image.x + 6, infoTop, infoW, infoH},
                 layout.windowHeight,
                 {0.0f, 0.0f, 0.0f, 0.72f});
        fillRect({image.x + 6, infoTop, 3, infoH}, layout.windowHeight, color);
        text.draw(text.small(),
                  line1,
                  image.x + 14,
                  infoTop + 6 + text.small().ascent,
                  layout.windowHeight,
                  kUiText);
        text.draw(text.small(),
                  line2,
                  image.x + 14,
                  infoTop + 21 + text.small().ascent,
                  layout.windowHeight,
                  probe.hit ? kUiMuted : color);
    }

    void drawCameraPane(const Layout& layout, int cameraIndex) {
        const Rect& pane = layout.cameraPanes[cameraIndex];
        const Rect& image = layout.cameraImages[cameraIndex];
        const CameraFrame& frame = displayFrame(cameraIndex);

        fillRect(pane, layout.windowHeight, {0.020f, 0.027f, 0.037f, 1.0f});
        fillRect(image, layout.windowHeight, {0.0f, 0.0f, 0.0f, 1.0f});
        drawCameraImage(image, layout.windowHeight, cameraIndex);

        const Color& camColor = mounts[cameraIndex].color;
        strokeRect(image, layout.windowHeight, {camColor.r, camColor.g, camColor.b, 0.9f}, 2.0f);
        strokeRect(pane, layout.windowHeight, {0.16f, 0.20f, 0.25f, 1.0f}, 1.0f);

        std::string label = mounts[cameraIndex].name;
        label += "  |  ";
        label += std::to_string(frame.width);
        label += "x";
        label += std::to_string(frame.height);
        const std::string fovSuffix =
            "  VFOV " + std::to_string(static_cast<int>(std::lround(config.cameraFovY))) + " deg";
        if (text.textWidth(text.small(), label + fovSuffix) + 30 <= image.w) {
            label += fovSuffix;
        }

        const int labelW = std::min(image.w - 8,
                                    text.textWidth(text.small(), label) + 30);
        fillRect({image.x + 4, image.y + 4, labelW, 23},
                 layout.windowHeight,
                 {0.0f, 0.0f, 0.0f, 0.62f});
        fillRect({image.x + 4, image.y + 4, 4, 23}, layout.windowHeight, camColor);
        text.draw(text.small(),
                  label,
                  image.x + 14,
                  image.y + 8 + text.small().ascent,
                  layout.windowHeight,
                  kUiText,
                  true);

        drawProbeOverlay(layout, cameraIndex, image);
    }

    void drawHeader(const Layout& layout) {
        const int w = layout.windowWidth;
        const int hh = layout.headerHeight;

        fillRect({0, 0, w, hh}, layout.windowHeight, kHeaderBg);
        fillRect({0, hh - 2, w, 2}, layout.windowHeight, kUiAccent);

        const std::string title = "RM 3D SIMULATOR";
        char subtitleBuffer[128];
        std::snprintf(subtitleBuffer,
                      sizeof(subtitleBuffer),
                      "field %.0f x %.0f m  |  5 camera feeds",
                      config.fieldHalfX * 2.0f,
                      config.fieldHalfY * 2.0f);
        const std::string subtitle = subtitleBuffer;
        const std::string controls1 = "WASD move    Q/E rotate    R reset    Shift boost";
        const std::string controls2 = "Click a camera image to probe its pixel ray depth";

        char status1[128];
        char status2[128];
        std::snprintf(status1,
                      sizeof(status1),
                      "x %+.2f m   y %+.2f m   yaw %+.1f deg",
                      car.x,
                      car.y,
                      car.yaw * (180.0f / kPi));
        std::snprintf(status2,
                      sizeof(status2),
                      "v %.2f m/s   %.0f fps",
                      lastMeasuredSpeed,
                      fps);

        // Lay the header out from measured text widths so the three columns do
        // not overlap when the window is near its minimum size.
        const int leftBlockW = std::max(text.textWidth(text.title(), title),
                                        text.textWidth(text.small(), subtitle)) +
                               28;
        const int rightBlockW = std::max(text.textWidth(text.small(), status1),
                                         text.textWidth(text.small(), status2)) +
                                18;
        const int controlsW = std::max(text.textWidth(text.small(), controls1),
                                       text.textWidth(text.small(), controls2));
        int controlsX = leftBlockW + std::max(10,
                                              (w - leftBlockW - rightBlockW - controlsW) / 2);
        controlsX = std::min(controlsX, w - rightBlockW - controlsW - 10);
        controlsX = std::max(controlsX, leftBlockW + 8);

        text.draw(text.title(),
                  title,
                  14,
                  5 + text.title().ascent,
                  layout.windowHeight,
                  kUiText,
                  true);
        text.draw(text.small(),
                  subtitle,
                  15,
                  hh - 15,
                  layout.windowHeight,
                  kUiMuted);

        text.draw(text.small(),
                  controls1,
                  controlsX,
                  9 + text.small().ascent,
                  layout.windowHeight,
                  kUiText);
        text.draw(text.small(),
                  controls2,
                  controlsX,
                  27 + text.small().ascent,
                  layout.windowHeight,
                  kUiMuted);
        text.drawRight(text.small(),
                       status1,
                       w - 14,
                       9 + text.small().ascent,
                       layout.windowHeight,
                       kUiText);
        text.drawRight(text.small(),
                       status2,
                       w - 14,
                       27 + text.small().ascent,
                       layout.windowHeight,
                       kUiMuted);

        // Small separators keep the three header columns readable.
        if (controlsX - 14 > leftBlockW) {
            fillRect({controlsX - 14, 8, 1, hh - 18},
                     layout.windowHeight,
                     {0.16f, 0.21f, 0.27f, 1.0f});
        }
    }

    float cameraAspect() const {
        if (config.cameraHeight <= 0) {
            return 4.0f / 3.0f;
        }
        return static_cast<float>(config.cameraWidth) /
               static_cast<float>(config.cameraHeight);
    }

    float fieldAspect() const {
        const float fieldW = 2.0f * config.fieldHalfX + 0.65f;
        const float fieldH = 2.0f * config.fieldHalfY + 0.65f;
        return fieldW / std::max(0.1f, fieldH);
    }

    void renderWindowFrame(int width, int height) {
        if (width <= 0 || height <= 0) {
            return;
        }
        const Layout layout = makeLayout(width, height, cameraAspect(), fieldAspect());

        glViewport(0, 0, width, height);
        glDisable(GL_SCISSOR_TEST);
        glClearColor(0.010f, 0.014f, 0.020f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        drawTopPane(layout);

        beginUi(width, height);
        for (int i = 0; i < kCameraCount; ++i) {
            drawCameraPane(layout, i);
        }
        drawTopPaneLabel(layout);
        drawHeader(layout);

        glDisable(GL_TEXTURE_2D);
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        glDisable(GL_SCISSOR_TEST);
        glViewport(0, 0, width, height);
    }
};

// =============================================================================
//  Public math helpers
// =============================================================================

Vec3 operator+(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(const Vec3& v, float s) {
    return {v.x * s, v.y * s, v.z * s};
}

float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

float length(const Vec3& v) {
    return std::sqrt(dot(v, v));
}

Vec3 normalize(const Vec3& v) {
    const float len = length(v);
    if (len <= 1.0e-8f) {
        return {0.0f, 0.0f, 0.0f};
    }
    return v * (1.0f / len);
}

// =============================================================================
//  Simulator public methods
// =============================================================================

int headerHeightForWindow(int windowHeight) {
    return clampi(windowHeight / 15, 52, 64);
}

int sidebarWidthForWindow(int windowWidth) {
    return clampi(windowWidth / 5, 280, 340);
}

Simulator::Simulator(const SimConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

Simulator::~Simulator() = default;

void Simulator::initializeGraphics() {
    if (impl_->graphicsReady) {
        return;
    }

    HDC dc = wglGetCurrentDC();
    impl_->text.initialize(dc);

    glGenTextures(kCameraCount, impl_->textures);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glDisable(GL_DITHER);
    glDisable(GL_CULL_FACE);
    glDepthFunc(GL_LEQUAL);
    glShadeModel(GL_FLAT);

    impl_->graphicsReady = true;
}

void Simulator::reset() {
    impl_->reset();
}

void Simulator::update(const InputState& input, float dt) {
    impl_->update(input, dt);
}

void Simulator::captureCamera(int cameraIndex) {
    impl_->captureCameraFrame(cameraIndex);
}

const CameraFrame& Simulator::cameraFrame(int cameraIndex) const {
    return impl_->frames.at(static_cast<std::size_t>(cameraIndex));
}

CameraFrame& Simulator::cameraFrame(int cameraIndex) {
    return impl_->frames.at(static_cast<std::size_t>(cameraIndex));
}

void Simulator::setCameraDisplayFrame(int cameraIndex, const CameraFrame* frame) {
    if (cameraIndex < 0 || cameraIndex >= kCameraCount) {
        return;
    }
    impl_->displayFrames[static_cast<std::size_t>(cameraIndex)] =
        frame != nullptr ? frame : &impl_->frames[static_cast<std::size_t>(cameraIndex)];
}

float Simulator::cameraMountHeight() const {
    return impl_->config.cameraMountHeight;
}

void Simulator::setCameraMountHeight(float height) {
    impl_->setCameraMountHeight(height);
}

float Simulator::getRayDepth(int cameraIndex, float imageX, float imageY) const {
    float depth = std::numeric_limits<float>::infinity();
    if (!impl_->sampleRayDepth(cameraIndex, imageX, imageY, depth)) {
        return std::numeric_limits<float>::infinity();
    }
    return depth;
}

bool Simulator::getRayDepth(int cameraIndex,
                            float imageX,
                            float imageY,
                            float& outDepth) const {
    return impl_->sampleRayDepth(cameraIndex, imageX, imageY, outDepth);
}

Ray Simulator::getCameraRay(int cameraIndex, float imageX, float imageY) const {
    if (cameraIndex < 0 || cameraIndex >= kCameraCount) {
        return {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    }
    return impl_->rayForPose(impl_->capturePoses[static_cast<std::size_t>(cameraIndex)],
                            imageX,
                            imageY);
}

bool Simulator::getPixelWorld(int cameraIndex,
                              float imageX,
                              float imageY,
                              Vec3& outWorld) const {
    float depth = 0.0f;
    Ray ray;
    if (!impl_->sampleRayDepth(cameraIndex, imageX, imageY, depth, &ray)) {
        return false;
    }
    outWorld = ray.origin + ray.direction * depth;
    return true;
}

const DepthProbe& Simulator::depthProbe() const {
    return impl_->probe;
}

DepthProbe Simulator::handleMouseClick(int windowWidth,
                                       int windowHeight,
                                       int mouseX,
                                       int mouseY) {
    const SimConfig& cfg = config();
    const float cameraAspect = cfg.cameraHeight > 0
                                   ? static_cast<float>(cfg.cameraWidth) /
                                         static_cast<float>(cfg.cameraHeight)
                                   : 4.0f / 3.0f;
    const float fieldAspect = (2.0f * cfg.fieldHalfX + 0.65f) /
                              std::max(0.1f, 2.0f * cfg.fieldHalfY + 0.65f);
    const Layout layout = makeLayout(windowWidth, windowHeight, cameraAspect, fieldAspect);
    for (int i = 0; i < kCameraCount; ++i) {
        const Rect& image = layout.cameraImages[i];
        if (!image.contains(mouseX, mouseY)) {
            continue;
        }
        const CameraFrame& frame = impl_->frames[i];
        if (frame.width <= 0 || frame.height <= 0) {
            return impl_->probe;
        }

        const float u = (static_cast<float>(mouseX - image.x) /
                         static_cast<float>(std::max(1, image.w))) *
                        static_cast<float>(frame.width);
        const float v = (static_cast<float>(mouseY - image.y) /
                         static_cast<float>(std::max(1, image.h))) *
                        static_cast<float>(frame.height);

        DepthProbe result;
        result.cameraIndex = i;
        result.imageX = clampf(u, 0.0f, static_cast<float>(frame.width) - 0.001f);
        result.imageY = clampf(v, 0.0f, static_cast<float>(frame.height) - 0.001f);

        Ray ray;
        float depth = 0.0f;
        if (impl_->sampleRayDepth(i, result.imageX, result.imageY, depth, &ray)) {
            result.hit = true;
            result.depth = depth;
            result.world = ray.origin + ray.direction * depth;
        }

        impl_->probe = result;
        if (result.hit) {
            std::printf("[depth] cam %d pixel (%.1f, %.1f) -> %.4f m  world (%.3f, %.3f, %.3f)\n",
                        i,
                        result.imageX,
                        result.imageY,
                        result.depth,
                        result.world.x,
                        result.world.y,
                        result.world.z);
        } else {
            std::printf("[depth] cam %d pixel (%.1f, %.1f) -> no hit (sky)\n",
                        i,
                        result.imageX,
                        result.imageY);
        }
        std::fflush(stdout);
        return result;
    }
    return impl_->probe;
}

int Simulator::lightCount() const {
    return impl_->lightCount;
}

void Simulator::setLightCount(int count) {
    impl_->lightCount = clampi(count, 0, kMaxLights);
}

const LightSettings& Simulator::lightSettings(int index) const {
    return impl_->lights.at(static_cast<std::size_t>(index));
}

void Simulator::setLightSettings(int index, const LightSettings& settings) {
    if (index < 0 || index >= kMaxLights) {
        return;
    }
    impl_->lights[static_cast<std::size_t>(index)] = settings;
}

void Simulator::setLightPosition(int index, const Vec3& position) {
    if (index < 0 || index >= kMaxLights) {
        return;
    }
    impl_->lights[static_cast<std::size_t>(index)].position = position;
}

void Simulator::setLightIntensity(int index, float intensity) {
    if (index < 0 || index >= kMaxLights) {
        return;
    }
    impl_->lights[static_cast<std::size_t>(index)].intensity =
        std::max(0.0f, intensity);
}

void Simulator::resetLightRig() {
    impl_->resetLightRig();
}

const ArmConfig& Simulator::armConfig() const {
    return impl_->armConfig;
}

void Simulator::setArmBasePosition(const Vec3& basePosition) {
    impl_->armConfig.basePosition = basePosition;
}

void Simulator::setArmLinkLengths(float upperArmLength, float lowerArmLength) {
    impl_->armConfig.upperArmLength = clampf(upperArmLength, 0.08f, 0.8f);
    impl_->armConfig.lowerArmLength = clampf(lowerArmLength, 0.08f, 0.8f);
}

void Simulator::setGripperOpening(float openingMeters) {
    impl_->armConfig.gripperOpening = clampf(openingMeters, 0.0f, kGripperMaxOpening);
}

void Simulator::setServoLimits(float maxSpeedDeg, float maxAccelDeg) {
    impl_->armConfig.servoMaxSpeedDeg = std::max(1.0f, maxSpeedDeg);
    impl_->armConfig.servoMaxAccelDeg = std::max(1.0f, maxAccelDeg);
}

void Simulator::setArmTargetDegrees(int joint, float degrees) {
    impl_->setArmTargetRadians(joint, radians(degrees));
}

void Simulator::setArmTargetRadians(int joint, float radiansValue) {
    impl_->setArmTargetRadians(joint, radiansValue);
}

void Simulator::setArmTargetsDegrees(
    const std::array<float, kArmJointCount>& degrees) {
    for (int i = 0; i < kArmJointCount; ++i) {
        impl_->setArmTargetRadians(i, radians(degrees[static_cast<std::size_t>(i)]));
    }
}

void Simulator::setArmTargetsRadians(
    const std::array<float, kArmJointCount>& radiansValues) {
    for (int i = 0; i < kArmJointCount; ++i) {
        impl_->setArmTargetRadians(i, radiansValues[static_cast<std::size_t>(i)]);
    }
}

float Simulator::armTargetDegrees(int joint) const {
    if (joint < 0 || joint >= kArmJointCount) {
        return 0.0f;
    }
    return impl_->radToDeg(impl_->armJoints[static_cast<std::size_t>(joint)].targetRad);
}

float Simulator::armCurrentDegrees(int joint) const {
    if (joint < 0 || joint >= kArmJointCount) {
        return 0.0f;
    }
    return impl_->radToDeg(impl_->armJoints[static_cast<std::size_t>(joint)].currentRad);
}

float Simulator::armVelocityDegrees(int joint) const {
    if (joint < 0 || joint >= kArmJointCount) {
        return 0.0f;
    }
    return impl_->radToDeg(impl_->armJoints[static_cast<std::size_t>(joint)].velocityRad);
}

void Simulator::updateArm(float dt) {
    impl_->updateArm(dt);
}

void Simulator::resetArmPose() {
    impl_->resetArmPose();
}

const CarState& Simulator::car() const {
    return impl_->car;
}

bool Simulator::groundHeightAt(float x, float y, float& outHeight) const {
    const float h = impl_->sampleGround(x, y);
    if (h < -100.0f) {
        return false;
    }
    outHeight = h;
    return true;
}

void Simulator::setCarPose(float x, float y, float yaw) {
    impl_->car.x = x;
    impl_->car.y = y;
    impl_->car.yaw = std::remainder(yaw, kTwoPi);
    impl_->snapCarToTerrain();
    impl_->lastMeasuredSpeed = 0.0f;
}

void Simulator::dropCarFromHeight(float heightAboveGround) {
    float ground = 0.0f;
    if (!impl_->terrain.ready) {
        ground = 0.0f;
    } else {
        float z = 0.0f;
        float pitch = 0.0f;
        float roll = 0.0f;
        float spread = 0.0f;
        impl_->terrainPose(z, pitch, roll, spread);
        ground = z;
    }
    impl_->car.z = ground + std::max(0.0f, heightAboveGround);
    impl_->car.pitch = 0.0f;
    impl_->car.roll = 0.0f;
    impl_->car.verticalVelocity = 0.0f;
    impl_->car.airborne = true;
    impl_->updateCameraPoses();
}

bool Simulator::carModelLoaded() const {
    return impl_->carMesh.loaded;
}

const UserSwitches& Simulator::userSwitches() const {
    return impl_->userSwitches;
}

void Simulator::setUserSwitches(const UserSwitches& switches) {
    impl_->userSwitches = switches;
}

const CameraPose& Simulator::cameraPose(int cameraIndex) const {
    return impl_->poses.at(static_cast<std::size_t>(cameraIndex));
}

const CameraPose& Simulator::capturedCameraPose(int cameraIndex) const {
    return impl_->capturePoses.at(static_cast<std::size_t>(cameraIndex));
}

const CameraIntrinsics& Simulator::cameraIntrinsics(int cameraIndex) const {
    (void)cameraIndex;
    return impl_->intrinsics;
}

const SimConfig& Simulator::config() const {
    return impl_->config;
}

void Simulator::setFps(float fps) {
    impl_->fps = fps;
}

void Simulator::renderWindow(int windowWidth, int windowHeight) {
    impl_->renderWindowFrame(windowWidth, windowHeight);
}

}  // namespace sim
