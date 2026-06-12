#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>

namespace Diamond {

enum class CameraMovement { Forward, Backward, Left, Right };

class Camera
{
public:
    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;

    float Yaw;
    float Pitch;
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;

    // Orbit/pan state — the camera sits on a sphere of radius Distance around
    // Pivot. Input writes the Target* values; UpdateOrbit eases the live state
    // toward them each frame so orbit, pan, dolly, and focus all glide.
    glm::vec3 Pivot          { 0.0f, 0.0f, 0.0f };
    float     Distance         = 5.0f;
    float     OrbitSensitivity = 0.25f;  // degrees per pixel of mouse drag
    float     Smoothing        = 12.0f;  // exponential ease rate; higher = snappier

    float     TargetYaw      = 0.0f;
    float     TargetPitch    = 0.0f;
    float     TargetDistance = 5.0f;
    glm::vec3 TargetPivot    { 0.0f, 0.0f, 0.0f };

    Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 3.0f),
           glm::vec3 up       = glm::vec3(0.0f, 1.0f, 0.0f),
           float yaw = -90.0f, float pitch = 0.0f)
        : Front(glm::vec3(0.0f, 0.0f, -1.0f)),
          MovementSpeed(2.5f), MouseSensitivity(0.1f), Zoom(45.0f)
    {
        Position = position;
        WorldUp  = up;
        Yaw      = yaw;
        Pitch    = pitch;
        UpdateVectors();
        SyncOrbitTargets();
    }

    glm::mat4 GetViewMatrix() const
    {
        return glm::lookAt(Position, Position + Front, Up);
    }

    void ProcessKeyboard(CameraMovement direction, float deltaTime)
    {
        float v = MovementSpeed * deltaTime;
        if (direction == CameraMovement::Forward)  Position += Front * v;
        if (direction == CameraMovement::Backward) Position -= Front * v;
        if (direction == CameraMovement::Left)     Position -= Right * v;
        if (direction == CameraMovement::Right)    Position += Right * v;
    }

    void ProcessMouseMovement(float xOffset, float yOffset, bool constrainPitch = true)
    {
        Yaw   += xOffset * MouseSensitivity;
        Pitch += yOffset * MouseSensitivity;
        if (constrainPitch)
            Pitch = glm::clamp(Pitch, -89.0f, 89.0f);
        UpdateVectors();
    }

    void ProcessMouseScroll(float yOffset)
    {
        Zoom = glm::clamp(Zoom - yOffset, 1.0f, 45.0f);
    }

    void LookAt(glm::vec3 target)
    {
        glm::vec3 dir = glm::normalize(target - Position);
        Pitch = glm::degrees(std::asin(glm::clamp(dir.y, -1.0f, 1.0f)));
        Yaw   = glm::degrees(std::atan2(dir.z, dir.x));
        UpdateVectors();
    }

    // ---- Orbit / pan / dolly (editor camera) --------------------------------

    // Re-anchors the orbit sphere to the camera's current transform. Call after
    // moving the camera directly (fly mode, LookAt) so orbit/pan resume from
    // where the camera is now instead of easing back toward stale targets.
    void SyncOrbitTargets()
    {
        Pivot          = Position + Front * Distance;
        TargetYaw      = Yaw;
        TargetPitch    = Pitch;
        TargetDistance = Distance;
        TargetPivot    = Pivot;
    }

    // dx/dy in pixels of mouse drag. Dragging down moves the camera below the
    // pivot (looking up); dragging right swings it around to the left.
    void Orbit(float dx, float dy)
    {
        TargetYaw  += dx * OrbitSensitivity;
        TargetPitch = glm::clamp(TargetPitch + dy * OrbitSensitivity, -89.0f, 89.0f);
    }

    // Slides camera and pivot together parallel to the view plane. Scaled so a
    // point at pivot depth tracks the cursor 1:1 regardless of zoom level.
    void Pan(float dx, float dy, float viewportHeight)
    {
        float worldPerPixel = 2.0f * TargetDistance * std::tan(glm::radians(Zoom) * 0.5f)
                            / glm::max(viewportHeight, 1.0f);
        TargetPivot += (-Right * dx + Up * dy) * worldPerPixel;
    }

    // Scroll-wheel dolly — multiplicative so each notch feels equal at any range.
    void Dolly(float wheel)
    {
        TargetDistance = glm::clamp(TargetDistance * std::pow(0.9f, wheel), 0.1f, 500.0f);
    }

    // Frame a point at the given range, keeping the current view direction.
    void Focus(const glm::vec3& center, float distance)
    {
        TargetPivot    = center;
        TargetDistance = glm::max(distance, 0.1f);
    }

    // Eases live state toward targets (framerate-independent) and places the
    // camera on its orbit sphere. Skip while fly mode is driving the camera.
    void UpdateOrbit(float dt)
    {
        float t = 1.0f - std::exp(-Smoothing * dt);
        Yaw      += (TargetYaw      - Yaw)      * t;
        Pitch    += (TargetPitch    - Pitch)    * t;
        Distance += (TargetDistance - Distance) * t;
        Pivot    += (TargetPivot    - Pivot)    * t;
        UpdateVectors();
        Position = Pivot - Front * Distance;
    }

private:
    void UpdateVectors()
    {
        glm::vec3 front;
        front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
        front.y = sin(glm::radians(Pitch));
        front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
        Front = glm::normalize(front);
        Right = glm::normalize(glm::cross(Front, WorldUp));
        Up    = glm::normalize(glm::cross(Right, Front));
    }
};

} // namespace Diamond