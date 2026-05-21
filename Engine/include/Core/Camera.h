#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

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