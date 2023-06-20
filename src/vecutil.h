#pragma once

#include <glm/glm.hpp>

#define SQR(x) ((x) * (x))
 
glm::vec3 orthoAxis(const glm::vec3 &v) {
    glm::vec3 av = glm::abs(v);
    return (av.x > av.y ? (av.y > av.z ? glm::vec3{0.0f, 0.0f, 1.0f} : glm::vec3{0.0f, 1.0f, 0.0f}) : 
            (av.x > av.z ? glm::vec3{0.0f, 0.0f, 1.0f} : glm::vec3{1.0f, 0.0f, 0.0f}));
}

glm::vec3 orthoVector(const glm::vec3 &v) {
	return glm::cross(v, orthoAxis(v));
}

