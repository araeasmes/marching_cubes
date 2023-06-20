#pragma once

#include <glm/glm.hpp>

#include "BBox.h"

typedef float (*sdf_fun_type) (const glm::vec3& pt, void *);

struct SdfSphere {
    glm::vec3 c;
    float rad;
    float t;
};

float sdfSphere(const glm::vec3& pt, void* data) {
    SdfSphere *sph = (SdfSphere*) data;
    glm::vec3 r = pt - sph->c;
    float add = glm::length(glm::sin(r * 2.0f * glm::sin(sph->t * 0.03f)));
    return glm::length(r) - sph->rad + add;
}

void fillSDFSine(float pts_sdf[8], const BBox &box) {
    for (int i = 0; i < 8; ++i) {
        glm::vec3 pt = getBoxPointByIndex(box, i);
        pts_sdf[i] = 0.5 * sin(2 * pt.x * pt.z) - pt.y;
    }
}

void fillSDFSphere(float pts_sdf[8], const BBox &box) {
    for (int i = 0; i < 8; ++i) {
            glm::vec3 pt = getBoxPointByIndex(box, i);
            pts_sdf[i] = glm::distance(pt, glm::vec3(0.0f, 0.0f, 0.0f)) - 2.0f;
        }
}
