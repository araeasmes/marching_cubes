#pragma once

#include <glm/glm.hpp>

#include "BBox.h"
#include "vecutil.h"

typedef float (*sdf_fun_type) (const glm::vec3& pt, void *);

struct SdfSphere {
    glm::vec3 c;
    float rad;
    float t;
};

struct SdfSine {
    float t;
};

float smoothUnion(float a, float b, float k) {
    float h = glm::clamp(0.5f + 0.5f * (b - a) / k, 0.0f, 1.0f);
    return glm::mix(b, a, h) - k * h * (1.0 - h);
}

float smoothSubtraction(float a, float b, float k) {
    float h = glm::clamp(0.5f - 0.5f * (b + a) / k, 0.0f, 1.0f);
    return glm::mix(b, -a, h) + k * h * (1.0f - h);
}

float sdfSphere(const glm::vec3 &pt, void *data) {
    SdfSphere *sph = (SdfSphere*) data;
    glm::vec3 r = pt - sph->c;
    glm::vec3 r2 = r - glm::vec3(-2.0f, 0.0f, 0.0f);
    float l = glm::length(r) - sph->rad;
    float l2 = glm::length(r2) - sph->rad;
    float add = glm::length(glm::sin(r * 2.0f * glm::sin(sph->t * 0.03f)));
    return l + add * 0.3f;
}

float sdfSine(const glm::vec3 &pt, void *data) {
    SdfSine *s = (SdfSine*) data;
    return 0.5f * glm::sin(2.0f * pt.x * glm::cos(pt.z + s->t * 0.05) + s->t * 0.03f) - pt.y;
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
