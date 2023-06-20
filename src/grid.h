#pragma once

#include <glm/glm.hpp>

struct Grid {
    glm::vec3 start;
    glm::vec3 step;
    glm::vec<3, int> num_steps;
};

glm::vec3 getGridPoint(const Grid &g, int x, int y, int z) {
    return g.start + g.step * glm::vec3(x, y, z);
}

