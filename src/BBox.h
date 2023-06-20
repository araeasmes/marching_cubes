#pragma once

#include <glm/glm.hpp>


struct BBox {
    union {
        struct {
            glm::vec3 min;
            glm::vec3 max;
        } pt;
        glm::vec3 pts[2];
    };
};

glm::vec3 getBoxPointByMask(const BBox &box, int ix, int iy, int iz) {
    return glm::vec3(box.pts[ix].x, box.pts[iy].y, box.pts[iz].z);
}

glm::vec3 getBoxPointByIndex(const BBox &box, int i) {
    // assumes i = [0, 8)
    int x = i & 1;
    int y = (i >> 1) & 1;
    int z = (i >> 2) & 1;
    return getBoxPointByMask(box, x, y, z);
}