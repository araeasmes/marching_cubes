#include <algorithm>
#include <iostream>
#include <cstdint>

#define RLIGHTS_IMPLEMENTATION

#include "raylib.h"
#include "raymath.h"
#include "rcamera.h"
#include "rlights.h"
#include "rlgl.h"
#include "glm/glm.hpp"
#include <glm/gtx/string_cast.hpp>

#define PLATFORM_DESKTOP

#if defined(PLATFORM_DESKTOP)
    #define GLSL_VERSION            330
#else   // PLATFORM_RPI, PLATFORM_ANDROID, PLATFORM_WEB
    #define GLSL_VERSION            100
#endif


glm::vec3 orthoAxis(const glm::vec3 &v) {
    glm::vec3 av = glm::abs(v);
    return (av.x > av.y ? (av.y > av.z ? glm::vec3{0.0f, 0.0f, 1.0f} : glm::vec3{0.0f, 1.0f, 0.0f}) : 
            (av.x > av.z ? glm::vec3{0.0f, 0.0f, 1.0f} : glm::vec3{1.0f, 0.0f, 0.0f}));
}

glm::vec3 orthoVector(const glm::vec3 &v) {
	return glm::cross(v, orthoAxis(v));
}

struct BBox {
    union {
        struct {
            glm::vec3 min;
            glm::vec3 max;
        } pt;
        glm::vec3 pts[2];
    };
};

float plane_dist(const glm::vec3 &pt, const glm::vec3 &n, float d) {
    return glm::dot(pt, n) + d; 
}

struct MarchResult { 
    float sdf[8];
    int ind = -1;
    int num_verts = 0;
    int num_inds = 0;
    int indices[12 * 3];
    glm::vec3 verts[12];
};

struct RLCube {
    Vector3 pos;
    Vector3 dims;
};

void initMarchResult(MarchResult &out) {
    out.ind = -1;
    out.num_verts = 0;
    out.num_inds = 0;
    for (int i = 0; i < 12 * 3; ++i) {
        out.indices[i] = -1;
    } 
    for (int i = 0; i < 8; ++i)
        out.sdf[i] = 0.0f;
}

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

void fillPlaneSDF(float pts_sdf[8], const BBox &box, const glm::vec3 &plane_normal, float plane_d) {
    int table_ind = 0;

    for (int x = 0; x < 2; ++x) {
        for (int y = 0; y < 2; ++y) {
            for (int z = 0; z < 2; ++z) {
                int xyz_ind = x + y * 2 + z * 4;
                glm::vec3 pt = getBoxPointByMask(box, x, y, z);
                float sdf = plane_dist(pt, plane_normal, plane_d);
                pts_sdf[xyz_ind] = sdf;
                
                int i = (int) (sdf > 0);
                table_ind |= i << xyz_ind;
            }
        }
    }
}

void fillSDFByIndex(float pts_sdf[8], int ind) {
    for (int i = 0; i < 8; ++i) {
        pts_sdf[i] = (ind & 1) ? 0.5f : -0.5f;
        ind >>= 1;
    }
}

void fillSDFSine(float pts_sdf[8], const BBox &box) {
    for (int i = 0; i < 8; ++i) {
        glm::vec3 pt = getBoxPointByIndex(box, i);
        pts_sdf[i] = 0.5 * sin(2 * pt.x * pt.z) - pt.y;
    }
}

int edge_mask(int p1, int p2) {
    // p1, p2 = [0, 8)
    return ~(p1 ^ p2);
}

int edge_index(int p1, int p2) {
    int mask = ~(p1 ^ p2);
    int tmp = (mask & p1);
    int edge_group = 0;
    int edge_ingroup = 0;
    for (int j = 2; j >= 0; --j) {
        if ((~mask) & (1 << j)) {
            edge_group = j;
        }
        if (mask & (1 << j)) {
            edge_ingroup <<= 1;
            edge_ingroup |= 1 & (tmp >> j);
        }
    }

    // std::cout << "(" << p1 << ", " << p2 << ") => group = " << edge_group << ", ingroup = " << edge_ingroup << "\n"; 

    return edge_group * 4 + edge_ingroup;
}

// j = [0, 3) represents the direction
int edge_index_from_pt_dir(int p, int j) {
    int mask = (~0) ^ (1 << j);
    int tmp = mask & p;
    int edge_group = j;
    int edge_ingroup = 0;
    for (int i = 2; i >= 0; --i) {
        if (i == j)
            continue;
        edge_ingroup <<= 1;
        edge_ingroup |= 1 & (tmp >> i);
    }
    return edge_group * 4 + edge_ingroup;
}

void edge_to_points(int &p1, int &p2, int e) {
    int edge_dir = e / 4;
    int edge_ingroup = e % 4;
    p1 = 0;
    p2 = 0;
    int ei = 1;
    for (int j = 2; j >= 0; --j) {
        p1 <<= 1;
        if (edge_dir != j) {
            p1 |= (edge_ingroup >> ei) & 1;  
            ei--;
        }
    }
    p2 = p1 ^ (1 << edge_dir);
}

void march(MarchResult &out, float pts_sdf[8], const BBox &box) {
    initMarchResult(out);

    int visited[8] = {0};
    int queue[8] = {-1};
    int queue_i = 0;
    int queue_end = 0;
    
    out.ind = 0;
    for (int i = 0; i < 8; ++i) {
        out.ind <<= 1;
        out.ind |= (pts_sdf[i] >= 0.0f);
        out.sdf[i] = pts_sdf[i];
    }
    
    for (int i = 0; i < 8; ++i) {
        if (visited[i] == 2)
            continue;

        visited[i] = 2;
        if (pts_sdf[i] < 0)
            continue;

        int comp_ind = 0;

        int comp_edge_inds[12];
        std::fill(comp_edge_inds, comp_edge_inds + 12, -1);
        bool edge_dirs[12];

        queue[queue_end++] = i;
        while (queue_i != queue_end) {
            int ind = queue[queue_i++];

            for (int j = 0; j < 3; ++j) {
                int k = ind ^ (1 << j);
                
                if (pts_sdf[k] >= 0) {
                    if (visited[k] == 0) {
                        queue[queue_end++] = k;
                        visited[k] = 1;
                    }
                } else {
                    float sdf_dif = pts_sdf[ind] - pts_sdf[k];
                    float t = 0.5f;
                    if (fabsf(sdf_dif) > FLT_EPSILON) 
                        t = pts_sdf[ind] / sdf_dif;

                    glm::vec3 pt_i = getBoxPointByIndex(box, ind);
                    glm::vec3 pt_k = getBoxPointByIndex(box, k);
                    glm::vec3 pt = (1.0f - t) * pt_i + pt_k * t;
                    out.verts[out.num_verts] = pt;

                    int edge_ind = edge_index(ind, k);
                    comp_edge_inds[edge_ind] = out.num_verts;
                    edge_dirs[edge_ind] = ind < k;

                    comp_ind++;
                    out.num_verts++;
                }
                visited[ind] = 2;
            }
        }
        
        if (comp_ind < 3)
            continue;
        
        int edge_neighbors[12][6];
        int edge_n_cnt[12];
        std::fill(edge_n_cnt, edge_n_cnt + 12, 0);
        
        for (int e = 0; e < 12; ++e) {
            if (comp_edge_inds[e] == -1)
                continue;

            int edge_dir = e / 4;
            int edge_ingroup = e % 4;

            int p1 = 0;
            int p2 = 0;
            edge_to_points(p1, p2, e);
            int dir = edge_dirs[e];
            
            int js[2];
            int js_cnt = 0;
            
            for (int j = 0; j < 3; ++j) {
                if (j == edge_dir)
                    continue;
                dir ^= (p1 >> j) & 1;
                js[js_cnt++] = j;
            }
            dir ^= (edge_dir == 1);
            if (dir) {
                std::swap(js[0], js[1]);
            }

            for (int ij = 0; ij < 2; ++ij) {
                int j = js[ij];
                int p3 = p1 ^ (1 << j);
                int p4 = p2 ^ (1 << j);
                int tmp_e;
                tmp_e = edge_index(p1, p3);
                if (comp_edge_inds[tmp_e] != -1) {
                    edge_neighbors[e][edge_n_cnt[e]] = tmp_e;    
                    edge_n_cnt[e]++;
                }
                tmp_e = edge_index(p3, p4);
                if (comp_edge_inds[tmp_e] != -1) {
                    edge_neighbors[e][edge_n_cnt[e]] = tmp_e;    
                    edge_n_cnt[e]++;
                }
                tmp_e = edge_index(p2, p4);
                if (comp_edge_inds[tmp_e] != -1) {
                    edge_neighbors[e][edge_n_cnt[e]] = tmp_e;    
                    edge_n_cnt[e]++;
                }
                std::swap(p1, p2);
            }
        }

        int connections_left[12];
        std::fill(connections_left, connections_left + 12, 2);
        int component_cnt = 0;
        int edge_component[12];
        std::fill(edge_component, edge_component + 12, -1);

        // 6 and 12 are kinda arbitrary here
        int components[6][12];
        int components_size[6];
        std::fill(components_size, components_size + 6, 0);

        int visited_edges[12];
        std::fill(visited_edges, visited_edges + 12, 0);

        // find the "seeding" edges - ones which have only 2 neighbors
        for (int e = 0; e < 12; ++e) {
            if (comp_edge_inds[e] == -1)
                continue;
            if (connections_left[e] != 2)
                continue;
            if (edge_n_cnt[e] != 2)
                continue;
            // here connections_left[e] == edge_n_cnt[e]

            int comp = component_cnt++;

            int edge_st[12];
            int edge_st_size = 1;
            edge_st[0] = e;
            while(edge_st_size) {
                int cur_e = edge_st[--edge_st_size]; 
                visited_edges[cur_e] = 1;
                edge_component[cur_e] = comp;
                components[comp][components_size[comp]++] = cur_e;

                if (connections_left[cur_e] != edge_n_cnt[cur_e])
                    continue;

                for (int j = 0; j < edge_n_cnt[cur_e]; ++j) {
                    int next_e = edge_neighbors[cur_e][j]; 
                    connections_left[cur_e]--;

                    if (!visited_edges[next_e]) {
                        edge_st[edge_st_size++] = next_e;
                        visited_edges[next_e] = 1;
                    }

                    for (int k = 0; k < edge_n_cnt[next_e]; ++k) {
                        if (edge_neighbors[next_e][k] == cur_e) {
                            edge_n_cnt[next_e]--;
                            std::swap(edge_neighbors[next_e][k], edge_neighbors[next_e][edge_n_cnt[next_e]]);
                            break;
                        }
                    }
                    connections_left[next_e]--;
                }
                edge_n_cnt[cur_e] = 0;
            }
        }
        
        for (int j = 0; j < component_cnt; ++j) {
            if (components_size[j] < 3) {
                std::cout << "ALERTA, SMALL COMPONENTS\n";
            }
            // reorder edges if there was an edge such that it had more than 2 connections,
            // but we added it in the middle of the component - such edges should be at the start and end of the array
            int left = 0;
            int right = components_size[j] - 1;

            while (left < components_size[j] && edge_n_cnt[components[j][left]] == 0)
                left++;
            while (right >= 0 && edge_n_cnt[components[j][right]] == 0)
                right--;

            if (right >= 0 && left < components_size[j] &&
                    left != 0 && right != components_size[j] - 1) {
                std::cerr << "ALERTA, BOTH ENDS ARE IN THE WRONG PLACE\n";
            }

            if (left != 0 && left < components_size[j]) {
                std::reverse(components[j], components[j] + left + 1);
                std::reverse(components[j], components[j] + components_size[j]);
            } else if (right != components_size[j] - 1 && right >= 0) {
                std::reverse(components[j] + right, components[j] + components_size[j]);
                std::reverse(components[j], components[j] + components_size[j]);
            }

            int e1 = components[j][0];
            for (int k = 2; k < components_size[j]; ++k) {
                int e2 = components[j][k - 1];
                int e3 = components[j][k];
                out.indices[out.num_inds++] = comp_edge_inds[e1];
                out.indices[out.num_inds++] = comp_edge_inds[e2];
                out.indices[out.num_inds++] = comp_edge_inds[e3];
            }

        }

        // connect leftovers 
        for (int j = 0; j < component_cnt; ++j) {
            int last = components_size[j] - 1;
            int ledge = components[j][0];
            int redge = components[j][last];
            int left_cnt = edge_n_cnt[ledge]; 
            int right_cnt = edge_n_cnt[redge];

            // shouldn't happen:
            if ((left_cnt > 0) ^ (right_cnt > 0))
                std::cout << "not both???\n";

            if (left_cnt && right_cnt) {
#ifdef PRINT_STUFF
                std::cout << left_cnt << " - " << right_cnt << "\n"; 
                std::cout << "left edge = " << ledge << "\n";
                for (int k = 0; k < left_cnt; ++k) {
                    std::cout << edge_neighbors[ledge][k] << " ";
                }
                std::cout << std::endl;

                std::cout << "right edge = " << redge << "\n";
                for (int k = 0; k < right_cnt; ++k) {
                    std::cout << edge_neighbors[redge][k] << " ";
                }
                std::cout << std::endl;
#endif
                for (int k = 0; k < left_cnt; ++k) {
                    if (edge_neighbors[ledge][k] == redge) {
                        std::swap(edge_neighbors[ledge][k], edge_neighbors[ledge][left_cnt - 1]);
                        left_cnt--;
                        break;
                    }
                }
                for (int k = 0; k < right_cnt; ++k) {
                    if (edge_neighbors[redge][k] == ledge) {
                        std::swap(edge_neighbors[redge][k], edge_neighbors[redge][right_cnt - 1]);
                        right_cnt--;
                        break;
                    }
                }

                if (left_cnt != 2) {
                    std::cerr << "NOT TWO LEFTOVERS, ALERT!\n";
                    continue;
                }

                int a = ledge;
                int b = redge;
                int c = edge_neighbors[ledge][0];
                int d = edge_neighbors[ledge][1];

                // add abc, cbd triangles
                // swap c and d if the triangles generate normals in different directions
                // which is equivalent to crossing each other instead of creating a quad 

                int ia = comp_edge_inds[a];
                int ib = comp_edge_inds[b];
                int ic = comp_edge_inds[c];
                int id = comp_edge_inds[d];


                const glm::vec3 &pt_a = out.verts[ia];
                const glm::vec3 &pt_b = out.verts[ib];
                const glm::vec3 &pt_c = out.verts[ic];
                const glm::vec3 &pt_d = out.verts[id];

                glm::vec3 abc = glm::cross(pt_b - pt_a, pt_c - pt_a);
                glm::vec3 cbd = glm::cross(pt_c - pt_d, pt_b - pt_d);
                if (glm::dot(abc, cbd) < 0) {
                    std::swap(c, d);
                    std::swap(ic, id);
                }

                out.indices[out.num_inds++] = ia;
                out.indices[out.num_inds++] = ib;
                out.indices[out.num_inds++] = ic;

                out.indices[out.num_inds++] = ic;
                out.indices[out.num_inds++] = ib;
                out.indices[out.num_inds++] = id;
            }


        } 
    }
}

MarchResult all_marches[256];

void genMarches(const BBox boxes[256]) {
    float pt_sdf[8];
    for (int ind = 0; ind < 256; ++ind) {
        fillSDFByIndex(pt_sdf, ind); 
        march(all_marches[ind], pt_sdf, boxes[ind]);
    }
}

Mesh genFromMarch(const MarchResult &res) {
    Mesh mesh = {};
    mesh.vertexCount = 12;
    mesh.triangleCount = 12;

    mesh.vertices = (float *)MemAlloc(mesh.vertexCount*3*sizeof(float));    // 3 vertices, 3 coordinates each (x, y, z)
    mesh.texcoords = (float *)MemAlloc(mesh.vertexCount*2*sizeof(float));   // 3 vertices, 2 coordinates each (x, y)
    mesh.normals = (float *)MemAlloc(mesh.vertexCount*3*sizeof(float));     // 3 vertices, 3 coordinates each (x, y, z)
    mesh.indices = (unsigned short *)MemAlloc(mesh.triangleCount * 3 * sizeof(unsigned short));

    for (int i = 0; i < res.num_verts; ++i) {
        mesh.vertices[i * 3 + 0] = res.verts[i].x;        
        mesh.vertices[i * 3 + 1] = res.verts[i].y;        
        mesh.vertices[i * 3 + 2] = res.verts[i].z;        

        mesh.normals[i * 3 + 0] = 0.0f;
        mesh.normals[i * 3 + 1] = 0.0f;
        mesh.normals[i * 3 + 2] = 0.0f;
        mesh.texcoords[i * 2 + 0] = 0.0f;
        mesh.texcoords[i * 2 + 1] = 0.0f;
    }

    for (int i = 0; i < res.num_inds; ++i) {
        mesh.indices[i] = res.indices[i];
    }
    for (int i = 0; i < res.num_inds; i += 3) {
        int ia = res.indices[i + 0];
        int ib = res.indices[i + 1];
        int ic = res.indices[i + 2];
        glm::vec3 normal = glm::cross(res.verts[ib] - res.verts[ia], res.verts[ic] - res.verts[ia]);
        normal = glm::normalize(normal);
        mesh.normals[ia * 3 + 0] = normal.x;
        mesh.normals[ia * 3 + 1] = normal.y;
        mesh.normals[ia * 3 + 2] = normal.z;

        mesh.normals[ib * 3 + 0] = normal.x;
        mesh.normals[ib * 3 + 1] = normal.y;
        mesh.normals[ib * 3 + 2] = normal.z;

        mesh.normals[ic * 3 + 0] = normal.x;
        mesh.normals[ic * 3 + 1] = normal.y;
        mesh.normals[ic * 3 + 2] = normal.z;
    }

    for (int i = res.num_inds; i < 12 * 3; ++i) {
        mesh.indices[i] = 0;
    }
    
    UploadMesh(&mesh, true);

    return mesh;
}

void updateMeshFromMarch(Mesh &mesh, const MarchResult &res) {
    for (int i = 0; i < res.num_verts; ++i) {
        mesh.vertices[i * 3 + 0] = res.verts[i].x;        
        mesh.vertices[i * 3 + 1] = res.verts[i].y;        
        mesh.vertices[i * 3 + 2] = res.verts[i].z;        

        mesh.normals[i * 3 + 0] = 0.0f;
        mesh.normals[i * 3 + 1] = 0.0f;
        mesh.normals[i * 3 + 2] = 0.0f;
        mesh.texcoords[i * 2 + 0] = 0.0f;
        mesh.texcoords[i * 2 + 1] = 0.0f;
    }

    for (int i = 0; i < res.num_inds; ++i) {
        mesh.indices[i] = res.indices[i];
    }
    for (int i = res.num_inds; i < 12 * 3; ++i) {
        mesh.indices[i] = 0;
    }

    // 0// Vertex buffer: positions
    // 1// Vertex buffer: texcoords
    // 2// Vertex buffer: normals
    // 3// Vertex buffer: colors
    // 4// Vertex buffer: tangents
    // 5// Vertex buffer: texcoords2
    // 6// Vertex buffer: indices

    UpdateMeshBuffer(mesh, 0, mesh.vertices, sizeof(float) * 12 * 3, 0);
    UpdateMeshBuffer(mesh, 6, mesh.indices, sizeof(unsigned short) * 12 * 3, 0);
    UpdateMeshBuffer(mesh, 2, mesh.normals, sizeof(float) * 12 * 3, 0);
    UpdateMeshBuffer(mesh, 5, mesh.texcoords, sizeof(float) * 12 * 2, 0);
}


Mesh genPlane(glm::vec3 pos, float extents, glm::vec3 planeNorm) {
    Mesh mesh = {};
    mesh.triangleCount = 2;
    mesh.vertexCount = 4;

    mesh.vertices = (float *)MemAlloc(mesh.vertexCount*3*sizeof(float));    // 3 vertices, 3 coordinates each (x, y, z)
    mesh.texcoords = (float *)MemAlloc(mesh.vertexCount*2*sizeof(float));   // 3 vertices, 2 coordinates each (x, y)
    mesh.normals = (float *)MemAlloc(mesh.vertexCount*3*sizeof(float));     // 3 vertices, 3 coordinates each (x, y, z)
    mesh.indices = (unsigned short *)MemAlloc(mesh.triangleCount * 3 * sizeof(unsigned short));

    glm::vec3 axes[2];
    axes[0] = orthoVector(planeNorm);
    axes[1] = glm::cross(axes[0], planeNorm);
    float signs[2] = {-1.0f, 1.0f};

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            glm::vec3 p = pos + extents * (signs[i] * axes[0] + signs[j] * axes[1]);
            int ind = i * 2 + j;
            mesh.vertices[ind * 3 + 0] = p.x;
            mesh.vertices[ind * 3 + 1] = p.y;
            mesh.vertices[ind * 3 + 2] = p.z;

            mesh.normals[ind * 3 + 0] = planeNorm.x;
            mesh.normals[ind * 3 + 1] = planeNorm.y;
            mesh.normals[ind * 3 + 2] = planeNorm.z;
            mesh.texcoords[ind * 2 + 0] = 0;
            mesh.texcoords[ind * 2 + 1] = 0;
        }
    }
    
    mesh.indices[0] = 0;
    mesh.indices[1] = 1;
    mesh.indices[2] = 2;
    mesh.indices[3] = 2;
    mesh.indices[4] = 1;
    mesh.indices[5] = 3;

    UploadMesh(&mesh, false);

    return mesh;
}

static Mesh genMesh() {
    Mesh mesh = {};
    mesh.triangleCount = 1;
    mesh.vertexCount = 3;
    mesh.vertices = (float *)MemAlloc(mesh.vertexCount*3*sizeof(float));    // 3 vertices, 3 coordinates each (x, y, z)
    mesh.texcoords = (float *)MemAlloc(mesh.vertexCount*2*sizeof(float));   // 3 vertices, 2 coordinates each (x, y)
    mesh.normals = (float *)MemAlloc(mesh.vertexCount*3*sizeof(float));     // 3 vertices, 3 coordinates each (x, y, z)

    // Vertex at (0, 0, 0)
    mesh.vertices[0] = 0;
    mesh.vertices[1] = 0;
    mesh.vertices[2] = 0;
    mesh.normals[0] = 0;
    mesh.normals[1] = 1;
    mesh.normals[2] = 0;
    mesh.texcoords[0] = 0;
    mesh.texcoords[1] = 0;

    // Vertex at (1, 0, 2)
    mesh.vertices[3] = 1;
    mesh.vertices[4] = 0;
    mesh.vertices[5] = 2;
    mesh.normals[3] = 0;
    mesh.normals[4] = 1;
    mesh.normals[5] = 0;
    mesh.texcoords[2] = 0.5f;
    mesh.texcoords[3] = 1.0f;

    // Vertex at (2, 0, 0)
    mesh.vertices[6] = 2;
    mesh.vertices[7] = 0;
    mesh.vertices[8] = 0;
    mesh.normals[6] = 0;
    mesh.normals[7] = 1;
    mesh.normals[8] = 0;
    mesh.texcoords[4] = 1;
    mesh.texcoords[5] = 0;

    // Upload mesh data from CPU (RAM) to GPU (VRAM) memory
    UploadMesh(&mesh, false);

    return mesh;
}

// #define ALL_BOXES_TEST

int main(void) {
    const int screenWidth = 1280;
    const int screenHeight = 720;

    SetTraceLogLevel(LOG_ERROR);
    InitWindow(screenWidth, screenHeight, "raylib marching cubes stuff");

    BBox box({glm::vec3(-2.0f, 1.0f, -2.0f), glm::vec3(2.0f, 5.0f, 2.0f)});

    glm::vec3 box_c = (box.pt.min + box.pt.max) * 0.5f;
    glm::vec3 box_dif = box.pt.max - box.pt.min;
    Vector3 box_pos = {box_c.x, box_c.y, box_c.z};
    Vector3 box_dims = {box_dif.x, box_dif.y, box_dif.z}; 

    rlDisableBackfaceCulling();
    Quaternion q_model = {0.0f, 0.0f, 0.0f, 1.0f};
    Vector2 mouse_delta;

    MarchResult march_res;
    float march_sdf[8];

    constexpr int XBOXES = 16;
#ifdef ALL_BOXES_TEST
    constexpr int YBOXES = 1;
#else
    constexpr int YBOXES = 6;
#endif
    constexpr int ZBOXES = 16;

    BBox all_boxes[XBOXES * YBOXES * ZBOXES];
    RLCube rl_boxes[XBOXES * YBOXES * ZBOXES];

    float box_side = 0.25f;
    float box_spacing = 0.0f;

#ifdef ALL_BOXES_TEST
    box_side = 2;
    box_spacing = 2;
#endif

    float box_start_x = 0.0f;
    float box_start_y = YBOXES / 2 * -box_side;
    float box_start_z = 0.0f;

    glm::vec3 box_dim(0.5 * box_side);

    float cur_box_x = box_start_x;
    for (int x = 0; x < XBOXES; ++x) {
        float cur_box_y = box_start_y;
        for (int y = 0; y < YBOXES; ++y) {
            float cur_box_z = box_start_z;
            for (int z = 0; z < ZBOXES; ++z) {
                int ind = x * YBOXES * ZBOXES + y * ZBOXES + z;
                glm::vec3 box_pt(cur_box_x, cur_box_y, cur_box_z);
                all_boxes[ind].pt.min = box_pt - box_dim;
                all_boxes[ind].pt.max = box_pt + box_dim;
                rl_boxes[ind].pos = {box_pt.x, box_pt.y, box_pt.z};
                rl_boxes[ind].dims = { box_side, box_side, box_side};

                cur_box_z += box_side + box_spacing;
            }
            cur_box_y += box_side + box_spacing;
        }
        cur_box_x += box_side + box_spacing;
    }

    MarchResult sine_marches[XBOXES * YBOXES * ZBOXES];
    Mesh march_meshes[XBOXES * YBOXES * ZBOXES];

#ifdef ALL_BOXES_TEST
    for (int i = 0; i < 256; ++i) {
        fillSDFByIndex(march_sdf, i);
        march(all_marches[i], march_sdf, all_boxes[i]);
        march_meshes[i] = genFromMarch(all_marches[i]);
    }
#else
    for (int i = 0; i < XBOXES * YBOXES * ZBOXES; ++i) {
        fillSDFSine(march_sdf, all_boxes[i]);
        march(sine_marches[i], march_sdf, all_boxes[i]);
        march_meshes[i] = genFromMarch(sine_marches[i]);
    }
#endif

    // Load basic lighting shader
    Shader shader = LoadShader(TextFormat("resources/shaders/glsl%i/lighting.vs", GLSL_VERSION),
                               TextFormat("resources/shaders/glsl%i/lighting.fs", GLSL_VERSION));
    // Get some required shader locations
    shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");
    // NOTE: "matModel" location name is automatically assigned on shader loading, 
    // no need to get the location again if using that uniform name
    //shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(shader, "matModel");
    
    // Ambient light level (some basic lighting)
    int ambientLoc = GetShaderLocation(shader, "ambient");
    float ambient_arr[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
    SetShaderValue(shader, ambientLoc, ambient_arr, SHADER_UNIFORM_VEC4);
    shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");
    Light lights[MAX_LIGHTS] = { 0 };
    lights[0] = CreateLight(LIGHT_DIRECTIONAL, (Vector3){ -2, 1, -2 }, Vector3Zero(), YELLOW, shader);
    lights[1] = CreateLight(LIGHT_DIRECTIONAL, (Vector3){ 2, 1, 2 }, Vector3Zero(), RED, shader);
    lights[2] = CreateLight(LIGHT_DIRECTIONAL, (Vector3){ -2, 1, 2 }, Vector3Zero(), GREEN, shader);
    lights[3] = CreateLight(LIGHT_DIRECTIONAL, (Vector3){ 2, 1, -2 }, Vector3Zero(), BLUE, shader);


    Matrix identity_matrix = MatrixIdentity();
    
    Mesh march_mesh = genFromMarch(march_res);
    Material march_material = LoadMaterialDefault();
    march_material.maps[0].color = {100, 100, 255, 255};
    march_material.shader = shader;
    Matrix march_matrix = MatrixIdentity();

    // Define the camera to look into our 3d world (position, target, up vector)
    Camera camera = { 0 };
    camera.position = (Vector3){ 0.0f, 20.0f, -10.0f };    // Camera position
    camera.target = (Vector3){ 10.0f, 0.0f, 10.0f };      // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 60.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type

    int cameraMode = CAMERA_FIRST_PERSON;

    DisableCursor();                    // Limit cursor to relative movement inside the window

    SetTargetFPS(60);                   // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())        // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        // Switch camera mode
        if (IsKeyPressed(KEY_ONE))
        {
            cameraMode = CAMERA_FREE;
            camera.up = (Vector3){ 0.0f, 1.0f, 0.0f }; // Reset roll
        }

        if (IsKeyPressed(KEY_TWO))
        {
            cameraMode = CAMERA_FIRST_PERSON;
            camera.up = (Vector3){ 0.0f, 1.0f, 0.0f }; // Reset roll
        }

        if (IsKeyPressed(KEY_THREE))
        {
            cameraMode = CAMERA_THIRD_PERSON;
            camera.up = (Vector3){ 0.0f, 1.0f, 0.0f }; // Reset roll
        }

        if (IsKeyPressed(KEY_FOUR))
        {
            cameraMode = CAMERA_ORBITAL;
            camera.up = (Vector3){ 0.0f, 1.0f, 0.0f }; // Reset roll
        }

        // Switch camera projection
        if (IsKeyPressed(KEY_P))
        {
            if (camera.projection == CAMERA_PERSPECTIVE)
            {
                // Create isometric view
                cameraMode = CAMERA_THIRD_PERSON;
                // Note: The target distance is related to the render distance in the orthographic projection
                camera.position = (Vector3){ 0.0f, 2.0f, -100.0f };
                camera.target = (Vector3){ 0.0f, 2.0f, 0.0f };
                camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
                camera.projection = CAMERA_ORTHOGRAPHIC;
                camera.fovy = 20.0f; // near plane width in CAMERA_ORTHOGRAPHIC
                CameraYaw(&camera, -135 * DEG2RAD, true);
                CameraPitch(&camera, -45 * DEG2RAD, true, true, false);
            }
            else if (camera.projection == CAMERA_ORTHOGRAPHIC)
            {
                // Reset to default view
                cameraMode = CAMERA_THIRD_PERSON;
                camera.position = (Vector3){ 0.0f, 2.0f, 10.0f };
                camera.target = (Vector3){ 0.0f, 2.0f, 0.0f };
                camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
                camera.projection = CAMERA_PERSPECTIVE;
                camera.fovy = 60.0f;
            }
        }

        mouse_delta = GetMouseDelta();
        mouse_delta.x /= (float) screenWidth;
        mouse_delta.y /= (float) screenHeight;
        Vector3 fwd = {0.0f, 0.0f, -1.0f};
        Vector3 new_fwd = fwd;
        new_fwd.x += mouse_delta.x;
        new_fwd.y += mouse_delta.y;
        new_fwd = Vector3Normalize(new_fwd);
        q_model = QuaternionMultiply(QuaternionFromVector3ToVector3(fwd, new_fwd),
                q_model);


        // Update camera computes movement internally depending on the camera mode
        // Some default standard keyboard/mouse inputs are hardcoded to simplify use
        // For advance camera controls, it's reecommended to compute camera movement manually
        UpdateCamera(&camera, cameraMode);                  // Update camera

        // Update the shader with the camera view vector (points towards { 0.0f, 0.0f, 0.0f })
        float cameraPos[3] = { camera.position.x, camera.position.y, camera.position.z };
        SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], cameraPos, SHADER_UNIFORM_VEC3);

        // Check key inputs to enable/disable lights
        if (IsKeyPressed(KEY_Y)) { lights[0].enabled = !lights[0].enabled; }
        if (IsKeyPressed(KEY_R)) { lights[1].enabled = !lights[1].enabled; }
        if (IsKeyPressed(KEY_G)) { lights[2].enabled = !lights[2].enabled; }
        if (IsKeyPressed(KEY_B)) { lights[3].enabled = !lights[3].enabled; }
        for (int i = 0; i < MAX_LIGHTS; i++) UpdateLightValues(shader, lights[i]);        
    

/*
        // Camera PRO usage example (EXPERIMENTAL)
        // This new camera function allows custom movement/rotation values to be directly provided
        // as input parameters, with this approach, rcamera module is internally independent of raylib inputs
        UpdateCameraPro(&camera,
            (Vector3){
                (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP))*0.1f -      // Move forward-backward
                (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN))*0.1f,    
                (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT))*0.1f -   // Move right-left
                (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))*0.1f,
                0.0f                                                // Move up-down
            },
            (Vector3){
                GetMouseDelta().x*0.05f,                            // Rotation: yaw
                GetMouseDelta().y*0.05f,                            // Rotation: pitch
                0.0f                                                // Rotation: roll
            },
            GetMouseWheelMove()*2.0f);                              // Move to target (zoom)
*/
        //----------------------------------------------------------------------------------
        //

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(RAYWHITE);

            BeginMode3D(camera);
                
                glm::vec3 pt0 = getBoxPointByIndex(box, 0);
                glm::vec3 pt1 = getBoxPointByIndex(box, 1);
                glm::vec3 pt2 = getBoxPointByIndex(box, 2);
                glm::vec3 pt4 = getBoxPointByIndex(box, 4);

                // draw cubes to indicate axes directions
                DrawCube((Vector3){pt0.x, pt0.y, pt0.z}, 0.25f, 0.25f, 0.25f, BLACK);
                DrawCube((Vector3){pt1.x, pt1.y, pt1.z}, 0.25f, 0.25f, 0.25f, RED);
                DrawCube((Vector3){pt2.x, pt2.y, pt2.z}, 0.25f, 0.25f, 0.25f, GREEN);
                DrawCube((Vector3){pt4.x, pt4.y, pt4.z}, 0.25f, 0.25f, 0.25f, BLUE);

#ifdef ALL_BOXES_TEST
                for (int i = 0; i < 256; ++i) {
                    // DrawCubeWires(rl_boxes[i].pos, rl_boxes[i].dims.x, rl_boxes[i].dims.y, rl_boxes[i].dims.z, GREEN);
                    DrawMesh(march_meshes[i], march_material, identity_matrix);
                    for (int j = 0; j < 8; ++j) {
                        if (all_marches[i].sdf[j] >= 0.0f) {
                            glm::vec3 corner = getBoxPointByIndex(all_boxes[i], j);
                            DrawCube({corner.x, corner.y, corner.z}, 0.25f, 0.25f, 0.25f, RED);
                        }
                    }
                }
#else
                for (int i = 0; i < XBOXES * YBOXES * ZBOXES; ++i) {
                    DrawCubeWires(rl_boxes[i].pos, rl_boxes[i].dims.x, rl_boxes[i].dims.y, rl_boxes[i].dims.z, GREEN);
                    DrawMesh(march_meshes[i], march_material, identity_matrix);
                    for (int j = 0; j < 8; ++j) {
                        if (sine_marches[i].sdf[j] >= 0.0f) {
                            glm::vec3 corner = getBoxPointByIndex(all_boxes[i], j);
                            DrawCube({corner.x, corner.y, corner.z}, 0.05f, 0.05f, 0.05f, RED);
                        }
                    }
                }
#endif
            EndMode3D();

            // Draw info boxes
            DrawRectangle(5, 5, 330, 100, Fade(SKYBLUE, 0.5f));
            DrawRectangleLines(5, 5, 330, 100, BLUE);

            DrawText("Camera controls:", 15, 15, 10, BLACK);
            DrawText("- Move keys: W, A, S, D, Space, Left-Ctrl", 15, 30, 10, BLACK);
            DrawText("- Look around: arrow keys or mouse", 15, 45, 10, BLACK);
            DrawText("- Camera mode keys: 1, 2, 3, 4", 15, 60, 10, BLACK);
            DrawText("- Zoom keys: num-plus, num-minus or mouse scroll", 15, 75, 10, BLACK);
            DrawText("- Camera projection key: P", 15, 90, 10, BLACK);

            DrawRectangle(600, 5, 195, 100, Fade(SKYBLUE, 0.5f));
            DrawRectangleLines(600, 5, 195, 100, BLUE);

            DrawText("Camera status:", 610, 15, 10, BLACK);
            DrawText(TextFormat("- Mode: %s", (cameraMode == CAMERA_FREE) ? "FREE" :
                                              (cameraMode == CAMERA_FIRST_PERSON) ? "FIRST_PERSON" :
                                              (cameraMode == CAMERA_THIRD_PERSON) ? "THIRD_PERSON" :
                                              (cameraMode == CAMERA_ORBITAL) ? "ORBITAL" : "CUSTOM"), 610, 30, 10, BLACK);
            DrawText(TextFormat("- Projection: %s", (camera.projection == CAMERA_PERSPECTIVE) ? "PERSPECTIVE" :
                                                    (camera.projection == CAMERA_ORTHOGRAPHIC) ? "ORTHOGRAPHIC" : "CUSTOM"), 610, 45, 10, BLACK);
            DrawText(TextFormat("- Position: (%06.3f, %06.3f, %06.3f)", camera.position.x, camera.position.y, camera.position.z), 610, 60, 10, BLACK);
            DrawText(TextFormat("- Target: (%06.3f, %06.3f, %06.3f)", camera.target.x, camera.target.y, camera.target.z), 610, 75, 10, BLACK);
            DrawText(TextFormat("- Up: (%06.3f, %06.3f, %06.3f)", camera.up.x, camera.up.y, camera.up.z), 610, 90, 10, BLACK);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    UnloadMesh(march_mesh);

    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}

