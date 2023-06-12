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

#define MAX_COLUMNS 20

#define FLT_EPSILON      1.192092896e-07F        // smallest such that 1.0+FLT_EPSILON != 1.0


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

        int cur_comp[12] = {-1};
        std::fill(cur_comp, cur_comp + 12, -1);
        int comp_ind = 0;

        int comp_edge_inds[12];
        std::fill(comp_edge_inds, comp_edge_inds + 12, -1);

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

                    comp_edge_inds[edge_index(ind, k)] = out.num_verts;

                    cur_comp[comp_ind++] = out.num_verts;
                    out.num_verts++;
                }
                visited[ind] = 2;
            }
        }
        
        if (comp_ind < 3)
            continue;
        
        int ordered_inds[12];
        std::fill(ordered_inds, ordered_inds + 12, -1);

        int cur_e = -1;
        int i_ordered = 0;
        for (int e = 0; e < 12; ++e) {
            if (comp_edge_inds[e] == -1)
                continue;
            cur_e = e;
            while (cur_e != -1) {
                ordered_inds[i_ordered++] = comp_edge_inds[cur_e];
                comp_edge_inds[cur_e] = -1; 

                int edge_dir = cur_e / 4;
                int edge_ingroup = cur_e % 4;

                int p1 = 0;
                int p2 = 0;
                edge_to_points(p1, p2, cur_e);
                
                int next_e = -1;

                for (int j = 0; j < 3; ++j) {
                    if (j == edge_dir)
                        continue;
                    
                    int p3 = p1 ^ (1 << j);
                    int p4 = p2 ^ (1 << j);
                    int tmp_e = edge_index(p1, p3);
                    if (comp_edge_inds[tmp_e] != -1) {
                        next_e = tmp_e;
                        break;
                    }
                    tmp_e = edge_index(p2, p4);
                    if (comp_edge_inds[tmp_e] != -1) {
                        next_e = tmp_e;
                        break;
                    }
                    tmp_e = edge_index(p3, p4);
                    if (comp_edge_inds[tmp_e] != -1) {
                        next_e = tmp_e;
                        break;
                    }
                }
                cur_e = next_e;
            }
        }

        /*
        glm::vec3 pt0 = out.verts[out.num_verts - comp_ind];
        glm::vec3 pt1 = out.verts[out.num_verts - comp_ind + 1];
        glm::vec3 pt2 = out.verts[out.num_verts - comp_ind + 2];
        glm::vec3 dir = glm::cross(pt1 - pt0, pt2 - pt0);
        // dir = glm::vec3(0.0f, 1.0f, 0.0f);
        std::sort(out.verts + out.num_verts - comp_ind, out.verts + out.num_verts, 
                [&](const glm::vec3 &a, const glm::vec3 &b) { return glm::dot(glm::cross(a - pt0, b - pt0), dir) < 0.0f; });
        */

        int id1 = ordered_inds[0];
        for (int j = 2; j < comp_ind; ++j) {
            int id2 = ordered_inds[j - 1];
            int id3 = ordered_inds[j];
            out.indices[out.num_inds++] = id1;
            out.indices[out.num_inds++] = id2;
            out.indices[out.num_inds++] = id3;
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
        mesh.normals[i * 3 + 1] = 1.0f;
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
    mesh.texcoords[5] =0;

    // Upload mesh data from CPU (RAM) to GPU (VRAM) memory
    UploadMesh(&mesh, false);

    return mesh;
}

int main(void) {
    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "raylib marching cubes stuff");

    BBox box({glm::vec3(-2.0f, 1.0f, -2.0f), glm::vec3(2.0f, 5.0f, 2.0f)});

    glm::vec3 box_c = (box.pt.min + box.pt.max) * 0.5f;
    glm::vec3 box_dif = box.pt.max - box.pt.min;
    Vector3 box_pos = {box_c.x, box_c.y, box_c.z};
    Vector3 box_dims = {box_dif.x, box_dif.y, box_dif.z}; 

    glm::vec3 plane_pos(0.0f, 0.0f, 0.0f);
    glm::vec3 plane_normal(0.0f, 1.0f, 0.0f);
    glm::vec3 plane_normal_rot = plane_normal;
    float plane_extents = 6.0f;
    float plane_d = -glm::dot(plane_normal, plane_pos);

    Matrix plane_mat = MatrixIdentity();
    Mesh plane_mesh = genPlane(plane_pos, plane_extents, plane_normal);
    Material plane_material = LoadMaterialDefault();
    plane_material.maps[0].color = {230, 110, 150, 255};

    rlDisableBackfaceCulling();
    Quaternion q_model = {0.0f, 0.0f, 0.0f, 1.0f};
    Vector2 mouse_delta;

    MarchResult march_res;
    float march_sdf[8];

    constexpr int XBOXES = 16;
    constexpr int YBOXES = 1;
    constexpr int ZBOXES = 16;

    BBox all_boxes[XBOXES * YBOXES * ZBOXES];
    RLCube rl_boxes[XBOXES * YBOXES * ZBOXES];

    float box_side = 2.0f;
    float box_spacing = 2.0f;

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
    // genMarches(all_boxes);
    for (int i = 0; i < XBOXES * YBOXES * ZBOXES; ++i) {
        fillSDFSine(march_sdf, all_boxes[i]);
        march(sine_marches[i], march_sdf, all_boxes[i]);
        //march_meshes[i] = genFromMarch(sine_marches[i]);
    }

    for (int i = 0; i < 256; ++i) {
        fillSDFByIndex(march_sdf, i);
        march(all_marches[i], march_sdf, all_boxes[i]);
        march_meshes[i] = genFromMarch(all_marches[i]);
    }

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
    lights[0] = CreateLight(LIGHT_POINT, (Vector3){ -2, 1, -2 }, Vector3Zero(), YELLOW, shader);
    lights[1] = CreateLight(LIGHT_POINT, (Vector3){ 2, 1, 2 }, Vector3Zero(), RED, shader);
    lights[2] = CreateLight(LIGHT_POINT, (Vector3){ -2, 1, 2 }, Vector3Zero(), GREEN, shader);
    lights[3] = CreateLight(LIGHT_POINT, (Vector3){ 2, 1, -2 }, Vector3Zero(), BLUE, shader);


    Matrix identity_matrix = MatrixIdentity();
    
    fillPlaneSDF(march_sdf, box, plane_normal_rot, plane_d);
    march(march_res, march_sdf, box); 

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
        plane_mat = QuaternionToMatrix(q_model);
        Vector3 tmp_vec = Vector3RotateByQuaternion({plane_normal.x, plane_normal.y, plane_normal.z}, q_model);
        plane_normal_rot = {tmp_vec.x, tmp_vec.y, tmp_vec.z};
        plane_normal_rot = glm::normalize(plane_normal_rot);
        plane_d = -glm::dot(plane_normal_rot, plane_pos);


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
        fillPlaneSDF(march_sdf, box, plane_normal_rot, plane_d);
        march(march_res, march_sdf, box); 
        updateMeshFromMarch(march_mesh, march_res);

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(RAYWHITE);

            BeginMode3D(camera);
                
                glm::vec3 pt0 = getBoxPointByIndex(box, 0);
                glm::vec3 pt1 = getBoxPointByIndex(box, 1);
                glm::vec3 pt2 = getBoxPointByIndex(box, 2);
                glm::vec3 pt4 = getBoxPointByIndex(box, 4);
                DrawCube((Vector3){pt0.x, pt0.y, pt0.z}, 0.25f, 0.25f, 0.25f, BLACK);
                DrawCube((Vector3){pt1.x, pt1.y, pt1.z}, 0.25f, 0.25f, 0.25f, RED);
                DrawCube((Vector3){pt2.x, pt2.y, pt2.z}, 0.25f, 0.25f, 0.25f, GREEN);
                DrawCube((Vector3){pt4.x, pt4.y, pt4.z}, 0.25f, 0.25f, 0.25f, BLUE);

                if (march_res.num_verts) {
                    glm::vec3 pt_start = march_res.verts[0];
                    DrawSphere((Vector3) {pt_start.x, pt_start.y, pt_start.z}, 0.2f, RED);
                    pt_start += plane_normal_rot;
                    DrawSphere((Vector3) {pt_start.x, pt_start.y, pt_start.z}, 0.15f, RED);
                }

                DrawCubeWires(box_pos, box_dims.x, box_dims.y, box_dims.z, GREEN);
                
                /*
                for (int i = 0; i < XBOXES * YBOXES * ZBOXES; ++i) {
                    // DrawCubeWires(rl_boxes[i].pos, rl_boxes[i].dims.x, rl_boxes[i].dims.y, rl_boxes[i].dims.z, GREEN);
                    DrawMesh(march_meshes[i], march_material, identity_matrix);
                    for (int j = 0; j < 8; ++j) {
                        if (all_marches[i].sdf[j] >= 0.0f) {
                            glm::vec3 corner = getBoxPointByIndex(all_boxes[i], j);
                            DrawCube({corner.x, corner.y, corner.z}, 0.25f, 0.25f, 0.25f, RED);
                        }
                    }
                }
                */
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

                DrawMesh(march_mesh, march_material, march_matrix);

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
    UnloadMesh(plane_mesh);
    UnloadMesh(march_mesh);

    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}

