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

#include "BBox.h"
#include "grid.h"
#include "meshgen.h"
#include "marchingcubes.h"
#include "vecutil.h"
#include "sdfs.h"

#define PLATFORM_DESKTOP

#if defined(PLATFORM_DESKTOP)
    #define GLSL_VERSION            330
#else   // PLATFORM_RPI, PLATFORM_ANDROID, PLATFORM_WEB
    #define GLSL_VERSION            100
#endif


float plane_dist(const glm::vec3 &pt, const glm::vec3 &n, float d) {
    return glm::dot(pt, n) + d; 
}

struct RLCube {
    Vector3 pos;
    Vector3 dims;
};


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


MarchResult all_marches[256];

void genMarches(const BBox boxes[256]) {
    float pt_sdf[8];
    for (int ind = 0; ind < 256; ++ind) {
        fillSDFByIndex(pt_sdf, ind); 
        march_no_table(all_marches[ind], pt_sdf, boxes[ind]);
    }
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

    constexpr int XBOXES = 32;
#ifdef ALL_BOXES_TEST
    constexpr int YBOXES = 1;
#else
    constexpr int YBOXES = 16;
#endif
    constexpr int ZBOXES = 32;

    BBox *all_boxes = new BBox[XBOXES * YBOXES * ZBOXES];
    RLCube *rl_boxes = new RLCube[XBOXES * YBOXES * ZBOXES];

    float box_side = 0.20f;
    float box_spacing = 0.0f;

    SdfSphere sdf_sphere_data = {glm::vec3(0.0f), 3.0f, 0.0f};
    Grid grid = {glm::vec3(-3.0f), glm::vec3(0.2f), glm::vec<3, int>(30, 30, 30)};
    std::vector<glm::vec3> grid_pts;
    std::vector<int> grid_inds;
    int grid_pts_cnt;
    int grid_inds_cnt;
    march(grid_pts, grid_inds, grid_pts_cnt, grid_inds_cnt, grid, &sdfSphere, (void*)&sdf_sphere_data);
    Mesh grid_mesh = genFromGridMarch(grid_pts, grid_inds, grid_pts_cnt, grid_inds_cnt);

    float frame_cnt = 0.0f;

#ifdef ALL_BOXES_TEST
    box_side = 2;
    box_spacing = 2;
#endif

    float box_start_x = XBOXES / 2 * -box_side;
    float box_start_y = YBOXES / 2 * -box_side;
    float box_start_z = ZBOXES / 2 * -box_side;

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

    MarchResult *sine_marches = new MarchResult[XBOXES * YBOXES * ZBOXES];
    Mesh *march_meshes = new Mesh[XBOXES * YBOXES * ZBOXES];

#ifdef ALL_BOXES_TEST
    for (int i = 0; i < 256; ++i) {
        fillSDFByIndex(march_sdf, i);
        march_no_table(all_marches[i], march_sdf, all_boxes[i]);
        march_meshes[i] = genFromMarch(all_marches[i]);
    }
#else
    for (int i = 0; i < XBOXES * YBOXES * ZBOXES; ++i) {
        fillSDFSine(march_sdf, all_boxes[i]);
        march_no_table(sine_marches[i], march_sdf, all_boxes[i]);
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
    lights[0] = CreateLight(LIGHT_DIRECTIONAL, Vector3Zero(), (Vector3){ -2, 1, -2 }, YELLOW, shader);
    lights[1] = CreateLight(LIGHT_DIRECTIONAL, Vector3Zero(), (Vector3){ 2, 1, 2 }, RED, shader);
    lights[2] = CreateLight(LIGHT_DIRECTIONAL, Vector3Zero(), (Vector3){ -2, 1, 2 }, GREEN, shader);
    lights[3] = CreateLight(LIGHT_DIRECTIONAL, Vector3Zero(), (Vector3){ 2, 1, -2 }, BLUE, shader);


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

        frame_cnt += 1.0f;
        sdf_sphere_data.t = frame_cnt;
        march(grid_pts, grid_inds, grid_pts_cnt, grid_inds_cnt, grid, &sdfSphere, (void*)&sdf_sphere_data);
        updateMeshFromGridMarch(grid_mesh, grid_pts, grid_inds, grid_pts_cnt, grid_inds_cnt);

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

            ClearBackground(GRAY);

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
                DrawMesh(grid_mesh, march_material, identity_matrix);
                for (int i = 0; i < XBOXES * YBOXES * ZBOXES; ++i) {
                    // DrawCubeWires(rl_boxes[i].pos, rl_boxes[i].dims.x, rl_boxes[i].dims.y, rl_boxes[i].dims.z, GREEN);
                    // DrawMesh(march_meshes[i], march_material, identity_matrix);
                    for (int j = 0; j < 8; ++j) {
                        if (sine_marches[i].sdf[j] >= 0.0f) {
                            glm::vec3 corner = getBoxPointByIndex(all_boxes[i], j);
                            // DrawCube({corner.x, corner.y, corner.z}, 0.05f, 0.05f, 0.05f, RED);
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

    delete [] all_boxes;
    delete [] rl_boxes;

    delete [] sine_marches;
    delete [] march_meshes;

    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}

