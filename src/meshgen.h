#pragma once

#include <glm/glm.hpp>

#include "raylib.h"

#include "marchingcubes.h"
#include "vecutil.h"


void updateMeshFromGridMarch(Mesh &mesh, 
    const std::vector<glm::vec3> &pts, const std::vector<int> &inds, int pt_cnt, int ind_cnt) {
    for (int i = 0; i < pts.size(); ++i) {
        mesh.vertices[i * 3 + 0] = pts[i].x;        
        mesh.vertices[i * 3 + 1] = pts[i].y;        
        mesh.vertices[i * 3 + 2] = pts[i].z;        

        mesh.normals[i * 3 + 0] = 0.0f;
        mesh.normals[i * 3 + 1] = 0.0f;
        mesh.normals[i * 3 + 2] = 0.0f;
    }

    for (size_t i = 0; i < inds.size(); ++i) {
        mesh.indices[i] = inds[i];
    }

    for (size_t i = 0; i < ind_cnt; i += 3) {
        int ia = inds[i + 0];
        int ib = inds[i + 1];
        int ic = inds[i + 2];
        glm::vec3 normal = glm::cross(pts[ib] - pts[ia], pts[ic] - pts[ia]);
        float len = glm::length(normal);
        if (len > FLT_EPSILON)
            normal /= len;
        mesh.normals[ia * 3 + 0] += normal.x;
        mesh.normals[ia * 3 + 1] += normal.y;
        mesh.normals[ia * 3 + 2] += normal.z;

        mesh.normals[ib * 3 + 0] += normal.x;
        mesh.normals[ib * 3 + 1] += normal.y;
        mesh.normals[ib * 3 + 2] += normal.z;

        mesh.normals[ic * 3 + 0] += normal.x;
        mesh.normals[ic * 3 + 1] += normal.y;
        mesh.normals[ic * 3 + 2] += normal.z;
    }
    for (int i = 0; i < pt_cnt; ++i) {
        float len = 0.0f;
        len += SQR(mesh.normals[i * 3 + 0]);
        len += SQR(mesh.normals[i * 3 + 1]);
        len += SQR(mesh.normals[i * 3 + 2]);
        len = glm::sqrt(len);
        if (len <= FLT_EPSILON)
            continue;
        mesh.normals[i * 3 + 0] /= len;
        mesh.normals[i * 3 + 1] /= len;
        mesh.normals[i * 3 + 2] /= len;
    }

    // 0// Vertex buffer: positions
    // 1// Vertex buffer: texcoords
    // 2// Vertex buffer: normals
    // 3// Vertex buffer: colors
    // 4// Vertex buffer: tangents
    // 5// Vertex buffer: texcoords2
    // 6// Vertex buffer: indices

    UpdateMeshBuffer(mesh, 0, mesh.vertices, sizeof(float) * pts.size(), 0);
    UpdateMeshBuffer(mesh, 6, mesh.indices, sizeof(unsigned short) * inds.size(), 0);
    UpdateMeshBuffer(mesh, 2, mesh.normals, sizeof(float) * pts.size(), 0);
    // UpdateMeshBuffer(mesh, 5, mesh.texcoords, sizeof(float) * 12 * 2, 0);
}

Mesh genFromGridMarch(const std::vector<glm::vec3> &pts, const std::vector<int> &inds, int pt_cnt, int ind_cnt) {
    Mesh mesh = {};
    mesh.vertexCount = pts.size();
    mesh.triangleCount = inds.size() / 3;
    
    mesh.vertices = (float *)MemAlloc(mesh.vertexCount*3*sizeof(float));    // 3 vertices, 3 coordinates each (x, y, z)
    // mesh.texcoords = (float *)MemAlloc(mesh.vertexCount*2*sizeof(float));   // 3 vertices, 2 coordinates each (x, y)
    mesh.normals = (float *)MemAlloc(mesh.vertexCount*3*sizeof(float));     // 3 vertices, 3 coordinates each (x, y, z)
    mesh.indices = (unsigned short *)MemAlloc(mesh.triangleCount * 3 * sizeof(unsigned short));

    for (size_t i = 0; i < pts.size(); ++i) {
        mesh.vertices[i * 3 + 0] = pts[i].x;
        mesh.vertices[i * 3 + 1] = pts[i].y;
        mesh.vertices[i * 3 + 2] = pts[i].z;

        mesh.normals[i * 3 + 0] = 0.0f;
        mesh.normals[i * 3 + 1] = 0.0f;
        mesh.normals[i * 3 + 2] = 0.0f;
    }
    for (size_t i = 0; i < inds.size(); ++i) {
        mesh.indices[i] = inds[i];
    }
    for (size_t i = 0; i < ind_cnt; i += 3) {
        int ia = inds[i + 0];
        int ib = inds[i + 1];
        int ic = inds[i + 2];
        glm::vec3 normal = glm::cross(pts[ib] - pts[ia], pts[ic] - pts[ia]);
        float len = glm::length(normal);
        if (len > FLT_EPSILON)
            normal /= len;
        mesh.normals[ia * 3 + 0] += normal.x;
        mesh.normals[ia * 3 + 1] += normal.y;
        mesh.normals[ia * 3 + 2] += normal.z;

        mesh.normals[ib * 3 + 0] += normal.x;
        mesh.normals[ib * 3 + 1] += normal.y;
        mesh.normals[ib * 3 + 2] += normal.z;

        mesh.normals[ic * 3 + 0] += normal.x;
        mesh.normals[ic * 3 + 1] += normal.y;
        mesh.normals[ic * 3 + 2] += normal.z;
    }
    for (int i = 0; i < pt_cnt; ++i) {
        float len = 0.0f;
        len += SQR(mesh.normals[i * 3 + 0]);
        len += SQR(mesh.normals[i * 3 + 1]);
        len += SQR(mesh.normals[i * 3 + 2]);
        len = glm::sqrt(len);
        if (len <= FLT_EPSILON)
            continue;
        mesh.normals[i * 3 + 0] /= len;
        mesh.normals[i * 3 + 1] /= len;
        mesh.normals[i * 3 + 2] /= len;
    }

    UploadMesh(&mesh, true);
    
    return mesh;
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
