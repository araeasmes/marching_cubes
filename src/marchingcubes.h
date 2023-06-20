#pragma once

#include <glm/glm.hpp>

#include <algorithm>
#include <iostream>
#include <vector>

#include "BBox.h"
#include "grid.h"
#include "sdfs.h"

struct MarchResult { 
    float sdf[8];
    int ind = -1;
    int num_verts = 0;
    int num_inds = 0;
    int indices[12 * 3];
    int edge_inds[12 * 3];
    glm::vec3 verts[12];
    int edges[12];
};

int edge_mask(int p1, int p2);
int edge_index(int p1, int p2);

glm::vec<3, int> pt_ind_to_vec(int p);

// j = [0, 3) represents the direction
int edge_index_from_pt_dir(int p, int j);

void edge_to_points(int &p1, int &p2, int e);

void fillSDFByIndex(float pts_sdf[8], int ind);

int sdfToIndex(float sdf[8]);

void initMarchResult(MarchResult &out);

void march_no_table(MarchResult &out, float pts_sdf[8], const BBox &box);

void generateTable(std::vector<int> edges[256], std::vector<int> indices[256]);

void march(std::vector<glm::vec3> &pts, std::vector<int> &inds, int &edges_added, int &inds_added,
            Grid &g, sdf_fun_type sdf_fun, void* sdf_data) {
    std::vector<int> edges[256];
    std::vector<int> indices[256];
    generateTable(edges, indices);

    int num_pts = (g.num_steps.x + 1) * (g.num_steps.y + 1) * (g.num_steps.z + 1);
    int yz_pts = (g.num_steps.y + 1) * (g.num_steps.z + 1);
    int zx_pts = (g.num_steps.z + 1) * (g.num_steps.x + 1);
    int xy_pts = (g.num_steps.x + 1) * (g.num_steps.y + 1);
    int edges_x = g.num_steps.x * yz_pts;
    int edges_y = g.num_steps.y * zx_pts;
    int edges_z = g.num_steps.z * xy_pts;

    int num_edges = edges_x + edges_y + edges_z;
    
    auto getGridInd = [&](glm::vec<3, int> pt_xyz) -> int 
        { return xy_pts * pt_xyz.z + (g.num_steps.x + 1) * pt_xyz.y + pt_xyz.x; };

    std::vector<float> sdf(num_pts);
    for (int z = 0; z <= g.num_steps.z; ++z) {
        for (int y = 0; y <= g.num_steps.y; ++y) {
            for (int x = 0; x <= g.num_steps.x; ++x) {
                int ind = getGridInd({x, y, z}); // xy_pts * z + (g.num_steps.x + 1) * y + x;
                glm::vec3 pt = getGridPoint(g, x, y, z);
                sdf[ind] = sdf_fun(pt, sdf_data);
            }
        }
    }

    pts.resize(num_edges, glm::vec3(0.0f));
    std::fill(pts.begin(), pts.end(), glm::vec3(0.0f));
    edges_added = 0;
    // a very rough estimate, 21 is the max number of inds in a single cube:
    inds.resize(g.num_steps.x * g.num_steps.y * g.num_steps.z * 21, 0);
    std::fill(inds.begin(), inds.end(), 0);
    inds_added = 0;
    std::vector<int> edge_remapping(num_edges, -1);

    int edge_dir_skip[3] = {0, edges_x, edges_x + edges_y};

    for (int z = 0; z < g.num_steps.z; ++z) {
        for (int y = 0; y < g.num_steps.y; ++y) {
            for (int x = 0; x < g.num_steps.x; ++x) {
                glm::vec<3, int> cur_xyz(x, y, z);
                int lut_ind = 0;
                for (int i = 0; i < 8; ++i) {
                    glm::vec<3, int> pi = pt_ind_to_vec(i);
                    int ind = getGridInd(cur_xyz + pi);
                    lut_ind |= int(sdf[ind] > 0) << i;
                }
                const std::vector<int> &cur_edges = edges[lut_ind];
                const std::vector<int> &cur_inds = indices[lut_ind];
                
                int local_remap[12];
                std::fill(local_remap, local_remap + 12, -1);

                for (int edge : cur_edges) {
                    int p1_ind, p2_ind;
                    edge_to_points(p1_ind, p2_ind, edge);

                    glm::vec<3, int> p1_grid = cur_xyz + pt_ind_to_vec(p1_ind);
                    glm::vec<3, int> p2_grid = cur_xyz + pt_ind_to_vec(p2_ind);

                    int edge_dir = edge / 4;
                    int edge_mult = 1;

                    int grid_edge = 0;
                    for (int j = 0; j < 3; ++j) {
                        if (j != edge_dir) {
                            grid_edge += p1_grid[j] * edge_mult;
                            edge_mult *= (g.num_steps[j] + 1);
                        }
                    }
                    grid_edge += p1_grid[edge_dir] * edge_mult;
                    grid_edge += edge_dir_skip[edge_dir];
                    
                    if (edge_remapping[grid_edge] == -1) {
                        edge_remapping[grid_edge] = edges_added;
                        local_remap[edge] = edges_added;

                        glm::vec3 p1 = getGridPoint(g, p1_grid.x, p1_grid.y, p1_grid.z);
                        glm::vec3 p2 = getGridPoint(g, p2_grid.x, p2_grid.y, p2_grid.z);
                        float sdf1 = sdf[getGridInd(p1_grid)];
                        float sdf2 = sdf[getGridInd(p2_grid)];
                        float sdf_dif = sdf1 - sdf2;
                        float t = 0.5f;
                        if (fabsf(sdf_dif) > FLT_EPSILON)
                            t = sdf1 / sdf_dif;

                        pts[edges_added] = (1.0f - t) * p1 + p2 * t;
                        edges_added++;
                    } else {
                        local_remap[edge] = edge_remapping[grid_edge];
                    }
                }
                for (int ind : cur_inds) {
                    inds[inds_added++] = local_remap[ind];
                }
            }
        }
    }
}

// IMPLEMENTATION START

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

    return edge_group * 4 + edge_ingroup;
}

glm::vec<3, int> pt_ind_to_vec(int p) {
    return glm::vec<3, int>(p & 1, (p >> 1) & 1, (p >> 2) & 1);
}

void fillSDFByIndex(float pts_sdf[8], int ind) {
    for (int i = 0; i < 8; ++i) {
        pts_sdf[i] = (ind & 1) ? 0.5f : -0.5f;
        ind >>= 1;
    }
}

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


void march_no_table(MarchResult &out, float pts_sdf[8], const BBox &box) {
    initMarchResult(out);

    int visited[8] = {0};
    int queue[8] = {-1};
    int queue_i = 0;
    int queue_end = 0;
    
    out.ind = sdfToIndex(pts_sdf);
    for (int i = 0; i < 8; ++i) {
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
                    out.edges[out.num_verts] = edge_ind;

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
            
            for (int j = 1; j < 3; ++j) {
                int axis = (edge_dir + j) % 3;
                dir ^= (p1 >> axis) & 1;
                js[j - 1] = axis;
            }
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
                std::cerr << "ALERTA, SMALL COMPONENTS\n";
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
                out.edge_inds[out.num_inds] = e1;
                out.indices[out.num_inds++] = comp_edge_inds[e1];

                out.edge_inds[out.num_inds] = e2;
                out.indices[out.num_inds++] = comp_edge_inds[e2];

                out.edge_inds[out.num_inds] = e3;
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
                std::cerr << "not both???\n";

            if (left_cnt && right_cnt) {
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

                out.edge_inds[out.num_inds] = a;
                out.indices[out.num_inds++] = ia;

                out.edge_inds[out.num_inds] = b;
                out.indices[out.num_inds++] = ib;

                out.edge_inds[out.num_inds] = c;
                out.indices[out.num_inds++] = ic;

                out.edge_inds[out.num_inds] = c;
                out.indices[out.num_inds++] = ic;

                out.edge_inds[out.num_inds] = b;
                out.indices[out.num_inds++] = ib;
                
                out.edge_inds[out.num_inds] = d;
                out.indices[out.num_inds++] = id;
            }
        } 
    }
}

int sdfToIndex(float sdf[8]) {
    int ind = 0;
    for (int i = 0; i < 8; ++i) {
        ind |= int(sdf[i] > 0) << i;
    }
    return ind;
}

void generateTable(std::vector<int> edges[256], std::vector<int> indices[256]) {
    float sdf[8];
    MarchResult mr;
    BBox box = {glm::vec3(0.0f), glm::vec3(1.0f)};

    initMarchResult(mr);
    for (int i = 0; i < 256; ++i) {
        fillSDFByIndex(sdf, i);
        march_no_table(mr, sdf, box);

        edges[i].resize(mr.num_verts);
        indices[i].resize(mr.num_inds);
        for (int j = 0; j < mr.num_verts; ++j)
            edges[i][j] = mr.edges[j];
        for (int j = 0; j < mr.num_inds; ++j)
            indices[i][j] = mr.edge_inds[j];
    }
}
