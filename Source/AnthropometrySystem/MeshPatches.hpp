#pragma once
#include <vector>
#include <set>
#include "../../libs/glm/glm.hpp"
#include <unordered_map>
#include "DisjointSets.hpp"
#include <include/gl.h>
class Mesh;
struct Patch_t
{
	Patch_t() {}
	Patch_t(int n, glm::vec3 normal)
	{
		num = n;
		normals.push_back(normal);
	}
	int num;
	std::vector<glm::vec3> normals;
	std::vector<int> vertsIDs;
	glm::vec3 avgNormal;
	std::set<int> adjacentPatches;
};

void MergePatches(Mesh *mesh, DisjointSet &ds, float threshold, std::unordered_map<GLushort, std::set<GLushort> > &vertexGraph);
std::vector<float> BuildFeatureMap(Mesh *&mesh, Mesh *&mesh1, std::unordered_map<GLushort, std::set<GLushort>> &vertexGraph,
													int MAX_ITERS = 3, float ITER_STEP = 0.005f, float ANGLE_THRESHOLD = 0.99f);
