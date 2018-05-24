#include "MeshPatches.hpp"
#include "../Core/GPU/Mesh.h" 
#include "DisjointSets.hpp"
#include "../../libs/GL/glew.h"
#include "../Core/GPU/BaseMesh.hpp"
#include <queue>
/*
void MergePatches(Mesh *mesh, DisjointSet &ds, float threshold, std::unordered_map<GLushort, std::set<GLushort>> &vertexGraph)
{
	std::unordered_map<int, Patch_t> patches;
	int setofi, setofit;
	for (int i = 0, numVerts = mesh->positions.size(); i < numVerts; i++)
	{// BUILD PATCHES
		setofi = ds.Find(i + 1);

		if (patches.find(setofi) == patches.end())
		{
			patches[setofi] = Patch_t(1, mesh->normals[i]);
		}
		else
		{
			patches[setofi].num++;
			patches[setofi].normals.push_back(mesh->normals[i]);
		}
		patches[setofi].vertsIDs.push_back(i);
	}

	for (auto it = patches.begin(); it != patches.end(); it++)
	{// CALCULATE AVERAGE NORMAL FOR EVERY PATCH
		it->second.avgNormal = glm::vec3(0.f);
		for (int i = 0, sz = it->second.normals.size(); i < sz; i++)
		{
			it->second.avgNormal += it->second.normals[i];
		}
		it->second.avgNormal /= (float)it->second.num;
	}


	for (int i = 0, numVerts = mesh->positions.size(); i < numVerts; i++)
	{// COMPUTE ADJACENCY INFO FOR EVERY PATCH
		setofi = ds.Find(i + 1);
		for (auto it = vertexGraph[i].begin(); it != vertexGraph[i].end(); it++)
		{
			setofit = ds.Find(*it + 1);
			if (setofi != setofit)
			{
				patches[setofi].adjacentPatches.emplace(setofit);
				patches[setofit].adjacentPatches.emplace(setofi);
			}
		}
	}
	float maxDot = 0;
	bool canMerge;

	for (auto it = patches.begin(); it != patches.end(); it++)
	{
		canMerge = false;
		maxDot = -999.f;
		int bestIdx;
		for (auto it2 = it->second.adjacentPatches.begin(); it2 != it->second.adjacentPatches.end(); it2++)
		{
			float dotProd = glm::dot(patches[*it2].avgNormal, it->second.avgNormal);
			if (dotProd > threshold)
			{
				int adjpatch = *(patches[*it2].vertsIDs.begin());
				ds.Union(*(it->second.vertsIDs.begin()) + 1, adjpatch + 1);
			}
			//	continue;
			//dotProd /= (it->second.num + patches[*it2].num);
			//if (dotProd > maxDot)
			//{
			//	maxDot = dotProd;
			//	bestIdx = *it2;
			//	canMerge = true;
			//}
		}
		//if (canMerge)
		//{
		//int adjpatch = *(patches[bestIdx].vertsIDs.begin());
		//ds.Union(*(it->second.vertsIDs.begin()) + 1, adjpatch + 1);
		//}

	}
}*/


void MergePatches(Mesh *mesh, DisjointSet &ds, float threshold, std::unordered_map<GLushort, std::set<GLushort>> &vertexGraph)
{
	std::unordered_map<int, Patch_t> patches;
	int setofi, setofit;
	std::vector<bool> visited(mesh->positions.size(), false);
	std::queue<int> Q;
	for (int i = 0, numVerts = mesh->positions.size(); i < numVerts; i++)
	{// BUILD PATCHES
		setofi = ds.Find(i + 1);
		Q = std::queue<int>(); // ??????
		if (!visited[i])
		{
			Q.emplace(i);
		}
		if (patches.find(setofi) == patches.end())
		{
			patches[setofi] = Patch_t(1, mesh->normals[i]);
		}
		else
		{
			patches[setofi].num++;
			patches[setofi].normals.push_back(mesh->normals[i]);
		}
		patches[setofi].vertsIDs.push_back(i);
	}

	for (auto it = patches.begin(); it != patches.end(); it++)
	{// CALCULATE AVERAGE NORMAL FOR EVERY PATCH
		it->second.avgNormal = glm::vec3(0.f);
		for (int i = 0, sz = it->second.normals.size(); i < sz; i++)
		{
			it->second.avgNormal += it->second.normals[i];
		}
		it->second.avgNormal /= (float)it->second.num;
	}


	for (int i = 0, numVerts = mesh->positions.size(); i < numVerts; i++)
	{// COMPUTE ADJACENCY INFO FOR EVERY PATCH
		setofi = ds.Find(i + 1);
		for (auto it = vertexGraph[i].begin(); it != vertexGraph[i].end(); it++)
		{
			setofit = ds.Find(*it + 1);
			if (setofi != setofit)
			{
				patches[setofi].adjacentPatches.emplace(setofit);
				patches[setofit].adjacentPatches.emplace(setofi);
			}
		}
	}
	float maxDot = 0;
	bool canMerge;

	for (auto it = patches.begin(); it != patches.end(); it++)
	{
		canMerge = false;
		maxDot = -999.f;
		int bestIdx;
		for (auto it2 = it->second.adjacentPatches.begin(); it2 != it->second.adjacentPatches.end(); it2++)
		{
			float dotProd = glm::dot(patches[*it2].avgNormal, it->second.avgNormal);
			if (dotProd > threshold)
			{
				int adjpatch = *(patches[*it2].vertsIDs.begin());
				ds.Union(*(it->second.vertsIDs.begin()) + 1, adjpatch + 1);
			}
			//	continue;
			//dotProd /= (it->second.num + patches[*it2].num);
			//if (dotProd > maxDot)
			//{
			//	maxDot = dotProd;
			//	bestIdx = *it2;
			//	canMerge = true;
			//}
		}
		/*if (canMerge)
		{
		int adjpatch = *(patches[bestIdx].vertsIDs.begin());
		ds.Union(*(it->second.vertsIDs.begin()) + 1, adjpatch + 1);
		}*/

	}
}



std::vector<float> BuildFeatureMap(Mesh *&mesh, Mesh *&mesh1, std::unordered_map<GLushort, std::set<GLushort>> &vertexGraph, 
											int MAX_ITERS, float ITER_STEP, float ANGLE_THRESHOLD)
{
	// Load meshes
	DisjointSet ds(mesh->positions.size());

	//std::unordered_map<int, Patch_t> patches;
	//return std::vector<float>();

	int cnt = 0;
	for (int i = 0; i < mesh->positions.size(); i++)
	{// JOIN SETS OF VERTICES ON A "BEST NEIGHBOR POLICY"
		ds.Union(i + 1, i + 1);
	}
	//*MissingCode*: ~/Documents/junk.cpp
	//for (float thresh = 0.95f; thresh >= 0.1f; thresh -= 0.1f)
	//for (int i = 0; i < 3; i++)
	MergePatches(mesh, ds, 0.9, vertexGraph);
	//	MergePatches(mesh, ds, 0.1);

	int maxadj = -1;
	std::vector<GLubyte> adjacentPatchesCount(mesh->positions.size(), 0);
	for (int i = 0, numVerts = mesh->positions.size(); i < numVerts; i++)
	{
		for (std::set<GLushort>::iterator it = vertexGraph[i].begin(); it != vertexGraph[i].end(); it++)
		{
			if (ds.Find(*it + 1) != ds.Find(i + 1))
			{
				adjacentPatchesCount[i]++;
				if (adjacentPatchesCount[i] > maxadj)
					maxadj = adjacentPatchesCount[i];
			}
		}
	}

	std::vector<int> ids;
	for (int i = 0, numVerts = mesh->positions.size(); i < numVerts; i++)
		ids.push_back(ds.Find(i + 1));

	std::unordered_map<int, glm::vec3> colors;
	for (int i = 0; i < mesh->positions.size(); i++)
		colors[ids[i]] = glm::vec3(float(rand() % 101) / 100.f, float(rand() % 101) / 100.f, float(rand() % 101) / 100.f);

	TVertexList verts, verts1;
	//#define LVLS 3
	std::vector<float> ret;

	for (int i = 0, numVerts = mesh->positions.size(); i < numVerts; i++)
	{
		glm::vec3 v1 = colors[ids[i]];
		verts1.push_back(VertexFormat(mesh->positions[i], v1, mesh->normals[i]));
		float fm = pow((float)adjacentPatchesCount[i] / (float)maxadj, 4);
		ret.push_back(fm);
		verts.push_back(VertexFormat(mesh->positions[i], glm::vec3(0, fm, fm * 0.5), mesh->normals[i]));
		//verts.push_back(VertexFormat(mesh->positions[i], glm::vec3(variations[i] / maxDotSum), mesh->normals[i]));
	}

	mesh->InitFromData(verts, mesh->indices);

	mesh1->InitFromData(verts1, mesh->indices);

	return ret;
}