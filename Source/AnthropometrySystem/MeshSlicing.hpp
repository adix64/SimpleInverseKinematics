#pragma once
#include <vector>
#include <set>
#include "../../libs/glm/glm.hpp"
#include <unordered_map>
#include "../Core/GPU/Mesh.h" 

struct Bone
{
	glm::vec3 pos;
	Bone *parent = NULL;
	glm::vec3 color = glm::vec3(1);
	std::vector<Bone*> children;
	
	uint64_t id;
	glm::uvec3 colorFBOid;
	
	bool pickable = false;

	Bone(glm::vec3 &pPos, Bone* pParent = NULL)
		: pos(pPos), parent(pParent) {}
};
