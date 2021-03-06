#pragma once
#include <Component/SimpleScene.h>
#include <Component/Transform/Transform.h>
#include <Core/GPU/Mesh.h>
#include <Core/GPU/CubeMapFBO.h>
#include <Core/GPU/Texture2D.h>
#include <Core/GPU/ShadowCubeMapFBO.h>
#include "Gizmo.hpp"
#include "Camera.hpp"
#include "Grid.hpp"
#include "FullscreenQuad.hpp"
#include <Core/GPU/Framebuffer.hpp>
#include "ColorGenerator.hpp"
#include <Core\GPU\Sprite.hpp>
#include "DisjointSets.hpp"
#include "TextRendering.h"
#include <unordered_map>
#include <set>
typedef std::vector<VertexFormat> TVertexList;
typedef std::vector<uint32_t> TIndexList;
typedef std::vector<glm::vec3> CurvePointList;
class Bone;

inline uint64_t calculateColorHash(glm::uvec3 v)
{
	uint64_t out = 0x0000000000000000;
	out = out | v.x;
	out = out | v.y << 8;
	out = out | v.z << 16;
	return out;
}

using namespace std;

class IKsystem : public SimpleScene
{
	enum ActiveToolType { SELECT_TOOL, MOVE_TOOL, ROTATE_TOOL, PLANE_SLICE_TOOL};
	public:
		IKsystem();
		~IKsystem();

		void Init() override;
	private:
		void LoadMaterials();
		void LoadMeshes();
		void LoadShaders();

		void FrameStart() override;
		void Update(float deltaTimeSeconds) override;
		void FrameEnd() override;
		void RenderBody();
		void AddBone(glm::vec3 position, Bone *parent, glm::vec3 color = glm::vec3(1));
		void AddBoneAtScreenPoint(glm::vec2 screenSpacePos);
		
		void IKSolverUpdate();
		void _IKSolverUpdate(Bone *crtBone);

		void DrawLine(Shader *shader, glm::vec3 &p1, glm::vec3 &p2, glm::vec3 &color = glm::vec3(1));
		void InitIKsystem();
		void RenderSimpleMesh(Mesh *mesh, Shader *shader, const glm::mat4 &modelMatrix, Texture2D* texture1 = NULL, Texture2D* texture2 = NULL, glm::vec3 color = glm::vec3(0, 0, 0));
		//void ForceRedraw();

		void OnInputUpdate(float deltaTime, int mods) override;
		void OnKeyPress(int key, int mods) override;
		void OnKeyRelease(int key, int mods) override;
		void OnMouseMove(int mouseX, int mouseY, int deltaX, int deltaY) override;
		void OnMouseBtnPress(int mouseX, int mouseY, int button, int mods) override;
		void OnMouseBtnRelease(int mouseX, int mouseY, int button, int mods) override;
		void OnMouseScroll(int mouseX, int mouseY, int offsetX, int offsetY) override;
		void OnWindowResize(int width, int height) override;
		void RenderButtons();
private:
	
	glm::vec3 camPivot;
	Gizmo *gizmo;
	Camera camera;

	glm::mat4 model_matrix, view_matrix, projection_matrix;
	BaseMesh *lineMesh;
	BaseMesh *gizmoLine, *gizmoCone;
	
	Grid *grid;
	BaseMesh *pointMesh;
	Sprite *fsQuad, *textSprite;
	TextRenderer mTextRenderer;
	int selectedIndex = -1;

	bool keyStates[256];
	bool m_LMB = false, m_RMB = false, m_MMB = false;
	bool m_altDown = false;

	float m_deltaTime;
	int m_width = 1280, m_height = 720;

	
	lab::Framebuffer colorPickingFB;
	GLubyte *readPixels;
	unsigned int quadTexture;
	glm::vec3 gizmoPos;
	glm::ivec2 prev_mousePos;
	glm::vec2 prev_ssdir = glm::vec2(0, 0);
	ColorGenerator colorGen;
	Texture2D *selectToolPic, *moveToolPic, *rotateToolPic, *buttonUp, *buttonDown, *buttonDisabled, *planeSliceToolPic;
	Texture2D *displayShadedPic, *displayNormalsPic, *displayPatchesPic, *displaySobelPic, *displayVertsPic, *displayEdgesPic, *changeBGPic;
	Sprite *m_sprite;
	bool showColorPickingFB = false;
	
	ActiveToolType toolType = SELECT_TOOL;

	ColorGenerator colorGenerator;
	std::unordered_map<uint64_t, Bone*> boneHashes;
	std::vector<Bone*> allBones;
	Bone *activeBone = NULL, *effector = NULL;

	struct DebugPoint { glm::vec3 pos, color; };
	std::vector<DebugPoint> debugPoints;

	glm::vec3 crtEffectorPos, prevEffectorPos;
};