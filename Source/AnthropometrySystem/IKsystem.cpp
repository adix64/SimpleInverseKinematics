#include "IKsystem.h"
#include <set>
#include <vector>
#include <string>
#include <iostream>
#include "DisjointSets.hpp"
#include <Core/Engine.h>
#include <algorithm>
#include "MeshSlicing.hpp"
#include "MeshPatches.hpp"
#include <chrono>
#define PI 3.1415926f

int bodyDrawMode = 0;
bool drawBodyWireframe = false;
bool drawBodyPoints = false;
bool drawFeatureMap = false;
bool invertColor = false;
int backgroundID = 0;
glm::vec3 backgroundColors[5] = {glm::vec3(0.7f, 0.85f, 1.f), glm::vec3(1),
								 glm::vec3(0.33), glm::vec3(0.66), glm::vec3(0.5)};
#define GUI_FRACTION 16


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IKsystem::LoadShaders()
{
	{// DULL COLOR
		Shader *shader = new Shader("DullColorShader");
		shader->AddShader("Shaders/pointVertexShader.glsl", GL_VERTEX_SHADER);
		shader->AddShader("Shaders/fragment.glsl", GL_FRAGMENT_SHADER);
		shader->CreateAndLink();
		shaders[shader->GetName()] = shader;
	}
	{// FULL-SCREEN SHADER
		Shader *shader = new Shader("FullScreenShader");
		shader->AddShader("Shaders/fullscreenVertex.glsl", GL_VERTEX_SHADER);
		shader->AddShader("Shaders/fullscreenFragment.glsl", GL_FRAGMENT_SHADER);
		shader->CreateAndLink();
		shaders[shader->GetName()] = shader;
	}

	{// DULL COLOR
		Shader *shader = new Shader("Text");
		shader->AddShader("Shaders/textVertex.glsl", GL_VERTEX_SHADER);
		shader->AddShader("Shaders/textFragment.glsl", GL_FRAGMENT_SHADER);
		shader->CreateAndLink();
		shaders[shader->GetName()] = shader;
	}

	//DEFAULT LIT GREY SHADER(hardcoded DirLight in shader)
	{
		Shader *shader = new Shader("default");
		shader->AddShader("Shaders/defaultVertexShader.glsl", GL_VERTEX_SHADER);
		shader->AddShader("Shaders/defaultFragmentShader.glsl", GL_FRAGMENT_SHADER);
		shader->CreateAndLink();
		shaders[shader->GetName()] = shader;
	}

	//FLAT COLOR SHADER
	{
		Shader *shader = new Shader("flat");
		shader->AddShader("Shaders/defaultVertexShader.glsl", GL_VERTEX_SHADER);
		shader->AddShader("Assets/Shaders/Color.FS.glsl", GL_FRAGMENT_SHADER);
		shader->CreateAndLink();
		shaders[shader->GetName()] = shader;
	}

	//VERTEX COLOR SHADER
	{
		Shader *shader = new Shader("VertexColor");
		shader->AddShader("Shaders/defaultVertexShader.glsl", GL_VERTEX_SHADER);
		shader->AddShader("Assets/Shaders/VertexColor.FS.glsl", GL_FRAGMENT_SHADER);
		shader->CreateAndLink();
		shaders[shader->GetName()] = shader;
	}
}

int FurthestPointAlongDir(std::unordered_map<int, glm::vec3> &verts, glm::vec3 &direction) {
	uint farthestIDX = 0;
	GLfloat farthest = glm::dot(verts[0], direction);
	for (auto it = verts.begin(); it != verts.end(); it++) {
		GLfloat dist = glm::dot(it->second, direction);
		if (dist > farthest) {
			farthestIDX = it->first;
			farthest = dist;
		}
	}
	return farthestIDX;
}

int ClosestSliceToPoint(std::unordered_map<int, glm::vec3> &centroids, glm::vec3 pPoint, float thresh = 99999.f)
{
	float minDist = 99999.f, dist;
	int bestID;
	for (auto it = centroids.begin(); it != centroids.end(); it++)
	{
		dist = glm::length(it->second - pPoint);
		if (dist < minDist)
		{
			minDist = dist;
			bestID = it->first;
		}
	}
	if (minDist < thresh)
		return bestID;
	else
		return -666;
}

std::vector<glm::vec3> ggColors{
	glm::vec3(0,0.2,1), glm::vec3(1,0.5,0), glm::vec3(0.5,0.5,0), glm::vec3(0,1,0), glm::vec3(0,1, 0.5), glm::vec3(0,0.5,1), glm::vec3(0,0,1), glm::vec3(0,1,1),
	glm::vec3(1,0,0), glm::vec3(1,0.5,0), glm::vec3(0.5,0.5,0), glm::vec3(0,1,0), glm::vec3(0,1, 0.5), glm::vec3(0,0.5,1), glm::vec3(0,0,1), glm::vec3(0,1,1),
	glm::vec3(1,0,0), glm::vec3(1,0.5,0), glm::vec3(0.5,0.5,0), glm::vec3(0,1,0), glm::vec3(0,1, 0.5), glm::vec3(0,0.5,1), glm::vec3(0,0,1), glm::vec3(0,1,1),
	glm::vec3(1,0,0), glm::vec3(1,0.5,0), glm::vec3(0.5,0.5,0), glm::vec3(0,1,0), glm::vec3(0,1, 0.5), glm::vec3(0,0.5,1), glm::vec3(0,0,1), glm::vec3(0,1,1)
};

IKsystem::IKsystem()
{
	
	std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

	m_width = 800; m_height = 450;
	glClearDepth(1);
	glEnable(GL_DEPTH_TEST);
	LoadShaders();
	camPivot = glm::vec3(0);
	model_matrix = glm::mat4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
	view_matrix = glm::lookAt(glm::vec3(-5, 10, 75), glm::vec3(5, 10, 0), glm::vec3(0, 1, 0));

	//wireframe draw mode
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	camera = Camera(glm::vec3(0, 60, 80), glm::vec3(0, 30, 0), glm::vec3(0, 1, 0));
	memset(keyStates, 0, 256);


	gizmo = new Gizmo();
	gizmo->Init(shaders["DullColorShader"], &view_matrix, &projection_matrix);

	grid = new Grid();
	grid->Init(shaders["DullColorShader"], &view_matrix, &projection_matrix);

	colorPickingFB.generate(m_width, m_height);

	readPixels = (GLubyte*)malloc(3 * m_width * m_height);

	fsQuad = new Sprite(shaders["FullScreenShader"], &m_width, &m_height, glm::vec3(-1, -1, 0), glm::vec3(1, 1, 0));
	textSprite = new Sprite(shaders["FullScreenShader"], &m_width, &m_height, glm::vec3(-1, -1, 0), glm::vec3(1, 1, 0));


	std::vector<glm::uvec3> reservedColors = {
		glm::uvec3(255, 0, 0), glm::uvec3(0, 255, 0), glm::uvec3(0, 0, 255),
		glm::uvec3(255, 255, 0), glm::uvec3(255, 0, 255), glm::uvec3(0, 127, 127),
		glm::uvec3(127, 0, 0), glm::uvec3(0, 127, 0), glm::uvec3(0, 0, 127),
		glm::uvec3(127, 127, 0), glm::uvec3(127, 0, 127), glm::uvec3(0, 127, 127),
		glm::uvec3(0, 128, 128), glm::uvec3(128, 0, 0), glm::uvec3(0, 128, 0),
		glm::uvec3(0, 0, 128), glm::uvec3(128, 128, 0), glm::uvec3(128, 0, 128),
		glm::uvec3(0, 128, 128), glm::uvec3(64, 64, 64), glm::uvec3(64, 64, 127),
		glm::uvec3(64, 127, 127), glm::uvec3(64, 127, 191),	glm::uvec3(191, 127, 191),
		glm::uvec3(191, 0, 191), glm::uvec3(191, 64, 191)
	};
	colorGen.SetReservedColors(reservedColors);
	mTextRenderer.Init(shaders["Text"], "Assets/Fonts/crkdownr.ttf" );
	//TextOutliner.Init(shaders["Text"], "Assets/Fonts/crkdwno2.ttf");
	InitIKsystem();
	
	
	//BuildFeatureMap(mesh, mesh1);
	OnWindowResize(800, 450);

	std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
	cout << "Processing time : " << duration << endl;

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IKsystem::AddBone(glm::vec3 pos, Bone *parent, glm::vec3 color)
{
	
	allBones.push_back(new Bone(pos, parent));
	activeBone = allBones[allBones.size() - 1];
	activeBone->color = glm::vec3(color);
	if (parent)
		parent->children.push_back(activeBone);
	activeBone->colorFBOid = colorGen.getNextColor();
	boneHashes[calculateColorHash(activeBone->colorFBOid)] = activeBone;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IKsystem::InitIKsystem()
{
	int cnt = 0;
	
	AddBone(glm::vec3(0), NULL);// , glm::vec3(0, 1, 0));
	AddBone(glm::vec3(0,10,0), activeBone);
	AddBone(glm::vec3(0, 20, 0), activeBone);
	AddBone(glm::vec3(0, 30, 0), activeBone);

	//EFFECTOR
	AddBone(glm::vec3(0, 30, 0), NULL, glm::vec3(0,1,0));
	effector = activeBone;
	effector->pickable = true;

	crtEffectorPos = prevEffectorPos = glm::vec3(0, 30, 0);

		//mDummyPlane = activeBone;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

IKsystem::~IKsystem()
{
	//distruge shader
	//distruge mesh incarcat
	delete lineMesh;
	delete pointMesh;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IKsystem::AddBoneAtScreenPoint(glm::vec2 screenSpacePos)
{
	float leftLimit = (float)m_width / (float)GUI_FRACTION;
	if (screenSpacePos.x < leftLimit)
		return;
	screenSpacePos.x -= leftLimit;
	screenSpacePos.x *= 1 + leftLimit / (float)m_width;

	screenSpacePos.x = screenSpacePos.x / (float)m_width * 2.f - 1.f;
	screenSpacePos.y = (m_height - screenSpacePos.y) / (float)m_height * 2.f - 1.f;
	
	glm::vec4 worldSpacePos = glm::inverse(projection_matrix * view_matrix) * glm::vec4(screenSpacePos.x, screenSpacePos.y, 0.95, 1);

	worldSpacePos.w = 1.0 / worldSpacePos.w;
	worldSpacePos.x *= worldSpacePos.w;
	worldSpacePos.y *= worldSpacePos.w;
	worldSpacePos.z *= worldSpacePos.w;
#pragma message "HANDLE DIVISION BY ZER(0add)!!!"
	const glm::vec3 camPos = camera.GetPosition();
	glm::vec3 dirVec = glm::normalize(glm::vec3(worldSpacePos) - camPos);
	worldSpacePos = glm::vec4(camPos - dirVec * camPos.z / dirVec.z, 1); //TODO
	allBones.push_back(new Bone(glm::vec3(worldSpacePos), activeBone));
	activeBone = allBones[allBones.size() - 1];
	activeBone->colorFBOid = colorGen.getNextColor();
	printf("[COLOR GENERATOR]: Next Color: %d %d %d \n", activeBone->colorFBOid.x, activeBone->colorFBOid.y, activeBone->colorFBOid.z);
	boneHashes[calculateColorHash(activeBone->colorFBOid)] = activeBone;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IKsystem::LoadMaterials()
{ 
	selectToolPic = new Texture2D();
	selectToolPic->Load2D("Assets/selectTool.png", GL_REPEAT);

	moveToolPic = new Texture2D();
	moveToolPic->Load2D("Assets/moveTool.png", GL_REPEAT);

	rotateToolPic = new Texture2D();
	rotateToolPic->Load2D("Assets/rotateTool.png", GL_REPEAT);

	buttonUp= new Texture2D();
	buttonUp->Load2D("Assets/buttonUp.png", GL_REPEAT);

	buttonDown = new Texture2D();
	buttonDown->Load2D("Assets/buttonDown.png", GL_REPEAT);
	
	buttonDisabled = new Texture2D();
	buttonDisabled->Load2D("Assets/buttonDisabled.png", GL_REPEAT);
	
	planeSliceToolPic = new Texture2D();
	planeSliceToolPic->Load2D("Assets/planeSliceTool.png", GL_REPEAT);

	displayShadedPic = new Texture2D();
	displayShadedPic->Load2D("Assets/displayShaded.png", GL_REPEAT);
	displayNormalsPic = new Texture2D();
	displayNormalsPic->Load2D("Assets/displayNormals.png", GL_REPEAT);
	displayPatchesPic = new Texture2D();
	displayPatchesPic->Load2D("Assets/displayPatches.png", GL_REPEAT);
	displaySobelPic = new Texture2D();
	displaySobelPic->Load2D("Assets/displaySobel.png", GL_REPEAT);
	displayVertsPic = new Texture2D();
	displayVertsPic->Load2D("Assets/displayVerts.png", GL_REPEAT);
	displayEdgesPic = new Texture2D();
	displayEdgesPic->Load2D("Assets/displayEdges.png", GL_REPEAT);
	changeBGPic = new Texture2D();
	changeBGPic->Load2D("Assets/changeBackground.png", GL_REPEAT);
	m_sprite = new Sprite(shaders["FullScreenShader"], &m_width, &m_height);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IKsystem::LoadMeshes()
{
	lineMesh = generateLineMesh();
	pointMesh = generatePointMesh();
	Mesh* mesh = new Mesh("male");
	mesh->LoadMesh(RESOURCE_PATH::MODELS + "Characters", "male2.obj");
	meshes[mesh->GetMeshID()] = mesh;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IKsystem::Init()
{
	LoadMaterials();
	LoadMeshes();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IKsystem::RenderSimpleMesh(Mesh *mesh, Shader *shader, const glm::mat4 & modelMatrix, Texture2D* texture1, Texture2D* texture2, glm::vec3 color)
{
	if (!mesh || !shader || !shader->GetProgramID())
		return;

	// render an object using the specified shader and the specified position
	glUseProgram(shader->program);

	// Bind model matrix
	GLint loc_model_matrix = glGetUniformLocation(shader->program, "Model");
	glUniformMatrix4fv(loc_model_matrix, 1, GL_FALSE, glm::value_ptr(modelMatrix));

	// Bind view matrix
	int loc_view_matrix = glGetUniformLocation(shader->program, "View");
	glUniformMatrix4fv(loc_view_matrix, 1, GL_FALSE, glm::value_ptr(view_matrix));

	int loc_projection_matrix = glGetUniformLocation(shader->program, "Projection");
	glUniformMatrix4fv(loc_projection_matrix, 1, GL_FALSE, glm::value_ptr(projection_matrix));

	if (texture1)
	{
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, texture1->GetTextureID());
		glUniform1i(glGetUniformLocation(shader->GetProgramID(), "texture1"), 0);
	}

	int colLoc = glGetUniformLocation(shader->GetProgramID(), "color");
	if (colLoc >= 0)
	{
		glUniform3f(colLoc, color.x, color.y, color.z);
	}
	// Draw the object
	glBindVertexArray(mesh->GetBuffers()->VAO);
	glDrawElements(mesh->GetDrawMode(), static_cast<int>(mesh->indices.size()), GL_UNSIGNED_SHORT, 0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IKsystem::RenderBody()
{
	glEnable(GL_DEPTH_TEST);

	glLineWidth(2);
	glm::mat4 modelMatrix = glm::mat4(1);// glm::scale(glm::mat4(1), glm::vec3(0.5f));


	glm::vec3 meshColor, wireframeColor, pointsColor;
	switch (bodyDrawMode)
	{
	case 0:
		meshColor = glm::vec3(0.8, 1, 1);
		wireframeColor = glm::vec3(0.5, 1.2, 1.6);
		pointsColor = glm::vec3(10, 10, 1);
		break;
	case 1:
		meshColor = glm::vec3(1);
		wireframeColor = glm::vec3(10);
		pointsColor = glm::vec3(0.5);
		break;
	case 2:
		meshColor = glm::vec3(1);
		wireframeColor = glm::vec3(0.5);
		pointsColor = glm::vec3(1.2);
		break;
	default:
		meshColor = glm::vec3(0.8, 1, 1);
		wireframeColor = glm::vec3(0.5, 1.2, 1.6);
		pointsColor = glm::vec3(10, 10, 1);
		break;
	}

	shaders["default"]->Use();
	glUniform1i(glGetUniformLocation(shaders["default"]->GetProgramID(), "mode"), bodyDrawMode);

	glUniform1i(glGetUniformLocation(shaders["default"]->GetProgramID(), "invertColor"), invertColor);

	Mesh *m = meshes["male"];// (bodyDrawMode != 3) ? meshes["male"] : meshes["male1"];
	glCullFace(GL_BACK);
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		RenderSimpleMesh(m, shaders["default"], modelMatrix, NULL, NULL, meshColor);
	}
	if (drawBodyWireframe)
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		RenderSimpleMesh(m, shaders["default"], modelMatrix, NULL, NULL, wireframeColor);
	}
	if (drawBodyPoints)
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		m->SetDrawMode(GL_POINTS);
		float l = glm::length(camera.m_pos - glm::vec3(0, 50, 0)) / 50.f;
		glPointSize(3 * (2.f - glm::clamp(l, 0.f, 1.f)));
		RenderSimpleMesh(m, shaders["default"], modelMatrix, NULL, NULL, pointsColor);
		m->SetDrawMode(GL_TRIANGLES);

	}

	glDepthMask(GL_TRUE);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//void IKsystem::RenderSceneText(std::vector<Bone *> &vec, glm::vec3 &color)
//{
//#define RENDER_LABELS
//#ifdef RENDER_LABELS
//	glDisable(GL_DEPTH_TEST);
//	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//	char periVal[32];
//	for (int i = 0; i < vec.size(); i++)
//	{//DRAW INTERSECTION CURVES' CENTERS
//		for (auto it = vec[i]->centroids.begin(); it != vec[i]->centroids.end(); it++)
//		{
//			if (!vec[i]->clusterVisibility[it->first])
//				continue;
//			glm::vec4 sspoint = projection_matrix * view_matrix * glm::translate(glm::mat4(1), it->second) * glm::vec4(0, 0, 0, 1);
//			float x = sspoint.x / sspoint.w, y = sspoint.y / sspoint.w;
//			memset(periVal, 0, 32);
//			sprintf_s(periVal, "%.0f", vec[i]->winnerPerimeter * 2.f);
//			mTextRenderer.RenderText(std::string(periVal), x, y, 0.4, glm::vec3(0.85, 0.45, 0));
//		}
//
//	}
//	glEnable(GL_DEPTH_TEST);
//#endif
//}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IKsystem::FrameStart()
{
}

void IKsystem::DrawLine(Shader *shader, glm::vec3 &p1, glm::vec3 &p2, glm::vec3 &color)
{
	glLineWidth(5);
	float dist = glm::distance(p2, p1);
	if (dist < 0.0001f)
		return;

	glm::mat4 scaleMat = glm::scale(glm::mat4(1), glm::vec3(dist));
	glm::mat4 rotationMat = glm::mat4(1);
	glm::mat4 translationMat = glm::translate(glm::mat4(1), glm::vec3(p1));
	glm::vec3 lineDir = glm::normalize(p2 - p1);
	
	float angleCosine = glm::dot(lineDir, glm::vec3(0, 1, 0));

	
	if (fabs(angleCosine + 1.f) < 0.0001f) 
	{// 180 degrees
		rotationMat = glm::rotate(glm::mat4(1), PI, glm::vec3(1, 0, 0));
	}
	else if (fabs(angleCosine - 1.f) > 0.0001f)
	{ //0 degrees doesn't require rot
		rotationMat = glm::rotate(glm::mat4(1), acos(angleCosine), glm::cross(glm::vec3(0, 1, 0), lineDir));
	}
	
	glUniformMatrix4fv(shaders["DullColorShader"]->GetUniformLocation(std::string("model_matrix")),
								1, false, glm::value_ptr(translationMat * rotationMat * scaleMat));
	lineMesh->draw(GL_LINES);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IKsystem::IKSolverUpdate()
{
	glm::vec3 rootPosition = allBones[0]->pos;

	int endBoneId = allBones.size() - 2;
	
	allBones[endBoneId]->pos = effector->pos;
	while (endBoneId > 0)
	{
		allBones[endBoneId - 1]->pos = allBones[endBoneId]->pos + glm::normalize(allBones[endBoneId - 1]->pos - allBones[endBoneId]->pos) * 10.f;
		endBoneId--;
	}

	glm::vec3 rootOffset = allBones[0]->pos - rootPosition;
	for (int i = 0; i < allBones.size() - 1; i++)
	{
		allBones[i]->pos -= rootOffset;
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IKsystem::Update(float deltaTimeSeconds)
{
	crtEffectorPos = effector->pos;

	if (glm::distance(crtEffectorPos, prevEffectorPos) > 0.0001)
	{
		IKSolverUpdate();
	}

	m_deltaTime = deltaTimeSeconds;

	colorPickingFB.unbind();
	glEnable(GL_DEPTH_TEST);
	
	glLineWidth(3);
	glViewport(m_width / GUI_FRACTION , 0, m_width - m_width / GUI_FRACTION - m_width / 3, m_height);
	glClearColor(backgroundColors[backgroundID].r, backgroundColors[backgroundID].g, backgroundColors[backgroundID].b, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	
	
	//RenderBody();

	glUseProgram(shaders["DullColorShader"]->GetProgramID());

	view_matrix = camera.GetViewMatrix();
	//projection_matrix = glm::perspective(45.f, (float)m_width/ (float)m_height, 1.f, 200.f);
	glUniformMatrix4fv(shaders["DullColorShader"]->GetUniformLocation(std::string("view_matrix")), 1, false, glm::value_ptr(view_matrix));
	glUniformMatrix4fv(shaders["DullColorShader"]->GetUniformLocation(std::string("projection_matrix")), 1, false, glm::value_ptr(projection_matrix));
	//foloseste shaderul
	glEnable(GL_DEPTH_TEST);
	grid->DrawGrid(glm::scale(glm::mat4(1), glm::vec3(0.5f)), glm::vec3(0,0,0));

///////////DRAW POINTS
	glUseProgram(shaders["DullColorShader"]->GetProgramID());
	glDisable(GL_DEPTH_TEST);
	model_matrix = glm::mat4(1);
	glUniformMatrix4fv(shaders["DullColorShader"]->GetUniformLocation(std::string("model_matrix")), 1, false, glm::value_ptr(model_matrix));
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glUniform3f(shaders["DullColorShader"]->GetUniformLocation(std::string("color")), 1, 1, 1);
	
	glPointSize(14);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	if (activeBone != NULL)
	{
		if (toolType == MOVE_TOOL || toolType == ROTATE_TOOL)
		{
			gizmo->SetVisible(true);
			gizmoPos = activeBone->pos;
		}
		else
		{
			gizmo->SetVisible(false);
		}	
	}
	else
	{
		gizmo->SetVisible(false);
	}
	
#define DRAW_PLANES_AND_POINTS
#ifdef DRAW_PLANES_AND_POINTS
	glDisable(GL_DEPTH_TEST);
	
	glUseProgram(shaders["DullColorShader"]->GetProgramID());
	glUniformMatrix4fv(shaders["DullColorShader"]->GetUniformLocation(std::string("view_matrix")), 1, false, glm::value_ptr(view_matrix));
	glUniformMatrix4fv(shaders["DullColorShader"]->GetUniformLocation(std::string("projection_matrix")), 1, false, glm::value_ptr(projection_matrix));

	for (int i = 0; i < allBones.size(); i++)
	{
		model_matrix = glm::translate(glm::mat4(1), allBones[i]->pos);
		glUniform3f(shaders["DullColorShader"]->GetUniformLocation(std::string("color")), 
			allBones[i]->color.r, allBones[i]->color.g, allBones[i]->color.b);
		glUniformMatrix4fv(shaders["DullColorShader"]->GetUniformLocation(std::string("model_matrix")), 1, false, glm::value_ptr(model_matrix));
		pointMesh->draw(GL_POINTS);
		for (int j = 0; j < allBones[i]->children.size(); j++)
		{
			DrawLine(shaders["DullColorShader"], allBones[i]->pos, allBones[i]->children[j]->pos);
		}
	}
#endif


#define DEBUG_DRAW_POINTS
#ifdef DEBUG_DRAW_POINTS
	for (int i = 0; i < debugPoints.size(); i++)
	{//DRAW INTERSECTION CURVES' CENTERS

		model_matrix = glm::translate(glm::mat4(1), debugPoints[i].pos);
		glUniform3f(shaders["DullColorShader"]->GetUniformLocation(std::string("color")), debugPoints[i].color.r, debugPoints[i].color.g, debugPoints[i].color.b);
		glUniformMatrix4fv(shaders["DullColorShader"]->GetUniformLocation(std::string("model_matrix")), 1, false, glm::value_ptr(model_matrix));
		pointMesh->draw(GL_POINTS);


	}
#endif


	glDisable(GL_DEPTH_TEST);

	//mTextOutliner.RenderText(std::string("This is sample text"), .0f, .0f, 2.0f, glm::vec3(0));
	
	gizmo->Render(camera, gizmoPos);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////// COLOR PICKING FB ///////////////////////////////////////////////////////////////////// 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	colorPickingFB.bind();
	glClearColor(0.0f, 0.0f, 0.0f, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(m_width / GUI_FRACTION, 0, m_width - m_width / GUI_FRACTION - m_width / 3, m_height);
	glEnable(GL_DEPTH_TEST);
	
	glUseProgram(shaders["DullColorShader"]->GetProgramID());
	glUniformMatrix4fv(shaders["DullColorShader"]->GetUniformLocation(std::string("view_matrix")), 1, false, glm::value_ptr(view_matrix));
	glUniformMatrix4fv(shaders["DullColorShader"]->GetUniformLocation(std::string("projection_matrix")), 1, false, glm::value_ptr(projection_matrix));
	for (int i = 0; i < allBones.size(); i++)
	{
		if (!allBones[i]->pickable)
			continue;
		model_matrix = glm::translate(glm::mat4(1), allBones[i]->pos);
		glUniform3f(shaders["DullColorShader"]->GetUniformLocation(std::string("color")),
												allBones[i]->colorFBOid.x / 255.f, 
												allBones[i]->colorFBOid.y / 255.f, 
												allBones[i]->colorFBOid.z / 255.f);
		glUniformMatrix4fv(shaders["DullColorShader"]->GetUniformLocation(std::string("model_matrix")), 1, false, glm::value_ptr(model_matrix));
		pointMesh->draw(GL_POINTS);
	}

	glLineWidth(8);
	gizmo->Render(camera, gizmoPos);
	
	colorPickingFB.unbind();
	
	RenderButtons();
	
	colorPickingFB.bind();
	
	glReadPixels(0, 0, m_width, m_height, GL_RGB, GL_UNSIGNED_BYTE, readPixels);
	
	glDeleteTextures(1, &quadTexture);
	
	quadTexture = loadTexture(readPixels, m_width, m_height);
	
	colorPickingFB.unbind();
	
}

////////////////////////////////////////////////////////////////////////////////
void IKsystem::RenderButtons()
{
	glViewport(0, 0, m_width, m_height);
	
	Texture2D *selectBtnBG = buttonUp, *moveBtnBG = buttonUp, *planeSliceBtnBG = buttonUp, *rotateBtnBG = buttonUp;
	Texture2D *shadedMeshBG = buttonUp, *normalsMeshBG = buttonUp, *patchesMeshBG = buttonUp, *sobelMeshBG = buttonUp;
	switch(toolType)
	{
	case SELECT_TOOL:
		selectBtnBG = buttonDown;
		break;
	case MOVE_TOOL:
		moveBtnBG = buttonDown;
		break;
	case ROTATE_TOOL:
		rotateBtnBG = buttonDown;
		break;
	case PLANE_SLICE_TOOL:
		planeSliceBtnBG = buttonDown;
		break;
	default:
		break;
	}

	switch (bodyDrawMode)
	{
	case 0:
		shadedMeshBG = buttonDown;
		break;
	case 1:
		normalsMeshBG = buttonDown;
		break;
	case 2:
		patchesMeshBG = buttonDown;
		break;
	case 3:
		sobelMeshBG = buttonDown;
		break;
	default:
		shadedMeshBG = buttonDown;
		break;
	}

	


	m_sprite->SetCorners(glm::vec3(-1, 0.8, 0), glm::vec3(-0.9, 1, 0));
	m_sprite->Render(selectBtnBG);
	m_sprite->Render(selectToolPic);

	m_sprite->SetCorners(glm::vec3(-1, 0.6, 0), glm::vec3(-0.9, 0.8, 0));
	m_sprite->Render(moveBtnBG);
	m_sprite->Render(moveToolPic);

	m_sprite->SetCorners(glm::vec3(-1, 0.4, 0), glm::vec3(-0.9, 0.6, 0));
	m_sprite->Render(rotateBtnBG);
	m_sprite->Render(rotateToolPic);


	m_sprite->SetCorners(glm::vec3(-1, 0.2, 0), glm::vec3(-0.9, 0.4, 0));
	m_sprite->Render(planeSliceBtnBG);
	m_sprite->Render(planeSliceToolPic);

	m_sprite->SetCorners(glm::vec3(-1, 0.2-0.05, 0), glm::vec3(-0.9, 0.1 - 0.05, 0));
	m_sprite->Render(shadedMeshBG);
	m_sprite->Render(displayShadedPic);

	m_sprite->SetCorners(glm::vec3(-1, 0.1 - 0.05, 0), glm::vec3(-0.9, 0 - 0.05, 0));
	m_sprite->Render(normalsMeshBG);
	m_sprite->Render(displayNormalsPic);

	m_sprite->SetCorners(glm::vec3(-1, 0 - 0.05, 0), glm::vec3(-0.9, -0.1 - 0.05, 0));
	m_sprite->Render(patchesMeshBG);
	m_sprite->Render(displayPatchesPic);

	m_sprite->SetCorners(glm::vec3(-1, -0.1 - 0.05, 0), glm::vec3(-0.9, -0.2 - 0.05, 0));
	m_sprite->Render(sobelMeshBG);
	m_sprite->Render(displaySobelPic);

	m_sprite->SetCorners(glm::vec3(-1, -0.2 - 0.1, 0), glm::vec3(-0.95, -0.3 - 0.1, 0));
	m_sprite->Render(drawBodyPoints?buttonDown:buttonUp);
	m_sprite->Render(displayVertsPic);

	m_sprite->SetCorners(glm::vec3(-0.95, -0.2 - 0.1, 0), glm::vec3(-0.9, -0.3 - 0.1, 0));
	m_sprite->Render(drawBodyWireframe ? buttonDown : buttonUp);
	m_sprite->Render(displayEdgesPic);

	m_sprite->SetCorners(glm::vec3(-1, -0.3 - 0.1, 0), glm::vec3(-0.9, -0.5 - 0.1, 0));
	m_sprite->Render(buttonUp);
	m_sprite->Render(changeBGPic);

	//COLOR PICKING FB
	colorPickingFB.bind();
	m_sprite->SetCorners(glm::vec3(-1, 0.8, 0), glm::vec3(-0.9, 1, 0));
	m_sprite->Render(glm::vec3(1, 1, 0));
	
	m_sprite->SetCorners(glm::vec3(-1, 0.6, 0), glm::vec3(-0.9, 0.8, 0));
	m_sprite->Render(glm::vec3(1, 0, 1));

	m_sprite->SetCorners(glm::vec3(-1, 0.4, 0), glm::vec3(-0.9, 0.6, 0));
	m_sprite->Render(glm::vec3(1, 0, 0.5));
	
	m_sprite->SetCorners(glm::vec3(-1, 0.2, 0), glm::vec3(-0.9, 0.4, 0));
	m_sprite->Render(glm::vec3(0, 1, 1));
	//
	m_sprite->SetCorners(glm::vec3(-1, 0.2 - 0.05, 0), glm::vec3(-0.9, 0.1 - 0.05, 0));
	m_sprite->Render(glm::vec3(0.25, 0.25, 0.25));

	m_sprite->SetCorners(glm::vec3(-1, 0.1 - 0.05, 0), glm::vec3(-0.9, 0 - 0.05, 0));
	m_sprite->Render(glm::vec3(0.25, 0.25, 0.5));

	m_sprite->SetCorners(glm::vec3(-1, 0 - 0.05, 0), glm::vec3(-0.9, -0.1 - 0.05, 0));
	m_sprite->Render(glm::vec3(0.25, 0.5, 0.5));

	m_sprite->SetCorners(glm::vec3(-1, -0.1 - 0.05, 0), glm::vec3(-0.9, -0.2 - 0.05, 0));
	m_sprite->Render(glm::vec3(0.25, 0.5, 0.75));

	m_sprite->SetCorners(glm::vec3(-1, -0.2 - 0.1, 0), glm::vec3(-0.95, -0.3 - 0.1, 0));
	m_sprite->Render(glm::vec3(0.75, 0.5, 0.75));

	m_sprite->SetCorners(glm::vec3(-0.95, -0.2 - 0.1, 0), glm::vec3(-0.9, -0.3 - 0.1, 0));
	m_sprite->Render(glm::vec3(0.75, 0, 0.75));

	m_sprite->SetCorners(glm::vec3(-1, -0.3 - 0.1, 0), glm::vec3(-0.9, -0.5 - 0.1, 0));
	m_sprite->Render(glm::vec3(0.75, 0.25, 0.75));

	colorPickingFB.unbind();
#define	DEBUG_MODE_READPIXELS
#ifdef DEBUG_MODE_READPIXELS
	if (showColorPickingFB)
		fsQuad->Render(quadTexture);
#endif

}
//functie chemata dupa ce am terminat cadrul de desenare (poate fi folosita pt modelare/simulare)
void notifyEndFrame() {}
//functei care e chemata cand se schimba dimensiunea ferestrei initiale
void IKsystem::OnWindowResize(int width, int height)
{
	mTextRenderer.Resize(width, height);
	//mTextOutliner.Resize(width, height);
	//reshape
	if (height == 0) height = 1;
	m_width = width; m_height = height;
	float aspect = (float)width / (float)height;
	colorPickingFB.destroy();
	colorPickingFB.generate(width, height);
	free(readPixels);
	readPixels = (GLubyte*)malloc(3 * width * height);
	aspect = (float)(m_width - m_width / GUI_FRACTION - m_width / 3) / (float)height;
	projection_matrix = glm::perspective(45.0f, aspect, 1.f, 200.f);
}

void IKsystem::FrameEnd()
{
	//DrawCoordinatSystem();
}

// Documentation for the input functions can be found in: "/Source/Core/Window/InputController.h" or
// https://github.com/UPB-Graphics/Framework-EGC/blob/master/Source/Core/Window/InputController.h

void IKsystem::OnInputUpdate(float deltaTime, int mods)
{
	float speed = 2;

	if (!window->MouseHold(GLFW_MOUSE_BUTTON_RIGHT))
	{
		glm::vec3 up = glm::vec3(0, 1, 0);
		glm::vec3 right = GetSceneCamera()->transform->GetLocalOXVector();
		glm::vec3 forward = GetSceneCamera()->transform->GetLocalOZVector();
		forward = glm::normalize(glm::vec3(forward.x, 0, forward.z));
	}
}

void IKsystem::OnKeyPress(int key, int mods)
{
	keyStates[key] = true;

	if (key == GLFW_KEY_1)
		bodyDrawMode = 0;
	else if (key == GLFW_KEY_2)
		bodyDrawMode = 1;
	else if (key == GLFW_KEY_3)
		bodyDrawMode = 2;
	else if (key == GLFW_KEY_4)
		drawBodyWireframe = !drawBodyWireframe;
	else if (key == GLFW_KEY_5)
		drawBodyPoints = !drawBodyPoints;
	else if (key == GLFW_KEY_6)
		drawFeatureMap = !drawFeatureMap;
	else
	if (key == GLFW_KEY_LEFT_ALT || key == GLFW_KEY_RIGHT_ALT)
	{
		glfwSetInputMode(window->GetGLFWWindow(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		m_altDown = true;
	}else	
	if (key == 27)
	{
		exit(0);
	}
	else
	if (key == 32)
	{
		for(auto s : shaders)
		{
			s.second->Reload();
		}			
	}else
	if (key == GLFW_KEY_P)
	{
		showColorPickingFB = !showColorPickingFB;
	}else
	if (key == GLFW_KEY_Z)
	{
		camPivot = glm::vec3(0);
		camera = Camera(glm::vec3(0, 60, 80), glm::vec3(0, 30, 0), glm::vec3(0, 1, 0));
	}
	else if (key == GLFW_KEY_Q)
	{
		toolType = SELECT_TOOL;
	}
	else if (key == GLFW_KEY_I)
	{
		invertColor = !invertColor;
	}
	else if (key == GLFW_KEY_W)
	{
		toolType = MOVE_TOOL;
		gizmo->crtMode = Gizmo::GizmoMode::MOVE_MODE;
	}
	else if (key == GLFW_KEY_E)
	{
		toolType = ROTATE_TOOL;
		gizmo->crtMode = Gizmo::GizmoMode::ROTATE_MODE;
	}
	else if (key == GLFW_KEY_S)
	{
		toolType = PLANE_SLICE_TOOL;
	}

}

void IKsystem::OnKeyRelease(int key, int mods)
{
	if (key == GLFW_KEY_LEFT_ALT || key == GLFW_KEY_RIGHT_ALT)
	{
		glfwSetInputMode(window->GetGLFWWindow(), GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		m_altDown = false;
		return;
	}
	keyStates[key] = false;
	
}

//void IKsystem::ForceRedraw()
//{
//	FrameStart();
//	Update(0.02);
//	FrameEnd();
//	window->SwapBuffers();
//}


////////////////////////////////////////////////////////////////////////////////////////////////////////

void IKsystem::OnMouseMove(int mouseX, int mouseY, int deltaX, int deltaY)
{
	if (m_altDown)
	{
		float dx = (float)deltaX;
		float dy = (float)deltaY;

		if (m_LMB)
		{
			//camera.RotateAroundPointX(-dy * 0.00499f, camPivot);
			//camera.RotateAroundPointY(-dx * 0.00499f , camPivot);

			if (activeBone == NULL)
			{
				camera.RotateAroundPointX(-dy * 0.00499f, camPivot);
				camera.RotateAroundPointY(-dx * 0.00499f, camPivot);
			}
			else
			{
				camera.RotateAroundPointX(-dy * 0.00499f, gizmoPos);// activeBone->position);
				camera.RotateAroundPointY(-dx * 0.00499f, gizmoPos);// activeBone->position);
			}
			camera.FixOZRotationYup();
			//ForceRedraw();
		}
		else if (m_RMB)
		{
			camera.TranslateAlongZ((-dy + dx) * 0.01f);
			//ForceRedraw();
		}
		else if (m_MMB)
		{
#define mmbspeed 0.05f
			camera.TranslateAlongX(-dx * mmbspeed);
			camera.TranslateAlongY(dy * mmbspeed);
			camPivot += glm::vec3(-dx * mmbspeed * camera.m_right + dy * mmbspeed * camera.m_up);
			camera.FixOZRotationYup();
			//ForceRedraw();
		}
		
		//glfwSetCursorPos(window->GetGLFWWindow(), m_width / 2, m_height / 2);
	}
	else
	{
		float dx = mouseX - prev_mousePos.x;
		float dy = prev_mousePos.y - mouseY;
		
		if (activeBone != NULL)
		{
			glm::vec2 dir = glm::vec2(mouseX, -mouseY) - glm::vec2(prev_mousePos.x, -prev_mousePos.y);
			if (gizmo->getSelectedX())
			{
				glm::vec2 ssdir = glm::vec2(projection_matrix * view_matrix * glm::translate(glm::mat4(1), gizmoPos) * glm::vec4(-1, 0, 0, 1));
				ssdir -= glm::vec2(projection_matrix * view_matrix * glm::translate(glm::mat4(1), gizmoPos) * glm::vec4(-0.5, 0, 0, 1));
		
				float dirlen = sqrt(glm::length(dir));
				float ssdirlen = glm::length(ssdir);
				if (dirlen > 0.001 && ssdirlen > 0.001) {
					float dotprod = glm::dot(glm::normalize(dir), glm::normalize(ssdir));// - prev_ssdir));
					
					if (gizmo->crtMode == Gizmo::GizmoMode::MOVE_MODE)
						activeBone->pos = glm::vec3(glm::translate(glm::mat4(1), glm::vec3(-dirlen * dotprod, 0, 0)) *
							glm::vec4(activeBone->pos, 1));
					//else //TODO
						//activeBone->normal = glm::vec3(glm::rotate(glm::mat4(1), dirlen * dotprod* 0.1f, glm::vec3(1, 0, 0)) * glm::vec4(activeBone->normal, 0));
					
					gizmoPos = activeBone->pos;
				}
				prev_ssdir = ssdir;

			}
			else if (gizmo->getSelectedY())
			{
				glm::vec2 ssdir = glm::vec2(projection_matrix * view_matrix * glm::translate(glm::mat4(1), gizmoPos) * glm::vec4(0, 1, 0, 1));
				ssdir -= glm::vec2(projection_matrix * view_matrix * glm::translate(glm::mat4(1), gizmoPos) * glm::vec4(0, 0.5, 0, 1));
		
				float dirlen = sqrt(glm::length(dir));
				float ssdirlen = glm::length(ssdir);
				if (dirlen > 0.001 && ssdirlen > 0.001)
				{
					float dotprod = glm::dot(glm::normalize(dir), glm::normalize(ssdir));// - prev_ssdir));
					if (gizmo->crtMode == Gizmo::GizmoMode::MOVE_MODE)
						activeBone->pos = glm::vec3(glm::translate(glm::mat4(1), glm::vec3(0, dirlen * dotprod, 0)) *
							glm::vec4(activeBone->pos, 1));
					//else //TODO
						//activeBone->normal = glm::vec3(glm::rotate(glm::mat4(1), -dirlen * dotprod* 0.1f, glm::vec3(0, 1, 0)) * glm::vec4(activeBone->normal, 0));
					gizmoPos = activeBone->pos;
				}
				prev_ssdir = ssdir;
			}
			else if (gizmo->getSelectedZ())
			{
				glm::vec2 ssdir = glm::vec2(projection_matrix * view_matrix * glm::translate(glm::mat4(1), gizmoPos) * glm::vec4(0, 0, 1, 1));
				ssdir -= glm::vec2(projection_matrix * view_matrix * glm::translate(glm::mat4(1), gizmoPos) * glm::vec4(0, 0, 0.5, 1));
			
				float dirlen = sqrt(glm::length(dir));
				float ssdirlen = glm::length(ssdir);
				if (dirlen > 0.001 && ssdirlen > 0.001) {
					float dotprod = glm::dot(glm::normalize(dir), glm::normalize(ssdir));// - prev_ssdir));

					if (gizmo->crtMode == Gizmo::GizmoMode::MOVE_MODE)
						activeBone->pos = glm::vec3(glm::translate(glm::mat4(1), glm::vec3(0,0,dirlen * dotprod)) *
							glm::vec4(activeBone->pos, 1));
					//else //TODO
						//activeBone->normal = glm::vec3(glm::rotate(glm::mat4(1), -dirlen * dotprod * 0.1f, glm::vec3(0, 0, 1)) * glm::vec4(activeBone->normal, 0));
					gizmoPos = activeBone->pos;
				}
				prev_ssdir = ssdir;
			}
		}
	}	
	prev_mousePos = glm::ivec2(mouseX, mouseY);
}

void IKsystem::OnMouseBtnPress(int mouseX, int mouseY, int button, int mods)
{	
	if (button == 1)
	{
		m_LMB = true;
		if (!m_altDown)
		{
			glm::uvec3 readPx = glm::uvec3(readPixels[(m_width * (m_height - mouseY) + mouseX) * 3],
				readPixels[(m_width * (m_height - mouseY) + mouseX) * 3 + 1],
				readPixels[(m_width * (m_height - mouseY) + mouseX) * 3 + 2]);
			if (readPx.r == 255 && readPx.g == 0 && readPx.b == 0)
			{
				gizmo->setSelectedX(true);
			}
			else if (readPx.r == 0 && readPx.g == 255 && readPx.b == 0)
			{
				gizmo->setSelectedY(true);
			}
			else if (readPx.r == 0 && readPx.g == 0 && readPx.b == 255)
			{
				gizmo->setSelectedZ(true);
			}
			else if (readPx.r == 255 && readPx.g == 255 && readPx.b == 0)
			{
				toolType = SELECT_TOOL;
				//gizmo->SetVisible(false);
			}
			else if (readPx.r == 255 && readPx.g == 0 && readPx.b == 255)
			{
				toolType = MOVE_TOOL;
				gizmo->crtMode = Gizmo::GizmoMode::MOVE_MODE;
			}
			else if (readPx.r == 255 && readPx.g == 0 && readPx.b == 127)
			{
				toolType = ROTATE_TOOL;
				gizmo->crtMode = Gizmo::GizmoMode::ROTATE_MODE;
			}
			else if (readPx.r == 0 && readPx.g == 255 && readPx.b == 255)
			{
				toolType = PLANE_SLICE_TOOL;
				gizmo->SetVisible(false);
			}
			//m_sprite->Render(glm::vec3(0.25, 0.25, 0.25));
			//m_sprite->Render(glm::vec3(0.25, 0.25, 0.5));
			//m_sprite->Render(glm::vec3(0.25, 0.5, 0.5));
			//m_sprite->Render(glm::vec3(0.25, 0.5, 0.75));

			else if (readPx.r == 64 && readPx.g == 64 && readPx.b == 64)
			{
				bodyDrawMode = 0;
			}
			else if (readPx.r == 64 && readPx.g == 64 && readPx.b == 127)
			{
				bodyDrawMode = 1;
			}
			else if (readPx.r == 64 && readPx.g == 127 && readPx.b == 127)
			{
				bodyDrawMode = 2;
				invertColor = false;
			}
			else if (readPx.r == 64 && readPx.g == 127 && readPx.b == 191)
			{
				bodyDrawMode = 3;
				invertColor = true;
			}
			else if (readPx.r == 191 && readPx.g == 127 && readPx.b == 191) 
			{
				drawBodyPoints = !drawBodyPoints;
			}
			else if (readPx.r == 191 && readPx.g == 0 && readPx.b == 191)
			{
				drawBodyWireframe = !drawBodyWireframe;
			}
			else if (readPx.r == 191 && readPx.g == 64 && readPx.b == 191)
			{
				backgroundID = (backgroundID + 1) % 5;
			}
			else
			{

				if (toolType == SELECT_TOOL)
				{
					uint64_t id = calculateColorHash(readPx);
					std::unordered_map<uint64_t, Bone*>::iterator it = boneHashes.find(id);
					if (it != boneHashes.end())
					{
						activeBone = it->second;
						selectedIndex = calculateColorHash(readPx);
						gizmoPos = activeBone->pos;
					}
					else activeBone = NULL;
					/////////////////////////////////////////////////////////////////
				}
				else if (toolType == MOVE_TOOL || toolType == ROTATE_TOOL)
				{
					uint64_t id = calculateColorHash(readPx);
					std::unordered_map<uint64_t, Bone*>::iterator it = boneHashes.find(id);
					if (it != boneHashes.end())
					{
						activeBone = it->second;
						selectedIndex = calculateColorHash(readPx);
						gizmoPos = activeBone->pos;
					}
					else activeBone = NULL;
				}
				else if(toolType == PLANE_SLICE_TOOL)
				{
					AddBoneAtScreenPoint(glm::vec2(mouseX, mouseY));
					if(activeBone)
						gizmoPos = activeBone->pos;
				}
			}
		}
		else
		{
			//glfwSetCursorPos(window->GetGLFWWindow(), m_width / 2, m_height / 2);
		}
	}
	else if (button == 2)
	{
		m_RMB = true;
		if (!m_altDown) {
			//????????????????? TODO 
		}
	}
	else
		m_MMB = true;

}

void IKsystem::OnMouseBtnRelease(int mouseX, int mouseY, int button, int mods)
{
	gizmo->setSelectedX(false);
	gizmo->setSelectedY(false);
	gizmo->setSelectedZ(false);
	if (button == 1)
		m_LMB = false;
	else if (button == 2)
		m_RMB = false;
	else
		m_MMB = false;
}

void IKsystem::OnMouseScroll(int mouseX, int mouseY, int offsetX, int offsetY)
{
	////glfwSetCursorPos(window->GetGLFWWindow(), m_width / 2, m_height / 2);
	//float dx = m_width / 2 - mouseX;
	//float dy = m_height / 2 - mouseY;
	camera.TranslateAlongZ(offsetY);
	//ForceRedraw();
}