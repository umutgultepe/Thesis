#pragma once
#include "StdAfx.h"
using namespace physx;
using namespace std;
struct ObjVertex
 {
 float Position[3];        //Vertex
 float TexCoord[2];         //Texcoord0
 float Normal[3];     //Normal
 };





struct FaceGroup
{
	int firstIndice;
	int lastIndice;
	int materialIndice;
	int faceCount;
};

struct ObjGroup
{
	string name;
	int faceGroupCount;
	std::vector<FaceGroup*> faceGroups;
};

class ObjObject
{
private:
	int	vertexCount,
		normalCount,
		textureCount,
		faceCount,
		materialCount,
		textureFileCount,
		faceGroupCount,
		groupCount;
	float	scaleFactor;
	String name;
	std::vector<FaceGroup*> faceGroups;
	std::vector<float*> verticeCoordinates;
	std::vector<float*> initialVerticeCoordinates;
	std::vector<float*> normals;
	std::vector<float*> textureCoordinates;
	std::vector<ObjGroup*> groupList;
	std::vector<int*> faceIndices;
	std::vector<Ogre::MaterialPtr> materialList;
	std::vector<bool> fixedVertices;
	float COM[3];
	 MeshPtr object ;

public:

	ObjObject(void);
	ObjObject(const char* filename);
	~ObjObject(void);
	void loadIntoShader();
	void drawObject();
	void Reset();
	float* moveToOrigin();
	 void Scale(float scaleRatio);
	 void translate(float veec[3]);
	float* placeOnXZ(bool aboveOrBelow);
	float unitize();
	PxClothMeshDesc* loadPhysxCloth(PxSceneDesc* SceneDesc,PxClothFabric* &fabric,PxClothParticle* &points,PxTransform* tr,PxPhysics* gPhysicsSDK);
	void updateWithPhysics(PxScene* gScene,PxReal timeStep);
	float getScaleFactor();
	void loadIntoOgre(Ogre::SceneManager* sceneManager,Ogre::String name);
	void loadMtl(string folder,string filename);
	void updateWithPhysics(PxScene* gScene);
	Ogre::Entity* entity;
	void analyzeFixedVertices();
	void setVisible(bool visible);
	PxCloth* cloth; 
private:
	void loadMaterial(int index);
	 std::vector<PxU32> indices;
	  std::vector<PxVec3> normal;
	int* getFaces(string s);
	float* getArray(string s,int beginPoint);
};

