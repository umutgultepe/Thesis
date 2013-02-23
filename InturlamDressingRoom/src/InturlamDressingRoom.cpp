/*
-----------------------------------------------------------------------------
Filename:    InturlamDressingRoom.cpp
-----------------------------------------------------------------------------


This source file is generated by the
___                   _              __    __ _                  _ 
/___\__ _ _ __ ___    /_\  _ __  _ __/ / /\ \ (_)______ _ _ __ __| |
//  // _` | '__/ _ \  //_\\| '_ \| '_ \ \/  \/ / |_  / _` | '__/ _` |
/ \_// (_| | | |  __/ /  _  \ |_) | |_) \  /\  /| |/ / (_| | | | (_| |
\___/ \__, |_|  \___| \_/ \_/ .__/| .__/ \/  \/ |_/___\__,_|_|  \__,_|
|___/                 |_|   |_|                                 
Ogre 1.7.x Application Wizard for VC10 (July 2011)
http://code.google.com/p/ogreappwizards/
-----------------------------------------------------------------------------
*/
#include "StdAfx.h"
#include "InturlamDressingRoom.h"
#define Y_OFFSET 50
#define SCALING_FACTOR 5
#define MODEL_TORSO_HEIGHT 1180 //mm
#define MODEL_SHOULDER_WIDTH 450 //mm
#define USE_KINECT 0
#define USE_USER_SCALING 0
float userWidthScale=1;
float userHeightScale=1;
float userDepthScale=1;
extern float sphereRadii[];
extern float estimatedShoulderWidth;
extern float estimatedTorsoHeight;
extern float estimatedBodyHeight;

using namespace Ogre;

int numberOfCapsules=0;

OgreBites::ParamsPanel* help;
PxClothCollisionData col_data;
physx::PxClothCollisionSphere* box_collider;
std::vector<float> radii;
std::vector<Ogre::SceneNode*> ListedNodes;
//-------------------------------------------------------------------------------------
InturlamDressingRoom::InturlamDressingRoom(void)
{
	simulating=false;
	firstStep=true;
	simulationCreated=false;
	gDefaultFilterShader=PxDefaultSimulationFilterShader;
	gPhysicsSDK=0;
	usingGPU=true;
}
//-------------------------------------------------------------------------------------
InturlamDressingRoom::~InturlamDressingRoom(void)
{
	
	if (box_collider)
		delete [] box_collider;
	if (gScene)
		gScene->release();
	if (gPhysicsSDK)
		gPhysicsSDK->release();


}
void SetupDepthMaterial()
{
	// Create the texture
	Ogre::TexturePtr depthTexture = Ogre::TextureManager::getSingleton().createManual(
		"MyDepthTexture", // name
		ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		TEX_TYPE_2D,      // type
		m_Width, m_Height,         // width & height
		0,                // number of mipmaps
		PF_BYTE_BGRA,     // pixel format
		TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);


	// Create a material using the texture
	MaterialPtr material = MaterialManager::getSingleton().create(
		"DepthTextureMaterial", // name
		ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	material->getTechnique(0)->getPass(0)->createTextureUnitState("MyDepthTexture");
	material->getTechnique(0)->getPass(0)->setSceneBlending(SBT_TRANSPARENT_ALPHA);
}

void InturlamDressingRoom::createCapsule(const Ogre::String& strName, const float r,const float r2,const float d, const int nRings , const int nSegments )
{
	MeshPtr pSphere = MeshManager::getSingleton().createManual(strName, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	SubMesh *pSphereVertex = pSphere->createSubMesh();

	pSphere->sharedVertexData = new VertexData();
	VertexData* vertexData = pSphere->sharedVertexData;

	// define the vertex format
	VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
	size_t currOffset = 0;
	// positions
	vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_POSITION);
	currOffset += VertexElement::getTypeSize(VET_FLOAT3);
	// normals
	vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_NORMAL);
	currOffset += VertexElement::getTypeSize(VET_FLOAT3);
	// two dimensional texture coordinates
	vertexDecl->addElement(0, currOffset, VET_FLOAT2, VES_TEXTURE_COORDINATES, 0);
	currOffset += VertexElement::getTypeSize(VET_FLOAT2);

	// allocate the vertex buffer
	vertexData->vertexCount = (nRings + 1) * (nSegments+1);
	HardwareVertexBufferSharedPtr vBuf = HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), vertexData->vertexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
	VertexBufferBinding* binding = vertexData->vertexBufferBinding;
	binding->setBinding(0, vBuf);
	float* pVertex = static_cast<float*>(vBuf->lock(HardwareBuffer::HBL_DISCARD));

	// allocate index buffer
	pSphereVertex->indexData->indexCount = 6 * nRings * (nSegments + 1);
	pSphereVertex->indexData->indexBuffer = HardwareBufferManager::getSingleton().createIndexBuffer(HardwareIndexBuffer::IT_16BIT, pSphereVertex->indexData->indexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
	HardwareIndexBufferSharedPtr iBuf = pSphereVertex->indexData->indexBuffer;
	unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(HardwareBuffer::HBL_DISCARD));

	float fDeltaRingAngle = (Math::PI / nRings);
	float fDeltaSegAngle = (2 * Math::PI / nSegments);
	unsigned short wVerticeIndex = 0 ;

	// Generate the group of rings for the sphere
	for( int ring = 0; ring <= nRings/2; ring++ ) {
		float r0 = r * sinf (ring * fDeltaRingAngle);
		float y0 = d+r * cosf (ring * fDeltaRingAngle);

		// Generate the group of segments for the current ring
		for(int seg = 0; seg <= nSegments; seg++) {
			float x0 = r0 * sinf(seg * fDeltaSegAngle);
			float z0 = r0 * cosf(seg * fDeltaSegAngle);

			// Add one vertex to the strip which makes up the sphere
			*pVertex++ = x0;
			*pVertex++ = y0;
			*pVertex++ = z0;

			Vector3 vNormal = Vector3(x0, y0, z0).normalisedCopy();
			*pVertex++ = vNormal.x;
			*pVertex++ = vNormal.y;
			*pVertex++ = vNormal.z;

			*pVertex++ = (float) seg / (float) nSegments;
			*pVertex++ = (float) ring / (float) nRings;

			if (ring != nRings) {
				// each vertex (except the last) has six indices pointing to it
				*pIndices++ = wVerticeIndex + nSegments + 1;
				*pIndices++ = wVerticeIndex;               
				*pIndices++ = wVerticeIndex + nSegments;
				*pIndices++ = wVerticeIndex + nSegments + 1;
				*pIndices++ = wVerticeIndex + 1;
				*pIndices++ = wVerticeIndex;
				wVerticeIndex ++;
			}
		}; // end for seg
	} // end for ring



	for( int ring = nRings/2+1; ring <= nRings; ring++ ) {
		float r0 = r2 * sinf (ring * fDeltaRingAngle);
		float y0 = r2 * cosf (ring * fDeltaRingAngle);

		// Generate the group of segments for the current ring
		for(int seg = 0; seg <= nSegments; seg++) {
			float x0 = r0 * sinf(seg * fDeltaSegAngle);
			float z0 = r0 * cosf(seg * fDeltaSegAngle);

			// Add one vertex to the strip which makes up the sphere
			*pVertex++ = x0;
			*pVertex++ = y0;
			*pVertex++ = z0;

			Vector3 vNormal = Vector3(x0, y0, z0).normalisedCopy();
			*pVertex++ = vNormal.x;
			*pVertex++ = vNormal.y;
			*pVertex++ = vNormal.z;

			*pVertex++ = (float) seg / (float) nSegments;
			*pVertex++ = (float) ring / (float) nRings;

			if (ring != nRings) {
				// each vertex (except the last) has six indices pointing to it
				*pIndices++ = wVerticeIndex + nSegments + 1;
				*pIndices++ = wVerticeIndex;               
				*pIndices++ = wVerticeIndex + nSegments;
				*pIndices++ = wVerticeIndex + nSegments + 1;
				*pIndices++ = wVerticeIndex + 1;
				*pIndices++ = wVerticeIndex;
				wVerticeIndex ++;
			}
		}; // end for seg
	} // end for ring




	// Unlock
	vBuf->unlock();
	iBuf->unlock();
	// Generate face list
	pSphereVertex->useSharedVertices = true;

	// the original code was missing this line:
	pSphere->_setBounds( AxisAlignedBox( Vector3(-r-r2, -r2, -r-r2), Vector3(r+r2, r+d, r+r2) ), false );
	pSphere->_setBoundingSphereRadius(r+d);
	// this line makes clear the mesh is loaded (avoids memory leaks)
	pSphere->load();
}

void InturlamDressingRoom::createSphere(const float r, const int nRings, const int nSegments)
{
	MeshPtr pSphere = MeshManager::getSingleton().createManual("sphere_r"+StringConverter::toString(r), ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	SubMesh *pSphereVertex = pSphere->createSubMesh();

	pSphere->sharedVertexData = new VertexData();
	VertexData* vertexData = pSphere->sharedVertexData;

	// define the vertex format
	VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
	size_t currOffset = 0;
	// positions
	vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_POSITION);
	currOffset += VertexElement::getTypeSize(VET_FLOAT3);
	// normals
	vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_NORMAL);
	currOffset += VertexElement::getTypeSize(VET_FLOAT3);
	// two dimensional texture coordinates
	vertexDecl->addElement(0, currOffset, VET_FLOAT2, VES_TEXTURE_COORDINATES, 0);
	currOffset += VertexElement::getTypeSize(VET_FLOAT2);

	// allocate the vertex buffer
	vertexData->vertexCount = (nRings + 1) * (nSegments+1);
	HardwareVertexBufferSharedPtr vBuf = HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), vertexData->vertexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
	VertexBufferBinding* binding = vertexData->vertexBufferBinding;
	binding->setBinding(0, vBuf);
	float* pVertex = static_cast<float*>(vBuf->lock(HardwareBuffer::HBL_DISCARD));

	// allocate index buffer
	pSphereVertex->indexData->indexCount = 6 * nRings * (nSegments + 1);
	pSphereVertex->indexData->indexBuffer = HardwareBufferManager::getSingleton().createIndexBuffer(HardwareIndexBuffer::IT_16BIT, pSphereVertex->indexData->indexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
	HardwareIndexBufferSharedPtr iBuf = pSphereVertex->indexData->indexBuffer;
	unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(HardwareBuffer::HBL_DISCARD));

	float fDeltaRingAngle = (Math::PI / nRings);
	float fDeltaSegAngle = (2 * Math::PI / nSegments);
	unsigned short wVerticeIndex = 0 ;

	// Generate the group of rings for the sphere
	for( int ring = 0; ring <= nRings; ring++ ) {
		float r0 = r * sinf (ring * fDeltaRingAngle);
		float y0 = r * cosf (ring * fDeltaRingAngle);

		// Generate the group of segments for the current ring
		for(int seg = 0; seg <= nSegments; seg++) {
			float x0 = r0 * sinf(seg * fDeltaSegAngle);
			float z0 = r0 * cosf(seg * fDeltaSegAngle);

			// Add one vertex to the strip which makes up the sphere
			*pVertex++ = x0;
			*pVertex++ = y0;
			*pVertex++ = z0;

			Vector3 vNormal = Vector3(x0, y0, z0).normalisedCopy();
			*pVertex++ = vNormal.x;
			*pVertex++ = vNormal.y;
			*pVertex++ = vNormal.z;

			*pVertex++ = (float) seg / (float) nSegments;
			*pVertex++ = (float) ring / (float) nRings;

			if (ring != nRings) {
				// each vertex (except the last) has six indices pointing to it
				*pIndices++ = wVerticeIndex + nSegments + 1;
				*pIndices++ = wVerticeIndex;               
				*pIndices++ = wVerticeIndex + nSegments;
				*pIndices++ = wVerticeIndex + nSegments + 1;
				*pIndices++ = wVerticeIndex + 1;
				*pIndices++ = wVerticeIndex;
				wVerticeIndex ++;
			}
		}; // end for seg
	} // end for ring

	// Unlock
	vBuf->unlock();
	iBuf->unlock();
	// Generate face list
	pSphereVertex->useSharedVertices = true;

	// the original code was missing this line:
	pSphere->_setBounds( AxisAlignedBox( Vector3(-r, -r, -r), Vector3(r, r, r) ), false );
	pSphere->_setBoundingSphereRadius(r);
	// this line makes clear the mesh is loaded (avoids memory leaks)
	pSphere->load();
}

ManualObject* InturlamDressingRoom::createLine(Ogre::String name,Vector3 startPos,Vector3 endPos,Ogre::ColourValue color)
{
	ManualObject* myManualObject =  mSceneMgr->createManualObject(name); 

	MaterialPtr myManualObjectMaterial = MaterialManager::getSingleton().create(name+"material",Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME); 
	myManualObjectMaterial->setReceiveShadows(false); 
	myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setDiffuse(color); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setAmbient(color); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(color); 



	myManualObject->begin(name+"material", Ogre::RenderOperation::OT_LINE_LIST); 
	myManualObject->position(startPos); 
	myManualObject->position(endPos); 
	// etc 
	myManualObject->end(); 
	return myManualObject;
}

void InturlamDressingRoom::buildAxes()
{
	SceneNode* root=mSceneMgr->getRootSceneNode()->createChildSceneNode("Axes");
	root->attachObject(createLine("xAxis",Vector3(0,0,0),Vector3(10,0,0),Ogre::ColourValue::Red));
	root->attachObject(createLine("yAxis",Vector3(0,0,0),Vector3(0,10,0),Ogre::ColourValue::Green));
	root->attachObject(createLine("zAxis",Vector3(0,0,0),Vector3(0,0,10),Ogre::ColourValue::Blue));
}

SceneNode* InturlamDressingRoom::createLimb(Ogre::String limbName,Ogre::String childNodeName,float startRadius, Ogre::Vector3 endPosition,float endRadius,Ogre::SceneNode* node,bool inheritOrientation,Quaternion orientation)
{

	float distance=endPosition.length();
	Ogre::String name="capsule"+Ogre::StringConverter::toString(numberOfCapsules++);
	createCapsule(name,endRadius,startRadius,distance);
	Ogre::Entity* limb = mSceneMgr->createEntity(limbName, name);
	node->attachObject(limb);
	Ogre::Vector3 up(0,1,0);

	Ogre::SceneNode* cNode=node->createChildSceneNode(childNodeName,Ogre::Vector3(0,distance,0));
	Ogre::Quaternion rotQ=up.getRotationTo(endPosition);
	node->setOrientation(rotQ);
	node->setInitialState();

	node->rotate(orientation);
	node->setInheritOrientation(inheritOrientation);
	return cNode;
}

const physx::PxU32 pairInd[]={	0,1,
	0,3,
	3,4,
	4,6,
	6,12,
	3,5,
	5,7,
	7,13,
	1,9,
	9,11,
	11,15,
	1,8,
	8,10,
	10,14,
	0,4,
	0,5};

float radius_modifier=1;



void InturlamDressingRoom::createSphereAndCapsule(Ogre::Bone* bone,Ogre::SceneNode* parentNode,int level)
{
	//if (bone->numChildren()>0 && level<11)
	//{
	//	Ogre::Bone::ChildNodeIterator childIterator=bone->getChildIterator();
	//	while(childIterator.hasMoreElements())
	//	{
	//		Ogre::Bone* childBone=(Ogre::Bone*)childIterator.getNext();
	//		Ogre::Vector3 childLocalPosition=childBone->getPosition();
	//		Ogre::Vector3 childPosition=bone->convertLocalToWorldPosition(childLocalPosition);
	//		Ogre::Vector3 worldOffset=childPosition-bonePosition;
	//		Ogre::String limbName=bone->getName()+" to "+ childBone->getName();
	//		Ogre::SceneNode* boneNode=parentNode->createChildSceneNode(limbName + " node");
	//		Ogre::SceneNode* childJointNode=createLimb(limbName,childBone->getName()+"node",0.2,worldOffset,0.1,boneNode,childBone->getInheritOrientation(),childBone->getOrientation());
	//		createSphereAndCapsule(childBone,childJointNode,childPosition,level+1);
	//	}
	//	parentNode->setInheritOrientation(bone->getInheritOrientation());
	////	parentNode->setOrientation(bone->getOrientation());
	//}
	//else
	//{	  
	//	 Entity* endPart=mSceneMgr->createEntity(bone->getName()+"Sphere","sphere_r0.3");
	//	  parentNode->attachObject(endPart);
	//}

	if (bone->numChildren()>0 && level<7)
	{
		Ogre::Bone::ChildNodeIterator childIterator=bone->getChildIterator();
		while(childIterator.hasMoreElements())
		{
			Ogre::Bone* childBone=(Ogre::Bone*)childIterator.getNext();
			Ogre::Vector3 childLocalPosition=childBone->getPosition();
			Ogre::String limbName=bone->getName()+" to "+ childBone->getName();
			Ogre::SceneNode* boneNode=parentNode->createChildSceneNode(limbName + " Node");
			Ogre::SceneNode* childJointNode=createLimb(limbName,childBone->getName()+"Node",0.2,childLocalPosition,0.1,boneNode,childBone->getInheritOrientation(),bone->getOrientation());
			createSphereAndCapsule(childBone,childJointNode,level+1);
		}
		parentNode->setInheritOrientation(bone->getInheritOrientation());
	}
	else
	{	  
		Entity* endPart=mSceneMgr->createEntity(bone->getName()+"Sphere","sphere_r0.3");
		parentNode->attachObject(endPart);
	}


}

void InturlamDressingRoom::createVisualHuman()
{
	int mirror=1;
	createSphere(0.3);
	Ogre::Bone* RootBone=femaleBody->getSkeleton()->getBone("Root");
	rootColliderNode=clothHandle->createChildSceneNode("RootNode");
	createSphereAndCapsule(RootBone,rootColliderNode);


}

PxSceneDesc InturlamDressingRoom::initializePhysics()
{

	gFoundation=PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
	if (!gFoundation)
	{
		MessageBox( NULL,"PxCreateFoundation failed!","Something Wrong With PhysX", MB_OK | MB_ICONERROR | MB_TASKMODAL);
		exit(0);
	}


	gManager=&PxProfileZoneManager::createProfileZoneManager(gFoundation);
	if (!gManager)
	{
		MessageBox( NULL,"Profile Zone Manager Creation failed!","Something Wrong With PhysX", MB_OK | MB_ICONERROR | MB_TASKMODAL);
		exit(0);
	}

	pxtask::CudaContextManagerDesc cudaContextManagerDesc;
	pxtask::CudaContextManager* mCudaContextManager = pxtask::createCudaContextManager(*gFoundation,cudaContextManagerDesc,gManager);

	if( mCudaContextManager )
	{
		if( !mCudaContextManager->contextIsValid() )
		{
			mCudaContextManager->release();
			mCudaContextManager = NULL;
		}
	}

	gPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION,*gFoundation,PxTolerancesScale(),true,gManager);

	if(gPhysicsSDK == NULL)	
	{
		MessageBox( NULL,"Error creating PhysX3 device!","Something Wrong With PhysX", MB_OK | MB_ICONERROR | MB_TASKMODAL);
		exit(0);
	}

	if(!PxInitExtensions(*gPhysicsSDK))
		cerr<< "PxInitExtensions failed!" <<endl;


	PxSceneDesc	sceneDesc(gPhysicsSDK->getTolerancesScale());
	sceneDesc.gravity=PxVec3(0.0f, -9.8f, 0.0f);

	if(!sceneDesc.cpuDispatcher) {
		PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
		if(!mCpuDispatcher)
			cerr<<"PxDefaultCpuDispatcherCreate failed!"<<endl;
		sceneDesc.cpuDispatcher = mCpuDispatcher;
	} 

	if (!sceneDesc.gpuDispatcher && mCudaContextManager )
	{
		printf("gpu dispatcher done!\n");
		sceneDesc.gpuDispatcher = mCudaContextManager->getGpuDispatcher();
	}


	if(!sceneDesc.filterShader)
		sceneDesc.filterShader  = gDefaultFilterShader;

	gScene = gPhysicsSDK->createScene(sceneDesc);


	if (!gScene)
		cerr<<"createScene failed!"<<endl;



	//gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE,				 1.0);
	//gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES,	1.0f);
	//gScene->setVisualizationParameter(PxVisualizationParameter::eDEFORMABLE_MESH, 1.0f);
	//gScene->setVisualizationParameter(PxVisualizationParameter::eDEFORMABLE_SELFCOLLISIONS, 1.0f);
	//gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_DYNAMIC, 1.0f);
	//gScene->setVisualizationParameter(PxVisualizationParameter::eDEFORMABLE_SHAPES, 1.0f);

	PxMaterial* mMaterial = gPhysicsSDK->createMaterial(0.5,0.5,0.5);

	//Create actors 
	//1) Create ground plane
	PxReal d = 0.0f;	 
	PxTransform pose = PxTransform(PxVec3(0.0f, 0.0f, 0.0f),PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));

	PxRigidStatic* plane = gPhysicsSDK->createRigidStatic(pose);
	if (!plane)
		cerr<<"create plane failed!"<<endl;

	PxShape* shape = plane->createShape(PxPlaneGeometry(), *mMaterial);
	if (!shape)
		cerr<<"create shape failed!"<<endl;
	gScene->addActor(*plane);

	return sceneDesc;
}

const physx::PxU32 pairIndHang[]={1,3,
	0,1,
	0,4,
	0,7,
	0,10,
	0,13,
	1,4,
	1,7,
	4,5,
	5,6,
	7,8,
	8,9,
	10,11,
	11,12,
	13,14,
	14,15,
	16,17};

void InturlamDressingRoom::setupHumanCollider()
{
	//PxClothCollisionData* cd= new PxClothCollisionData;
	col_data.setToDefault();

	box_collider=new physx::PxClothCollisionSphere[16];


	for (int i=0;i<16;i++)
	{
		//	Ogre::Bone* tBone=skeleton->getBone(boneStrings[i]);
		Ogre::String nodeName=boneStrings[i]+"Node";
		Ogre::SceneNode* gNode=mSceneMgr->getSceneNode(nodeName);
		Vector3 localPosition=gNode->_getDerivedPosition()*Vector3(SCALING_FACTOR,SCALING_FACTOR,SCALING_FACTOR)+Vector3(0,Y_OFFSET,0);
		if (boneStrings[i]=="Root")
		{
			box_collider[i].pos.x=localPosition.x;
			box_collider[i].pos.y=localPosition.y;
			box_collider[i].pos.z=localPosition.z;
			box_collider[i].radius=0.01;
		}
		else
		{
			box_collider[i].pos.x=localPosition.x;
			box_collider[i].pos.y=localPosition.y;
			box_collider[i].pos.z=localPosition.z;
			//box_collider[i].radius=2.1;
			box_collider[i].radius=sphereRadii[i]/40;
		}
	}


	col_data.spheres=box_collider;
	col_data.numSpheres=16;
	col_data.numPairs=16;
	col_data.pairIndexBuffer=pairInd;

}
PxVec3 clothPos;
PxVec3 wind;
float curScale=1;

void InturlamDressingRoom::createCloth(PxSceneDesc sceneDesc)
{
	PxClothMeshDesc meshDesc;
	meshDesc.setToDefault();
	clothPos.x=0;
	clothPos.y=0;
	clothPos.z=0;
	wind.x=0;
	wind.y=0;
	wind.z=0;
	PxClothFabric* fabric;
	PxClothParticle* points;
	PxTransform tr;

	meshDesc=*lowerCloth->loadPhysxCloth(&sceneDesc,fabric,points,&tr,gPhysicsSDK);

	bool withHanger=true;

	setupHumanCollider();	
	//col_data.setToDefault();

	if (col_data.isValid())
		cloth = gPhysicsSDK->createCloth(tr,*fabric,points,col_data, PxClothFlag::eSWEPT_CONTACT |  PxClothFlag::eGPU );


	if(cloth) {	
		PxClothPhaseSolverConfig bendCfg;	 
		bendCfg.solverType= PxClothPhaseSolverConfig::eSTIFF;
		bendCfg.stiffness = 1;
		bendCfg.stretchStiffness = 0.50;
		bendCfg.stretchLimit=0.60;
		cloth->setSolverFrequency(60);

		cloth->setPhaseSolverConfig(PxClothFabricPhaseType::eBENDING,		bendCfg) ;	
		cloth->setPhaseSolverConfig(PxClothFabricPhaseType::eSTRETCHING,	bendCfg) ;	
		cloth->setPhaseSolverConfig(PxClothFabricPhaseType::eSHEARING,		bendCfg) ;	
		cloth->setPhaseSolverConfig(PxClothFabricPhaseType::eSTRETCHING_HORIZONTAL, bendCfg) ;
		//	cloth->setClothFlag(physx::PxClothFlag::eGPU,true); 
		cloth->setDampingCoefficient(0.2f);	   
		cloth->setFrictionCoefficient(0.5f); 
		cloth->setCollisionMassScale(20.0f);
		cloth->setInertiaScale(0.5);
		cloth->setClothFlag(PxClothFlag::eGPU,usingGPU);

		gScene->addActor(*cloth); 
	}

	lowerCloth->cloth=cloth;
	clothDirection=still;
	clothRotation=none;
	guyMoves=standing;
}

void InturlamDressingRoom::createSimulation()
{
	#if USE_USER_SCALING
	userHeightScale=SCALING_FACTOR*estimatedTorsoHeight/MODEL_TORSO_HEIGHT;
	userWidthScale=SCALING_FACTOR*estimatedShoulderWidth/MODEL_SHOULDER_WIDTH;
	userDepthScale=(userHeightScale+userWidthScale)/2;
	#else
	userHeightScale=SCALING_FACTOR;
	userWidthScale=SCALING_FACTOR;
	userDepthScale=SCALING_FACTOR;
	#endif

	clothHandle=mSceneMgr->getRootSceneNode()->createChildSceneNode();
	clothNode=clothHandle->createChildSceneNode("ClothNode");
	femaleNode=clothHandle->createChildSceneNode("FemaleHandle");
	lowerClothHandle=clothHandle->createChildSceneNode("lowerClothHandle",Vector3(0,-Y_OFFSET,0));

	upperCloth=new SkeletalMesh(mKinect);
	femaleBody=new SkeletalMesh(mKinect);

	//Sundress
	upperCloth->loadMesh(mSceneMgr,clothNode,"UpperCloth","Cloth_Shape.003.mesh");
	femaleBody->loadMesh(mSceneMgr,femaleNode,"FemaleModel","FemaleBody.mesh");
	lowerCloth=new ObjObject("../../media/wavefront/lowerDressv2.obj");

	//Kimono
	//upperCloth->loadMesh(mSceneMgr,clothNode,"UpperCloth","kimono.mesh");
	//femaleBody->loadMesh(mSceneMgr,femaleNode,"FemaleModel","FemaleBody.mesh");
	//lowerCloth=new ObjObject("../../media/wavefront/lowerKimono.obj");

	//Tunic-Vest
	//upperCloth->loadMesh(mSceneMgr,clothNode,"UpperCloth","Layer_12za.mesh");
	//femaleBody->loadMesh(mSceneMgr,femaleNode,"FemaleModel","FemaleBody.mesh");
	//lowerCloth=new ObjObject("../../media/wavefront/lowerTunic.obj");

	lowerCloth->Scale(SCALING_FACTOR);
	float vec3[3]={0,Y_OFFSET,0};
	lowerCloth->translate(vec3);
	lowerCloth->loadIntoOgre(mSceneMgr, "lowerClothMesh");
	Ogre::Entity* lowerClothEntity=mSceneMgr->createEntity("LowerCloth","lowerClothMesh");
	lowerClothHandle->attachObject(lowerClothEntity);

	createVisualHuman();
	//setupHumanCollider();
	createCloth(initializePhysics());

	femaleNode->scale(userWidthScale,userHeightScale,userDepthScale);
	clothNode->scale(userWidthScale,userHeightScale,userDepthScale);
	rootColliderNode->scale(SCALING_FACTOR,SCALING_FACTOR,SCALING_FACTOR);
	//clothNode->setVisible(false);
	//femaleNode->setVisible(false);
	rootColliderNode->setVisible(false);
	simulationCreated=true;
}


//-------------------------------------------------------------------------------------
void InturlamDressingRoom::createScene(void)
{
	#if USE_KINECT//Kinect And Stuff
	SetupDepthMaterial();
	mKinect=new KinectController(false);
	mKinect->createRTT(mRoot,mTrayMgr);
	Ogre::OverlayElement* mDepthPanel = Ogre::OverlayManager::getSingleton().createOverlayElement("Panel","DepthPanel");
	mDepthPanel->setMaterialName("DepthTextureMaterial");
	mDepthPanel->setMetricsMode(Ogre::GMM_RELATIVE);
	mDepthPanel->setWidth(0.25);
	mDepthPanel->setHeight(0.25*480/640);
	mDepthPanel->setHorizontalAlignment(GHA_RIGHT);
	mDepthPanel->setVerticalAlignment(GVA_BOTTOM);
	mDepthPanel->setLeft(-mDepthPanel->getWidth());
	mDepthPanel->setTop(-mDepthPanel->getHeight());
	mTrayMgr->getTraysLayer()->add2D((Ogre::OverlayContainer*)mDepthPanel);
	mDepthPanel->show();
	#else
	createSimulation();
	#endif
	buildAxes();
	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
	Ogre::Light* l = mSceneMgr->createLight("MainLight");
	l->setPosition(0,30,0);
	StringVector items;
	items.push_back("Passed Frames");
	items.push_back("Calibration Time");
	items.push_back("Body Height");
	items.push_back("Shoulder Width");
	mTrayMgr->hideLogo();
	help = mTrayMgr->createParamsPanel(TL_NONE, "HelpMessage", 200, items);
    help->hide();
}
bool justCalibrated=true;

void InturlamDressingRoom::updateCollisionSpheres()
{
	//Ogre::Vector3 scale=clothHandle->getScale();

	//clothHandle->setScale(Ogre::Vector3(1));

	for (int i=0;i<16;i++)
	{
		//	Ogre::Bone* tBone=skeleton->getBone(boneStrings[i]);
		Ogre::String nodeName=boneStrings[i]+"Node";
		Ogre::SceneNode* gNode=mSceneMgr->getSceneNode(nodeName);
		Vector3 localPosition=gNode->_getDerivedPosition()+Vector3(0,Y_OFFSET,0);
		if (boneStrings[i]=="Root")
		{
			box_collider[i].pos.x=localPosition.x;
			box_collider[i].pos.y=localPosition.y;
			box_collider[i].pos.z=localPosition.z;
			//box_collider[i].radius=0.1;
		}
		else
		{
			box_collider[i].pos.x=localPosition.x;
			box_collider[i].pos.y=localPosition.y;
			box_collider[i].pos.z=localPosition.z;
			//box_collider[i].radius=0.2;
		}
	}

	cloth->setCollisionSpheres(box_collider);
	//clothHandle->setScale(scale);
}

void InturlamDressingRoom::updateCloth()
{

	updateCollisionSpheres();

	Vector3 ts=lowerClothHandle->getPosition()+Vector3(0,Y_OFFSET,0);
	PxVec3 trans=PxVec3(ts.x,ts.y,ts.z);

	//Ogre::Quaternion qRot=Quaternion::IDENTITY;
	Ogre::Quaternion qRot=bodyRotation;


	PxQuat quat=PxQuat(qRot.x,qRot.y,qRot.z,qRot.w);


	if (justCalibrated)
	{
		cloth->setGlobalPose(PxTransform(trans,quat));
		justCalibrated=false;
	}
	else
		cloth->setTargetPose(PxTransform(trans,quat));

}

void InturlamDressingRoom::updateJoints(Ogre::Bone* bone,int level)
{
	if (bone->numChildren()>0 && level<7)
	{
		Ogre::Bone::ChildNodeIterator childIterator=bone->getChildIterator();
		while(childIterator.hasMoreElements())
		{
			Ogre::Bone* childBone=(Ogre::Bone*)childIterator.getNext();
			if (!bone->getInheritOrientation() && level>0)
			{
				Ogre::String limbName=bone->getName()+" to "+ childBone->getName();
				Ogre::SceneNode* boneNode=mSceneMgr->getSceneNode(limbName + " Node");
				Ogre::Quaternion qI = boneNode->getInitialOrientation();
				boneNode->resetOrientation();
				boneNode->setOrientation(bodyRotation*bone->getOrientation()*qI);
			}
			updateJoints(childBone,level+1);
		}
	}
}

void InturlamDressingRoom::updateVisualHuman()
{

	Ogre::Vector3 leftHip=mKinect->getRealCoordinate(XN_SKEL_LEFT_HIP);
	Ogre::Vector3 rightHip=mKinect->getRealCoordinate(XN_SKEL_RIGHT_HIP);
	Ogre::Vector3 hipVector=(rightHip-leftHip);
	Ogre::Vector3 initialVector=Ogre::Vector3(1,0,0);
	hipVector.y=0;
	bodyRotation=hipVector.getRotationTo(initialVector);
	Ogre::Bone* rootBone=femaleBody->getSkeleton()->getBone("Root");
	updateJoints(rootBone);


}

long long milliseconds_now() {
    static LARGE_INTEGER s_frequency;
    static BOOL s_use_qpc = QueryPerformanceFrequency(&s_frequency);
    if (s_use_qpc) {
        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        return (1000LL * now.QuadPart) / s_frequency.QuadPart;
    } else {
        return GetTickCount();
    }
}

PxReal timeStep=0;
int initialDelay=0;
float totalCalibrationTime=0;
bool calibrated=false;
long long cal_start;
long long cal_end;
bool InturlamDressingRoom::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
	#if USE_KINECT
	if (simulating && simulationCreated)
		timeStep+=evt.timeSinceLastFrame;
	if (mKinect->addTime(evt.timeSinceLastFrame))
	{
		if (simulating && simulationCreated)
		{
			lowerCloth->updateWithPhysics(gScene,timeStep);
			timeStep=0;
		}
		if (mKinect->isUserActive())
		{
			if (mKinect->m_UserGenerator.GetSkeletonCap().IsCalibrated(mKinect->activeUser))
			{
				if (calibrated && simulationCreated)
				{
					upperCloth->setUserID(mKinect->activeUser);
					Ogre::Vector3 targetPos=upperCloth->updateMesh();
					femaleBody->setUserID(mKinect->activeUser);
					femaleBody->updateMesh();
					updateVisualHuman();
					lowerClothHandle->setPosition(targetPos*Vector3(SCALING_FACTOR,SCALING_FACTOR,SCALING_FACTOR)+Vector3(0,-Y_OFFSET,0));
					lowerClothHandle->setOrientation(upperCloth->getBoneOrientation(BONE_ROOT));
					updateCloth();
				}
				else if (initialDelay>6)
				{
					if (addFrame(&mKinect->m_DepthGenerator,&mKinect->m_UserGenerator,mKinect->activeUser))
					{
						calibrated=true;
						cal_end=milliseconds_now();
						totalCalibrationTime+=((float)cal_end-(float)cal_start)/1000;	
						help->setParamValue("Calibration Time",StringConverter::toString(totalCalibrationTime));
						help->setParamValue("Body Height",StringConverter::toString(estimatedBodyHeight));
						help->setParamValue("Shoulder Width",StringConverter::toString(estimatedShoulderWidth));
						createSimulation();
					}
				}
				else
				{
					initialDelay+=1;	
					help->setParamValue("Passed Frames",StringConverter::toString(initialDelay));
					if (initialDelay>6) cal_start=milliseconds_now();
				}
			}
			else if (simulationCreated)
			{
				femaleBody->resetBonesToInitialState();
				upperCloth->resetBonesToInitialState();
				updateVisualHuman();
			}
		}
		else if (simulationCreated)
		{
			femaleBody->resetBonesToInitialState();
			upperCloth->resetBonesToInitialState();
			updateVisualHuman();
		}
	}
	#else
	timeStep=evt.timeSinceLastFrame;
	lowerCloth->updateWithPhysics(gScene,timeStep);
	#endif
	return BaseApplication::frameRenderingQueued(evt);
}

bool InturlamDressingRoom::keyPressed( const OIS::KeyEvent &arg )
{
	if (arg.key==OIS::KC_SPACE)
		simulating=!simulating;
	else if (arg.key==OIS::KC_L)
	{
		mCamera->setPosition(Ogre::Vector3(0,0,80));
		mCamera->lookAt(Ogre::Vector3(0,0,-300));
	}
	else if (arg.key==OIS::KC_H)
	{
		femaleNode->flipVisibility();
	}
	else if (arg.key==OIS::KC_ADD)
	{
		PxReal scale=cloth->getInertiaScale();
		if (scale<=0.9)
			cloth->setInertiaScale(scale+0.1);

		/* curScale+=0.01;
		clothNode->setScale(curScale,curScale,curScale);*/
	}
	else if (arg.key==OIS::KC_SUBTRACT)
	{
		PxReal scale=cloth->getInertiaScale();
		if (scale>=0.1)
			cloth->setInertiaScale(scale-0.1);
		//curScale-=0.01;
		//clothNode->setScale(curScale,curScale,curScale);
	}
	else if (arg.key==OIS::KC_J)
	{
		if (help->getTrayLocation() == OgreBites::TL_NONE)
        {
            mTrayMgr->moveWidgetToTray(help, OgreBites::TL_TOPRIGHT, 0);
            help->show();
        }
        else
        {
            mTrayMgr->removeWidgetFromTray(help);
            help->hide();
        }
	}
	else if (arg.key==OIS::KC_TAB)
	{
		usingGPU=!usingGPU;
		cloth->setClothFlag(PxClothFlag::eGPU,usingGPU);
		//cloth->setClothFlag(PxClothFlag::eG
	}
	else if (arg.key==OIS::KC_NUMPAD8)
	{
		//Ogre::SceneNode* ax=mSceneMgr->getSceneNode("Axes");
		//ax->translate(0,0,1);
		clothDirection=moveDirection::forward;
		//		 cloth->setExternalAcceleration(PxVec3(0,0,1));

	}
	else if (arg.key==OIS::KC_NUMPAD5)
	{
		clothDirection=moveDirection::backward;
		//cloth->setExternalAcceleration(PxVec3(0,0,-1));
		//Ogre::SceneNode* ax=mSceneMgr->getSceneNode("Axes");
		//ax->translate(0,0,-1);
	}
	else if (arg.key==OIS::KC_NUMPAD6)
	{
		clothDirection=moveDirection::right;
		//cloth->setExternalAcceleration(PxVec3(1,0,0));
		//Ogre::SceneNode* ax=mSceneMgr->getSceneNode("Axes");
		//ax->translate(0,1,0);
	}
	else if (arg.key==OIS::KC_NUMPAD4)
	{
		clothDirection=moveDirection::left;
		//cloth->setExternalAcceleration(PxVec3(-1,0,0));
		//Ogre::SceneNode* ax=mSceneMgr->getSceneNode("Axes");
		//ax->translate(0,-1,0);
	}
	else if (arg.key==OIS::KC_NUMPAD9)
	{

		clothRotation=clockwise;

		//Ogre::SceneNode* ax=mSceneMgr->getSceneNode("Axes");
		//ax->translate(1,0,0);
	}
	else if (arg.key==OIS::KC_NUMPAD7)
	{
		clothRotation=counterclockwise;

		//Ogre::SceneNode* ax=mSceneMgr->getSceneNode("Axes");
		//ax->translate(-1,0,0);
	}
	else if (arg.key==OIS::KC_NUMPAD3)
	{

		guyMoves=gforward;

		//Ogre::SceneNode* ax=mSceneMgr->getSceneNode("Axes");
		//ax->translate(1,0,0);
	}
	else if (arg.key==OIS::KC_NUMPAD1)
	{
		guyMoves=gbackward;

		//Ogre::SceneNode* ax=mSceneMgr->getSceneNode("Axes");
		//ax->translate(-1,0,0);
	}


	return BaseApplication::keyPressed(arg);
}

bool InturlamDressingRoom::keyReleased( const OIS::KeyEvent &arg )
{
	if (arg.key==OIS::KC_NUMPAD8 || arg.key==OIS::KC_NUMPAD5 || arg.key==OIS::KC_NUMPAD6 || arg.key==OIS::KC_NUMPAD4)
	{
		//Ogre::SceneNode* ax=mSceneMgr->getSceneNode("Axes");
		//ax->translate(0,0,1);
		clothDirection=still;

	}
	else if (arg.key==OIS::KC_NUMPAD7 || arg.key==OIS::KC_NUMPAD9)
	{
		clothRotation=none;
	}
	else if (arg.key==OIS::KC_NUMPAD1 || arg.key==OIS::KC_NUMPAD3)
	{
		guyMoves=standing;
	}


	return BaseApplication::keyReleased(arg);
}


#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
	INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
	int main(int argc, char *argv[])
#endif
	{
		// Create application object
		InturlamDressingRoom app;

		try {
			app.go();
		} catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
			MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
			std::cerr << "An exception has occured: " <<
				e.getFullDescription().c_str() << std::endl;
#endif
		}

		return 0;
	}

#ifdef __cplusplus
}
#endif
