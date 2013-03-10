/*
-----------------------------------------------------------------------------
Filename:    InturlamDressingRoom.h
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
#ifndef __InturlamDressingRoom_h_
#define __InturlamDressingRoom_h_
#include "StdAfx.h"
#include "BaseApplication.h"
#include "ClothResizer.h"
#include "KinectController.h"
#include "SkeletalMesh.h"
#include "NUI_Controller.h"

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#include "../res/resource.h"
#endif
#include "ObjObject.h"
using namespace physx;


#define CLOTH_COUNT 4
#define INITIAL_CLOTH_INDEX 0

enum clothIndex
{
	SUNDRESS = 1,
	KIMONO = 2,
	JEANS_SUIT = 3,
	ASTRONAUT = 4,
	FULL_DRESS=5,
};




static Ogre::String skeletonClothNames[]={
	"Cloth_Shape.003.mesh",
	"kimono.mesh",
	"BlueJeans.mesh",
	"FlightSuit.mesh",
	"",
	};

static Ogre::String PhysicsClothNames[]={
	"../../media/wavefront/lowerDressv2.obj",
	"../../media/wavefront/lowerKimono.obj",
	"",
	"",
	"../../media/wavefront/FullDress.obj"
};


enum moveDirection
{
	forward,
	backward,
	left,
	right,
	still
};

enum rotateDirection
{
	clockwise,
	counterclockwise,
	none
};

enum guyDirection
{
	gforward,
	gbackward,
	standing
};

class PhysxErrorReporter:  public PxDefaultErrorCallback
{
	public:
		virtual void reportError (PxErrorCode::Enum code, const char *message, const char *file, int line){  
			if (code==PxErrorCode::eINVALID_PARAMETER)
				MessageBox( NULL, message, "Invalid Parameter!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
			else if (code==PxErrorCode::eINVALID_OPERATION)
			{
				if (line==195)
					return;
				MessageBox( NULL, message, "Invalid Operation!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
			}
			else if (code==PxErrorCode::eOUT_OF_MEMORY)
				MessageBox( NULL, message, "Out Of Memory!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
			else if (code==PxErrorCode::eABORT || code==PxErrorCode::eINTERNAL_ERROR)
				MessageBox( NULL, message, "Fatal Error!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
			else
			{
				PxDefaultErrorCallback::reportError(code,message,file,line);
				return;
			}
			exit(1);
		};
};


class InturlamDressingRoom : public BaseApplication
{
public:
    InturlamDressingRoom(void);
    virtual ~InturlamDressingRoom(void);

protected:
    virtual void createScene(void);
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	virtual bool keyPressed( const OIS::KeyEvent &arg );
    virtual bool keyReleased( const OIS::KeyEvent &arg );
	void buildAxes();
	void createCapsule(const Ogre::String& strName, const float r,const float r2,const float d, const int nRings = 16, const int nSegments = 16);
	void createVisualHuman();
	Ogre::SceneNode* createLimb(Ogre::String limbName,Ogre::String childNodeName,float startRadius, Ogre::Vector3 endPosition,float endRadius,Ogre::SceneNode* node,bool inheritOrientation,Quaternion orientation);
	Ogre::SceneNode* createLimb(Ogre::String limbName,float startRadius, Ogre::Vector3 endPosition,float endRadius,Ogre::SceneNode* jointNode);

	ManualObject* createLine(Ogre::String name,Vector3 startPos,Vector3 endPos,Ogre::ColourValue color);
	void createSphere(const float r, const int nRings = 16, const int nSegments = 16);

	void loadPhysicsPiece(Ogre::MeshPtr meshData);
	void createSphereAndCapsule(Ogre::Bone* bone,Ogre::SceneNode* parentNode,int level=0);
	void updateJoints(Ogre::Bone* bone,int level=0);
	void updateVisualHuman();  
	void createSimulation();
	void setupHumanCollider();
	void updateCollisionSpheres();
	void updateVisualHumanOrientation();
	Ogre::Quaternion bodyRotation;
	KinectController* mKinect;
	PxSceneDesc initializePhysics();
	 void createCloth(PxSceneDesc sceneDesc);
	 void updateAcceleration(PxVec3 add);
	 void processMoves();
	 void updateCloth();
	 void changeCloth(int index);
	 void loadClothes();
	 moveDirection clothDirection;
	rotateDirection clothRotation;
	guyDirection guyMoves;
	SkeletalMesh* femaleBody;
	SkeletalMesh* upperCloth;
	

	int currentClothIndex;
	bool usingGPU;
	bool simulationCreated;
	NUI_Controller* mNui;
	ObjObject* lowerCloth;
	PxFoundation* gFoundation;
	pxtask::CudaContextManager* mCudaContextManager;
	PxDefaultCpuDispatcher* mCpuDispatcher;
	PxProfileZoneManager* gManager;
	PxPhysics* gPhysicsSDK ;
	PhysxErrorReporter gDefaultErrorCallback;
	PxDefaultAllocator gDefaultAllocatorCallback;
	PxSimulationFilterShader gDefaultFilterShader;
	PxSceneDesc* gSceneDesc;
	PxScene* gScene;
	PxCloth* cloth; 
	Ogre::SceneNode*				rootColliderNode;
	Ogre::SceneNode*				femaleNode;
	Ogre::SceneNode* 				clothNode;
	Ogre::SceneNode* 				clothHandle;
	Ogre::SceneNode*				lowerClothHandle;

	std::vector<SkeletalMesh*> skeletalMeshes;
	std::vector<ObjObject*> physicsMeshes;

	bool simulating;
	bool firstStep;
};

#endif // #ifndef __InturlamDressingRoom_h_
