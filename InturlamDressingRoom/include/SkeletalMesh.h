#pragma once
#include "StdAfx.h"
#include "ClothResizer.h"
#include "KinectController.h"
#include "NUI_Controller.h"
#ifndef __SkeletalMesh_h_
#define __SkeletalMesh_h_
#define TOTAL_BONE_COUNT 28
#define ACTIVE_BONE_COUNT 17
class KinectController;

enum boneID
{
	BONE_STOMACH= 0,
	BONE_WAIST=1,
	BONE_ROOT = 2,
	BONE_CHEST = 3,
	BONE_LEFT_HUMERUS =4,
	BONE_RIGHT_HUMERUS=5,
	BONE_LEFT_ULNA=6,
	BONE_RIGHT_ULNA=7,
	BONE_LEFT_THIGH=8,
	BONE_RIGHT_THIGH=9,
	BONE_LEFT_CALF=10,
	BONE_RIGHT_CALF=11,
	BONE_LEFT_HAND=12,
	BONE_RIGHT_HAND=13,
	BONE_LEFT_FOOT=14,
	BONE_RIGHT_FOOT=15,
	BONE_NECK=16,
	BONE_LEG_EXTENSION_UP_L=17,
	BONE_LEG_EXTENSION_UP_R=18,
	BONE_LEG_EXTENSION_HIP_L=19,
	BONE_LEG_EXTENSION_HIP_R=20,
	BONE_LEG_EXTENSION_HIP2_L=21,
	BONE_LEG_EXTENSION_HIP2_R=22,
	BONE_LEG_EXTENSION_DUMMY_L=23,
	BONE_LEG_EXTENSION_DUMMY_R=24,
	BONE_WAIST_EXTENTION_L=25,
	BONE_WAIST_EXTENTION_R=26,
	BONE_GROIN_EXTENTION=27,
	BONE_UNKNOWN=28,
};
static Ogre::String boneStrings[] = 
	{"Stomach",
	"Waist",
	"Root",
	"Chest",
	"Humerus.L",
	"Humerus.R",
	"Ulna.L",
	"Ulna.R",
	"Thigh.L",
	"Thigh.R",
	"Calf.L",
	"Calf.R",//AFter this bone, the others are not used in skeleton animation
	"Hand.L",
	"Hand.R",
	"Foot.L",
	"Foot.R",
	"Neck",
	"LegExtensionUp.L",
	"LegExtensionUp.R",
	"LegExtensionHip.L",
	"LegExtensionHip.R",
	"LegExtensionHip2.L",
	"LegExtensionHip2.R",
	"Dummy.L",
	"Dummy.R",
	"WaistExtent.L",
	"WaistExtent.R",
	"GroinsExtent",
	"unknown"};

static Ogre::String jointStrings[] = 
	{"Torso",
	"Torso",
	"Torso",
	"Torso",
	"Shoulder.L",
	"Shoulder.R",
	"Elbow.L",
	"Elbow.R",
	"Hip.L",
	"Hip.R",
	"Knee.L",
	"Knee.R",//AFter this bone, the others are not used in skeleton animation
	"Hand.L",
	"Hand.R",
	"Foot.L",
	"Foot.R",
	"Neck",
	"unknown"};
static XnSkeletonJoint jointIDs[] = 
{XN_SKEL_TORSO,
XN_SKEL_WAIST,
XN_SKEL_WAIST,
XN_SKEL_TORSO,
XN_SKEL_LEFT_SHOULDER,
XN_SKEL_RIGHT_SHOULDER,
XN_SKEL_LEFT_ELBOW,
XN_SKEL_RIGHT_ELBOW,
XN_SKEL_LEFT_HIP,
XN_SKEL_RIGHT_HIP,
XN_SKEL_LEFT_KNEE,
XN_SKEL_RIGHT_KNEE,
XN_SKEL_LEFT_HAND,
XN_SKEL_RIGHT_HAND,
XN_SKEL_LEFT_FOOT,
XN_SKEL_RIGHT_FOOT,
(XnSkeletonJoint)0};

static _NUI_SKELETON_POSITION_INDEX nuiIDs[]=
{
	NUI_SKELETON_POSITION_HIP_CENTER,
	NUI_SKELETON_POSITION_SPINE,
	NUI_SKELETON_POSITION_HIP_CENTER,
	NUI_SKELETON_POSITION_SHOULDER_CENTER,
	NUI_SKELETON_POSITION_ELBOW_LEFT,
	NUI_SKELETON_POSITION_ELBOW_RIGHT,
	NUI_SKELETON_POSITION_WRIST_LEFT,
	NUI_SKELETON_POSITION_WRIST_RIGHT,
	NUI_SKELETON_POSITION_KNEE_LEFT,
	NUI_SKELETON_POSITION_KNEE_RIGHT,
	NUI_SKELETON_POSITION_ANKLE_LEFT,
	NUI_SKELETON_POSITION_ANKLE_RIGHT,
	NUI_SKELETON_POSITION_HAND_LEFT,
	NUI_SKELETON_POSITION_HAND_RIGHT,
	NUI_SKELETON_POSITION_FOOT_LEFT,
	NUI_SKELETON_POSITION_FOOT_RIGHT,
	NUI_SKELETON_POSITION_HEAD,
	(_NUI_SKELETON_POSITION_INDEX)-1
};

//static _NUI_SKELETON_POSITION_INDEX nuiIDs[]=
//{
//	NUI_SKELETON_POSITION_SPINE,
//	NUI_SKELETON_POSITION_HIP_CENTER,
//	NUI_SKELETON_POSITION_HIP_CENTER,
//	NUI_SKELETON_POSITION_SPINE,
//	NUI_SKELETON_POSITION_SHOULDER_LEFT,
//	NUI_SKELETON_POSITION_SHOULDER_RIGHT,
//	NUI_SKELETON_POSITION_ELBOW_LEFT,
//	NUI_SKELETON_POSITION_ELBOW_RIGHT,
//	NUI_SKELETON_POSITION_HIP_LEFT,
//	NUI_SKELETON_POSITION_HIP_RIGHT,
//	NUI_SKELETON_POSITION_KNEE_LEFT,
//	NUI_SKELETON_POSITION_KNEE_RIGHT,
//	NUI_SKELETON_POSITION_WRIST_LEFT,
//	NUI_SKELETON_POSITION_WRIST_RIGHT,
//	NUI_SKELETON_POSITION_FOOT_LEFT,
//	NUI_SKELETON_POSITION_FOOT_RIGHT,
//	(_NUI_SKELETON_POSITION_INDEX)-1
//};


static NUI_Vector4* boneOrientations=0;
class SkeletalMesh
{
public:

	SkeletalMesh(void);
	SkeletalMesh(KinectController* kinect);
	~SkeletalMesh(void);

	Ogre::Entity* Mesh;
	Ogre::String EntityName;
	Ogre::SceneNode* Node;
	Ogre::SkeletonInstance* Skeleton;
	std::vector<bool> boneExists;
	Ogre::Vector3 mGoalDirection;
	Ogre::Vector3 origTorsoPos;
	KinectController* gKinect;
	XnUserID userID;
	bool bNewUser;

	double m_SmoothingFactor;
	int m_SmoothingDelta;

	Ogre::Entity* loadMesh(Ogre::SceneManager* g_SceneManager,Ogre::SceneNode* parentNode,Ogre::String entityName,Ogre::String meshID);

	Ogre::Vector3 updateMesh();
	Ogre::Vector3 updateMesh(NUI_Controller* nui);
	
	Real rollManually(const Ogre::String& modelBoneName, Real modifier);
	void setupBone(const String& name,const Degree& yaw,const Degree& pitch,const Degree& roll);
	void setupBone(const String& name,const Ogre::Quaternion& q);
	void setupBone(const String& name,const Ogre::Radian& angle, const Vector3 axis);
	void resetBonesToInitialState();
	void transformBone(const Ogre::String& modelBoneName, XnSkeletonJoint skelJoint, bool flip=false);
	void transformBone(const Ogre::String& modelBoneName, NUI_SKELETON_BONE_ORIENTATION skelJoint, bool flip=true,Quaternion factor=Quaternion::IDENTITY);
	void setOriginalTorsoPosition();
	Quaternion convertNUItoOgre(NUI_SKELETON_BONE_ORIENTATION sj,bool flip=true);

	inline void setVisible(bool visibleOrNot)
	{	
		Mesh->setVisible(visibleOrNot);
	}
	inline void flipVisibility()
	{
		Mesh->setVisible(!Mesh->getVisible());
	}


	Ogre::Quaternion getBoneOrientation(boneID bone);
	inline Ogre::Skeleton* getSkeleton()
	{
		return  Mesh->getSkeleton();
	}
	inline void setController(KinectController* kinect)
	{
		gKinect=kinect;
	}
	inline void setUserID(XnUserID id)
	{
		userID=id;
	}
	inline void clearUserID()
	{
		userID=0;
	}

};

#endif