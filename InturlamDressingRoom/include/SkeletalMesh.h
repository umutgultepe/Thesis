#pragma once
#include "StdAfx.h"
#include "KinectController.h"
#ifndef __SkeletalMesh_h_
#define __SkeletalMesh_h_
#define ACTIVE_BONE_COUNT 12
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
	BONE_UNKNOWN=16
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
	"LegExtensionUp.L",
	"LegExtensionUp.R",
	"LegExtensionHip.L",
	"LegExtensionHip.R",
	"LegExtensionHip2.L",
	"LegExtensionHip2.R",
	"Dummy.L",
	"Dummy.R",
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

	void setupBone(const String& name,const Degree& yaw,const Degree& pitch,const Degree& roll);
	void setupBone(const String& name,const Ogre::Quaternion& q);
	void setupBone(const String& name,const Ogre::Radian& angle, const Vector3 axis);
	void resetBonesToInitialState();
	void transformBone(const Ogre::String& modelBoneName, XnSkeletonJoint skelJoint, bool flip=false);

	void setVisible(bool visibleOrNot);

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