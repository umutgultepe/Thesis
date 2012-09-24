/*****************************************************************************
*                                                                            *
*  Sinbad Sample Application                                                 *
*  Copyright (C) 2010 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  OpenNI is free software: you can redistribute it and/or modify            *
*  it under the terms of the GNU Lesser General Public License as published  *
*  by the Free Software Foundation, either version 3 of the License, or      *
*  (at your option) any later version.                                       *
*                                                                            *
*  OpenNI is distributed in the hope that it will be useful,                 *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              *
*  GNU Lesser General Public License for more details.                       *
*                                                                            *
*  You should have received a copy of the GNU Lesser General Public License  *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.            *
*                                                                            *
*****************************************************************************/

// This sample is based on the Character sample from the OgreSDK.

#ifndef __Sinbad_H__
#define __Sinbad_H__
#include "StdAfx.h"
#define SHOW_DEPTH 1
#define SHOW_BAR 0
#include "HandTracker.h"
#include "PointDrawer.h"
#if SHOW_DEPTH && DEPTH_BAR
#	error SHOW_DEPTH and SHOW_BAR are mutually exclusive
#endif 

#include "SkeletonPoseDetector.h"

using namespace Ogre;

#define Z_BACK_LIMIT 3500
#define Z_FORWARD_LIMIT 500
#define X_LIMIT 1000
#define Z_OUTER_BACK_LIMIT 4000
#define Z_OUTER_FORWARD_LIMIT 0000
#define X_OUTER_LIMIT 2000


// Note: wont work as expected for > 5 users in scene
static unsigned int g_UsersColors[] = {/*0x70707080*/0 ,0x80FF0000,0x80FF4500,0x80FF1493,0x8000ff00, 0x8000ced1,0x80ffd700};
#define GetColorForUser(i) g_UsersColors[(i)%(sizeof(g_UsersColors)/sizeof(unsigned int))]

#define VALIDATE_GENERATOR(type, desc, generator)				\
{																\
	rc = m_Context.EnumerateExistingNodes(nodes, type);			\
	if (nodes.IsEmpty())										\
{															\
	printf("No %s generator!\n", desc);						\
	return 1;												\
}															\
	(*(nodes.Begin())).GetInstance(generator);					\
}
#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
{																\
	printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
	return rc;													\
}
using namespace OgreBites;

class KinectController
{
private:

	// all the animations our character has, and a null ID
	// some of these affect separate body parts and will be blended together
public:
	xn::Context m_Context;
#if SHOW_DEPTH
	xn::DepthGenerator m_DepthGenerator;
#endif
	xn::UserGenerator m_UserGenerator;
	xn::HandsGenerator m_HandsGenerator;
	xn::ImageGenerator m_ImageGenerator;
	xn::GestureGenerator m_GestureGenerator;
	xn::SceneAnalyzer m_SceneAnalyzer;
	XnVPointDrawer* m_pDrawer;
	XnVSessionManager* m_pSessionManager;
	XnVFlowRouter* m_gFlower;
	Handstate mHandState;
	double m_SmoothingFactor;
	int m_SmoothingDelta;
	bool deleteUser;
	bool m_front;	
	bool suppress;
	bool lookingForPose;
	bool trackingUser;
	float elapsedTime;
	std::vector<XnUserID> candidateUsers;
	std::vector<int> candidateUserInterestRate;
	std::vector<XnPoint3D> candidateUserCOMs;
	XnUserID activeUser;
	XnUserID interestedUser;
	OgreBites::ParamsPanel* m_help;
	OgreBites::SdkTrayManager *m_pTrayMgr;
	StartPoseDetector * m_pStartPoseDetector;
	EndPoseDetector * m_pEndPoseDetector;
	XnCallbackHandle m_hPoseCallbacks;
	XnCallbackHandle m_hUserCallbacks;
	XnCallbackHandle m_hCalibrationCallbacks;
	XnCallbackHandle m_hUserExitCallBack;
	XnCallbackHandle m_hUserReEnterCallBack;
	
	HandTracker* m_HandTracker;
	//RTT stuff
	Ogre::SceneNode* mUserNode;
	Ogre::Entity* mUserEntity;
	Ogre::OverlayElement* mRegionGuide;
	Ogre::SceneManager* mSceneMgr;
	Ogre::Root* mRoot;
	Handstate mLeftHandState;
	Handstate mRightHandState;
	XnPoint3D mLeftHandPosition;
	XnPoint3D mRighttHandPosition;
	XnVector3D mLeftHandSpeed;
	XnVector3D mRightHandSpeed;

	unsigned int lastUpdateTime;


	std::vector<XnPoint3D> mRightHandBuffer; 
	std::vector<XnPoint3D> mLeftHandBuffer;

	//Method to call when the user is seen
	//Probably guide the user to the active area
	void userSeen(const XnUserID nUserId);
	bool checkUserCOM(XnUserID user);
	bool checkOuterLimits(XnUserID user);

	static void XN_CALLBACK_TYPE NewUser(xn::UserGenerator& generator, const XnUserID nUserId, void* pCookie);
	static void XN_CALLBACK_TYPE UserExit(xn::UserGenerator& generator, const XnUserID nUserId, void* pCookie);
	static void XN_CALLBACK_TYPE UserReEnter(xn::UserGenerator& generator, const XnUserID nUserId, void* pCookie);
	static  void XN_CALLBACK_TYPE LostUser(xn::UserGenerator& generator, const XnUserID nUserId, void* pCookie);
	static  void XN_CALLBACK_TYPE CalibrationStart(xn::SkeletonCapability& skeleton, const XnUserID nUserId, void* pCookie);
	static  void XN_CALLBACK_TYPE CalibrationEnd(xn::SkeletonCapability& skeleton, const XnUserID nUserId, XnBool bSuccess, void* pCookie);
	static void XN_CALLBACK_TYPE PoseDetected(xn::PoseDetectionCapability& poseDetection, const XnChar* strPose, XnUserID nId, void* pCookie);
	static void XN_CALLBACK_TYPE PoseLost(xn::PoseDetectionCapability& poseDetection, const XnChar* strPose, XnUserID nId, void* pCookie);

	KinectController(bool useHandTracker);
	~KinectController();
	void changeCandidateID(XnUserID ID);
	XnStatus initPrimeSensor(bool useHandTracker);
	Image* takeSnapshot();
	bool updateUserNode(XnUserID ID);
	bool addTime(Real deltaTime);
	void injectKeyDown(const OIS::KeyEvent& evt);
	void injectKeyUp(const OIS::KeyEvent& evt);
	XnPoint3D getRelativeCoordinate(XnSkeletonJoint joint);	
	XnPoint3D getRelativeHandCoordinate(XnSkeletonJoint joint);
	XnVector3D getHandSpeed(XnSkeletonJoint joint);
	
	Ogre::Vector3 getRealCoordinate(XnSkeletonJoint joint);
	Ogre::Matrix3 getRealOrientation(XnSkeletonJoint joint);

	void updateRelativeHandCoordinates();
	void createRTT(Ogre::Root* ogreRoot,OgreBites::SdkTrayManager* mTrayMgr);
	bool checkInterestRate(XnUserID user);
	bool isUserActive();
	Handstate getHandstate(bool leftOrRight);
	
};

#endif
