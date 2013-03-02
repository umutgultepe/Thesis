#pragma once
#include "StdAfx.h"
#include "HandTracker.h"
#include "SkeletalMesh.h"

#ifndef USE_KINECT
#define USE_KINECT 0
#define USE_USER_SCALING 0
#endif
enum 
{
	TORSO_HEIGHT=0,
	HEAD_WIDTH = 1,
	BODY_HEIGHT=2,
	HIP_HEIGHT=3,
	ELBOW_FINGERTIP=4,
	WRIST_FINGERTIP=5,
	SHOULDER_WIDTH=6,
	HIP_WIDTH=7,
	HEAD_HEIGHT = 8,
};

enum 
{
	SHOULDER_WIDTH_TO_HEAD_WIDTH=0,
	TORSO_HEIGHT_TO_HEAD_HEIGHT=1,
	TORSO_HEIGHT_TO_BODY_HEIGHT=2,
	TORSO_HEIGHT_TO_HIP_HEIGHT=3,
	TORSO_HEIGHT_TO_ELBOW_FINGERTIP=4,
	TORSO_HEIGHT_TO_WRIST_FINGERTIP=5,
};

static int processedFrameCount=0;
static float bodyMeasurements[]={0,0,0,0,0,0,0,0,0};
static float sleevelessProportions[]={3.1, 5.3 , 0.63, 1.26 , 3.3 , 16.9 };

bool convertMetaDataToIpl(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID);
bool optimizeDepthMap(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID);
void getSphereSizes(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID);
void measureBody(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID);
void estimateParameters();
bool addFrame(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID);