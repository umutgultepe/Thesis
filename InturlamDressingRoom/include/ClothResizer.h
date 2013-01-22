#pragma once
#include "StdAfx.h"
#include "HandTracker.h"
#include "SkeletalMesh.h"
static float sphereRadii[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
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








static float estimatedShoulderWidth=0;
static float estimatedTorsoHeight=0;

static float bodyMeasurements[]={0,0,0,0,0,0,0,0,0};
static float sleevelessProportions[]={3, 4.7 , 0.67, 1.18 , 2.36 , 4.73 };

bool convertMetaDataToIpl(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID, IplImage* uImage, IplImage* dImage);
void optimizeDepthMap(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID);
void getSphereSizes(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID);
void measureBody(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID);