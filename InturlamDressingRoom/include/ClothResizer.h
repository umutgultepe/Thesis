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

static float bodyMeasurements[]={0,0,0,0,0,0,0,0,0};


bool convertMetaDataToIpl(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID, IplImage* uImage, IplImage* dImage);
void optimizeDepthMap(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID);
void getSphereSizes(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID);
void measureBody(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID);