#pragma once
#include "StdAfx.h"
#include "HandTracker.h"
#include "SkeletalMesh.h"
static float sphereRadii[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


bool convertMetaDataToIpl(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID, IplImage* uImage, IplImage* dImage);
void optimizeDepthMap(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID);
void getSphereSizes(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID);