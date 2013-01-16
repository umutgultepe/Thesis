#pragma once
#include "StdAfx.h"
#include "HandTracker.h"
class ClothResizer
{
public:
	ClothResizer(void);
	~ClothResizer(void);

};

bool convertMetaDataToIpl(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID, IplImage* uImage, IplImage* dImage);
void addDepthToBuffer(xn::DepthMetaData dmd,xn::SceneMetaData smd);
