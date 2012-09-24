#include "StdAfx.h"
#include "SkeletonPoseDetector.h"
#include "PointDrawer.h"

#ifndef HAND_TRACKER_HPP
#define HAND_TRACKER_HPP

#ifndef SHOW_DEPTH
#define SHOW_DEPTH 1
#endif
#ifndef SHOW_BAR
#define SHOW_BAR 0
#endif

#define averaging_size 15



const uint m_Width = 640;
const uint m_Height = 480;
const int RGBX= 1280;
const int RGBY = 1024;
enum Handstate
{
	HAND_UNKNOWN,
	HAND_OPEN,
	HAND_CLOSED
};

class HandTracker
{

private:
	int ROIWidth;
	int ROIHeight;
	float startDistance;
	bool setUp;

	XnUserID userID;
	xn::DepthGenerator* m_DepthGenerator;
	xn::UserGenerator* m_UserGenerator;
	StartPoseDetector * m_pStartPoseDetector;
	EndPoseDetector * m_pEndPoseDetector;
	XnVPointDrawer* m_pDrawer;

public:
	HandTracker(xn::DepthGenerator* depth,xn::UserGenerator* user,StartPoseDetector * startPoseDetector,EndPoseDetector* endPoseDetector,XnVPointDrawer* PDrawer);
	~HandTracker(void);
	void initROI(XnUserID user);
	void resetROI();
	void setUserID(XnUserID user);
	bool isSetUp();
	void UpdateDepthTexture(bool m_front,Handstate* leftHandState,Handstate* rightHandState,XnPoint3D* leftHandRelativePosition,XnPoint3D* rightHandRelativePosition);
	
};

#endif