#include "StdAfx.h"
#include "..\include\ClothResizer.h"


int gaussian_m=3,
	gaussian_n=3,
	gaussian_e=0.95;


IplImage* dImage;
IplImage* uImage;
CvSize dSize=cvSize(640,480);


bool convertMetaDataToIpl(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID)
{

	bool m_front=true;
	xn::DepthMetaData dmd;
	xn::SceneMetaData smd;
	dpg->GetMetaData(dmd);
	XnStatus tStatus=ug->GetUserPixels(userID, smd);
	if (tStatus!=XN_STATUS_OK)
		return false;
	
	const XnDepthPixel* pDepth = dmd.Data();
	const XnLabel* pUsersLBLs = smd.Data();

	for (size_t j = 0; j < m_Height; j++)
	{
		for(size_t i = 0; i < m_Width; i++)
		{		
			uint fixed_i = i;// fix i if we are mirrored
			if (!m_front)
				fixed_i = m_Width - i;				
			unsigned int color = 0;// determine color
			if (userID != 0)// if we have a candidate, filter out the rest
			{
				if  (userID == pUsersLBLs[j*m_Width + fixed_i])
				{
					cvSet2D(uImage,fixed_i,j,cvScalar(1));
					XnPoint3D projPoint;
					projPoint.X=fixed_i;
					projPoint.Y=j;
					projPoint.Z=0;
					XnPoint3D realPoint;
					dpg->ConvertProjectiveToRealWorld(1,&projPoint,&realPoint);
					cvSet2D(dImage,fixed_i,j,cvScalar(realPoint.Z));			
				}
				else
				{
					cvSet2D(uImage,fixed_i,j,cvScalar(0));
					cvSet2D(dImage,fixed_i,j,cvScalar(0));	
				}
			}		
		}
	}
	return true;
}

void optimizeDepthMap(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID)
{
	
	bool tStatus=convertMetaDataToIpl( dpg,ug,userID,  uImage, dImage);	//Convert Xn Matrices to OpenCV Matrices for easier calculation.
	CvScalar depthMean=cvAvg(dImage,uImage);							//Get teh Average Depth Value of the User Pixels
	cvNot(uImage,uImage);												//Invert the user pixels to paint the rest of the image with average user depth
	cvSet(dImage,depthMean,uImage);										 
	cvSmooth(dImage,dImage,CV_GAUSSIAN,gaussian_m,gaussian_n,gaussian_e);//Perform Gaussian Smoothing, depth map is optimized.
	cvNot(uImage,uImage);	
	cvErode(uImage,uImage,0,2);		//Smoothen the User Map as well
	cvDilate(uImage,uImage,0,2);
}


void getSphereSizes(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID)
{
	
	for (int i=0;i<16;i++)
	{
		xn::SkeletonCapability pUserSkel = ug->GetSkeletonCap();	
		XnSkeletonJoint sJoint=jointIDs[i];
		for (int j=0;j<i;j++)				//If joint is processed before, do not repeat it
		{
			if (jointIDs[j]==sJoint)
				sphereRadii[i]=sphereRadii[j];
		}
		XnSkeletonJointPosition realPosition;
		XnPoint3D projPosition;
		pUserSkel.GetSkeletonJointPosition(userID, sJoint, realPosition);
		dpg->ConvertRealWorldToProjective(1,&realPosition.position,&projPosition);	//Get the projective coordinate
		CvScalar tPosition=cvScalar(projPosition.X,projPosition.Y);					//Get cv
		int x_init=projPosition.X;
		int y_init=projPosition.Y;
		int step=0;
		int radius=0;
		XnPoint3D endOfJoint=projPosition;
		while( step<640)					//Slowly enlarge the joint sphere until it reaches the end of a bone in one direction.
		{
			if (x_init-step>-1)
			{
				if( !(bool)cvGetReal2D(uImage,x_init-step,y_init) )
				{
					endOfJoint.X-=(step-1);
					dpg->ConvertProjectiveToRealWorld(1,&endOfJoint,&endOfJoint);
					radius=abs(realPosition.position.X-endOfJoint.X);
					break;
				}
			}
			if (x_init+step<640)
			{
				if( !(bool)cvGetReal2D(uImage,x_init+step,y_init) )
				{
					endOfJoint.X+=(step-1);
					dpg->ConvertProjectiveToRealWorld(1,&endOfJoint,&endOfJoint);
					radius=abs(realPosition.position.X-endOfJoint.X);
					break;
				}
			}
			if (step<480)
			{
				if (y_init-step>-1)
				{
					if( !(bool)cvGetReal2D(uImage,x_init,y_init-step) )
					{
						endOfJoint.Y-=(step-1);
						dpg->ConvertProjectiveToRealWorld(1,&endOfJoint,&endOfJoint);
						radius=abs(realPosition.position.Y-endOfJoint.Y);
						break;
					}
				}
				if (y_init+step<480)
				{
					if( !(bool)cvGetReal2D(uImage,x_init,y_init+step) )
					{
						endOfJoint.Y+=(step-1);
						dpg->ConvertProjectiveToRealWorld(1,&endOfJoint,&endOfJoint);
						radius=abs(realPosition.position.Y-endOfJoint.Y);
						break;
					}
				}
			}
		}
		sphereRadii[i]=radius;
	}
}


void processFrame(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID)
{
	dImage=cvCreateImage(dSize,IPL_DEPTH_16U,1);
	uImage=cvCreateImage(dSize,IPL_DEPTH_1U,1);

	optimizeDepthMap(dpg,ug,userID);
	getSphereSizes(dpg,ug,userID);

	//Do not forget to release Images to prevent memory leak
	cvReleaseImage(&dImage);
	cvReleaseImage(&uImage);
}


void measureBody(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID)
{
	//Head Width Measurement
	xn::SkeletonCapability pUserSkel = ug->GetSkeletonCap();	
	XnSkeletonJointPosition head,neck,lShoulder,rShoulder,lFoot,rFoot,lHip,rHip,lElbow,rElbow,lHand,rHand,lShoulder,rShoulder,torso ;
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_HEAD, head);
	int tempX=head.position.X;
	int tempY=head.position.Y;

	while(cvGetReal2D(uImage,--tempX,tempY)>0);	//Extend the line horizontally until it reaches the borders of the head.
	int leftX=++tempX;
	tempX=head.position.X;
	while(cvGetReal2D(uImage,++tempX,tempY)>0);
	int rightX=--tempX;

	XnPoint3D headPoints[2];
	headPoints[0].X=leftX;
	headPoints[0].Y=tempY;
	headPoints[1].X=rightX;
	headPoints[1].Y=tempY;
	dpg->ConvertProjectiveToRealWorld(2,headPoints,headPoints);
	bodyMeasurements[HEAD_WIDTH]=abs(headPoints[0].X-headPoints[1].X);

	//Head Height Measurement
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_NECK, neck);
	XnPoint3D headNeckReal[2]={head.position,neck.position};	
	dpg->ConvertProjectiveToRealWorld(2,headNeckReal,headNeckReal);
	bodyMeasurements[HEAD_HEIGHT]=abs(headNeckReal[0].Y-headNeckReal[1].Y);


	//Body Height Measurement
	
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_LEFT_FOOT, lFoot);
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_RIGHT_FOOT, rFoot);	
	XnPoint3D feet[2]={lFoot.position,rFoot.position};
	dpg->ConvertProjectiveToRealWorld(2,feet,feet);
	float lowPointY=(feet[0].Y+feet[1].Y)/2;
	int tempX=head.position.X;
	int tempY=head.position.Y;
	while(cvGetReal2D(uImage,tempX,++tempY)>0);	//Extend the line verticallu until it reaches the top of the head.
	XnPoint3D topPoint;
	topPoint.X=tempX;
	topPoint.Y=--tempY;
	dpg->ConvertProjectiveToRealWorld(1,&topPoint,&topPoint);
	bodyMeasurements[BODY_HEIGHT]=abs(topPoint.Y-lowPointY);

	//Hip Height Measurement
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_LEFT_HIP, lHip);
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_RIGHT_HIP, rHip);	
	XnPoint3D hips[2]={lHip.position,rHip.position};
	dpg->ConvertProjectiveToRealWorld(2,hips,hips);
	bodyMeasurements[HIP_HEIGHT]=(abs(hips[0].Y-feet[0].Y)+abs(hips[1].Y-feet[1].Y))/2;

	//Elbow-Fingertip Measurement
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_LEFT_ELBOW, lElbow);
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_RIGHT_ELBOW, rElbow);
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_LEFT_HAND, lHand);
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_RIGHT_HAND, rHand);

	XnPoint3D lArmBottom,lArmTop,rArmBottom,rArmTop;
	tempX=lElbow.position.X;
	tempY=lElbow.position.Y;
	while(cvGetReal2D(uImage,tempX,--tempY)>0);	//Extend the line vertically until it reaches the borders of the arm.
	lArmBottom.X=tempX;
	lArmBottom.Y=++tempY;
	tempX=lHand.position.X;
	tempY=lHand.position.Y;
	while(cvGetReal2D(uImage,tempX,++tempY)>0);	//Extend the line vertically until it reaches the borders of the arm.
	lArmTop.X=tempX;
	lArmTop.Y=--tempY;
	tempX=rElbow.position.X;
	tempY=rElbow.position.Y;
	while(cvGetReal2D(uImage,tempX,--tempY)>0);	//Extend the line vertically until it reaches the borders of the arm.
	rArmBottom.X=tempX;
	rArmBottom.Y=++tempY;
	tempX=rHand.position.X;
	tempY=rHand.position.Y;
	while(cvGetReal2D(uImage,tempX,++tempY)>0);	//Extend the line vertically until it reaches the borders of the arm.
	rArmTop.X=tempX;
	rArmTop.Y=--tempY;

	XnPoint3D armPoints[4]={lArmBottom,lArmTop,rArmBottom,rArmTop};
	dpg->ConvertProjectiveToRealWorld(4,armPoints,armPoints);
	bodyMeasurements[ELBOW_FINGERTIP]=(abs(armPoints[1].Y-armPoints[0].Y)+abs(armPoints[3].Y-armPoints[2].Y))/2;

	//Wrist to Fingertip Measurement
	XnPoint3D hands[2]={lHand.position,rHand.position};
	dpg->ConvertProjectiveToRealWorld(2,hands,hands);
	bodyMeasurements[WRIST_FINGERTIP]=(abs(armPoints[1].Y-hands[0].Y)+abs(armPoints[3].Y-hands[1].Y))/2;

	//SHoulder Width Measurement
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_LEFT_SHOULDER, lShoulder);
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_RIGHT_SHOULDER, rShoulder);
	XnPoint3D shoulders[2]={lShoulder.position,rShoulder.position};
	dpg->ConvertProjectiveToRealWorld(2,shoulders,shoulders);
	bodyMeasurements[SHOULDER_WIDTH]=abs(shoulders[0].X-shoulders[1].X);

	//Hip Width Measurement

	XnPoint3D lEnd,rEnd;
	tempX=lHip.position.X;
	tempY=lHip.position.Y;
	while(cvGetReal2D(uImage,--tempX,tempY)>0);	//Extend the line vertically until it reaches the borders of the arm.
	lEnd.X=++tempX;
	lEnd.Y=tempY;

	tempX=rHip.position.X;
	tempY=rHip.position.Y;
	while(cvGetReal2D(uImage,++tempX,tempY)>0);	//Extend the line vertically until it reaches the borders of the arm.
	rEnd.X=--tempX;
	rEnd.Y=tempY;
	XnPoint3D hipEnds[2]={lEnd,rEnd};
	dpg->ConvertProjectiveToRealWorld(2,hipEnds,hipEnds);
	bodyMeasurements[HIP_WIDTH]=abs(hipEnds[0].X-hipEnds[1].X);

	//Torso Height Measurement

	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_TORSO, torso);
	dpg->ConvertProjectiveToRealWorld(1,&torso.position,&torso.position);
	bodyMeasurements[TORSO_HEIGHT]=abs(torso.position.Y-lowPointY);


}

//Code is prepared for sleeveless dress for now
void estimateParameters()
{
	estimatedShoulderWidth=(bodyMeasurements[SHOULDER_WIDTH]+bodyMeasurements[HEAD_WIDTH]*sleevelessProportions[SHOULDER_WIDTH_TO_HEAD_WIDTH])/2;
	estimatedTorsoHeight=bodyMeasurements[TORSO_HEIGHT]+bodyMeasurements[HEAD_HEIGHT]*sleevelessProportions[TORSO_HEIGHT_TO_HEAD_HEIGHT]+bodyMeasurements[HIP_HEIGHT]*sleevelessProportions[TORSO_HEIGHT_TO_HIP_HEIGHT]+
		+bodyMeasurements[ELBOW_FINGERTIP]*sleevelessProportions[TORSO_HEIGHT_TO_ELBOW_FINGERTIP]+bodyMeasurements[WRIST_FINGERTIP]*sleevelessProportions[TORSO_HEIGHT_TO_WRIST_FINGERTIP]+bodyMeasurements[BODY_HEIGHT]*sleevelessProportions[TORSO_HEIGHT_TO_BODY_HEIGHT];
	estimatedTorsoHeight=estimatedTorsoHeight/6;

}