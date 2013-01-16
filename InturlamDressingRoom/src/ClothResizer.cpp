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
					endOfJoint.X-=step;
					dpg->ConvertProjectiveToRealWorld(1,&endOfJoint,&endOfJoint);
					radius=abs(realPosition.position.X-endOfJoint.X);
					break;
				}
			}
			if (x_init+step<640)
			{
				if( !(bool)cvGetReal2D(uImage,x_init+step,y_init) )
				{
					endOfJoint.X+=step;
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
						endOfJoint.Y-=step;
						dpg->ConvertProjectiveToRealWorld(1,&endOfJoint,&endOfJoint);
						radius=abs(realPosition.position.Y-endOfJoint.Y);
						break;
					}
				}
				if (y_init+step<480)
				{
					if( !(bool)cvGetReal2D(uImage,x_init,y_init+step) )
					{
						endOfJoint.Y+=step;
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