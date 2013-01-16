#include "StdAfx.h"
#include "..\include\ClothResizer.h"


int gaussian_m=3,
	gaussian_n=3,
	gaussian_e=0.95;




ClothResizer::ClothResizer(void)
{
}


ClothResizer::~ClothResizer(void)
{
}




bool convertMetaDataToIpl(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID, IplImage* uImage, IplImage* dImage)
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
	CvSize dSize=cvSize(640,480);
	IplImage* dImage=cvCreateImage(dSize,IPL_DEPTH_16U,1);
	IplImage* uImage=cvCreateImage(dSize,IPL_DEPTH_1U,1);
	bool tStatus=convertMetaDataToIpl( dpg,ug,userID,  uImage, dImage);	//Convert Xn Matrices to OpenCV Matrices for easier calculation.
	CvScalar depthMean=cvAvg(dImage,uImage);							//Get teh Average Depth Value of the User Pixels
	cvNot(uImage,uImage);												//Invert the user pixels to paint the rest of the image with average user depth
	cvSet(dImage,depthMean,uImage);										 
	cvSmooth(dImage,dImage,CV_GAUSSIAN,gaussian_m,gaussian_n,gaussian_e);//Perform Gaussian Smoothing, depth map is optimized.
}




		

		