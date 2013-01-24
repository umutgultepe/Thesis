#include "StdAfx.h"
#include "..\include\ClothResizer.h"

float* radiiBuffer[29];
float* bodySizeBuffer[29];
int gaussian_m=3,
	gaussian_n=3,
	gaussian_e=0.95;

IplImage *uImage,*dImage,*showImage;

short minD=65536,maxD=0;

CvSize dSize=cvSize(m_Width,m_Height);
extern OgreBites::ParamsPanel* help;

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
	size_t j,i;
	try 
	{
	for ( j= 0; j < m_Height; j++)
	{
		char* uPtr=(char*)(uImage->imageData+j*uImage->widthStep);
		unsigned short* dPtr=(unsigned short*)(dImage->imageData+j*dImage->widthStep);
		for(i = 0; i < m_Width; i++)
		{		
			uint fixed_i = i;// fix i if we are mirrored
			if (!m_front)
				fixed_i = m_Width - i;				
			if (userID == pUsersLBLs[j*m_Width + fixed_i])// if we have a candidate, filter out the rest
			{	
				uPtr[fixed_i]=255;
				dPtr[fixed_i]=pDepth[j*m_Width + fixed_i];

				//if	(dPtr[fixed_i]< minD)
				//	minD=dPtr[fixed_i];
				//if	(dPtr[fixed_i]> maxD)
				//	maxD=dPtr[fixed_i];

			}		
		}
	}
	}
	catch (cv::Exception e)
	{
			ErrorDialog dlg;
			dlg.display(e.err + "\ni:" +  Ogre::StringConverter::toString(i) +  "\nj:" +  Ogre::StringConverter::toString(j));
			exit(0);
	}
	return true;
}

void showDepthImage()
{
	cvConvertImage(dImage,showImage);
	cvShowImage("Image", showImage);
	cvWaitKey();
	cvDestroyAllWindows();
}

bool optimizeDepthMap(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID)
{	

	if (!convertMetaDataToIpl( dpg,ug,userID))	//Convert Xn Matrices to OpenCV Matrices for easier calculation.
		return false;


	CvScalar depthMean=cvAvg(dImage,uImage);							//Get teh Average Depth Value of the User Pixels
	cvNot(uImage,uImage);												//Invert the user pixels to paint the rest of the image with average user depth
	//cvSet(dImage,depthMean,uImage);										 
	cvSmooth(dImage,dImage,CV_GAUSSIAN,gaussian_m,gaussian_n,gaussian_e);//Perform Gaussian Smoothing, depth map is optimized.

	cvNot(uImage,uImage);	
	cvErode(uImage,uImage,0,2);		//Smoothen the User Map as well
	cvDilate(uImage,uImage,0,2);
	return true;
}
Ogre::String windowName="Image";
void mouseEvent(int evt, int x, int y, int flags, void* param){
	HWND hw= (HWND)cvGetWindowHandle(windowName.c_str());
	windowName="x: "+ StringConverter::toString(x) + " , y: " + StringConverter::toString(y) ;
	SetWindowText(hw,(LPCTSTR)windowName.c_str());

}

void getSphereSizes(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID)
{
	xn::SkeletonCapability pUserSkel = ug->GetSkeletonCap();	
	for (int i=0;i<16;i++)
	{
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
		//cvShowImage(windowName.c_str(),uImage);
		//cvSetMouseCallback(windowName.c_str(), mouseEvent, 0);
		//cvWaitKey();

		XnPoint3D endOfJoint=projPosition;
		while( step<640)					//Slowly enlarge the joint sphere until it reaches the end of a bone in one direction.
		{
			unsigned char* iPtr=(unsigned char*)uImage->imageData+y_init*uImage->widthStep+x_init;
			if (x_init-step>-1)
			{

				unsigned char tValue=*(iPtr-step);
				if( !(bool)tValue )
				{
					endOfJoint.X-=(step-1);
					endOfJoint.Z=cvGetReal2D(dImage,endOfJoint.X,endOfJoint.Y);
					dpg->ConvertProjectiveToRealWorld(1,&endOfJoint,&endOfJoint);
					radius=abs(realPosition.position.X-endOfJoint.X);
					break;
				}
			}
			if (x_init+step<640)
			{
				unsigned char tValue=*(iPtr+step);
				if(!(bool)tValue )
				{
					endOfJoint.X+=(step-1);
					endOfJoint.Z=cvGetReal2D(dImage,endOfJoint.X,endOfJoint.Y);
					dpg->ConvertProjectiveToRealWorld(1,&endOfJoint,&endOfJoint);
					radius=abs(realPosition.position.X-endOfJoint.X);
					break;
				}
			}
			if (step<480)
			{
				if (y_init-step>-1)
				{
					unsigned char tValue=*(iPtr-step*uImage->widthStep);
					if( !(bool)tValue )
					{
						endOfJoint.Y-=(step-1);
						endOfJoint.Z=cvGetReal2D(dImage,endOfJoint.X,endOfJoint.Y);
						dpg->ConvertProjectiveToRealWorld(1,&endOfJoint,&endOfJoint);
						radius=abs(realPosition.position.Y-endOfJoint.Y);
						break;
					}
				}
				if (y_init+step<480)
				{
					unsigned char tValue=*(iPtr+step*uImage->widthStep);
					if( !(bool)tValue )
					{
						endOfJoint.Y+=(step-1);
						endOfJoint.Z=cvGetReal2D(dImage,endOfJoint.X,endOfJoint.Y);
						dpg->ConvertProjectiveToRealWorld(1,&endOfJoint,&endOfJoint);
						radius=abs(realPosition.position.Y-endOfJoint.Y);
						break;
					}
				}
			}
			step++;
		}
		sphereRadii[i]=radius;
	}
}

void measureBody(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID)
{
	//Head Width Measurement
	xn::SkeletonCapability pUserSkel = ug->GetSkeletonCap();	
	XnSkeletonJointPosition head,neck,lShoulder,rShoulder,lFoot,rFoot,lHip,rHip,lElbow,rElbow,lHand,rHand,torso ;
	XnPoint3D projHead;
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_HEAD, head);
	dpg->ConvertRealWorldToProjective(1,&head.position,&projHead);

	int tempX=projHead.X;
	int tempY=projHead.Y;
	unsigned char* headPtr=(unsigned char*) uImage->imageData+tempY*uImage->widthStep+tempX;
	unsigned char* iPtr=headPtr;
	int leftX=tempX;
	while(*--iPtr>0 && (headPtr-iPtr)<=tempX)
		leftX--;	//Extend the line horizontally until it reaches the borders of the head.
	int rightX=tempX;
	iPtr=headPtr;
	while(*++iPtr>0 && (iPtr-headPtr)<=(639-tempX))
		rightX++;
	XnPoint3D headPoints[2];
	headPoints[0].X=leftX;
	headPoints[0].Y=tempY;
	headPoints[0].Z=cvGetReal2D(dImage,leftX,tempY);
	headPoints[1].X=rightX;
	headPoints[1].Y=tempY;
	headPoints[2].Z=cvGetReal2D(dImage,rightX,tempY);
	dpg->ConvertProjectiveToRealWorld(2,headPoints,headPoints);
	bodyMeasurements[HEAD_WIDTH]=abs(headPoints[0].X-headPoints[1].X);

	//Head Height Measurement
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_NECK, neck);
	bodyMeasurements[HEAD_HEIGHT]=abs(head.position.Y-neck.position.Y);

	//Body Height Measurement
	
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_LEFT_FOOT, lFoot);
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_RIGHT_FOOT, rFoot);	
	XnPoint3D feet[2]={lFoot.position,rFoot.position};
	dpg->ConvertProjectiveToRealWorld(2,feet,feet);
	float lowPointY=(feet[0].Y+feet[1].Y)/2;
	tempX=head.position.X;
	tempY=head.position.Y;
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

bool processFrame(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID)
{
	if (!dImage)
	{
		dImage=cvCreateImage(dSize,IPL_DEPTH_16U,1);
		uImage=cvCreateImage(dSize,IPL_DEPTH_8U,1);
		showImage=cvCreateImage(dSize,IPL_DEPTH_8U,1);
	}
	cvSetZero(dImage);
	cvSetZero(uImage);


	if (!optimizeDepthMap(dpg,ug,userID))
		return false;
	getSphereSizes(dpg,ug,userID);

	measureBody(dpg,ug,userID);
	estimateParameters();

	return true;
}

void outputDataToCSV()
{
	std::ofstream*myfile=new std::ofstream("measurements.csv");

	
	if (myfile->is_open())
	{
			*myfile<<"Spheres,";
			for (int i=0;i<16;i++)
				*myfile<< Ogre::StringConverter::toString(i+1) +",";
			*myfile<<"\n";
			for (int j=0;j<29;j++)
			{
				*myfile<< Ogre::StringConverter::toString(j+1) +",";
				for (int i=0;i<16;i++)
					*myfile<< Ogre::StringConverter::toString(radiiBuffer[j][i]) +",";
				*myfile<<"\n";
			}
			*myfile<<"\n";
			*myfile<< "Average,";
			for (int i=0;i<16;i++)
					*myfile<< Ogre::StringConverter::toString(sphereRadii[i]) +",";
			*myfile<<"\n";
			*myfile<<"Body Sizes, Shoulder Width, Torso Height\n";
			for (int j=0;j<29;j++)
			{
				*myfile<< Ogre::StringConverter::toString(j+1) +",";
				for (int i=0;i<2;i++)
					*myfile<< Ogre::StringConverter::toString(bodySizeBuffer[j][i]) +",";
				*myfile<<"\n";
			}
			*myfile<<"\n";
			*myfile<< "Average,";
			*myfile<< Ogre::StringConverter::toString(estimatedShoulderWidth)+",";
			*myfile<< Ogre::StringConverter::toString(estimatedTorsoHeight)+",";
			*myfile<<"\n";
			myfile->close();
	}
}

bool addFrame(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID) //Data Collection for temporal Temporal Optimization
{
	if (!processFrame(dpg,ug,userID))
		return false;
	if (processedFrameCount==29)
	{
		for (int i=0;i<29;i++) //last frame data is already in the actual registers
		{
			for (int j=0;j<16;j++)
				sphereRadii[j]+=radiiBuffer[i][j];
			estimatedShoulderWidth+=bodySizeBuffer[i][0];
			estimatedTorsoHeight+=bodySizeBuffer[i][1];
			delete[] radiiBuffer[i];
			delete[] bodySizeBuffer[i];
		}
		for (int i=0;i<16;i++)
				sphereRadii[i]/=16;
		estimatedShoulderWidth/=16;
		estimatedTorsoHeight/=16;
		outputDataToCSV();
		for (int i=0;i<29;i++) //free the buffers
		{
			delete[] radiiBuffer[i];
			delete[] bodySizeBuffer[i];
		}
		//Do not forget to release Images to prevent memory leak
		cvReleaseImage(&dImage);
		cvReleaseImage(&uImage);
		cvReleaseImage(&dImage);

		return true;
	}
	else
	{
		radiiBuffer[processedFrameCount]=new float[16];
		memcpy(radiiBuffer[processedFrameCount],sphereRadii,16*sizeof(float));
		bodySizeBuffer[processedFrameCount]=new float[2];
		bodySizeBuffer[processedFrameCount][0]=estimatedShoulderWidth;
		bodySizeBuffer[processedFrameCount][1]=estimatedTorsoHeight;
		processedFrameCount++;
		return false;
	}
}
	


