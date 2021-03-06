#include "StdAfx.h"
#include "ClothResizer.h"

float* radiiBuffer[29];
float* bodySizeBuffer[29];
int gaussian_m=5,
	gaussian_n=5,
	gaussian_e=0.95;
#if USE_USER_SCALING
float sphereRadii[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0//Additional
						,0,0,0,0,0,0};
#else
float sphereRadii[] = 
	{1.3525,//BONE_STOMACH
	1.7325,//BONE_WAIST
	1,//BONE_ROOT
	1.5775,//BONE_CHEST
	0.9875,//BONE_LEFT_HUMERUS
	0.9875,//BONE_RIGHT_HUMERUS
	0.6125,//BONE_LEFT_ULNA
	0.6125,//BONE_RIGHT_ULNA
	1.3525,//BONE_LEFT_THIGH
	1.3525,//BONE_RIGHT_THIGH
	1.715,//BONE_LEFT_CALF
	1.715,//BONE_RIGHT_CAL3F
	0.01,//BONE_LEFT_HAND
	0.01,//BONE_RIGHT_HAND
	0.6,//BONE_LEFT_FOOT
	0.6,//BONE_RIGHT_FOOT
	0.6,//BONE_NECK
	2.125,//BONE_LE1.L
	2.125,//BONE_LE1.R
	2.3275,//BONE_LE2.L
	2.3275,//BONE_LE2.R
	2.4,//BONE_LEH.L
	2.4,//BONE_LEH.R
	2.2,//BONE_LEH2.L
	2.2,//BONE_LEH2.R
	2.3,//BONE_WaistExtent.L
	2.3,//BONE_WaistExtent.R
	2.125,//Groins
	0.01
};
	
//float sphereRadii[] = 
//	{1.3525,//BONE_STOMACH
//	1.7325,//BONE_WAIST
//	1,//BONE_ROOT
//	1.5775,//BONE_CHEST
//	0.9875,//BONE_LEFT_HUMERUS
//	0.9875,//BONE_RIGHT_HUMERUS
//	0.6125,//BONE_LEFT_ULNA
//	0.6125,//BONE_RIGHT_ULNA
//	1.3525,//BONE_LEFT_THIGH
//	1.3525,//BONE_RIGHT_THIGH
//	1.715,//BONE_LEFT_CALF
//	1.715,//BONE_RIGHT_CAL3F
//	0.42,//BONE_LEFT_HAND
//	0.42,//BONE_RIGHT_HAND
//	0.6,//BONE_LEFT_FOOT
//	0.6,//BONE_RIGHT_FOOT
//	2.125,//BONE_LE1.L
//	2.125,//BONE_LE1.R
//	2.3275,//BONE_LE2.L
//	2.3275,//BONE_LE2.R
//	2.4,//BONE_LEH.L
//	2.4,//BONE_LEH.R
//	2.5,//BONE_LEH2.L
//	2.5,//BONE_LEH2.R
//	2.6,//BONE_WaistExtent.L
//	2.6,//BONE_WaistExtent.R
//	2.125,//Groins
//	0.01
//};
	

#endif
float estimatedShoulderWidth=0;
float estimatedTorsoHeight=0;
float estimatedBodyHeight=0;

IplImage *uImage,*dImage,*showImage;
Ogre::String windowName="Image";
short minD=65535,maxD=0;


extern OgreBites::ParamsPanel* help;

void viewImage(IplImage* img)
{
	if (img->depth == IPL_DEPTH_16U)
	{
		cvConvertImage(img,showImage);
		cvShowImage("Image", showImage);
	}
	else
	{
		cvShowImage("Image",img);
	}
	cvWaitKey();
	cvDestroyAllWindows();
}

//Code is prepared for sleeveless dress for now
void estimateParameters()
{
	estimatedShoulderWidth=(bodyMeasurements[SHOULDER_WIDTH]+bodyMeasurements[HEAD_WIDTH]*sleevelessProportions[SHOULDER_WIDTH_TO_HEAD_WIDTH])/2;
	estimatedTorsoHeight=bodyMeasurements[TORSO_HEIGHT]+bodyMeasurements[HEAD_HEIGHT]*sleevelessProportions[TORSO_HEIGHT_TO_HEAD_HEIGHT]+bodyMeasurements[HIP_HEIGHT]*sleevelessProportions[TORSO_HEIGHT_TO_HIP_HEIGHT]+
		+bodyMeasurements[ELBOW_FINGERTIP]*sleevelessProportions[TORSO_HEIGHT_TO_ELBOW_FINGERTIP]+bodyMeasurements[WRIST_FINGERTIP]*sleevelessProportions[TORSO_HEIGHT_TO_WRIST_FINGERTIP]+bodyMeasurements[BODY_HEIGHT]*sleevelessProportions[TORSO_HEIGHT_TO_BODY_HEIGHT];
	estimatedTorsoHeight=estimatedTorsoHeight/6;
	estimatedBodyHeight=estimatedTorsoHeight/sleevelessProportions[TORSO_HEIGHT_TO_BODY_HEIGHT];
}

void mouseEvent(int evt, int x, int y, int flags, void* param){
	HWND hw= (HWND)cvGetWindowHandle(windowName.c_str());
	windowName="x: "+ StringConverter::toString(x) + " , y: " + StringConverter::toString(y) ;
	SetWindowText(hw,(LPCTSTR)windowName.c_str());
}

bool optimizeDepthMap()
{	
	cvErode(uImage,uImage,0,2);		//Smoothen the User Map as well
	cvDilate(uImage,uImage,0,2);
	CvScalar depthMean=cvAvg(dImage,uImage);							//Get teh Average Depth Value of the User Pixels
	cvNot(uImage,uImage);												//Invert the user pixels to paint the rest of the image with average user depth									 
	//viewImage(dImage);
	cvSet(dImage,depthMean,uImage);										 
	IplImage* tempImage=cvCreateImage(dSize,IPL_DEPTH_8U,1);
	cvConvertScale(dImage,tempImage,1.0/256);
	cvSmooth(tempImage,tempImage,CV_GAUSSIAN,7);//Perform Gaussian Smoothing, depth map is optimized.
	cvConvert(tempImage,dImage);
	cvScale(dImage,dImage,256);
	cvSet(dImage,cvScalar(0),uImage);	
	//viewImage(dImage);
	//cvSmooth(dImage,dImage,CV_GAUSSIAN,gaussian_m,gaussian_n,gaussian_e);//Perform Gaussian Smoothing, depth map is optimized.
	cvNot(uImage,uImage);
	cvReleaseImage(&tempImage);
	return true;
}

void outputDataToCSV()
{
	std::ofstream*myfile=new std::ofstream("..\\measurements.csv");

	
	if (myfile->is_open())
	{
			*myfile<<"sep=\t\n";
			*myfile<<"Spheres\t";
			for (int i=0;i<16;i++)
				*myfile<< jointStrings[i] +"\t";
			*myfile<<"\n";
			for (int j=0;j<29;j++)
			{
				*myfile<< Ogre::StringConverter::toString(j+1) +"\t";
				for (int i=0;i<16;i++)
					*myfile<< Ogre::StringConverter::toString(radiiBuffer[j][i]) +"\t";
				*myfile<<"\n";
			}
			*myfile<<"\n";
			*myfile<< "Average\t";
			for (int i=0;i<16;i++)
					*myfile<< Ogre::StringConverter::toString(sphereRadii[i]) +"\t";
			*myfile<<"\n";
			*myfile<<"Body Sizes\t Shoulder Width\t Torso Height\t Body Height\n";
			for (int j=0;j<29;j++)
			{
				*myfile<< Ogre::StringConverter::toString(j+1) +"\t";
				for (int i=0;i<3;i++)
					*myfile<< Ogre::StringConverter::toString(bodySizeBuffer[j][i]) +"\t";
				*myfile<<"\n";
			}
			*myfile<<"\n";
			*myfile<< "Average\t";
			*myfile<< Ogre::StringConverter::toString(estimatedShoulderWidth)+"\t";
			*myfile<< Ogre::StringConverter::toString(estimatedTorsoHeight)+"\t";
			*myfile<< Ogre::StringConverter::toString(estimatedBodyHeight)+"\t";
			*myfile<<"\n";
			myfile->close();
	}
}

#if USE_KINECT

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
		unsigned char* uPtr=(unsigned char*)(uImage->imageData+j*uImage->widthStep);
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



void getSphereSizes(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID)
{
	xn::SkeletonCapability pUserSkel = ug->GetSkeletonCap();	
	int x_init=0;
	int y_init=0;
	int step=0;
	XnPoint3D endOfJoint;
	try 
	{
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
			x_init=projPosition.X;
			y_init=projPosition.Y;
			step=0;
			int radius=0;
			//cvShowImage(windowName.c_str(),uImage);
			//cvSetMouseCallback(windowName.c_str(), mouseEvent, 0);
			//cvWaitKey();
		
			endOfJoint=projPosition;
			unsigned char* iPtr=(unsigned char*)uImage->imageData+y_init*uImage->widthStep+x_init;
			unsigned short* dPtr=(unsigned short*)(dImage->imageData+y_init*dImage->widthStep+x_init*2);//Multipy x_init by 2, since dImage is 16 bits- 2bytes
			while( step<640)					//Slowly enlarge the joint sphere until it reaches the end of a bone in one direction.
			{			
				if (x_init-step>-1)
				{
					unsigned char tValue=*(iPtr-step);
					if( tValue!=0 )
					{
						endOfJoint.X-=(step-1);
						endOfJoint.Z=*(dPtr-(step-1));
						dpg->ConvertProjectiveToRealWorld(1,&endOfJoint,&endOfJoint);
						radius=abs(realPosition.position.X-endOfJoint.X);
						break;
					}
				}
				if (x_init+step<640)
				{
					unsigned char tValue=*(iPtr+step);
					if( tValue!=0)
					{
						endOfJoint.X+=(step-1);
						endOfJoint.Z=*(dPtr+(step-1));
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
						if(  tValue!=0 )
						{
							endOfJoint.Y-=(step-1);
							endOfJoint.Z=*(dPtr-(step-1)*dImage->widthStep/2);	//Divide widthstep by 2, since step is in bytes an pointer increments in 2 bytes
							dpg->ConvertProjectiveToRealWorld(1,&endOfJoint,&endOfJoint);
							radius=abs(realPosition.position.Y-endOfJoint.Y);
							break;
						}
					}
					if (y_init+step<480)
					{
						unsigned char tValue=*(iPtr+step*uImage->widthStep);
						if( tValue!=0)
						{
							endOfJoint.Y+=(step-1);
							endOfJoint.Z=*(dPtr+(step-1)*dImage->widthStep/2);
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
	catch( Ogre::Exception& e ) {
			MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);

	}
	catch(cv::Exception e) {
		MessageBox( NULL, e.err.c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);

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

	int projHeadX=projHead.X;
	int projHeadY=projHead.Y;
	unsigned char* headPtr=(unsigned char*) uImage->imageData+projHeadY*uImage->widthStep+projHeadX;
	unsigned short* headDepthPtr=(unsigned short*) (dImage->imageData+projHeadY*dImage->widthStep+projHeadX*2);
	unsigned char* iPtr=headPtr;
	int leftX=projHeadX;
	int rightX=projHeadX;
	int leftStep=0;
	int rightStep=0;
	while(*(--iPtr)>0 && 0<leftX)
	{
		leftX--;	//Extend the line horizontally until it reaches the borders of the head.
		leftStep++;
	}
	iPtr=headPtr;
	while(*(++iPtr)>0 && rightX<639)
	{
		rightX++;
		rightStep++;
	}
	XnPoint3D headPoints[2];
	headPoints[0].X=leftX;
	headPoints[0].Y=projHeadY;
	headPoints[0].Z=*(headDepthPtr-leftStep);
	headPoints[1].X=rightX;
	headPoints[1].Y=projHeadY;
	headPoints[1].Z=*(headDepthPtr+rightStep);
	dpg->ConvertProjectiveToRealWorld(2,headPoints,headPoints);
	bodyMeasurements[HEAD_WIDTH]=abs(headPoints[0].X-headPoints[1].X);

	//Head Height Measurement
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_NECK, neck);
	bodyMeasurements[HEAD_HEIGHT]=abs(head.position.Y-neck.position.Y);

	//Body Height Measurement
	
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_LEFT_FOOT, lFoot);
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_RIGHT_FOOT, rFoot);	
	XnPoint3D feet[2]={lFoot.position,rFoot.position};
	float lowPointY=(feet[0].Y+feet[1].Y)/2;
	XnPoint3D topPoint;
	iPtr=headPtr-=uImage->widthStep;	//Initialize the pointer 1 pixel above, since decrement will take place after comparison.
	topPoint.X=projHeadX;
	topPoint.Y=projHeadY;
	int topStep=0;
	while(*iPtr>0 && (topPoint.Y>0))
	{
		iPtr-=uImage->widthStep;
		topPoint.Y--;
		topStep++;
	}
	topPoint.Z=*(headDepthPtr - topStep*dImage->widthStep/2);
	dpg->ConvertProjectiveToRealWorld(1,&topPoint,&topPoint);
	bodyMeasurements[BODY_HEIGHT]=abs(topPoint.Y-lowPointY);

	//Hip Height Measurement
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_LEFT_HIP, lHip);
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_RIGHT_HIP, rHip);	
	XnPoint3D hips[2]={lHip.position,rHip.position};
	bodyMeasurements[HIP_HEIGHT]=(abs(hips[0].Y-feet[0].Y)+abs(hips[1].Y-feet[1].Y))/2;

	//Elbow-Fingertip Measurement
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_LEFT_ELBOW, lElbow);
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_RIGHT_ELBOW, rElbow);
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_LEFT_HAND, lHand);
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_RIGHT_HAND, rHand);
	XnPoint3D lArmBottom,lArmTop,rArmBottom,rArmTop,
		lElbowReal=lElbow.position,
		lHandReal=lHand.position,
		rElbowReal=rElbow.position,
		rHandReal=rHand.position;


	XnPoint3D armPoints[4]={lElbowReal,lHandReal,rElbowReal,rHandReal};
	dpg->ConvertRealWorldToProjective(4,armPoints,armPoints);

	
	lArmTop.X=armPoints[1].X;
	lArmTop.Y=armPoints[1].Y;
	unsigned char* lHandPtr=(unsigned char*) uImage->imageData+(int)(lArmTop.Y-1)*uImage->widthStep+(int)lArmTop.X;//Initialize the pointer 1 pixel above, since decrement will take place after comparison.
	while(*lHandPtr>0  && (lArmTop.Y>0))
	{
		lHandPtr-=uImage->widthStep;
		lArmTop.Y--;	
	}	//Extend the line vertically until it reaches the borders of the arm.

	lArmBottom.X=armPoints[0].X;
	lArmBottom.Y=armPoints[0].Y;
	unsigned char* lElbowPtr=(unsigned char*) uImage->imageData+(int)(lArmBottom.Y+1)*uImage->widthStep+(int)lArmBottom.X;//Initialize the pointer 1 pixel above, since decrement will take place after comparison.
	while(*lElbowPtr>0  && (lArmBottom.Y<639))
	{
		lElbowPtr+=uImage->widthStep;
		lArmBottom.Y++;	
	}	//Extend the line vertically until it reaches the borders of the arm.

	rArmTop.X=armPoints[3].X;
	rArmTop.Y=armPoints[3].Y;
	unsigned char* rHandPtr=(unsigned char*) uImage->imageData+(int)(rArmTop.Y-1)*uImage->widthStep+(int)rArmTop.X;//Initialize the pointer 1 pixel above, since decrement will take place after comparison.
	while(*rHandPtr>0  && (rArmTop.Y>0))
	{
		rHandPtr-=uImage->widthStep;
		rArmTop.Y--;	
	}	//Extend the line vertically until it reaches the borders of the arm.


	rArmBottom.X=armPoints[2].X;
	rArmBottom.Y=armPoints[2].Y;
	unsigned char* rElbowPtr=(unsigned char*) uImage->imageData+(int)(rArmBottom.Y+1)*uImage->widthStep+(int)rArmBottom.X;//Initialize the pointer 1 pixel above, since decrement will take place after comparison.
	while(*rElbowPtr>0  && (rArmBottom.Y<639))
	{
		rElbowPtr+=uImage->widthStep;
		rArmBottom.Y++;	
	}	//Extend the line vertically until it reaches the borders of the arm.

	unsigned short* depthPtr=(unsigned short*) (dImage->imageData);


	XnPoint3D armEndsReal[4]={lArmBottom,lArmTop,rArmBottom,rArmTop};
	for (int i=0;i<4;i++)
		armEndsReal[i].Z=*(depthPtr+(int)armEndsReal[i].Y*dImage->widthStep/2+(int)armEndsReal[i].X);

	dpg->ConvertProjectiveToRealWorld(4,armEndsReal,armEndsReal);
	bodyMeasurements[ELBOW_FINGERTIP]=(abs(armEndsReal[1].Y-armEndsReal[0].Y)+abs(armEndsReal[3].Y-armEndsReal[2].Y))/2;

	//Wrist to Fingertip Measurement
	bodyMeasurements[WRIST_FINGERTIP]=(abs(armEndsReal[1].Y-lHandReal.Y)+abs(armEndsReal[3].Y-rHandReal.Y))/2;

	//SHoulder Width Measurement
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_LEFT_SHOULDER, lShoulder);
	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_RIGHT_SHOULDER, rShoulder);
	XnPoint3D shoulders[2]={lShoulder.position,rShoulder.position};
	bodyMeasurements[SHOULDER_WIDTH]=abs(shoulders[0].X-shoulders[1].X);

	//Hip Width Measurement

	XnPoint3D lEnd,rEnd;
	XnPoint3D projHips[2]={lHip.position,rHip.position};
	dpg->ConvertRealWorldToProjective(2,projHips,projHips);


	unsigned char* lEndPtr=(unsigned char*) uImage->imageData+(int)projHips[0].Y*uImage->widthStep+(int)projHips[0].X;
	leftX=projHips[0].X;
	leftStep=0;
	while(*(--lEndPtr)>0 && leftX>0)
	{
		leftX--;	//Extend the line horizontally until it reaches the borders of the head.
		leftStep++;
	}
	lEnd.X=leftX;
	lEnd.Y=projHips[0].Y;
	unsigned short* lEndDepth=(unsigned short*) (dImage->imageData+(int)lEnd.Y*dImage->widthStep+(int)lEnd.X*2);
	lEnd.Z=*lEndDepth;


	

	unsigned char* rEndPtr=(unsigned char*) uImage->imageData+(int)projHips[1].Y*uImage->widthStep+(int)projHips[1].X;
	rightX=projHips[1].X;
	rightStep=0;
	while(*(++rEndPtr)>0 && rightX<639)
	{
		rightX++;	//Extend the line horizontally until it reaches the borders of the head.
		rightStep++;
	}
	rEnd.X=rightX;
	rEnd.Y=projHips[1].Y;
	unsigned short* rEndDepth=(unsigned short*) (dImage->imageData+(int)rEnd.Y*dImage->widthStep+(int)rEnd.X*2);
	rEnd.Z=*rEndDepth;

		
	XnPoint3D hipEnds[2]={lEnd,rEnd};
	dpg->ConvertProjectiveToRealWorld(2,hipEnds,hipEnds);
	bodyMeasurements[HIP_WIDTH]=abs(hipEnds[0].X-hipEnds[1].X);

	//Torso Height Measurement

	pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_TORSO, torso);
	bodyMeasurements[TORSO_HEIGHT]=abs(torso.position.Y-lowPointY);


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
	if (!convertMetaDataToIpl( dpg,ug,userID))	//Convert Xn Matrices to OpenCV Matrices for easier calculation.
		return false;
	optimizeDepthMap();
	getSphereSizes(dpg,ug,userID);
	measureBody(dpg,ug,userID);
	estimateParameters();
	return true;
}
bool addFrame(xn::DepthGenerator* dpg,xn::UserGenerator* ug,XnUserID userID) //Data Collection for temporal Temporal Optimization
{
	if (processedFrameCount>29)
		return true;
	else if (processedFrameCount==29)
	{
		for (int i=0;i<29;i++) //last frame data is already in the actual registers
		{
			for (int j=0;j<16;j++)
				sphereRadii[j]+=radiiBuffer[i][j];
			estimatedShoulderWidth+=bodySizeBuffer[i][0];
			estimatedTorsoHeight+=bodySizeBuffer[i][1];
			estimatedBodyHeight+=bodySizeBuffer[i][2];
		}
		for (int i=0;i<16;i++)
				sphereRadii[i]/=30;
		for (int i=10;i<12;i++)		//Correct Torso, as they are measured by the same bone, and can cause errors.
				sphereRadii[i]*=1.4;
		for (int i=8;i<10;i++)		//Correct Torso, as they are measured by the same bone, and can cause errors.
				sphereRadii[i]*=1.2;
		for (int i=0;i<4;i++)		//Correct Torso, as they are measured by the same bone, and can cause errors.
				sphereRadii[i]*=0.75;
		estimatedShoulderWidth/=30;
		estimatedTorsoHeight/=30;
		estimatedBodyHeight/=30;
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
		processedFrameCount++;
		return true;
	}
	else
	{
		if (processFrame(dpg,ug,userID))
		{	
			radiiBuffer[processedFrameCount]=new float[16];
			memcpy(radiiBuffer[processedFrameCount],sphereRadii,16*sizeof(float));
			bodySizeBuffer[processedFrameCount]=new float[3];
			bodySizeBuffer[processedFrameCount][0]=estimatedShoulderWidth;
			bodySizeBuffer[processedFrameCount][1]=estimatedTorsoHeight;
			bodySizeBuffer[processedFrameCount][2]=estimatedBodyHeight;
			processedFrameCount++;
		}
		return false;
	}
}
	
#elif USE_NUI

void getSphereSizes(NUI_Controller* mNui)
{
	int x_init=0;
	int y_init=0;
	int step=0;
	try 
	{
		for (int i=0;i<16;i++)
		{
			NUI_SKELETON_POSITION_INDEX sJoint=nuiIDs[i];
			for (int j=0;j<i;j++)				//If joint is processed before, do not repeat it
			{
				if (nuiIDs[j]==sJoint)
					sphereRadii[i]=sphereRadii[j];
			}

			NUI_Vector4 realPosition=mNui->m_Points[sJoint];
			Vector2 endOfJoint;
			LONG x,y;
			USHORT depth;				
			NuiTransformSkeletonToDepthImage( realPosition, &x, &y, &depth ,mNui->m_DepthResolution);
			x_init=x;
			y_init=y;
			step=0;
			int radius=0;
			//cvShowImage(windowName.c_str(),uImage);
			//cvSetMouseCallback(windowName.c_str(), mouseEvent, 0);
			//cvWaitKey();
		
			endOfJoint.x=x;
			endOfJoint.y=x;

			UCHAR* iPtr=(UCHAR*)(uImage->imageData+y_init*uImage->widthStep+x_init);
			USHORT* dPtr=(USHORT*)(dImage->imageData+y_init*dImage->widthStep+x_init*2);//Multipy x_init by 2, since dImage is 16 bits- 2bytes
			while( step<m_Width)					//Slowly enlarge the joint sphere until it reaches the end of a bone in one direction.
			{			
				if (x_init-step>-1)
				{
					UCHAR tValue=*(iPtr-step);
					if( tValue!=0 )
					{
						endOfJoint.x-=(step-1);
						NUI_Vector4 trueEnd=NuiTransformDepthImageToSkeleton(endOfJoint.x,endOfJoint.y,*(dPtr-(step-1)),mNui->m_DepthResolution);
						radius=abs(realPosition.x-trueEnd.x);
						break;
					}
				}
				if (x_init+step<m_Width)
				{
					UCHAR tValue=*(iPtr+step);
					if( tValue!=0)
					{

						endOfJoint.x+=(step-1);
						NUI_Vector4 trueEnd=NuiTransformDepthImageToSkeleton(endOfJoint.x,endOfJoint.y,*(dPtr+(step-1)),mNui->m_DepthResolution);
						radius=abs(realPosition.x-trueEnd.x);
						break;
					}
				}
				if (step<m_Height)
				{
					if (y_init-step>-1)
					{
						UCHAR tValue=*(iPtr-step*uImage->widthStep);
						if(  tValue!=0 )
						{
							endOfJoint.y-=(step-1);
							NUI_Vector4 trueEnd=NuiTransformDepthImageToSkeleton(endOfJoint.x,endOfJoint.y,*(dPtr-(step-1)*dImage->widthStep/2),mNui->m_DepthResolution);
							radius=abs(realPosition.y-trueEnd.y);
							break;
						}
					}
					if (y_init+step<m_Height)
					{
						UCHAR tValue=*(iPtr+step*uImage->widthStep);
						if( tValue!=0)
						{
							endOfJoint.y+=(step-1);
							NUI_Vector4 trueEnd=NuiTransformDepthImageToSkeleton(endOfJoint.x,endOfJoint.y,*(dPtr+(step-1)*dImage->widthStep/2),mNui->m_DepthResolution);
							radius=abs(realPosition.y-trueEnd.y);
							break;
						}
					}
				}
				step++;
			}
			sphereRadii[i]=radius;
		}
	}
	catch( Ogre::Exception& e ) {
			MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);

	}
	catch(cv::Exception e) {
		MessageBox( NULL, e.err.c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);

	}
}

void measureBody(NUI_Controller* mNui)
{
	//Head Width Measurement
	NUI_Vector4 head,neck,lShoulder,rShoulder,lFoot,rFoot,lHip,rHip,lElbow,rElbow,lHand,rHand,torso ;
	head=mNui->m_Points[NUI_SKELETON_POSITION_HEAD];
	LONG projHeadX,projHeadY;
	USHORT depth;
	NuiTransformSkeletonToDepthImage( head, &projHeadX, &projHeadY, &depth ,mNui->m_DepthResolution);

	UCHAR* headPtr=(UCHAR*)( uImage->imageData+projHeadY*uImage->widthStep+projHeadX);
	USHORT* headDepthPtr=(USHORT*) (dImage->imageData+projHeadY*dImage->widthStep+projHeadX*2);
	UCHAR* iPtr=headPtr;
	LONG leftX=projHeadX;
	LONG rightX=projHeadX;
	LONG leftStep=0;
	LONG rightStep=0;
	while(*(--iPtr)>0 && 0<leftX)
	{
		leftX--;	//Extend the line horizontally until it reaches the borders of the head.
		leftStep++;
	}
	iPtr=headPtr;
	while(*(++iPtr)>0 && rightX<(m_Width-1))
	{
		rightX++;
		rightStep++;
	}

	NUI_Vector4 leftHead=NuiTransformDepthImageToSkeleton(leftX,projHeadY,*(headDepthPtr-leftStep),mNui->m_DepthResolution);
	NUI_Vector4 rightHead=NuiTransformDepthImageToSkeleton(rightX,projHeadY,*(headDepthPtr+leftStep),mNui->m_DepthResolution);

	bodyMeasurements[HEAD_WIDTH]=abs(rightHead.x-leftHead.y);

	//Head Height Measurement
	
	neck=mNui->m_Points[NUI_SKELETON_POSITION_SHOULDER_CENTER];
	bodyMeasurements[HEAD_HEIGHT]=abs(head.y-neck.y);

	//Body Height Measurement
	
	lFoot=mNui->m_Points[NUI_SKELETON_POSITION_FOOT_LEFT];
	rFoot=mNui->m_Points[NUI_SKELETON_POSITION_FOOT_RIGHT];
	float lowPointY=(lFoot.y+rFoot.y)/2;
	
	iPtr=headPtr-uImage->widthStep;	//Initialize the pointer 1 pixel above, since decrement will take place after comparison.
	LONG topY;
	int topStep=0;
	while(*iPtr>0 && (topY>0))
	{
		iPtr-=uImage->widthStep;
		topY--;
		topStep++;
	}
	NUI_Vector4 topPoint=NuiTransformDepthImageToSkeleton(projHeadX,topY,*(headDepthPtr - topStep*dImage->widthStep/2),mNui->m_DepthResolution);
	bodyMeasurements[BODY_HEIGHT]=abs(topPoint.y-lowPointY);

	//Hip Height Measurement

	lHip=mNui->m_Points[NUI_SKELETON_POSITION_HIP_LEFT];
	rHip=mNui->m_Points[NUI_SKELETON_POSITION_HIP_RIGHT];
	bodyMeasurements[HIP_HEIGHT]=(abs(lHip.y-lFoot.y)+abs(rHip.y-lHip.y))/2;

	//Elbow-Fingertip Measurement
	lElbow=mNui->m_Points[NUI_SKELETON_POSITION_ELBOW_LEFT];
	rElbow=mNui->m_Points[NUI_SKELETON_POSITION_ELBOW_RIGHT];
	lHand=mNui->m_Points[NUI_SKELETON_POSITION_HAND_LEFT];
	rHand=mNui->m_Points[NUI_SKELETON_POSITION_HAND_RIGHT];


	LONG lHandUpX,rHandUpX,lHandDownX,rHandDownX,lHandUpY,lHandDownY,rHandUpY,rHandDownY;
	NuiTransformSkeletonToDepthImage( lHand, &lHandUpX, &lHandUpY, &depth ,mNui->m_DepthResolution);
	NuiTransformSkeletonToDepthImage( rHand, &rHandUpX, &rHandUpY, &depth ,mNui->m_DepthResolution);
	NuiTransformSkeletonToDepthImage( lElbow, &lHandDownX, &lHandDownY, &depth ,mNui->m_DepthResolution);
	NuiTransformSkeletonToDepthImage( rElbow, &rHandDownX, &rHandDownY, &depth ,mNui->m_DepthResolution);

	UCHAR* lHandPtr=(UCHAR*)( uImage->imageData+(int)(lHandUpY-1)*uImage->widthStep+(int)lHandUpX);//Initialize the pointer 1 pixel above, since decrement will take place after comparison.
	while(*lHandPtr>0  && (lHandUpY>0))
	{
		lHandPtr-=uImage->widthStep;
		lHandUpY--;	
	}	//Extend the line vertically until it reaches the borders of the arm.


	UCHAR* lElbowPtr=(UCHAR*) (uImage->imageData+(int)(lHandDownY+1)*uImage->widthStep+(int)lHandDownX);//Initialize the pointer 1 pixel above, since decrement will take place after comparison.
	while(*lElbowPtr>0  && (lHandDownY<(m_Width-1)))
	{
		lElbowPtr+=uImage->widthStep;
		lHandDownY++;	
	}	//Extend the line vertically until it reaches the borders of the arm.

	UCHAR* rHandPtr=(UCHAR*) (uImage->imageData+(int)(rHandUpY-1)*uImage->widthStep+(int)rHandUpX);//Initialize the pointer 1 pixel above, since decrement will take place after comparison.
	while(*rHandPtr>0  && (rHandUpY>0))
	{
		rHandPtr-=uImage->widthStep;
		rHandUpY--;	
	}	//Extend the line vertically until it reaches the borders of the arm.

	UCHAR* rElbowPtr=(UCHAR*)(uImage->imageData+(int)(rHandDownY+1)*uImage->widthStep+(int)rHandDownX);//Initialize the pointer 1 pixel above, since decrement will take place after comparison.
	while(*rElbowPtr>0  && (rHandDownY<(m_Height-1)))
	{
		rElbowPtr+=uImage->widthStep;
		rHandDownY++;	
	}	//Extend the line vertically until it reaches the borders of the arm.

	USHORT* depthPtr=(USHORT*) (dImage->imageData);
	NUI_Vector4 leftArmDown=NuiTransformDepthImageToSkeleton(lHandDownX,lHandDownY,*(depthPtr+lHandDownY*dImage->widthStep/2+lHandDownX),mNui->m_DepthResolution);
	NUI_Vector4 leftArmUp=NuiTransformDepthImageToSkeleton(lHandUpX,lHandUpY,*(depthPtr+lHandUpY*dImage->widthStep/2+lHandUpX),mNui->m_DepthResolution);
	NUI_Vector4 rightArmDown=NuiTransformDepthImageToSkeleton(rHandDownX,rHandDownY,*(depthPtr+rHandDownY*dImage->widthStep/2+rHandDownX),mNui->m_DepthResolution);
	NUI_Vector4 rightArmUp=NuiTransformDepthImageToSkeleton(rHandUpX,rHandUpY,*(depthPtr+rHandUpY*dImage->widthStep/2+rHandUpX),mNui->m_DepthResolution);
	bodyMeasurements[ELBOW_FINGERTIP]=(abs(rightArmUp.y-rightArmDown.y)+abs(leftArmUp.y-leftArmDown.y))/2;
	//Wrist to Fingertip Measurement
	bodyMeasurements[WRIST_FINGERTIP]=abs(leftArmUp.y-mNui->m_Points[NUI_SKELETON_POSITION_WRIST_LEFT].y)+abs(rightArmUp.y-mNui->m_Points[NUI_SKELETON_POSITION_WRIST_RIGHT].y)/2;

	//SHoulder Width Measurement
	lShoulder=mNui->m_Points[NUI_SKELETON_POSITION_SHOULDER_LEFT];
	rShoulder=mNui->m_Points[NUI_SKELETON_POSITION_SHOULDER_RIGHT];
	bodyMeasurements[SHOULDER_WIDTH]=abs(rShoulder.x-lShoulder.x);

	//Hip Width Measurement
	LONG lHipX,lHipY,rHipX,rHipY;
	NuiTransformSkeletonToDepthImage(lHip,&lHipX,&lHipY,&depth,mNui->m_DepthResolution);
	NuiTransformSkeletonToDepthImage(rHip,&rHipX,&rHipY,&depth,mNui->m_DepthResolution);

	UCHAR* lEndPtr=(UCHAR*)(uImage->imageData+(int)lHipY*uImage->widthStep+(int)lHipX);
	leftX=lHipX;
	leftStep=0;
	while(*(--lEndPtr)>0 && leftX>0)
	{
		leftX--;	//Extend the line horizontally until it reaches the borders of the head.
		leftStep++;
	}
	NUI_Vector4 leftHipEnd=NuiTransformDepthImageToSkeleton(leftX,lHipY,*((USHORT*)(dImage->imageData+(int)lHipY*dImage->widthStep+(int)leftX*2)),mNui->m_DepthResolution);

	UCHAR* rEndPtr=(UCHAR*) (uImage->imageData+(int)rHipY*uImage->widthStep+(int)rHipX);
	rightX=rHipX;
	rightStep=0;
	while(*(++rEndPtr)>0 && rightX<(m_Width-1))
	{
		rightX++;	//Extend the line horizontally until it reaches the borders of the head.
		rightStep++;
	}
	NUI_Vector4 rightHipEnd=NuiTransformDepthImageToSkeleton(rightX,rHipY,*((USHORT*)(dImage->imageData+(int)rHipY*dImage->widthStep+(int)rightX*2)),mNui->m_DepthResolution);
	bodyMeasurements[HIP_WIDTH]=abs(rightHipEnd.x-leftHipEnd.x);

	//Torso Height Measurement

	torso=mNui->m_Points[NUI_SKELETON_POSITION_SPINE];
	bodyMeasurements[TORSO_HEIGHT]=abs(torso.y-lowPointY);


}

bool processFrame(NUI_Controller* mNui)
{
	optimizeDepthMap();
	getSphereSizes(mNui);
	measureBody(mNui);
	estimateParameters();
	return true;
}
bool addFrame(NUI_Controller* mNui) //Data Collection for temporal Temporal Optimization
{
	if (processedFrameCount>29)
		return true;
	else if (processedFrameCount==29)
	{
		for (int i=0;i<29;i++) //last frame data is already in the actual registers
		{
			for (int j=0;j<16;j++)
				sphereRadii[j]+=radiiBuffer[i][j];
			estimatedShoulderWidth+=bodySizeBuffer[i][0];
			estimatedTorsoHeight+=bodySizeBuffer[i][1];
			estimatedBodyHeight+=bodySizeBuffer[i][2];
		}
		for (int i=0;i<16;i++)
				sphereRadii[i]/=30;
		for (int i=10;i<12;i++)		//Correct Torso, as they are measured by the same bone, and can cause errors.
				sphereRadii[i]*=1.4;
		for (int i=8;i<10;i++)		//Correct Torso, as they are measured by the same bone, and can cause errors.
				sphereRadii[i]*=1.2;
		for (int i=0;i<4;i++)		//Correct Torso, as they are measured by the same bone, and can cause errors.
				sphereRadii[i]*=0.75;
		estimatedShoulderWidth/=30;
		estimatedTorsoHeight/=30;
		estimatedBodyHeight/=30;
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
		processedFrameCount++;
		return true;
	}
	else
	{
		if (processFrame(mNui))
		{	
			radiiBuffer[processedFrameCount]=new float[16];
			memcpy(radiiBuffer[processedFrameCount],sphereRadii,16*sizeof(float));
			bodySizeBuffer[processedFrameCount]=new float[3];
			bodySizeBuffer[processedFrameCount][0]=estimatedShoulderWidth;
			bodySizeBuffer[processedFrameCount][1]=estimatedTorsoHeight;
			bodySizeBuffer[processedFrameCount][2]=estimatedBodyHeight;
			processedFrameCount++;
		}
		return false;
	}
}
#endif 


