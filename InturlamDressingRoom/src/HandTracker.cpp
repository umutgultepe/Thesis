#include "StdAfx.h"
#include "HandTracker.h"
#include "KinectController.h"
#ifndef SHOW_DEPTH
#define SHOW_DEPTH 1
#endif
#ifndef SHOW_BAR
#define SHOW_BAR 0
#endif

float lastRatio=-1;
//static unsigned int g_UsersColors[] = {/*0x70707080*/0 ,0x80FF0000,0x80FF4500,0x80FF1493,0x8000ff00, 0x8000ced1,0x80ffd700};
std::vector<float> leftBuffer(averaging_size);
std::vector<float> rightBuffer(averaging_size);
#define GetColorForUser(i) g_UsersColors[(i)%(sizeof(g_UsersColors)/sizeof(unsigned int))]
//Hand parameters
#define HAND_TRESHOLD_UP 2.8
#define HAND_TRESHOLD_DOWN 4.0
#define ZInterval 70
#define Hand_Torso_Distance 250

HandTracker::HandTracker(xn::DepthGenerator* depth,xn::UserGenerator* user,StartPoseDetector * startPoseDetector,EndPoseDetector* endPoseDetector,XnVPointDrawer* PDrawer)
{
	m_DepthGenerator=depth;
	m_UserGenerator=user;
	m_pStartPoseDetector=startPoseDetector;
	m_pEndPoseDetector=endPoseDetector;
	m_pDrawer=PDrawer;
	userID=0;
	setUp=false;
}

bool HandTracker::isSetUp()
{
	return setUp;
}

void HandTracker::setUserID(XnUserID user)
{
	if (userID!=user)
	{
		userID=user;
		setUp=false;
		//cvDestroyWindow("Image:");
	}
	
}

HandTracker::~HandTracker(void)
{
}

void HandTracker::initROI(XnUserID user)
{
	setUp=false;
	if (m_UserGenerator->GetSkeletonCap().IsTracking(userID))
	{
		
		xn::SceneMetaData smd;
		XnStatus tStatus=m_UserGenerator->GetUserPixels(0, smd);
		if (tStatus==XN_STATUS_OK)
		{
			userID=user;
			XnSkeletonJointPosition headPosition,headProjective,neckPosition,neckProjective;
			m_UserGenerator->GetSkeletonCap().GetSkeletonJointPosition(user,XN_SKEL_HEAD,headPosition);
			m_DepthGenerator->ConvertRealWorldToProjective(1,&headPosition.position,&headProjective.position);
			m_UserGenerator->GetSkeletonCap().GetSkeletonJointPosition(user,XN_SKEL_NECK,neckPosition);
			m_DepthGenerator->ConvertRealWorldToProjective(1,&neckPosition.position,&neckProjective.position);


		
			const XnLabel* pUsersLBLs = smd.Data();
			int leftPixel=0,
				rightPixel=0,
				topPixel=0,
				downPixel=neckProjective.position.Y-headProjective.position.Y;

			int headYPos=headProjective.position.Y;
			int headXPos=headProjective.position.X;
			//cvNamedWindow("Image:", CV_WINDOW_AUTOSIZE);

			while(user == pUsersLBLs[headYPos*m_Width + headXPos + (rightPixel++)])
			{
				if (headXPos + rightPixel>=m_Width)
					return;
			}
			while(user == pUsersLBLs[headYPos*m_Width + headXPos - (leftPixel++)])
			{
				if (headXPos - leftPixel<0)
					return;
			}
			while(user == pUsersLBLs[(headYPos-(topPixel++))*m_Width + headXPos])
			{
				if (headYPos-topPixel<0)
					return;
			}
		
			ROIWidth=(rightPixel+leftPixel)*1.2;
			ROIHeight=topPixel+downPixel*1.2;


			if (ROIWidth<10)
				return;
			if (ROIHeight<10)
				return;
			startDistance=headPosition.position.Z;
			setUp=true;
		}
	}
}

float processHandImage(IplImage* img)
{
	CvMemStorage* m_Storage=cvCreateMemStorage(0);
	CvMemStorage* m_DefectStorage=cvCreateMemStorage(0);
	CvSeq* Contours=0;
	CvSeq* ConvexHull=0;
	CvConvexityDefect* defectArray; 
	CvSeq* defects;  

	cvFindContours(img,m_Storage,&Contours);
	cvZero(img);

	if(Contours!=0)
	{
		ConvexHull=cvConvexHull2(Contours,0, CV_COUNTER_CLOCKWISE,     0); 
		defects = cvConvexityDefects( Contours,  
								ConvexHull,  
								m_DefectStorage);  

		int nomdef = defects->total; // defect amount                
		if (nomdef>1)
		{
	           
					// Alloc memory for defect set.     
			 defectArray = (CvConvexityDefect*)malloc(sizeof(CvConvexityDefect)*nomdef);  
              
			// Get defect set.      
			cvCvtSeqToArray(defects,defectArray, CV_WHOLE_SEQ);  
			float totalDepth=0;
			for (int i=0;i<nomdef;i++)
			{
				totalDepth+=defectArray[i].depth;
			}
			lastRatio=totalDepth/nomdef;
			free(defectArray);

		}
		if(ConvexHull!=0)
			cvClearSeq(ConvexHull);
		if(defects!=0)
			cvClearSeq(defects);
		cvClearSeq(Contours);
		
	}
	//cvClearMemStorage(m_Storage);
	//cvClearMemStorage(m_DefectStorage);
	cvReleaseMemStorage(&m_Storage);
	cvReleaseMemStorage(&m_DefectStorage);

	return lastRatio;
}

void HandTracker::UpdateDepthTexture(bool m_front,Handstate* leftHandState,Handstate* rightHandState,XnPoint3D* leftHandRelativePosition,XnPoint3D* rightHandRelativePosition)
	{
		TexturePtr texture = TextureManager::getSingleton().getByName("MyDepthTexture");
		// Get the pixel buffer
		HardwarePixelBufferSharedPtr pixelBuffer = texture->getBuffer();

		// Lock the pixel buffer and get a pixel box
		pixelBuffer->lock(HardwareBuffer::HBL_DISCARD); 
		const PixelBox& pixelBox = pixelBuffer->getCurrentLock();

		unsigned char* pDest = static_cast<unsigned char*>(pixelBox.data);

	
		// Get label map 
		xn::SceneMetaData smd;
		xn::DepthMetaData dmd;

		XnStatus tStatus=m_UserGenerator->GetUserPixels(0, smd);
		if (tStatus==XN_STATUS_OK)
		{
		m_DepthGenerator->GetMetaData(dmd);

		const XnLabel* pUsersLBLs = smd.Data();
		const XnDepthPixel* pDepth = dmd.Data();

		Image* img=0;
		ColourValue cv;
		int userPix=0;
		int outPix=0;


		//Drawing DepthMap
		for (size_t j = 0; j < m_Height; j++)
		{
			pDest = static_cast<unsigned char*>(pixelBox.data) + j*pixelBox.rowPitch*4;
			for(size_t i = 0; i < m_Width; i++)
			{		
				uint fixed_i = i;// fix i if we are mirrored
				if(!m_front)
					fixed_i = m_Width - i;				
				//unsigned int color = GetColorForUser(pUsersLBLs[j*m_Width + fixed_i]);// determine color
				unsigned int color = 0;// determine color
				if (userID != 0)// if we have a candidate, filter out the rest
				{
					if  (userID == pUsersLBLs[j*m_Width + fixed_i])
					{
						color = GetColorForUser(1);
						if( j > m_Height*(1 - m_pStartPoseDetector->GetDetectionPercent()))
							color |= 0xFF070707;
						if( j < m_Height*(m_pEndPoseDetector->GetDetectionPercent()))
							color &= 0x20F0F0F0;

						if (img)
						{
							userPix++;
							cv=img->getColourAt(i,j,0);
							if (cv.r > 0.5)
								outPix++;
						}
					}
					//else
					//	color = 0;

				}		
				*((unsigned int*)pDest) = color;	// write to output buffer
				pDest+=4;
			}
		}
		//Hand Tracking
		XnSkeletonJointPosition handProjective,handPosition;
		int centerXPos,centerYPos,ROIFixedWidth,ROIFixedHeight,ROIYNegative,ROIYPositive,
			pix_size,row_w,empty_byte;
		float handZPosition;
		IplImage* handImage=0;
		unsigned char* imgPtr=0;
		XnBool isTracking;

		
		//LeftHand
		if (isTracking=m_UserGenerator->GetSkeletonCap().IsTracking(userID) && setUp)
		{
			m_UserGenerator->GetSkeletonCap().GetSkeletonJointPosition(userID,XN_SKEL_LEFT_HAND,handPosition);
			m_DepthGenerator->ConvertRealWorldToProjective(1,&handPosition.position,&handProjective.position);

			centerXPos=handProjective.position.X;
			centerYPos=handProjective.position.Y;
			handZPosition=handProjective.position.Z;

			ROIFixedWidth=ROIWidth*(startDistance/handZPosition)/2;
			ROIFixedHeight=ROIHeight*(startDistance/handZPosition)/2;
			ROIYNegative=ROIFixedHeight*3/5;
			ROIYPositive=2*ROIFixedHeight-ROIYNegative;
			if (ROIYNegative>0 && ROIYPositive>0 && ROIFixedWidth>0)
			{
				handImage=cvCreateImage(cvSize(ROIFixedWidth*2-1,ROIFixedHeight*2-1),IPL_DEPTH_8U,1);
				imgPtr=(unsigned char*)handImage->imageData;
				pix_size = 1;
				row_w=handImage->width*pix_size;
				empty_byte=handImage->widthStep-row_w;
			}
			else
				setUp=false;
		}

		float ratio=0;
		float avgDepth=-1;
		float result=-1;
		*leftHandState=HAND_UNKNOWN;
		*rightHandState=HAND_UNKNOWN;
		if (isTracking && setUp)
		{
			unsigned int lowestX=740;
			unsigned int highestX=0;
			const XnLabel* userPtr = smd.Data();
			const XnDepthPixel* depthPtr = dmd.Data();
			for (int j = centerYPos-ROIYNegative+1; j < centerYPos+ROIYPositive; j++)
			{
				if (j<0 || j>=m_Height)
					continue;
				int yLevel=j*m_Width;
				for(int i = centerXPos-ROIFixedWidth+1; i < centerXPos+ROIFixedWidth; i++)
				{
					if (i<0 || i>=m_Width)
						continue;
					uint fixed_i = i;
					if(!m_front)
					{
						fixed_i = m_Width - i;
					}
					if  (userID == userPtr[yLevel + fixed_i] && (depthPtr[yLevel + fixed_i]>(handZPosition-ZInterval) && depthPtr[yLevel + fixed_i]<(handZPosition+ZInterval)))
					{
						(*imgPtr++)=0xF0;

						if(lowestX>fixed_i)
							lowestX=fixed_i;
						if(highestX<fixed_i)
							highestX=fixed_i;
					}
						else
						(*imgPtr++)=0x00;			
				}
				imgPtr+=empty_byte;
			}//Copy Image
			if (highestX!=0)
			ratio=(float)(highestX-lowestX)/(ROIFixedWidth*2);		
			//cvShowImage("Image:", handImage);
			//cvResizeWindow("Image:",ROIFixedWidth,ROIFixedHeight*2);
			avgDepth=processHandImage(handImage);
			float sum=/*ratio*avgDepth**/avgDepth;
			for (int i=0;i<averaging_size-1;i++)
			{
				leftBuffer.at(i)=leftBuffer.at(i+1);
				result+=leftBuffer.at(i);
			}
			leftBuffer.at(averaging_size-1)=sum;
			result+=sum;
			result/=averaging_size;
			XnSkeletonJointPosition torso,hand;
			m_UserGenerator->GetSkeletonCap().GetSkeletonJointPosition(userID,XN_SKEL_TORSO,torso);
			m_UserGenerator->GetSkeletonCap().GetSkeletonJointPosition(userID,XN_SKEL_LEFT_HAND,hand);
			if (result>HAND_TRESHOLD_UP && (torso.position.Z-hand.position.Z)>Hand_Torso_Distance)
				*leftHandState=HAND_OPEN;
			else if (result>0 && result<HAND_TRESHOLD_DOWN)
				*leftHandState=HAND_CLOSED;


			if (handImage)
				cvReleaseImage(&handImage);
		}

		//RightHand
		if (isTracking && setUp)
		{
			m_UserGenerator->GetSkeletonCap().GetSkeletonJointPosition(userID,XN_SKEL_RIGHT_HAND,handPosition);
			m_DepthGenerator->ConvertRealWorldToProjective(1,&handPosition.position,&handProjective.position);

			centerXPos=handProjective.position.X;
			centerYPos=handProjective.position.Y;
			handZPosition=handProjective.position.Z;

			ROIFixedWidth=ROIWidth*(startDistance/handZPosition)/2;
			ROIFixedHeight=ROIHeight*(startDistance/handZPosition)/2;
			ROIYNegative=ROIFixedHeight*3/5;
			ROIYPositive=2*ROIFixedHeight-ROIYNegative;
			if (ROIYNegative>0 && ROIYPositive>0 && ROIFixedWidth>0)
			{
				handImage=cvCreateImage(cvSize(ROIFixedWidth*2-1,ROIFixedHeight*2-1),IPL_DEPTH_8U,1);
				imgPtr=(unsigned char*)handImage->imageData;
				pix_size = 1;
				row_w=handImage->width*pix_size;
				empty_byte=handImage->widthStep-row_w;
			}
			else
				setUp=false;
		}
		

		ratio=0;
		avgDepth=-1;
		result=-1;
		
		if (isTracking && setUp)
		{
			unsigned int lowestX=740;
			unsigned int highestX=0;
			const XnLabel* userPtr = smd.Data();
			const XnDepthPixel* depthPtr = dmd.Data();
			for (int j = centerYPos-ROIYNegative+1; j < centerYPos+ROIYPositive; j++)
			{
				if (j<0 || j>=m_Height)
					continue;
				int yLevel=j*m_Width;
				for(int i = centerXPos-ROIFixedWidth+1; i < centerXPos+ROIFixedWidth; i++)
				{
					if (i<0 || i>=m_Width)
						continue;
					uint fixed_i = i;
					if(!m_front)
					{
						fixed_i = m_Width - i;
					}
					if  (userID == userPtr[yLevel + fixed_i] && (depthPtr[yLevel + fixed_i]>(handZPosition-ZInterval) && depthPtr[yLevel + fixed_i]<(handZPosition+ZInterval)))
					{
						(*imgPtr++)=0xF0;

						if(lowestX>fixed_i)
							lowestX=fixed_i;
						if(highestX<fixed_i)
							highestX=fixed_i;
					}
						else
						(*imgPtr++)=0x00;			
				}
				imgPtr+=empty_byte;
			}//Copy Image
			//cvShowImage("Image:", handImage);
			//cvResizeWindow("Image:",ROIFixedWidth,ROIFixedHeight*2);
			if (highestX!=0)
			ratio=(float)(highestX-lowestX)/(ROIFixedWidth*2);		
			avgDepth=processHandImage(handImage);
			float sum=avgDepth;
			if (sum<10)
			{
				for (int i=0;i<averaging_size-1;i++)
				{
					rightBuffer.at(i)=rightBuffer.at(i+1);
					result+=rightBuffer.at(i);
				}
				rightBuffer.at(averaging_size-1)=sum;
				result+=sum;
				result/=averaging_size;
				XnSkeletonJointPosition torso,hand;
				m_UserGenerator->GetSkeletonCap().GetSkeletonJointPosition(userID,XN_SKEL_TORSO,torso);
				m_UserGenerator->GetSkeletonCap().GetSkeletonJointPosition(userID,XN_SKEL_RIGHT_HAND,hand);
				if (result>HAND_TRESHOLD_UP && (torso.position.Z-hand.position.Z)>Hand_Torso_Distance)
					*rightHandState=HAND_OPEN;
				else if (result>0 && result<HAND_TRESHOLD_DOWN)
					*rightHandState=HAND_CLOSED;
			}


			if (handImage)
				cvReleaseImage(&handImage);
		}


		// Unlock the pixel buffer
		
		}
		pixelBuffer->unlock();
	}