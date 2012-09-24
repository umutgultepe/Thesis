#include "StdAfx.h"
#include "KinectController.h"
#include "InturlamDressingRoom.h"
#define USER_TIMEOUT 1.5
#define handPositionBufferLength 20
Ogre::OverlayElement* gRegionGuide;
//#define USE_CALIBRATION_FILE 0
XnPoint3D gHeadPosition={-1,-1,-1};



	bool KinectController::checkUserCOM(XnUserID user)
	{
		XnPoint3D COM;
		m_UserGenerator.GetCoM(user,COM);
		if (COM.X<X_LIMIT && COM.X>-X_LIMIT)
			if (COM.Z<Z_BACK_LIMIT && COM.Z>Z_FORWARD_LIMIT)
				return true;
		return false;
	}

	bool KinectController::checkOuterLimits(XnUserID user)
	{
		XnPoint3D COM;
		m_UserGenerator.GetCoM(user,COM);
		if (COM.X<X_OUTER_LIMIT && COM.X>-X_OUTER_LIMIT)
			if (COM.Z<Z_OUTER_BACK_LIMIT && COM.Z>Z_OUTER_FORWARD_LIMIT)
				return true;
		return false;
	}

	void XN_CALLBACK_TYPE KinectController::NewUser(xn::UserGenerator& generator, const XnUserID nUserId, void* pCookie)
	{
		// start looking for calibration pose for new users
		//generator.GetPoseDetectionCap().StartPoseDetection("Psi", nUserId);
		KinectController* This = (KinectController*)pCookie;
		 This->candidateUsers.push_back(nUserId);
	}

	void XN_CALLBACK_TYPE KinectController::UserExit(xn::UserGenerator& generator, const XnUserID nUserId, void* pCookie)
	{
		// start looking for calibration pose for new users
		//generator.GetPoseDetectionCap().StartPoseDetection("Psi", nUserId);
		KinectController* This = (KinectController*)pCookie;
		int index=0;
		for (std::vector<XnUserID>::iterator i=This->candidateUsers.begin();i!=This->candidateUsers.end();i++)
		{
			if (*i._Ptr==nUserId)
			{
				if (*i._Ptr==This->interestedUser)
					This->interestedUser=0;
				This->candidateUsers.erase(i);
				break;
			}
			index++;
		}

	}

	void XN_CALLBACK_TYPE KinectController::UserReEnter(xn::UserGenerator& generator, const XnUserID nUserId, void* pCookie)
	{
		// start looking for calibration pose for new users
		//generator.GetPoseDetectionCap().StartPoseDetection("Psi", nUserId);
		KinectController* This = (KinectController*)pCookie;
		 This->candidateUsers.push_back(nUserId);
	}

	void XN_CALLBACK_TYPE KinectController::LostUser(xn::UserGenerator& generator, const XnUserID nUserId, void* pCookie)
	{
		KinectController* This = (KinectController*)pCookie;
		int index=0;
		for (std::vector<XnUserID>::iterator i=This->candidateUsers.begin();i!=This->candidateUsers.end();i++)
		{
			if (*i._Ptr==nUserId)
			{

				This->candidateUsers.erase(i);
				break;
			}
			index++;
		}

		if(This->activeUser == nUserId )
		{
			This->changeCandidateID(0);
			This->m_pEndPoseDetector->SetUserId(0);
			This->m_pStartPoseDetector->Reset();
		}
	}

	void XN_CALLBACK_TYPE KinectController::CalibrationStart(xn::SkeletonCapability& skeleton, const XnUserID nUserId, void* pCookie)
	{
	}

	void XN_CALLBACK_TYPE KinectController::CalibrationEnd(xn::SkeletonCapability& skeleton, const XnUserID nUserId, XnBool bSuccess, void* pCookie)
	{
		KinectController* This = (KinectController*)pCookie;

		if (bSuccess)
		{
			// start tracking
			skeleton.StartTracking(nUserId);
			
			This->m_pStartPoseDetector->SetStartPoseState(true);
			This->m_pEndPoseDetector->SetUserId(nUserId);
			This->lookingForPose=false;
			This->trackingUser=true;
//			This->m_pQuitFlow->SetActive(NULL);

			This->suppress = true;

			This->m_HandTracker->initROI(This->activeUser);
		}
		else
		{
			This->m_UserGenerator.GetSkeletonCap().AbortCalibration(This->activeUser);
			This->m_UserGenerator.GetPoseDetectionCap().StopPoseDetection(This->activeUser);
			This->m_UserGenerator.GetPoseDetectionCap().StartPoseDetection("Psi",This->activeUser);
		}
	}

	void XN_CALLBACK_TYPE KinectController::PoseDetected(xn::PoseDetectionCapability& poseDetection, const XnChar* strPose, XnUserID nId, void* pCookie)
	{
		KinectController* This = (KinectController*)pCookie;

		// If we dont have an active candidate
		if(This->activeUser != 0)
		{
			//This->changeCandidateID(nId);
			This->m_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
			This->m_pStartPoseDetector->SetStartPoseState(true);
		}
	}

	void XN_CALLBACK_TYPE KinectController::PoseLost(xn::PoseDetectionCapability& poseDetection, const XnChar* strPose, XnUserID nId, void* pCookie)
	{
		KinectController* This = (KinectController*)pCookie;
		This->m_pStartPoseDetector->Reset();
	}

	KinectController::KinectController(bool useHandTracker):
	mLeftHandBuffer(handPositionBufferLength),
	mRightHandBuffer(handPositionBufferLength)
	{
		XnStatus rc;
		rc = initPrimeSensor(useHandTracker);
		activeUser=0;
		interestedUser=0;
		if (XN_STATUS_OK != rc)
		{
			ErrorDialog dlg;
			dlg.display("Error initing sensor");
			exit(0);
		}
		m_HandTracker=new HandTracker(&m_DepthGenerator,&m_UserGenerator,m_pStartPoseDetector,m_pEndPoseDetector,m_pDrawer);
		
	}

	KinectController::~KinectController()
	{
		m_Context.StopGeneratingAll();

		if (NULL != m_hPoseCallbacks)
		{
			m_UserGenerator.GetPoseDetectionCap().UnregisterFromPoseCallbacks(m_hPoseCallbacks);
			m_hPoseCallbacks = NULL;
		}
		if (NULL != m_hUserCallbacks)
		{
			m_UserGenerator.UnregisterUserCallbacks(m_hUserCallbacks);
			m_hUserCallbacks = NULL;
		}
		if (NULL != m_hUserExitCallBack)
		{
			m_UserGenerator.UnregisterFromUserExit(m_hUserExitCallBack);
			m_hUserExitCallBack = NULL;
		}
		if (NULL != m_hUserReEnterCallBack)
		{
			m_UserGenerator.UnregisterFromUserExit(m_hUserReEnterCallBack);
			m_hUserExitCallBack = NULL;
		}
		if (NULL != m_hCalibrationCallbacks)
		{
			m_UserGenerator.GetSkeletonCap().UnregisterCalibrationCallbacks(m_hCalibrationCallbacks);
			m_hCalibrationCallbacks = NULL;
		}

		m_Context.Shutdown();

		mSceneMgr->destroyAllCameras();
		mSceneMgr->destroyAllEntities();
		mSceneMgr->getRootSceneNode()->removeAndDestroyAllChildren();
		mRoot->destroySceneManager(mSceneMgr);
	}

	void KinectController::changeCandidateID(XnUserID ID)
	{
		activeUser=ID;
		if 	(m_HandTracker)
			m_HandTracker->setUserID(ID);
	}

	XnStatus KinectController::initPrimeSensor(bool useHandTracker)
	{
		m_front=true;

		m_hUserCallbacks = NULL;
		m_hPoseCallbacks = NULL;
		m_hCalibrationCallbacks = NULL;
		m_hUserReEnterCallBack=NULL;
		m_hUserExitCallBack=NULL;
		// Init OpenNI from XML
		XnStatus rc = XN_STATUS_OK;
		rc = m_Context.InitFromXmlFile("..\\..\\media\\openni.xml");
		CHECK_RC(rc, "InitFromXml");

		// Make sure we have all OpenNI nodes we will be needing for this sample
		xn::NodeInfoList nodes;

#if SHOW_DEPTH
		VALIDATE_GENERATOR(XN_NODE_TYPE_DEPTH, "Depth", m_DepthGenerator);
#endif 
		VALIDATE_GENERATOR(XN_NODE_TYPE_USER, "User", m_UserGenerator);
		//if (useHandTracker)
		//{
		VALIDATE_GENERATOR(XN_NODE_TYPE_HANDS, "Gesture", m_GestureGenerator);
		VALIDATE_GENERATOR(XN_NODE_TYPE_HANDS, "Hands", m_HandsGenerator);
		//}
		VALIDATE_GENERATOR(XN_NODE_TYPE_IMAGE, "Image", m_ImageGenerator);
		
		// Init NITE Controls (UI stuff)
		m_pSessionManager = new XnVSessionManager;
		rc = m_pSessionManager->Initialize(&m_Context, "Click,Wave", "RaiseHand");
		m_pSessionManager->SetQuickRefocusTimeout(0);

		if (useHandTracker)
		{
		m_pDrawer = new XnVPointDrawer(20, m_DepthGenerator); 
		m_gFlower = new XnVFlowRouter;
		m_gFlower->SetActive(m_pDrawer);
		m_pSessionManager->AddListener(m_gFlower);
		m_HandsGenerator.SetSmoothing(1);
		}
		else
		{
			m_pDrawer=NULL;
			m_gFlower=NULL;
		}
		// Create quit slider & add to session manager
		mLeftHandState=HAND_UNKNOWN;
		mRightHandState=HAND_UNKNOWN;
		lookingForPose=false;
		trackingUser=false;
		suppress = false;
		elapsedTime=0;
		deleteUser=false;
		// Init OpenNI nodes
	
		m_UserGenerator.RegisterUserCallbacks(KinectController::NewUser, KinectController::LostUser, this, m_hUserCallbacks);
		//m_UserGenerator.RegisterToUserExit(KinectController::UserExit,this,m_hUserExitCallBack);
		//m_UserGenerator.RegisterToUserReEnter(KinectController::UserReEnter,this,m_hUserReEnterCallBack);
		m_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(KinectController::PoseDetected, KinectController::PoseLost, this, m_hPoseCallbacks);
#if SHOW_DEPTH
		m_DepthGenerator.GetMirrorCap().SetMirror(m_front);
#endif
		m_ImageGenerator.GetMirrorCap().SetMirror(m_front);

		// Skeleton stuff
		m_SmoothingFactor = 0.6;
		m_SmoothingDelta = 0;
		xn::SkeletonCapability skel = m_UserGenerator.GetSkeletonCap();
		skel.SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
		skel.SetSmoothing(m_SmoothingFactor);
		m_UserGenerator.GetMirrorCap().SetMirror(m_front);
		skel.RegisterCalibrationCallbacks(KinectController::CalibrationStart, KinectController::CalibrationEnd, this, m_hCalibrationCallbacks);
		
		// Make sure OpenNI nodes start generating
		rc = m_Context.StartGeneratingAll();
		CHECK_RC(rc, "StartGenerating");

		m_pStartPoseDetector = new StartPoseDetector(3.0);
		m_pEndPoseDetector = new EndPoseDetector(m_UserGenerator, 2.0);
		m_pEndPoseDetector->SetUserId(0);

		return XN_STATUS_OK;
	}

	Image* KinectController::takeSnapshot()
	{
		xn::ImageMetaData imd;
		m_ImageGenerator.GetMetaData(imd);
	

		unsigned char* imgPtr=(unsigned char*)imd.Data();

		TexturePtr texture = TextureManager::getSingleton().getByName("MyRGBTexture");
		// Get the pixel buffer
		HardwarePixelBufferSharedPtr pixelBuffer = texture->getBuffer();

		// Lock the pixel buffer and get a pixel box
		pixelBuffer->lock(HardwareBuffer::HBL_DISCARD); 
		const PixelBox& pixelBox = pixelBuffer->getCurrentLock();

		unsigned char* pDest = static_cast<unsigned char*>(pixelBox.data);

	
		//Drawing RGB
		for (size_t j = 0; j < RGBY; j++)
		{
			pDest = static_cast<unsigned char*>(pixelBox.data) + j*pixelBox.rowPitch*4;
			for(size_t i = 0; i < RGBX; i++)
			{
				//for(int i=0;i<3;i++)
				*pDest=*(imgPtr+2);
				*(pDest+1)=*(imgPtr+1);
				*(pDest+2)=*imgPtr;
				pDest+=4;
				imgPtr+=3;
				
			}
		}
		pDest = static_cast<unsigned char*>(pixelBox.data);
		Ogre::Image* img=new Ogre::Image;
		img->loadDynamicImage(pDest,texture->getWidth(),texture->getHeight(),texture->getFormat());
		pixelBuffer->unlock();

		return img;
		
	}

	bool KinectController::updateUserNode(XnUserID ID)
	{
		XnPoint3D COM;
		m_UserGenerator.GetCoM(ID,COM);
		if (COM.Z<=0)
			return false;
		else
			mUserNode->setPosition(-COM.X/10,175-COM.Z/10,-10);
		return true;
	}

	bool KinectController::addTime(Real deltaTime)
	{

		try
		{
		m_Context.WaitNoneUpdateAll();
		m_pSessionManager->Update(&m_Context);
		}
		catch (std::exception e)
		{
			const char* c=e.what();
		}
		
		if (m_DepthGenerator.IsDataNew())
		{

				if (candidateUsers.size()==0)//THis means there are no users seen in the scene whatsoever
				{
					mRegionGuide->hide();//else hide the guide
					
				}
				else if (activeUser==0)	//If there is no active user, check the locations of the users
				{

					if (updateUserNode(candidateUsers.at(0)))		
					{
						mRegionGuide->show();
					}

					for (unsigned int i=0;i<candidateUsers.size();i++)
					{

						if (checkUserCOM(candidateUsers.at(i)))//If there is a user within the active region, activate him
						{
							changeCandidateID(candidateUsers.at(i));
							break;
						}
						
					}
				
				}
				if (activeUser!=0)//If there is an active user, which is using the AdVivum
				{
					if (checkUserCOM(activeUser))//If the user is still in the region
					{
						mRegionGuide->hide();
						if (deleteUser)
						{
							deleteUser=false;
							elapsedTime=0;
						}
					
						if (!m_UserGenerator.GetSkeletonCap().IsCalibrated(activeUser)) //If the user is not calibrated,do calibration 
						{

						#ifdef USE_CALIBRATION_FILE
						
						//m_HandTracker->initROI(This->activeUser);
							mStatistics->newUser();
							if (gManager->getActiveStateStack()->back()->m_Index!=MenuStateId)
								gManager->popAllAndPushAppState(gManager->findByName(StateNames.at(MenuStateId)));
							m_UserGenerator.GetSkeletonCap().LoadCalibrationDataFromFile(activeUser,"calib.inturlam");
							m_UserGenerator.GetSkeletonCap().StartTracking(activeUser);
							lookingForPose=false;
							trackingUser=true;
							suppress = true;
							m_HandTracker->initROI(activeUser);
							OgreFramework::getSingletonPtr()->m_pLog->logMessage("Loaded the calibration file");



						#else
							mRegionGuide->show();
							updateUserNode(activeUser);
							if (!lookingForPose)
							{
								m_UserGenerator.GetPoseDetectionCap().StartPoseDetection("Psi",activeUser);
								lookingForPose=true;
							}
						#endif
						}
						else if(!m_UserGenerator.GetSkeletonCap().IsTracking(activeUser))// If the user is calibrated, but not being tracked, start tracking
						{
	//						changeCandidateID(activeUser);
							m_UserGenerator.GetSkeletonCap().StartTracking(activeUser);
							trackingUser=true;
						}
					//	#ifndef USE_CALIBRATION_FILE
						else if (!m_HandTracker->isSetUp())
						{
							m_HandTracker->initROI(activeUser);
						}
					//	#endif
						else
							updateRelativeHandCoordinates();
					
					}
					else//If the active user stepped out of the active region
					{

						if (!checkOuterLimits(activeUser))//If the user stepped out too far
						{
							deleteUser=true;			//Immediately discard user
							elapsedTime=0;
						}
						if (elapsedTime>0)		
						{
							elapsedTime-=deltaTime;
							mRegionGuide->show();
							updateUserNode(candidateUsers.at(0));
							//Show return to zone guide, user is out of active region
						}
						else
						{
							if (deleteUser)
							{
								trackingUser=false;		//Lose the current user
								deleteUser=false;
								mRightHandState=HAND_UNKNOWN;
								mLeftHandState=HAND_UNKNOWN;
								elapsedTime=0;
								m_UserGenerator.GetSkeletonCap().Reset(activeUser);	
								mRegionGuide->hide();
								changeCandidateID(0);	//This clears the hand trackerd
							}
							else if (m_UserGenerator.GetSkeletonCap().IsTracking(activeUser))// If the user was being tracked
							{
								//Wait for three seconds to get back in the region
								//Then clear the user as the active user
								elapsedTime=USER_TIMEOUT;
								m_UserGenerator.GetSkeletonCap().StopTracking(activeUser);
								deleteUser=true;
							
							}
							#ifndef USE_CALIBRATION_FILE
							else if (lookingForPose)//If we were checking for pose, user was not calibrated
							{
								m_UserGenerator.GetPoseDetectionCap().StopPoseDetection(activeUser);//Stop Pose detection
								lookingForPose=false;
								changeCandidateID(0);	//This clears the hand tracker
							}
							#endif
						
						}
						//activeUser=0;		//This does not reset hand tracker
					}
				}
				

			if 	(m_HandTracker)
				m_HandTracker->UpdateDepthTexture(m_front,&mLeftHandState,&mRightHandState,&mLeftHandPosition,&mRighttHandPosition);
			return true;
			
		}
		else
			return false;
		//UpdateDepthTexture();

	}

	void KinectController::injectKeyDown(const OIS::KeyEvent& evt)
	{

		

		// keep track of the player's intended direction
		//Smoothing Factor.
		if(evt.key == OIS::KC_H)
		{
			m_SmoothingDelta = 1;
		}
		else if(evt.key == OIS::KC_N)
		{
			m_SmoothingDelta = -1;
		}

	}

	void KinectController::injectKeyUp(const OIS::KeyEvent& evt)
	{
		// keep track of the player's intended direction

		//Mirror.
		if(evt.key == OIS::KC_M)
		{
			m_front = !m_front;
#if SHOW_DEPTH
			m_DepthGenerator.GetMirrorCap().SetMirror(m_front);
#endif
		}

		if(evt.key == OIS::KC_H || evt.key == OIS::KC_N)
		{
			m_SmoothingDelta = 0;
		}

	}

	Ogre::Matrix3 KinectController::getRealOrientation(XnSkeletonJoint joint)
	{
		Ogre::Matrix3 pp;
	
		if (m_UserGenerator.GetSkeletonCap().IsTracking(activeUser) )
		{
			XnSkeletonJointOrientation realPos;
			m_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(activeUser,joint,realPos);
			for (int i=0;i<3;i++)
				for (int j=0;j<3;j++)
					pp[i][j]=realPos.orientation.elements[i+3*j];
			//pp.x=realPos.orientation;
			//pp.y=realPos.position.Y;
			//pp.z=realPos.position.Z;
		}
		return pp;

	}

	Ogre::Vector3 KinectController::getRealCoordinate(XnSkeletonJoint joint)
	{
		Ogre::Vector3 pp;
		pp.x=-1;
		pp.y=-1;
	
		if (m_UserGenerator.GetSkeletonCap().IsTracking(activeUser) )
		{
			XnSkeletonJointPosition realPos;
			m_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(activeUser,joint,realPos);
			pp.x=realPos.position.X;
			pp.y=realPos.position.Y;
			pp.z=realPos.position.Z;
		}
		return pp;

	}

	bool KinectController::isUserActive()
	{
		if (activeUser>0)
			return true;
		return false;
	}

	XnPoint3D KinectController::getRelativeCoordinate(XnSkeletonJoint joint)
	{
		XnPoint3D pp;
		pp.X=-1;
		pp.Y=-1;
	
		if (m_UserGenerator.GetSkeletonCap().IsTracking(activeUser) )
		{
			XnSkeletonJointPosition realPos;
			m_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(activeUser,joint,realPos);
			m_DepthGenerator.ConvertRealWorldToProjective(1,&realPos.position,&pp);
			pp.X=pp.X/m_Width;
			pp.Y=pp.Y/m_Height;
		}
		return pp;

	}
	
	XnPoint3D KinectController::getRelativeHandCoordinate(XnSkeletonJoint joint)
	{
		XnPoint3D tempPoint;
		if (joint==XN_SKEL_RIGHT_HAND)
			tempPoint= mRighttHandPosition;
		else 
			tempPoint= mLeftHandPosition;



		if(tempPoint.Y<0 )
			tempPoint.Y=0;
		else if (tempPoint.Y > 1 )
			tempPoint.Y=1;
		else if (!(tempPoint.Y>=0 && tempPoint.Y <=1))
			tempPoint.Y=0;

		if(tempPoint.X<0 )
			tempPoint.X=0;
		else if (tempPoint.X > 1 )
			tempPoint.X=1;
		else if (!(tempPoint.X>=0 && tempPoint.X <=1))
			tempPoint.X=0;

		return tempPoint;
	}
	
	void KinectController::updateRelativeHandCoordinates()
	{
		XnPoint3D ppLeft,ppRight;
		ppLeft.X=-1;
		ppLeft.Y=-1;
		ppRight.X=-1;
		ppRight.X=-1;
		float headToTorso=-1;
		if (m_UserGenerator.GetSkeletonCap().IsTracking(activeUser) )
		{
			XnSkeletonJoint joint=XN_SKEL_RIGHT_HAND;
			XnSkeletonJointPosition torsoPos,headPos,handPos;
			m_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(activeUser,XN_SKEL_HEAD,headPos);
			m_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(activeUser,XN_SKEL_TORSO,torsoPos);
			m_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(activeUser,joint,handPos);
			m_DepthGenerator.ConvertRealWorldToProjective(1,&handPos.position,&handPos.position);
			m_DepthGenerator.ConvertRealWorldToProjective(1,&torsoPos.position,&torsoPos.position);
			m_DepthGenerator.ConvertRealWorldToProjective(1,&headPos.position,&headPos.position);
			gHeadPosition=headPos.position;
			headToTorso=sqrt(pow(headPos.position.X-torsoPos.position.X,2)+pow(headPos.position.Y-torsoPos.position.Y,2));
			//float x=handPos.position.X-(torsoPos.position.X-headToTorso*0.4);
			//float y=handPos.position.Y-(headPos.position.Y-headToTorso*0.8);
			float x=handPos.position.X-(torsoPos.position.X);
			float y=handPos.position.Y-(headPos.position.Y-headToTorso*0.8);

			x=x/(1.5*headToTorso);
			y=y/(headToTorso*1.9);
			ppRight.X=x;
			ppRight.Y=y;
			
			joint=XN_SKEL_LEFT_HAND;
			m_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(activeUser,joint,handPos);
			m_DepthGenerator.ConvertRealWorldToProjective(1,&handPos.position,&handPos.position);
			x=handPos.position.X-(torsoPos.position.X-headToTorso*1.2);
			y=handPos.position.Y-(headPos.position.Y-headToTorso*0.8);
			x=x/(2.0*headToTorso);
			y=y/(headToTorso*1.9);
			ppLeft.X=x;
			ppLeft.Y=y;


		}
		//Right Hand Update
		mRightHandBuffer.erase(mRightHandBuffer.begin());
		mRightHandBuffer.push_back(ppRight);

		XnPoint3D tempPosition=mRighttHandPosition;

		mRighttHandPosition.X=0;
		mRighttHandPosition.Y=0;

		for (int i=0;i<handPositionBufferLength;i++)
		{
			mRighttHandPosition.X+=mRightHandBuffer.at(i).X;
			mRighttHandPosition.Y+=mRightHandBuffer.at(i).Y;
		}
		mRighttHandPosition.X/=handPositionBufferLength;
		mRighttHandPosition.Y/=handPositionBufferLength;
		//Speed update
		unsigned int tempTime=Ogre::Root::getSingleton().getTimer()->getMilliseconds();

		if (headToTorso>0)
		{
			
			mRightHandSpeed.X=(mRighttHandPosition.X-tempPosition.X)/(float)(tempTime-lastUpdateTime)*1000;
			mRightHandSpeed.Y=(mRighttHandPosition.Y-tempPosition.Y)/(float)(tempTime-lastUpdateTime)*1000;
		}


		//Left Hand Update
		mLeftHandBuffer.erase(mLeftHandBuffer.begin());
		mLeftHandBuffer.push_back(ppLeft);

		tempPosition=mLeftHandPosition;
		mLeftHandPosition.X=0;
		mLeftHandPosition.Y=0;

		for (int i=0;i<handPositionBufferLength;i++)
		{
			mLeftHandPosition.X+=mLeftHandBuffer.at(i).X;
			mLeftHandPosition.Y+=mLeftHandBuffer.at(i).Y;
		}
		mLeftHandPosition.X/=handPositionBufferLength;
		mLeftHandPosition.Y/=handPositionBufferLength;
		//Speed update
		if (headToTorso>0)
		{
			mLeftHandSpeed.X=(mLeftHandPosition.X-tempPosition.X)/(float)(tempTime-lastUpdateTime)*1000;
			mLeftHandSpeed.Y=(mLeftHandPosition.Y-tempPosition.Y)/(float)(tempTime-lastUpdateTime)*1000;
		}

		lastUpdateTime=tempTime;
	}
	
	XnVector3D KinectController::getHandSpeed(XnSkeletonJoint joint)
	{
		if (joint==XN_SKEL_LEFT_HAND)
		{
			return mLeftHandSpeed;
		}
		else if (joint==XN_SKEL_RIGHT_HAND)
		{
			return mRightHandSpeed;
		}
		else
			return XnVector3D();
	}

	void KinectController::createRTT(Ogre::Root* ogreRoot,OgreBites::SdkTrayManager* mTrayMgr)
	{
		mRoot=ogreRoot;
		Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create("RegionMat", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		  Ogre::TextureUnitState* tuisTexture = mat->getTechnique(0)->getPass(0)->createTextureUnitState("regionGuide.jpg");

		 Ogre::MeshManager::getSingleton().createPlane("RegionPlane", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::Plane(-Vector3::UNIT_Z, 0), 380, 350, 1, 1, true, 1, 1, 1, Ogre::Vector3::UNIT_Y);
		  Ogre::MeshManager::getSingleton().createPlane("UserPlane", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::Plane(-Vector3::UNIT_Z, 0), 60, 30, 1, 1, true, 1, 1, 1, Ogre::Vector3::UNIT_Y);

		mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);
		 Ogre::Entity*	mPlaneEnt = mSceneMgr->createEntity("RegionBG", "RegionPlane");

		 
		mPlaneEnt->setMaterialName("userBack");

		 Ogre::Camera* mCamera = mSceneMgr->createCamera("PlayerCam");

		// Position it at 500 in Z direction
		mCamera->setPosition(Ogre::Vector3(0,0,-50));
		// Look back along -Z
		mCamera->lookAt(Ogre::Vector3(0,0,0));
		mCamera->setNearClipDistance(5);
		mCamera->setProjectionType(PT_ORTHOGRAPHIC);
		mCamera->setOrthoWindow(380,350);
 
		

		Ogre::SceneNode* mPlaneNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		mPlaneNode->attachObject(mPlaneEnt);
		Ogre::TexturePtr rtt_texture = Ogre::TextureManager::getSingleton().createManual("RttTex",
																							Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, 
																							Ogre::TEX_TYPE_2D,
																							380, 350,
																							0, Ogre::PF_R8G8B8,
																							Ogre::TU_RENDERTARGET);
		Ogre::RenderTexture *renderTexture = rtt_texture->getBuffer()->getRenderTarget();
		renderTexture->addViewport(mCamera);
		renderTexture->getViewport(0)->setClearEveryFrame(true);
		renderTexture->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
		renderTexture->getViewport(0)->setOverlaysEnabled(false);
		//renderTexture->addListener(this);
		

		Ogre::MaterialPtr renderMaterial = Ogre::MaterialManager::getSingleton().create("RttMat", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		Ogre::Technique* matTechnique = renderMaterial->createTechnique();
		matTechnique->createPass();
		renderMaterial->getTechnique(0)->getPass(0)->setLightingEnabled(false);
		Ogre::TextureUnitState* texState=renderMaterial->getTechnique(0)->getPass(0)->createTextureUnitState("RttTex");
		renderMaterial->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBF_SOURCE_ALPHA,Ogre::SBF_ONE_MINUS_SOURCE_ALPHA);
		texState->setAlphaOperation(LBX_DOTPRODUCT, LBS_MANUAL,LBS_TEXTURE , 0.8);


		mRegionGuide = Ogre::OverlayManager::getSingleton().createOverlayElement("Panel","RegionPanel");
		mRegionGuide->setMaterialName("RttMat");
		mRegionGuide->setMetricsMode(Ogre::GMM_PIXELS);
		mRegionGuide->setWidth(380);
		mRegionGuide->setHeight(350);
		mRegionGuide->setHorizontalAlignment(GHA_LEFT);
		mRegionGuide->setVerticalAlignment(GVA_CENTER);
		mRegionGuide->setLeft(80);
		mRegionGuide->setTop(-175);
		
		
		

		mTrayMgr->getTraysLayer()->add2D((Ogre::OverlayContainer*)mRegionGuide);
		mRegionGuide->show();
	
		mUserEntity=mSceneMgr->createEntity("UserEntity","UserPlane");
		mUserEntity->setMaterialName("userMat");
		mUserNode=mSceneMgr->getRootSceneNode()->createChildSceneNode("UserNode",Ogre::Vector3(0,0,-10));
		mUserNode->attachObject(mUserEntity);
		gRegionGuide=mRegionGuide;
	}

	Handstate KinectController::getHandstate(bool leftOrRight)
	{
		if (leftOrRight)
			return mLeftHandState;
		else
			return mRightHandState;
	}