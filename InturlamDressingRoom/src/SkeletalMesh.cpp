#include "StdAfx.h"
#include "ClothResizer.h"
#include "SkeletalMesh.h"


SkeletalMesh::SkeletalMesh(void)
{

	m_SmoothingFactor = 0.6;
	m_SmoothingDelta = 0;
	bNewUser=true;
	boneExists.resize(ACTIVE_BONE_COUNT);
	for (int i=0;i<ACTIVE_BONE_COUNT;i++)
		boneExists.at(i)=false;
}

SkeletalMesh::SkeletalMesh(KinectController* kinect)
{
	m_SmoothingFactor = 0.6;
	m_SmoothingDelta = 0;
	gKinect=kinect;
	bNewUser=true;
		boneExists.resize(ACTIVE_BONE_COUNT);
	for (int i=0;i<ACTIVE_BONE_COUNT;i++)
		boneExists.at(i)=false;
}


SkeletalMesh::~SkeletalMesh(void)
{
}

void SkeletalMesh::setupBone(const String& name,const Degree& yaw,const Degree& pitch,const Degree& roll)
{
	Ogre::Bone* bone = Mesh->getSkeleton()->getBone(name);
	bone->setManuallyControlled(true);
	bone->setInheritOrientation(false);
		
	bone->resetOrientation();
		
	bone->yaw(yaw);
	bone->pitch(pitch);
	bone->roll(roll);
	
	//Matrix3 mat = bone->getLocalAxes();
	bone->setInitialState();

}
void SkeletalMesh::setupBone(const String& name,const Ogre::Quaternion& q)
{
	Ogre::Bone* bone = Mesh->getSkeleton()->getBone(name);
	bone->setManuallyControlled(true);
	bone->setInheritOrientation(false);
		
	bone->resetOrientation();
	bone->setOrientation(q);
	
	bone->setInitialState();
}

void SkeletalMesh::setupBone(const String& name,const Ogre::Radian& angle, const Vector3 axis)
{
		
	Quaternion q;
	q.FromAngleAxis(angle,axis);	 
	setupBone(name, q);

}

void SkeletalMesh::resetBonesToInitialState()
{
	Ogre::Skeleton::BoneIterator bIter=Skeleton->getBoneIterator();
	while (bIter.hasMoreElements())
	{
		bIter.getNext()->resetToInitialState();
	}
		
}

void SkeletalMesh::setVisible(bool visibleOrNot)
{
	Mesh->setVisible(visibleOrNot);
}

Ogre::Entity* SkeletalMesh::loadMesh(Ogre::SceneManager* g_SceneManager,Ogre::SceneNode* parentNode,Ogre::String entityName,Ogre::String meshName)
{
	Mesh=g_SceneManager->createEntity(entityName,meshName);
	parentNode->attachObject(Mesh);
	Skeleton=Mesh->getSkeleton();
	Skeleton->setBlendMode(ANIMBLEND_CUMULATIVE);	

	Ogre::Skeleton::BoneIterator bIter=Skeleton->getBoneIterator();
	while (bIter.hasMoreElements())
	{
		Ogre::Bone* tBone=bIter.getNext();
		Ogre::String tName=tBone->getName();


		Ogre::Quaternion q = Quaternion::IDENTITY;
		Quaternion q2= Quaternion::IDENTITY;
		Vector3 xAxis,yAxis,zAxis;

		Ogre::String name=tBone->getName();
		if (tBone->getName()=="Humerus.L")
		{
			q.FromAngleAxis(Ogre::Degree(90),Vector3(0,0,-1));
			q.ToAxes(xAxis,yAxis,zAxis);
			q2.FromAngleAxis(Ogre::Degree(90),xAxis);
			boneExists.at(BONE_LEFT_HUMERUS)=true;
	
		}
		else if  (tBone->getName()=="Humerus.R")
		{
			q.FromAngleAxis(Ogre::Degree(90),Vector3(0,0,1));
			q.ToAxes(xAxis,yAxis,zAxis);
			q2.FromAngleAxis(Ogre::Degree(90),xAxis);
			boneExists.at(BONE_RIGHT_HUMERUS)=true;
	
		}
		else if  (tBone->getName()=="Ulna.L")
		{
			q.FromAngleAxis(Ogre::Degree(90),Vector3(0,0,-1));		 
			q2.FromAngleAxis(Ogre::Degree(45),Vector3(0,-1,0));			
			boneExists.at(BONE_LEFT_ULNA)=true;

		}
		else if  (tBone->getName()=="Ulna.R")
		{
			q.FromAngleAxis(Ogre::Degree(90),Vector3(0,0,1));		 
			q2.FromAngleAxis(Ogre::Degree(45),Vector3(0,-1,0));	
			q2=q2.Inverse();
			boneExists.at(BONE_RIGHT_ULNA)=true;
		}
		
		else if (tBone->getName()=="Chest" )
		{
			q.FromAngleAxis(Ogre::Degree(0),Vector3(0,1,0));
			boneExists.at(BONE_CHEST)=true;
			setupBone(tBone->getName(),q);
			continue;
		}
		else if (tBone->getName()=="Stomach")
		{
			q.FromAngleAxis(Ogre::Degree(0),Vector3(0,1,0));
			boneExists.at(BONE_STOMACH)=true;
			setupBone(tBone->getName(),q);
			continue;
		}
		else 
		{
			q.FromAngleAxis(Ogre::Degree(180),Vector3(1,0,0));	 	
			q2.FromAngleAxis(Ogre::Degree(0),Vector3(0,1,0));
			if (tBone->getName()=="Thigh.L")
						boneExists.at(BONE_LEFT_THIGH)=true;
			else if (tBone->getName()=="Thigh.R")
						boneExists.at(BONE_RIGHT_THIGH)=true;
			else if (tBone->getName()=="Calf.L")
						boneExists.at(BONE_LEFT_CALF)=true;
			else if (tBone->getName()=="Calf.R")
						boneExists.at(BONE_RIGHT_CALF)=true;
			else if (tBone->getName()=="Root")
			{
				boneExists.at(BONE_ROOT)=true;
				continue;
			}
			else
			{
				//bones.push_back(BONE_UNKNOWN);
				continue;
			}

		}
		setupBone(tBone->getName(),q*q2);

	}
	setupBone("Root",Degree(0),Degree(0),Degree(0));
	//setupBone("Waist",Degree(0),Degree(0),Degree(0));


	return Mesh;
}

void SkeletalMesh::transformBone(const Ogre::String& modelBoneName, XnSkeletonJoint skelJoint, bool flip)
{
	// Get the model skeleton bone info
	Ogre::Skeleton* skel = Mesh->getSkeleton();
	Ogre::Bone* bone = skel->getBone(modelBoneName);
	Ogre::Quaternion qI = bone->getInitialOrientation();
	Ogre::Quaternion newQ = Quaternion::IDENTITY;

	// Get the openNI bone info
	xn::SkeletonCapability pUserSkel = gKinect->m_UserGenerator.GetSkeletonCap();		
	XnSkeletonJointOrientation jointOri;
	pUserSkel.GetSkeletonJointOrientation(userID, skelJoint, jointOri);

	static float deg = 0;
	if(jointOri.fConfidence > 0 )
	{
		XnVector3D col1 = xnCreatePoint3D(jointOri.orientation.elements[0], jointOri.orientation.elements[3], jointOri.orientation.elements[6]);
		XnVector3D col2 = xnCreatePoint3D(jointOri.orientation.elements[1], jointOri.orientation.elements[4], jointOri.orientation.elements[7]);
		XnVector3D col3 = xnCreatePoint3D(jointOri.orientation.elements[2], jointOri.orientation.elements[5], jointOri.orientation.elements[8]);
	
		Ogre::Matrix3 matOri(jointOri.orientation.elements[0],-jointOri.orientation.elements[1],jointOri.orientation.elements[2],
							-jointOri.orientation.elements[3],jointOri.orientation.elements[4],-jointOri.orientation.elements[5],
							jointOri.orientation.elements[6],-jointOri.orientation.elements[7],jointOri.orientation.elements[8]);
		Quaternion q;
			
		newQ.FromRotationMatrix(matOri);
			
		bone->resetOrientation(); //in order for the conversion from world to local to work.
		newQ = bone->convertWorldToLocalOrientation(newQ);
			
		bone->setOrientation(newQ*qI);			
	} 
}

Quaternion convertNUItoOgre(NUI_SKELETON_BONE_ORIENTATION sj,bool flip=true)
{
	Quaternion q;
	q.x=sj.absoluteRotation.rotationQuaternion.x;
	q.y=sj.absoluteRotation.rotationQuaternion.y;
	q.z=sj.absoluteRotation.rotationQuaternion.z;
	q.w=sj.absoluteRotation.rotationQuaternion.w;
	if (flip)
	{
		Ogre::Matrix3 rotM;
		Radian yaw,pitch,roll;
		q.ToRotationMatrix(rotM);
		rotM.ToEulerAnglesZXY(yaw,pitch,roll);
		rotM.FromEulerAnglesZXY(-yaw,-pitch,roll);
		q.FromRotationMatrix(rotM);
	}
	return q;

}


void SkeletalMesh::transformBone(const Ogre::String& modelBoneName, NUI_SKELETON_BONE_ORIENTATION skelJoint, bool flip,Quaternion factor)
{
	// Get the model skeleton bone info
	Ogre::Skeleton* skel = Mesh->getSkeleton();
	Ogre::Bone* bone = skel->getBone(modelBoneName);
	Ogre::Quaternion newQ=convertNUItoOgre(skelJoint,flip);
	bone->setOrientation(newQ*factor);			
	
	

		//Ogre::Quaternion qI = bone->getInitialOrientation();
	// Get the Kinect SDK bone info
	//Quaternion flipZ=Quaternion::IDENTITY;
	//flipZ.FromAngleAxis(Ogre::Degree(180),Ogre::Vector3(0,-1,0));
	//static float deg = 0;
	//Ogre::Vector3 v;
	//v=rotM.GetColumn(0);
	//v.z=-v.z;
	//rotM.SetColumn(0,v);
	//
	//v=rotM.GetColumn(1);
	//v.z=-v.z;
	//rotM.SetColumn(1,v);
	//
	//v=rotM.GetColumn(2);
	//v.x=-v.x;
	//v.y=-v.y;
	//rotM.SetColumn(2,v);

	//XnVector3D col1 = xnCreatePoint3D(jointOri.orientation.elements[0], jointOri.orientation.elements[3], jointOri.orientation.elements[6]);
	//XnVector3D col2 = xnCreatePoint3D(jointOri.orientation.elements[1], jointOri.orientation.elements[4], jointOri.orientation.elements[7]);
	//XnVector3D col3 = xnCreatePoint3D(jointOri.orientation.elements[2], jointOri.orientation.elements[5], jointOri.orientation.elements[8]);
	//
	//Ogre::Matrix3 matOri(jointOri.orientation.elements[0],-jointOri.orientation.elements[1],jointOri.orientation.elements[2],
	//					-jointOri.orientation.elements[3],jointOri.orientation.elements[4],-jointOri.orientation.elements[5],
	//					jointOri.orientation.elements[6],-jointOri.orientation.elements[7],jointOri.orientation.elements[8]);
	//Quaternion q;
	//newQ.FromRotationMatrix(matOri);
			
	//bone->resetOrientation(); //in order for the conversion from world to local to work.
	//newQ = bone->convertWorldToLocalOrientation(newQ);
			
	//bone->setOrientation(newQ*qI);			
	
}


Ogre::Quaternion SkeletalMesh::getBoneOrientation(boneID bone)
{

	Ogre::Skeleton* skel = Mesh->getSkeleton();
	Ogre::Bone* targetBone = skel->getBone(boneStrings[bone]);
	return targetBone->getOrientation();
}

Ogre::Vector3 SkeletalMesh::updateMesh()
{
		mGoalDirection = Vector3::ZERO;   // we will calculate this
		xn::SkeletonCapability pUserSkel =gKinect->m_UserGenerator.GetSkeletonCap();

		//set smoothing according to the players request.
		if(m_SmoothingDelta!=0)
		{
			m_SmoothingFactor += 0.01 * m_SmoothingDelta;
			if(m_SmoothingFactor >= 1)
				m_SmoothingFactor = 0.99;
			if(m_SmoothingFactor <= 0)
				m_SmoothingFactor = 0.00;
			pUserSkel.SetSmoothing(m_SmoothingFactor);
			Ogre::DisplayString blah = "H/N ";
			blah.append(Ogre::StringConverter::toString((Real)m_SmoothingFactor));
			//m_help->setParamValue("Smoothing", blah);
		}
		
		Ogre::Skeleton* skel = Mesh->getSkeleton();
		Ogre::Bone* rootBone = skel->getBone("Root");

		XnSkeletonJointPosition torsoPos;
		Vector3 newPos2=Vector3(0,0,0);

		if(bNewUser)
		{			
			pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_TORSO, torsoPos);
			if(torsoPos.fConfidence > 0.5)
			{
				origTorsoPos.x = -torsoPos.position.X;
				origTorsoPos.y = torsoPos.position.Y;
				origTorsoPos.z = -torsoPos.position.Z;
				bNewUser = false;
			}
		}


		Ogre::Skeleton::BoneIterator bIter=Skeleton->getBoneIterator();
		int i=0;

		for (int i=0;i<ACTIVE_BONE_COUNT;i++)
		{
			if (boneExists.at(i))
			{
				if (i==BONE_CHEST || i==BONE_STOMACH)
					transformBone(boneStrings[i],jointIDs[i],true);
				else
					transformBone(boneStrings[i],jointIDs[i]);
			}
		}



		if(!bNewUser)
		{			 
			pUserSkel.GetSkeletonJointPosition(userID, XN_SKEL_TORSO, torsoPos);
			Vector3 newPos;
			newPos.x = -torsoPos.position.X;
			newPos.y = torsoPos.position.Y;
			newPos.z = -torsoPos.position.Z;

			newPos2 = (newPos - origTorsoPos)/100;

			newPos2.y -= 0.3;

			if (newPos2.y < 0)
			{
				newPos2.y /= 2.5;

				if (newPos2.y < -1.5)
				{
					newPos2.y = -1.5;
				}
			}


			if(torsoPos.fConfidence > 0.5)
			{
				rootBone->setPosition(newPos2);
			}

		}


		Ogre::Vector3 leftHip=gKinect->getRealCoordinate(XN_SKEL_LEFT_HIP);
		Ogre::Vector3 rightHip=gKinect->getRealCoordinate(XN_SKEL_RIGHT_HIP);
		Ogre::Vector3 hipVector=(rightHip-leftHip);
		Ogre::Vector3 initialVector=Ogre::Vector3(1,0,0);
		hipVector.y=0;
		//Ogre::Quaternion bodyRotation=hipVector.getRotationTo(initialVector);
		Ogre::Quaternion bodyRotation=initialVector.getRotationTo(hipVector);
		
		Ogre::Quaternion qI=rootBone->getInitialOrientation();
		rootBone->setOrientation(bodyRotation*qI);

	
		return newPos2;
	
}

Ogre::Vector3 SkeletalMesh::updateMesh(NUI_Controller* nui)
{
	mGoalDirection = Vector3::ZERO;   // we will calculate this

	Ogre::Skeleton* skel = Mesh->getSkeleton();
	Ogre::Bone* rootBone = skel->getBone("Root");

	NUI_Vector4 torsoPos;
	Vector3 newPos2=Vector3(0,0,0);

	if(bNewUser)
	{			
		torsoPos=nui->m_Points[NUI_SKELETON_POSITION_HIP_CENTER];
		origTorsoPos.x = -torsoPos.x*1000;
		origTorsoPos.y = torsoPos.y*1000;
		origTorsoPos.z = -torsoPos.z*1000;
		bNewUser = false;
	}


	Ogre::Skeleton::BoneIterator bIter=Skeleton->getBoneIterator();
	int i=0;

	for (int i=0;i<ACTIVE_BONE_COUNT;i++)
	{
		if (boneExists.at(i))
		{
			if (i==BONE_CHEST)
				transformBone(boneStrings[i],nui->m_Orientations[nuiIDs[i]],true,Ogre::Quaternion(Ogre::Degree(180),Ogre::Vector3(0,1,0)));
			else if (i==BONE_STOMACH)
				transformBone(boneStrings[i],nui->m_Orientations[nuiIDs[i]],true,Ogre::Quaternion(Ogre::Degree(-180),Ogre::Vector3(0,1,0)));
			else if (i!=BONE_ROOT)
				transformBone(boneStrings[i],nui->m_Orientations[nuiIDs[i]]);
		}
	}



	if(!bNewUser)
	{			 
		torsoPos=nui->m_Points[NUI_SKELETON_POSITION_HIP_CENTER];
		Vector3 newPos;
		newPos.x = -torsoPos.x*1000;
		newPos.y = torsoPos.y*1000;
		newPos.z = -torsoPos.z*1000;

		newPos2 = (newPos - origTorsoPos)/100;

		newPos2.y -= 0.3;

		if (newPos2.y < 0)
		{
			newPos2.y /= 2.5;

			if (newPos2.y < -1.5)
			{
				newPos2.y = -1.5;
			}
		}

		rootBone->setPosition(newPos2);
	}


	//Ogre::Vector3 leftHip=gKinect->getRealCoordinate(XN_SKEL_LEFT_HIP);
	//Ogre::Vector3 rightHip=gKinect->getRealCoordinate(XN_SKEL_RIGHT_HIP);
	//Ogre::Vector3 hipVector=(rightHip-leftHip);
	//Ogre::Vector3 initialVector=Ogre::Vector3(1,0,0);
	//hipVector.y=0;

	////Ogre::Quaternion bodyRotation=hipVector.getRotationTo(initialVector);
	//Ogre::Quaternion qI=rootBone->getInitialOrientation();
	
	
	NUI_SKELETON_BONE_ORIENTATION hip=nui->m_Orientations[NUI_SKELETON_POSITION_HIP_CENTER];
	Ogre::Quaternion q=convertNUItoOgre(hip);
	Ogre::Quaternion q2(Ogre::Degree(180),Ogre::Vector3(0,1,0));
	Ogre::Quaternion rootOrientation=q*q2;
	Ogre::Matrix3 tMat;
	Radian yaw,pitch,roll;
	rootOrientation.ToRotationMatrix(tMat);
	tMat.ToEulerAnglesZXY(yaw,pitch,roll);
	tMat.FromEulerAnglesZXY(Radian(0),Radian(0),roll);
	rootOrientation.FromRotationMatrix(tMat);

	/*Quaternion q;
	q.x=hip.hierarchicalRotation.rotationQuaternion.x;
	q.y=hip.hierarchicalRotation.rotationQuaternion.y;
	q.z=hip.hierarchicalRotation.rotationQuaternion.z;
	q.w=hip.hierarchicalRotation.rotationQuaternion.w;*/
	rootBone->setOrientation(rootOrientation);

	
	return newPos2;
}