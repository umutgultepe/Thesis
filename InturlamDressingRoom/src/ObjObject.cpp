#include "StdAfx.h"
#include "ObjObject.h"
using namespace physx;
using namespace std;

 float convertToFloat(std::string const& s)
 {
   std::istringstream i(s);
   float x;
   i >> x;
   return x;
 }

  int convertToInt(std::string const& s)
 {
   std::istringstream i(s);
   int x;
   i >> x;
   return x;
 }

 float* ObjObject::getArray(string s,int beginPoint)
 {

	string num;
	int nextNum=beginPoint;
	int numberOfNumbers=0;
	std::vector<float> indices;
	int sLength=s.size();
	for (int i=beginPoint;i<sLength;i++)
	{
		if (s.at(i)==' ')
		{
			if(num.size()>0)
			{
				indices.push_back(convertToFloat(num));
				num.clear();
				numberOfNumbers++;
			}
			else
				continue;
		}	
		else
		{
			num.push_back(s.at(i));
			if(i==s.size()-1)
			{
				indices.push_back(convertToFloat(num));
				numberOfNumbers++;
			}
		}
	}
	float* fArray=new float[numberOfNumbers];
	for (int i=0;i<numberOfNumbers;i++)
		fArray[i]=indices[i];
	return fArray;
 }
 
 int* ObjObject::getFaces(string s)
 {
	
	string num;
	std::vector<int> indices;
	int nextNum=2;
	bool spaceAct=false;
	int numOfVertices=0;
	int sLength=s.size();
	int* fArray;
	bool pureVertex=true;
	for (int i=1;i<sLength;i++)
	{
		if(s.at(i)=='/' || (s.at(i)=='\\' ))
		{
			pureVertex=false;
			break;
		}
	}
	if (pureVertex)
	{
		fArray=new int[9];
		float* tt=getArray(s,2);
		fArray[0]=tt[0];
		fArray[1]=-1;
		fArray[2]=-1;
		fArray[3]=tt[1];
		fArray[4]=-1;
		fArray[5]=-1;
		fArray[6]=tt[2];
		fArray[7]=-1;
		fArray[8]=-1;
	}
	else
	{

		for (int i=1;i<sLength;i++)
		{
			if (s.at(i)==' ')
			{
				if(num.size()>0)
				{
					indices.push_back(convertToInt(num));
					num.clear();
					if(i!=s.size()-1)
						indices.push_back(-2);
					numOfVertices++;
				}
				else
					continue;
			}
			else if(s.at(i)=='/' || (s.at(i)=='\\' ))
			{
				if(num.size()>0)
				{
					indices.push_back(convertToInt(num));
					num.clear();
				}
				indices.push_back(-1);
			}		
			else
			{
				num.push_back(s.at(i));
				if(i==s.size()-1)
				{
					indices.push_back(convertToInt(num));
					numOfVertices++;
				}
			}
		}
		fArray=new int[numOfVertices*3];
		int index=0;
		for (unsigned int i=0;i<indices.size();i++)
		{
			if (indices.at(i)==-1)
			{
				index++;
				fArray[index]=-1;
			}
			else if (indices.at(i)==-2)
				index++;
			else
			{
				fArray[index]=indices.at(i)-1;
			}
		}
	}
	return fArray;
	//calculate num of vertices in the shape


 }

 float ObjObject::getScaleFactor()
 {
	return scaleFactor;
 }

 void ObjObject::Scale(float scaleRatio)
 {
	for(int i=0;i<vertexCount;i++)
	{
		for (int j=0;j<3;j++)
			verticeCoordinates.at(i)[j]*=scaleRatio;
	}
	scaleFactor=scaleFactor*scaleRatio;
 }

 void ObjObject::Reset()
 {
	for (int i=0;i<vertexCount;i++)
		memcpy(verticeCoordinates.at(i),initialVerticeCoordinates.at(i),3*sizeof(float));
	if (normalCount>0)
	{
		for (int i=0;i<normalCount;i++)
			memcpy(normals.at(i),initialNormals.at(i),3*sizeof(float));
	}
 }
 
 void ObjObject::analyzeFixedVertices()
 {
	 fixedVertices.resize(vertexCount);


	 for (int i=0;i<vertexCount;i++)
	 {
		 fixedVertices.at(i)=0;
	 }

	  for (unsigned int j=0;j<groupList.size();j++)
	 {
		 ObjGroup* curGroup=groupList.at(j);
		 for (int i=0;i<curGroup->faceGroupCount;i++)
		 {
			FaceGroup* curFaceGroup=curGroup->faceGroups.at(i);
			Ogre::String matName=materialList.at( curFaceGroup->materialIndice)->getName();
			if (matName.find("Free")==Ogre::String::npos)
			{
				for(int k = curFaceGroup->firstIndice; k <=curFaceGroup->lastIndice; k++ ) 
				{
				for (int l=0;l<3;l++)
					fixedVertices.at(faceIndices.at(k)[0+l*3])=true;
				}

		  }

	  }

		}
 }

ObjObject::ObjObject(void)
{
}



PxClothMeshDesc* ObjObject::loadPhysxCloth(PxSceneDesc* SceneDesc,PxClothFabric* &fabric,PxClothParticle* &points,PxTransform* tr,PxPhysics* gPhysicsSDK)
{
	 std::vector<PxVec3> pos;
 //std::vector<PxVec3> normal;

	PxClothMeshDesc* meshDesc=new PxClothMeshDesc;
	meshDesc->setToDefault();
	
	//Fill the geometry
	meshDesc->points.count= vertexCount;        
	meshDesc->triangles.count= faceCount;    
	meshDesc->points.stride= sizeof(PxVec3);  
	meshDesc->triangles.stride= 3*sizeof(PxU32);  
	meshDesc->points.data= (PxVec3*)malloc(sizeof(PxVec3)*meshDesc->points.count);    
	meshDesc->triangles.data= (PxU32*)malloc(sizeof(PxU32)*meshDesc->triangles.count*3);    
	meshDesc->edgeFlags = 0;
	
	//Fill the geometry
	int i;    
	PxVec3 *p = (PxVec3*)meshDesc->points.data;   


	pos.resize(meshDesc->points.count);
 	normal.resize(meshDesc->points.count);
	indices.resize(meshDesc->triangles.count*3);
	for (i = 0; i < vertexCount; i++) {                
		   p->x = verticeCoordinates.at(i)[0];
		  p->y = verticeCoordinates.at(i)[1];
		  p->z = verticeCoordinates.at(i)[2];   
		  p++;     
	}   

	memcpy(&pos[0].x, (meshDesc->points.data), sizeof(PxVec3)*meshDesc->points.count);
	

	//Fill the topology
	PxU32 *id = (PxU32*)meshDesc->triangles.data;  
	for (i = 0; i < faceCount; i++) {                                  
		*id++ = faceIndices.at(i)[0]; *id++ = faceIndices.at(i)[3]; *id++ = faceIndices.at(i)[6];  
		
	}
	
	memcpy(&indices[0], meshDesc->triangles.data, sizeof(PxU32)*meshDesc->triangles.count*3);

	//Make sure everything is fine so far
	if(!(meshDesc->isValid()))
	   cerr<<"Mesh invalid."<<endl;

	//Start cooking of fibres
	PxCookingParams cp; 
	
	PxCooking* cooking = PxCreateCooking(PX_PHYSICS_VERSION, (gPhysicsSDK->getFoundation()), cp);
	MemoryWriteBuffer buf;

	

	bool status = cooking->cookClothFabric(*meshDesc,SceneDesc->gravity, buf);
	if(!status) {
	   cerr<<"Problem cooking mesh.\nExiting ..."<<endl;
	   exit(1);
	}
	    
	fabric=gPhysicsSDK->createClothFabric(MemoryReadBuffer(buf.data));
	analyzeFixedVertices();
	tr->p = PxVec3(0,10,0); tr->q = PxQuat::createIdentity();

	points=(PxClothParticle*)malloc(sizeof(PxClothParticle)*meshDesc->points.count);
	p = (PxVec3*)meshDesc->points.data;  

	
	for(size_t i=0;i<meshDesc->points.count;i++) {
	   points[i].pos = *p;
	   float yS=verticeCoordinates.at(i)[1];
	  // Fixing the top corner points
	   if(fixedVertices.at(i)) 
		  points[i].invWeight =0;
	   else 
		  points[i].invWeight = 0.5f;
	   p++;
	}

	
	cooking->release();
	return meshDesc;
}

void ObjObject::saveInitial()
{
	for (int i=0;i<vertexCount;i++)
	{
		float* tF=new float[3];
		memcpy(tF,verticeCoordinates.at(i),3*sizeof(float));
		initialVerticeCoordinates.push_back(tF);
	}
	if (normalCount>0)
	{
		for (int i=0;i<normalCount;i++)
		{
			float* tF=new float[3];
			memcpy(tF,normals.at(i),3*sizeof(float));
			initialNormals.push_back(tF);
		}
	}
}


ObjObject::ObjObject(const char* filename)
{
	ifstream  objFile(filename);
	name=StringConverter::toString(filename);
	for (int i=0;i<4;i++)
		name.pop_back();
	string line;
	entity=0;
	scaleFactor=1;
	vertexCount=0;
	faceCount=0;
	textureCount=0;
	normalCount=0;
	faceGroupCount=0;
	cloth=0;
	groupCount=0;
	//Counting Pass
	int activeGroup=-1;
	ObjGroup* defG=new ObjGroup;
	defG->name="default";
	defG->faceGroupCount=0;
	if (objFile.is_open())
	{
		long beginP=objFile.tellg();
		while ( objFile.good() && !objFile.eof() )
		{
		  getline (objFile,line);
		  if (line.size()==0 || line.at(0)=='#')
			  continue;
		  else
			  if (0==line.compare(0,6,"mtllib"))
			  {
				  string ff=filename;
				  unsigned int i=ff.size()-1;
				  string folder;
				  while (ff.at(i)!='\\' && ff.at(i)!='/')
					  i--;
				  for (unsigned int j=0;j<i+1;j++)
					  folder.push_back(ff.at(j));
				  i=6;
				  string mtlFile;
				  while (line.at(i)==' ')
					  i++;
				  for (i;i<line.size();i++)
					  mtlFile.push_back(line.at(i));
				  loadMtl(folder,mtlFile);

			  }
			  else if (0==line.compare(0,6,"usemtl"))
			  {
				  unsigned int i=6;
				  string material;
				  while (line.at(i)==' ')
					  i++;
				  for ( i;i<line.size();i++)
					  material.push_back(line.at(i));
				  if (faceGroupCount>0)
				  {
					  faceGroups.at(faceGroupCount-1)->lastIndice=faceCount-1;
					  faceGroups.at(faceGroupCount-1)->faceCount= faceGroups.at(faceGroupCount-1)->lastIndice- faceGroups.at(faceGroupCount-1)->firstIndice+1;
				  }

				  FaceGroup* tempFG=new FaceGroup;
				  tempFG->firstIndice=faceCount;
				  faceGroupCount++;
				  bool found=false;
				  for (int i=0;i<materialCount;i++)
				  {
					  if(materialList.at(i)->getName().compare(material)==0)
						{
							tempFG->materialIndice=i;
							found=true;
							break;
						}
				  }
				  if(!found)
				  {
					  tempFG->materialIndice=-1;
					  printf("warning:material %s not found\n",material.c_str());
				  }
				  if (activeGroup==-1)
				  {
					  defG->faceGroups.push_back(tempFG);
					  defG->faceGroupCount++;
				  }
				  else
				  {
					  groupList.at(activeGroup)->faceGroups.push_back(tempFG);
					  groupList.at(activeGroup)->faceGroupCount++;
				  }
				  faceGroups.push_back(tempFG);
			  }
			  else if (line.at(0)=='v')
			  {
				  if (line.at(1)=='t')
				  {
					  textureCoordinates.push_back(getArray(line,3));
					 textureCoordinates.at(textureCount)[1]=- textureCoordinates.at(textureCount)[1];//Due to blender export anomaly
					  textureCount++;
				  }
				  else if(line.at(1)=='n')
				  {
					  normals.push_back(getArray(line,3));
					  normalCount++;
				  }
				  else
				  {
					  verticeCoordinates.push_back(getArray(line,2));
					  vertexCount++;
				  }
			  }
			  else if (line.at(0)=='f')
			  {
				  faceIndices.push_back(getFaces(line));
				  faceCount++;
			  }
			  else if (line.at(0)=='g')
			  {
				 unsigned int i=1;
				 string num;
				 while (line.at(i)==' ')
					 i++;
				 for (i;i<line.size();i++)
					 num.push_back(line.at(i));
				 bool existing=false;
				 for (int i=0;i<groupCount;i++)
				 {
					 if (groupList.at(i)->name.compare(num)==0)
					 {
						 activeGroup=i;
						 existing=true;
						 break;
					 }
				 }
				 if (!existing)
				 {
					 ObjGroup* tG=new ObjGroup;
					 tG->name=num;
					 tG->faceGroupCount=0;
					 groupList.push_back(tG);
					 activeGroup++;
					 groupCount++;
					 printf("found a group %d\n",groupCount);
				 }
				 
				 
			  }
		}
		if (defG->faceGroupCount>0)
		{
			groupList.push_back(defG);
			groupCount++;
		}
		if (faceGroupCount>0)
		{
			faceGroups.at(faceGroupCount-1)->lastIndice=faceCount-1;
			faceGroups.at(faceGroupCount-1)->faceCount= faceGroups.at(faceGroupCount-1)->lastIndice- faceGroups.at(faceGroupCount-1)->firstIndice+1;
		}
		objFile.close();
		printf("Finished Loading\n");
		COM[0]=0;
		COM[1]=0;
		COM[2]=0;
		for (int i=0;i<vertexCount;i++)
			for (int j=0;j<3;j++)
				COM[j]=COM[j]+verticeCoordinates.at(i)[j]/vertexCount;
		printf("Got COM\n");
		
	}
	else
		printf("Could not open file!\n");
}

void ObjObject::loadMtl(string folder,string filename)
{
	string fullFileName=folder;
	fullFileName.append(filename);
	ifstream  mtlFile(fullFileName);
	string line;
	materialCount=0;
	textureFileCount=0;
	printf("loading mtl file: %s\n",fullFileName.c_str());
	//Counting Pass
	if (mtlFile.is_open())
	{
		long beginP=mtlFile.tellg();
		while ( mtlFile.good() && !mtlFile.eof() )
		{
		  getline (mtlFile,line);
		  if (line.size()==0 || line.at(0)=='#')
			  continue;
		  else
		  {
			  unsigned int ss=0;
			   while (line.at(ss)==' ' || line.at(ss)=='\t')
					  ss++;
			  if (line.compare(ss,6,"newmtl")==0)
			  {
				  unsigned int i=ss+6;
				  string num;
				  while (line.at(i)==' ')
					  i++;
				  for (i;i<line.size();i++)
					  num.push_back(line.at(i));

				  Ogre::MaterialPtr tMat=Ogre::MaterialManager::getSingleton().create(num,Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);


				  //ObjMaterial* tt=new ObjMaterial;
				  materialList.push_back(tMat);
				  materialCount++;
				 	  materialList.at(materialCount-1)->setReceiveShadows(false); 
				materialList.at(materialCount-1)->getTechnique(0)->setLightingEnabled(true); 
				materialList.at(materialCount-1)->getTechnique(0)->getPass(0)->setCullingMode(CULL_NONE);
			  }
			  else if(line.compare(ss,2,"Ka")==0)
			  {
				  float* tt=getArray(line,ss+3);
				 materialList.at(materialCount-1)->getTechnique(0)->getPass(0)->setAmbient(Ogre::ColourValue(tt[0],tt[1],tt[2])); 
			  }
			  else if(line.compare(ss,2,"Kd")==0)
			  {
				  float* tt=getArray(line,ss+3);
				  materialList.at(materialCount-1)->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(tt[0],tt[1],tt[2])); 
			  }
			  else if(line.compare(ss,2,"Ks")==0)
			  {
				  float* tt=getArray(line,ss+3);
				  materialList.at(materialCount-1)->getTechnique(0)->getPass(0)->setSpecular(Ogre::ColourValue(tt[0],tt[1],tt[2])); 
			  
			  }
			  else if(line.compare(ss,2,"Ns")==0)
			  {
				  float* tt=getArray(line,ss+3);
				  materialList.at(materialCount-1)->getTechnique(0)->getPass(0)->setShininess(tt[0]);
				 
			  }
			  else if(line.compare(ss,6,"map_Kd")==0)
			  {
				  if (line.size()>ss+7)
				  {
					  unsigned int i=ss+6;
					  string num=folder;
					  string texFile;
					  while (line.at(i)==' ')
						  i++;
					  for (i;i<line.size();i++)
						  texFile.push_back(line.at(i));
					  materialList.at(materialCount-1)->getTechnique(0)->getPass(0)->createTextureUnitState()->setTextureName(texFile);

				  }
			  }
			  else if((line.at(ss)=='d') || (line.compare(ss,2,"Tr")==0))
			  {
				  float* tt;
				  if(line.at(ss)=='d') 
					tt=getArray(line,2);
				  else
					tt=getArray(line,3);
				  if (tt[0]<1)
				  {
					   materialList.at(materialCount-1)->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
					   materialList.at(materialCount-1)->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);

						Ogre::ColourValue diffuse=materialList.at(materialCount-1)->getTechnique(0)->getPass(0)->getDiffuse();
					  Ogre::ColourValue ambient=materialList.at(materialCount-1)->getTechnique(0)->getPass(0)->getAmbient();
					  diffuse.a=tt[0];
					  ambient.a=tt[0];
					materialList.at(materialCount-1)->getTechnique(0)->getPass(0)->setDiffuse(diffuse);
					 materialList.at(materialCount-1)->getTechnique(0)->getPass(0)->setAmbient(ambient);
				  }

			  }
		  }
		}
	}

}
//PxReal myTimestep = 1.0f/60.0f;
void ObjObject::updateWithPhysics(PxScene* gScene,PxReal timeStep)
{
	gScene->simulate(timeStep,NULL,false);

	while(!gScene->fetchResults())  ;

	PxClothReadData* pData = cloth->lockClothReadData();
	PxClothParticle* pParticles = const_cast<PxClothParticle*>(pData->particles);

	for (int i = 0; i < vertexCount; i++) {         
		if (!fixedVertices.at(i))
		{
			verticeCoordinates.at(i)[0]=pParticles[i].pos.x;
			verticeCoordinates.at(i)[1]=pParticles[i].pos.y;
			verticeCoordinates.at(i)[2]=pParticles[i].pos.z; 
		}
	}  

	pData->unlock();


	//update normals
	for(size_t i=0;i<indices.size();i+=3) {
		PxVec3 p1 = pParticles[indices[i]].pos;
		PxVec3 p2 = pParticles[indices[i+1]].pos;
		PxVec3 p3 = pParticles[indices[i+2]].pos;
		PxVec3 n  = (p2-p1).cross(p3-p1);

		normal[indices[i]]    += n/3.0f ; 
		normal[indices[i+1]]  += n/3.0f ; 
		normal[indices[i+2]]  += n/3.0f ; 			
	}

	for(size_t i=0;i<normal.size();i++) { 
		PxVec3& n  = normal[i];
		n= n.getNormalized();
	}

	for (unsigned int j=0;j<groupList.size();j++)
	{
		ObjGroup* curGroup=groupList.at(j);
		for (int i=0;i<curGroup->faceGroupCount;i++)
		{
			FaceGroup* curFaceGroup=curGroup->faceGroups.at(i);
			Ogre::String matName=materialList.at( curFaceGroup->materialIndice)->getName();
			if (matName.find("Free")==Ogre::String::npos)
				continue;		
			else
			{

				SubMesh *subMesh = object->getSubMesh(curGroup->name+"_face_"+StringConverter::toString(i));
				VertexData* vertexData =  subMesh->vertexData;
				HardwareVertexBufferSharedPtr vBuf= vertexData->vertexBufferBinding->getBindings().at(0);
				float* pVertex = static_cast<float*>(vBuf->lock(HardwareBuffer::HBL_NORMAL));
				for(int k = curFaceGroup->firstIndice; k <=curFaceGroup->lastIndice; k++ ) {

					for (int l=0;l<3;l++)
					{
						int vIndice=faceIndices.at(k)[0+l*3];

						*pVertex++ = verticeCoordinates.at(vIndice)[0];
						*pVertex++ = verticeCoordinates.at(vIndice)[1];
						*pVertex++ = verticeCoordinates.at(vIndice)[2];
						if (faceIndices.at(k)[2+l*3]!=-1)
						{
							*pVertex++ = normal.at(faceIndices.at(k)[0+l*3]).x;
							*pVertex++ = normal.at(faceIndices.at(k)[0+l*3]).y;
							*pVertex++ = normal.at(faceIndices.at(k)[0+l*3]).z;
						}
						else
						{
							*pVertex++=0;
							*pVertex++=0;
							*pVertex++=0;
						}
						pVertex+=2;
						//pVertex+=5;
					}
				}
				vBuf->unlock();
			}
		}
	}
}

void ObjObject::setVisible(bool visible)
{
	if (entity)
		entity->setVisible(visible);
}


void ObjObject::loadIntoOgre(Ogre::SceneManager* sceneManager,Ogre::String name)
 {
	 object = MeshManager::getSingleton().createManual(name, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	
	 for (unsigned int j=0;j<groupList.size();j++)
	 {
		 ObjGroup* curGroup=groupList.at(j);
		 for (int i=0;i<curGroup->faceGroupCount;i++)
		 {
			 FaceGroup* curFaceGroup=curGroup->faceGroups.at(i);
			 SubMesh *subMesh = object->createSubMesh(curGroup->name+"_face_"+StringConverter::toString(i));
			  subMesh->vertexData=new VertexData();
			VertexData* vertexData =  subMesh->vertexData;
			
			size_t currOffset = 0;
			VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
			vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_POSITION);
			currOffset += VertexElement::getTypeSize(VET_FLOAT3);
			// normals
			vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_NORMAL);
			currOffset += VertexElement::getTypeSize(VET_FLOAT3);
			// two dimensional texture coordinates
			vertexDecl->addElement(0, currOffset, VET_FLOAT2, VES_TEXTURE_COORDINATES, 0);
			currOffset += VertexElement::getTypeSize(VET_FLOAT2);
			vertexData->vertexCount =curFaceGroup->faceCount*3;
			HardwareVertexBufferSharedPtr vBuf = HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), vertexData->vertexCount, HardwareBuffer::HBU_DYNAMIC, false);
			VertexBufferBinding* binding = vertexData->vertexBufferBinding;
			binding->setBinding(0, vBuf);
			float* pVertex = static_cast<float*>(vBuf->lock(HardwareBuffer::HBL_DISCARD));

			subMesh->indexData->indexCount =curFaceGroup->faceCount*3;
			 subMesh->indexData->indexBuffer = HardwareBufferManager::getSingleton().createIndexBuffer(HardwareIndexBuffer::IT_16BIT, subMesh->indexData->indexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
			 HardwareIndexBufferSharedPtr iBuf = subMesh->indexData->indexBuffer;
			 unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(HardwareBuffer::HBL_NORMAL));
			 int index=0;
			 int k,l;
			for( k = curFaceGroup->firstIndice; k <=curFaceGroup->lastIndice; k++ ) {

				for (l=0;l<3;l++)
				{
					*pVertex++ = verticeCoordinates.at(faceIndices.at(k)[0+l*3])[0];
					*pVertex++ = verticeCoordinates.at(faceIndices.at(k)[0+l*3])[1];
					*pVertex++ = verticeCoordinates.at(faceIndices.at(k)[0+l*3])[2];
					if (faceIndices.at(k)[2+l*3]!=-1)
					{
						*pVertex++ = normals.at(faceIndices.at(k)[2+l*3])[0];
						*pVertex++ = normals.at(faceIndices.at(k)[2+l*3])[1];
						*pVertex++ = normals.at(faceIndices.at(k)[2+l*3])[2];
					}
					else
					{
						*pVertex++=0;
						*pVertex++=0;
						*pVertex++=0;
					}
					if (faceIndices.at(k)[1+l*3]!=-1)
					{
					*pVertex++ = textureCoordinates.at(faceIndices.at(k)[1+l*3])[0];
					*pVertex++ = textureCoordinates.at(faceIndices.at(k)[1+l*3])[1];
					}
					else
					{
						*pVertex++=0;
						*pVertex++=0;
					}
					*pIndices++=index++;
				}
				
			 }
			vBuf->unlock();
			iBuf->unlock();
			subMesh->setMaterialName(materialList.at( curFaceGroup->materialIndice)->getName());
			subMesh->useSharedVertices = false;
		
		 }


	  }
     // the original code was missing this line:
	 object->_setBounds( AxisAlignedBox::BOX_INFINITE, false );
    // object->_setBoundingSphereRadius(scaleFactor);
         // this line makes clear the mesh is loaded (avoids memory leaks)
       object->load();
	   //if (!Ogre::MeshManager::getSingleton().resourceExists(name+".mesh"))
	   //{
		  //Ogre::MeshSerializer* mSerializer=new Ogre::MeshSerializer();
		  //mSerializer->exportMesh(object.getPointer(),name+".mesh");
		  // delete mSerializer;
	   //}
	   
  }

//void ObjObject::loadIntoShader()
//{
//	ObjVertex* actVertex;
//	int index;
//	for (int k=0;k<groupCount;k++)
//	{
//		for(int l=0;l<groupList.at(k)->faceGroupCount;l++)
//		{
//			actVertex=new ObjVertex[3*groupList.at(k)->faceGroups.at(l)->faceCount];
//			index=0;
//			for (int i=groupList.at(k)->faceGroups.at(l)->firstIndice;i<groupList.at(k)->faceGroups.at(l)->lastIndice+1;i++)
//			{
//				for (int j=0;j<3;j++)
//				{
//					actVertex[3*index].Position[j]=verticeCoordinates.at(faceIndices.at(i)[0])[j];
//					actVertex[3*index+1].Position[j]=verticeCoordinates.at(faceIndices.at(i)[3])[j];
//					actVertex[3*index+2].Position[j]=verticeCoordinates.at(faceIndices.at(i)[6])[j];
//				}
//				if(faceIndices.at(i)[1]>-1)
//				{
//					actVertex[3*index].TexCoord[0]=textureCoordinates.at(faceIndices.at(i)[1])[0];
//					actVertex[3*index+1].TexCoord[0]=textureCoordinates.at(faceIndices.at(i)[4])[0];
//					actVertex[3*index+2].TexCoord[0]=textureCoordinates.at(faceIndices.at(i)[7])[0];
//
//					actVertex[3*index].TexCoord[1]=1-textureCoordinates.at(faceIndices.at(i)[1])[1];
//					actVertex[3*index+1].TexCoord[1]=1-textureCoordinates.at(faceIndices.at(i)[4])[1];
//					actVertex[3*index+2].TexCoord[1]=1-textureCoordinates.at(faceIndices.at(i)[7])[1];
//				
//				}
//				if(faceIndices.at(i)[2]>-1)
//				{
//					for (int j=0;j<3;j++)
//					{
//						actVertex[3*index].Normal[j]=normals.at(faceIndices.at(i)[2])[j];
//						actVertex[3*index+1].Normal[j]=normals.at(faceIndices.at(i)[5])[j];
//						actVertex[3*index+2].Normal[j]=normals.at(faceIndices.at(i)[8])[j];
//					}
//				}
//				index++;
//			}
//			glGenVertexArrays(1, &groupList.at(k)->faceGroups.at(l)->vertexArray);
//			glBindVertexArray(groupList.at(k)->faceGroups.at(l)->vertexArray);
//			glEnableVertexAttribArray(0);
//			if(faceIndices.at(groupList.at(k)->faceGroups.at(l)->firstIndice)[1]>-1);
//			glEnableVertexAttribArray(1);
//			//if(faceIndices.at(groupList.at(k)->faceGroups.at(l)->firstIndice)[2]>-1);
//			//glEnableVertexAttribArray(2);
//
//
//			glGenBuffers(1, &groupList.at(k)->faceGroups.at(l)->vertexBuffer);
//			glBindBuffer(GL_ARRAY_BUFFER,groupList.at(k)->faceGroups.at(l)->vertexBuffer);
//			glBufferData(GL_ARRAY_BUFFER,(groupList.at(k)->faceGroups.at(l)->faceCount)*3*sizeof(ObjVertex), actVertex, GL_STATIC_DRAW);
//			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,sizeof(ObjVertex), (GLvoid*)0);
//			glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(ObjVertex), (GLvoid*)(sizeof(actVertex[0].Position)));
//			//glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(ObjVertex), (GLvoid*)(sizeof(actVertex[0].Position)+sizeof(actVertex[0].TexCoord)));
//			glBindVertexArray(0);
//		}
//	}
//}


ObjObject::~ObjObject(void)
{
	for (int i=0;i<vertexCount;i++)
	delete[] verticeCoordinates.at(i);
	
	for (int i=0;i<normalCount;i++)
	delete[] normals.at(i);

	for (int i=0;i<textureCount;i++)
	delete[] textureCoordinates.at(i);

	for (int i=0;i<faceCount;i++)
	delete[] faceIndices.at(i);

	verticeCoordinates.clear();
	normals.clear();
	textureCoordinates.clear();
	faceIndices.clear();
}

float* ObjObject::moveToOrigin()
{
	float*  res=new float[3];

	for (int i=0;i<3;i++)
		res[i]=-COM[i];


	for (int i=0;i<vertexCount;i++)
		for (int j=0;j<3;j++)
			verticeCoordinates.at(i)[j]=verticeCoordinates.at(i)[j]-COM[j];

	COM[0]=0;
	COM[1]=0;
	COM[2]=0;

	return res;
}
void ObjObject::translate(float veec[3])

{

	for (int i=0;i<vertexCount;i++)
		for (int j=0;j<3;j++)
			verticeCoordinates.at(i)[j]=verticeCoordinates.at(i)[j]+veec[j];

	for (int j=0;j<3;j++)
			COM[j]=COM[j]+veec[j];
}

float* ObjObject::placeOnXZ(bool aboveOrBelow)
{
	float yy=verticeCoordinates.at(0)[1];
	if (aboveOrBelow)
	{
	for (int i=0;i<vertexCount;i++)
		if (yy>verticeCoordinates.at(i)[1])
			yy=verticeCoordinates.at(i)[1];
	}
	else
	{
		for (int i=0;i<vertexCount;i++)
			if (yy<verticeCoordinates.at(i)[1])
				yy=verticeCoordinates.at(i)[1];
	}
	for (int i=0;i<vertexCount;i++)
		verticeCoordinates.at(i)[1]=verticeCoordinates.at(i)[1]-yy;

	float* res=new float[3];
	res[0]=0;
	res[1]=-yy;
	res[2]=0;
	COM[1]=-yy;
	return res;

}

float ObjObject::unitize()
{
	float min[3],max[3];
	for (int j=0;j<3;j++)
	{
		min[j]=verticeCoordinates.at(0)[j];
		max[j]=verticeCoordinates.at(0)[j];
	}

	for (int i=0;i<vertexCount;i++)
		for (int j=0;j<3;j++)
		{
			if(min[j]>verticeCoordinates.at(i)[j])
				min[j]=verticeCoordinates.at(i)[j];
			if(max[j]<verticeCoordinates.at(i)[j])
				max[j]=verticeCoordinates.at(i)[j];
		}

	float scale=max[0]-min[0];
	if (scale<max[1]-min[1])
		scale=max[1]-min[1];
	if (scale<max[2]-min[2])
		scale=max[2]-min[2];

	scaleFactor=1/scale;
	for (int i=0;i<vertexCount;i++)
		for (int j=0;j<3;j++)
			//verticeCoordinates.at(i)[j]=(verticeCoordinates.at(i)[j]-COM[j])/scale+COM[j];
			verticeCoordinates.at(i)[j]*=scaleFactor;
	return scale;
}