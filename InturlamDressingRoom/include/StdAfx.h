#define WIN32_LEAN_AND_MEAN   
#include <windows.h>
#include <ole2.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>
#include "NuiApi.h"
typedef _Vector4 NUI_Vector4;

#include "Ogre.h"
#include "OIS.h"
#include <opencv\cv.h>
#include <opencv\cxcore.h>
#include <opencv\highgui.h>
#include <XnVDeviceGenerator.h>
#include <XnVNite.h>
#include <vector>
//#include <MyUserControl.h>
#include <XnTypes.h>
#include <XnV3DVector.h>

#include <OgreStringConverter.h>
#include <OgreErrorDialog.h>


#include <XnCppWrapper.h>
#include "SamplePlugin.h"
#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreLogManager.h>
#include <OgreRoot.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>

#include <OISEvents.h>
#include <OISInputManager.h>
#include <OISKeyboard.h>
#include <OISMouse.h>

#include "PxPhysics.h"
#include <SdkTrays.h>
#include <SdkCameraMan.h>


#include <PxPhysicsAPI.h> 
#include <PxExtensionsAPI.h>   
#include <PxGpuDispatcher.h>
#include <PxCudaContextManager.h>
#include <Cloth/PxClothReadData.h> 
#include "Stream.h"
 

using namespace Ogre;

//void WriteToTexture(const String &str, TexturePtr destTexture, Image::Box destRectangle, Font* font, const ColourValue &color, char justify ,  bool wordwrap );