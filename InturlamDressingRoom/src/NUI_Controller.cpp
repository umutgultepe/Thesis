#include "StdAfx.h"
#include "NUI_Controller.h"
NUI_Controller::NUI_Controller()
{
    Nui_Zero();
	Nui_Init();
}

NUI_Controller::~NUI_Controller()
{
	Nui_Shutdown();
    Nui_Zero();
}

void NUI_Controller::Nui_Zero()
{

    m_pNuiInstance = NULL;
    m_hNextDepthFrameEvent = NULL;
    m_hNextVideoFrameEvent = NULL;
    m_hNextSkeletonEvent = NULL;
    m_pDepthStreamHandle = NULL;
    m_pVideoStreamHandle = NULL;
    m_hThNuiProcess=NULL;
    m_hEvNuiProcessStop=NULL;
	mSkeletonUpdated=false;
    //ZeroMemory(m_Pen,sizeof(m_Pen));
    //m_SkeletonDC = NULL;
    //m_SkeletonBMP = NULL;
    //m_SkeletonOldObj = NULL;
    //m_PensTotal = 6;
    //ZeroMemory(m_Points,sizeof(m_Points));
    //m_LastSkeletonFoundTime = -1;
    //m_bScreenBlanked = false;
    //m_FramesTotal = 0;
    //m_LastFPStime = -1;
    //m_LastFramesTotal = 0;
	return;
}


bool NUI_Controller::Nui_Init()
{

	if (!m_pNuiInstance)
    {
        HRESULT hr = NuiCreateSensorByIndex(0, &m_pNuiInstance);
        if (FAILED(hr))
        {
            return hr;
			MessageBox( NULL, "Failed to create NUI instance", "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
        }
        m_instanceId = m_pNuiInstance->NuiDeviceConnectionId();
    }

	//Create Events
	m_hNextDepthFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
    m_hNextVideoFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
    m_hNextSkeletonEvent = CreateEvent( NULL, TRUE, FALSE, NULL );

	//Initialize NUI
	DWORD nuiFlags = NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON |  NUI_INITIALIZE_FLAG_USES_COLOR;
	HRESULT hr=m_pNuiInstance->NuiInitialize(nuiFlags);
	if( FAILED( hr ) )
    {
        MessageBox( NULL, "Failed to Init NUI", "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
        return false;
    }

	//Initialize Skeleton
	if (HasSkeletalEngine(m_pNuiInstance))
    {
        hr = m_pNuiInstance->NuiSkeletonTrackingEnable( m_hNextSkeletonEvent, 0 );
        if( FAILED( hr ) )
        {
            MessageBox( NULL, "Failed to Init Skeleton", "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
            return hr;
        }
    }

	//Initialize Video
	hr = m_pNuiInstance->NuiImageStreamOpen(
        NUI_IMAGE_TYPE_COLOR,
        NUI_IMAGE_RESOLUTION_640x480,
        0,
        2,
        m_hNextVideoFrameEvent,
        &m_pVideoStreamHandle );
    if( FAILED( hr ) )
    {
        MessageBox( NULL, "Failed to Init Video", "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
        return hr;
    }

	//Initialize Depth
	hr = m_pNuiInstance->NuiImageStreamOpen(
        HasSkeletalEngine(m_pNuiInstance) ? NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX : NUI_IMAGE_TYPE_DEPTH,
        NUI_IMAGE_RESOLUTION_320x240,
        0,
        2,
        m_hNextDepthFrameEvent,
        &m_pDepthStreamHandle );
    if( FAILED( hr ) )
    {
       MessageBox( NULL, "Failed to Init Depth", "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
        return hr;
    }

	// Start the Nui processing thread
    m_hEvNuiProcessStop=CreateEvent(NULL,FALSE,FALSE,NULL);
    m_hThNuiProcess=CreateThread(NULL,0,Nui_ProcessThread,this,0,NULL);

	return true;
}


void NUI_Controller::Nui_Shutdown()
{
    // Stop the Nui processing thread
    if(m_hEvNuiProcessStop!=NULL)
    {
        // Signal the thread
        SetEvent(m_hEvNuiProcessStop);

        // Wait for thread to stop
        if(m_hThNuiProcess!=NULL)
        {
            WaitForSingleObject(m_hThNuiProcess,INFINITE);
            CloseHandle(m_hThNuiProcess);
        }
        CloseHandle(m_hEvNuiProcessStop);
    }

    if (m_pNuiInstance)
    {
        m_pNuiInstance->NuiShutdown( );
    }
    if( m_hNextSkeletonEvent && ( m_hNextSkeletonEvent != INVALID_HANDLE_VALUE ) )
    {
        CloseHandle( m_hNextSkeletonEvent );
        m_hNextSkeletonEvent = NULL;
    }
    if( m_hNextDepthFrameEvent && ( m_hNextDepthFrameEvent != INVALID_HANDLE_VALUE ) )
    {
        CloseHandle( m_hNextDepthFrameEvent );
        m_hNextDepthFrameEvent = NULL;
    }
    if( m_hNextVideoFrameEvent && ( m_hNextVideoFrameEvent != INVALID_HANDLE_VALUE ) )
    {
        CloseHandle( m_hNextVideoFrameEvent );
        m_hNextVideoFrameEvent = NULL;
    }

}


//static const COLORREF g_JointColorTable[NUI_SKELETON_POSITION_COUNT] = 
//{
//    RGB(169, 176, 155), // NUI_SKELETON_POSITION_HIP_CENTER
//    RGB(169, 176, 155), // NUI_SKELETON_POSITION_SPINE
//    RGB(168, 230, 29), // NUI_SKELETON_POSITION_SHOULDER_CENTER
//    RGB(200, 0,   0), // NUI_SKELETON_POSITION_HEAD
//    RGB(79,  84,  33), // NUI_SKELETON_POSITION_SHOULDER_LEFT
//    RGB(84,  33,  42), // NUI_SKELETON_POSITION_ELBOW_LEFT
//    RGB(255, 126, 0), // NUI_SKELETON_POSITION_WRIST_LEFT
//    RGB(215,  86, 0), // NUI_SKELETON_POSITION_HAND_LEFT
//    RGB(33,  79,  84), // NUI_SKELETON_POSITION_SHOULDER_RIGHT
//    RGB(33,  33,  84), // NUI_SKELETON_POSITION_ELBOW_RIGHT
//    RGB(77,  109, 243), // NUI_SKELETON_POSITION_WRIST_RIGHT
//    RGB(37,   69, 243), // NUI_SKELETON_POSITION_HAND_RIGHT
//    RGB(77,  109, 243), // NUI_SKELETON_POSITION_HIP_LEFT
//    RGB(69,  33,  84), // NUI_SKELETON_POSITION_KNEE_LEFT
//    RGB(229, 170, 122), // NUI_SKELETON_POSITION_ANKLE_LEFT
//    RGB(255, 126, 0), // NUI_SKELETON_POSITION_FOOT_LEFT
//    RGB(181, 165, 213), // NUI_SKELETON_POSITION_HIP_RIGHT
//    RGB(71, 222,  76), // NUI_SKELETON_POSITION_KNEE_RIGHT
//    RGB(245, 228, 156), // NUI_SKELETON_POSITION_ANKLE_RIGHT
//    RGB(77,  109, 243) // NUI_SKELETON_POSITION_FOOT_RIGHT
//};


DWORD WINAPI NUI_Controller::Nui_ProcessThread(LPVOID pParam)
{
    NUI_Controller *pthis=(NUI_Controller *) pParam;
    return pthis->Nui_ProcessThread();
}
//
DWORD WINAPI NUI_Controller::Nui_ProcessThread()
{
    HANDLE                hEvents[4];
    int                    nEventIdx/*,t,dt*/;

    // Configure events to be listened on
    hEvents[0]= m_hEvNuiProcessStop;
    hEvents[1]= m_hNextDepthFrameEvent;
    hEvents[2]= m_hNextVideoFrameEvent;
    hEvents[3]= m_hNextSkeletonEvent;

#pragma warning(push)
#pragma warning(disable: 4127) // conditional expression is constant

    // Main thread loop
    while(1)
    {
        // Wait for an event to be signalled
        nEventIdx=WaitForMultipleObjects(sizeof(hEvents)/sizeof(hEvents[0]),hEvents,FALSE,100);

        // If the stop event, stop looping and exit
        if(nEventIdx==0)
            break;            
        // Process signal events
        //switch(nEventIdx)
        //{
        //    case 1:
        //        Nui_GotDepthAlert();
        //        break;

        //    case 2:
        //        Nui_GotVideoAlert();
        //        break;

        //    case 3:
        //        Nui_GotSkeletonAlert( );
        //        break;
        //}
		if ( WAIT_OBJECT_0 == WaitForSingleObject( m_hNextDepthFrameEvent, 0 ) )
        {
			Nui_GotDepthAlert();
        }

        if ( WAIT_OBJECT_0 == WaitForSingleObject( m_hNextVideoFrameEvent, 0 ) )
        {
            Nui_GotVideoAlert();
        }

        if (  WAIT_OBJECT_0 == WaitForSingleObject( m_hNextSkeletonEvent, 0 ) )
        {
            Nui_GotSkeletonAlert( );
        }


    }
#pragma warning(pop)

    return (0);
}
//
void NUI_Controller::Nui_GotVideoAlert( )
{
    NUI_IMAGE_FRAME pImageFrame;

    HRESULT hr = m_pNuiInstance->NuiImageStreamGetNextFrame(
        m_pVideoStreamHandle,
        0,
        &pImageFrame );
    if( FAILED( hr ) )
    {
        return;
    }

    INuiFrameTexture * pTexture = pImageFrame.pFrameTexture;
    NUI_LOCKED_RECT LockedRect;
    pTexture->LockRect( 0, &LockedRect, NULL, 0 );
    if( LockedRect.Pitch != 0 )
    {
        BYTE * pBuffer = (BYTE*) LockedRect.pBits;
    }
    else
    {
		MessageBox( NULL, "Buffer length of received texture is bogus\r", "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
    }

    m_pNuiInstance->NuiImageStreamReleaseFrame( m_pVideoStreamHandle, &pImageFrame );
}


#include "HandTracker.h"
IplImage* tImage=0;
bool textureUpdated=false;


static const int g_IntensityShiftByPlayerR[] = { 1, 2, 0, 2, 0, 0, 2, 0 };
static const int g_IntensityShiftByPlayerG[] = { 1, 2, 2, 0, 2, 0, 0, 1 };
static const int g_IntensityShiftByPlayerB[] = { 1, 0, 2, 2, 0, 2, 0, 2 };

void NUI_Controller::Nui_GotDepthAlert( )
{
    NUI_IMAGE_FRAME pImageFrame;

    HRESULT hr = m_pNuiInstance->NuiImageStreamGetNextFrame(
        m_pDepthStreamHandle,
        0,
        &pImageFrame );

    if( FAILED( hr ) )
    {
        return;
    }

    INuiFrameTexture * pTexture = pImageFrame.pFrameTexture;
    NUI_LOCKED_RECT LockedRect;
    pTexture->LockRect( 0, &LockedRect, NULL, 0 );
    if( LockedRect.Pitch != 0 )
    {
        BYTE * pBuffer = (BYTE*) LockedRect.pBits;

		//Show Result
		if (!tImage)
			tImage=cvCreateImage(dSize,IPL_DEPTH_8U,3);
		// draw the bits to the bitmap
		USHORT * pBufferRun = (USHORT*) pBuffer;
		for( int y = 0 ; y < m_Height ; y++ )
		{
			BYTE* dPtr=(BYTE*)(tImage->imageData+y*tImage->widthStep);
			for( int x = 0 ; x < m_Width ; x++ )
			{
				USHORT depth     = *pBufferRun++;
				USHORT realDepth = NuiDepthPixelToDepth(depth);
				USHORT player    = NuiDepthPixelToPlayerIndex(depth);
				BYTE intensity = static_cast<BYTE>(~(realDepth >> 4));
				//RGBQUAD quad = Nui_ShortToQuad_Depth( *pBufferRun++ );
				*dPtr++ = intensity >> g_IntensityShiftByPlayerB[player];;
				*dPtr++ = intensity >> g_IntensityShiftByPlayerG[player];
				*dPtr++ = intensity >> g_IntensityShiftByPlayerR[player];
			}
		}
		cvShowImage("result",tImage);cvWaitKey(2);
		textureUpdated=true;
    }
    else
    {
		MessageBox( NULL, "Buffer length of received texture is bogus\r" , "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
    }
    m_pNuiInstance->NuiImageStreamReleaseFrame( m_pDepthStreamHandle, &pImageFrame );
}







//void NUI_Controller::Nui_BlankSkeletonScreen(HWND hWnd)
//{
//    HDC hdc = GetDC( hWnd );
//    RECT rct;
//    GetClientRect(hWnd, &rct);
//    int width = rct.right;
//    int height = rct.bottom;
//    PatBlt( hdc, 0, 0, width, height, BLACKNESS );
//    ReleaseDC( hWnd, hdc );
//}
//
//void NUI_Controller::Nui_DrawSkeletonSegment( NUI_SKELETON_DATA * pSkel, int numJoints, ... )
//{
//    va_list vl;
//    va_start(vl,numJoints);
//
//    POINT segmentPositions[NUI_SKELETON_POSITION_COUNT];
//    int segmentPositionsCount = 0;
//
//    DWORD polylinePointCounts[NUI_SKELETON_POSITION_COUNT];
//    int numPolylines = 0;
//    int currentPointCount = 0;
//
//    // Note the loop condition: We intentionally run one iteration beyond the
//    // last element in the joint list, so we can properly end the final polyline.
//
//    for (int iJoint = 0; iJoint <= numJoints; iJoint++)
//    {
//        if (iJoint < numJoints)
//        {
//            NUI_SKELETON_POSITION_INDEX jointIndex = va_arg(vl,NUI_SKELETON_POSITION_INDEX);
//
//            if (pSkel->eSkeletonPositionTrackingState[jointIndex] != NUI_SKELETON_POSITION_NOT_TRACKED)
//            {
//                // This joint is tracked: add it to the array of segment positions.
//            
//                segmentPositions[segmentPositionsCount].x = m_Points[jointIndex].x;
//                segmentPositions[segmentPositionsCount].y = m_Points[jointIndex].y;
//                segmentPositionsCount++;
//                currentPointCount++;
//
//                // Fully processed the current joint; move on to the next one
//
//                continue;
//            }
//        }
//
//        // If we fall through to here, we're either beyond the last joint, or
//        // the current joint is not tracked: end the current polyline here.
//
//        if (currentPointCount > 1)
//        {
//            // Current polyline already has at least two points: save the count.
//
//            polylinePointCounts[numPolylines++] = currentPointCount;
//        }
//        else if (currentPointCount == 1)
//        {
//            // Current polyline has only one point: ignore it.
//
//            segmentPositionsCount--;
//        }
//        currentPointCount = 0;
//    }
//
//#ifdef _DEBUG
//    // We should end up with no more points in segmentPositions than the
//    // original number of joints.
//
//    assert(segmentPositionsCount <= numJoints);
//
//    int totalPointCount = 0;
//    for (int i = 0; i < numPolylines; i++)
//    {
//        // Each polyline should contain at least two points.
//    
//        assert(polylinePointCounts[i] > 1);
//
//        totalPointCount += polylinePointCounts[i];
//    }
//
//    // Total number of points in all polylines should be the same as number
//    // of points in segmentPositions.
//    
//    assert(totalPointCount == segmentPositionsCount);
//#endif
//
//    if (numPolylines > 0)
//    {
//        PolyPolyline(m_SkeletonDC, segmentPositions, polylinePointCounts, numPolylines);
//    }
//
//    va_end(vl);
//}
//
//void NUI_Controller::Nui_DrawSkeleton( bool bBlank, NUI_SKELETON_DATA * pSkel, HWND hWnd, int WhichSkeletonColor )
//{
//    HGDIOBJ hOldObj = SelectObject(m_SkeletonDC,m_Pen[WhichSkeletonColor % m_PensTotal]);
//    
//    RECT rct;
//    GetClientRect(hWnd, &rct);
//    int width = rct.right;
//    int height = rct.bottom;
//
//    if( m_Pen[0] == NULL )
//    {
//        m_Pen[0] = CreatePen( PS_SOLID, width / 80, RGB(255, 0, 0) );
//        m_Pen[1] = CreatePen( PS_SOLID, width / 80, RGB( 0, 255, 0 ) );
//        m_Pen[2] = CreatePen( PS_SOLID, width / 80, RGB( 64, 255, 255 ) );
//        m_Pen[3] = CreatePen( PS_SOLID, width / 80, RGB(255, 255, 64 ) );
//        m_Pen[4] = CreatePen( PS_SOLID, width / 80, RGB( 255, 64, 255 ) );
//        m_Pen[5] = CreatePen( PS_SOLID, width / 80, RGB( 128, 128, 255 ) );
//    }
//
//    if( bBlank )
//    {
//        PatBlt( m_SkeletonDC, 0, 0, width, height, BLACKNESS );
//    }
//
//    int scaleX = width; //scaling up to image coordinates
//    int scaleY = height;
//    float fx=0,fy=0;
//    int i;
//    for (i = 0; i < NUI_SKELETON_POSITION_COUNT; i++)
//    {
//        NuiTransformSkeletonToDepthImageF( pSkel->SkeletonPositions[i], &fx, &fy );
//        m_Points[i].x = (int) ( fx * scaleX + 0.5f );
//        m_Points[i].y = (int) ( fy * scaleY + 0.5f );
//    }
//
//    SelectObject(m_SkeletonDC,m_Pen[WhichSkeletonColor%m_PensTotal]);
//    
//    Nui_DrawSkeletonSegment(pSkel,4,NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_HEAD);
//    Nui_DrawSkeletonSegment(pSkel,5,NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT);
//    Nui_DrawSkeletonSegment(pSkel,5,NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT);
//    Nui_DrawSkeletonSegment(pSkel,5,NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_LEFT, NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT);
//    Nui_DrawSkeletonSegment(pSkel,5,NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_RIGHT, NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT);
//    
//    // Draw the joints in a different color
//    for (i = 0; i < NUI_SKELETON_POSITION_COUNT ; i++)
//    {
//        if (pSkel->eSkeletonPositionTrackingState[i] != NUI_SKELETON_POSITION_NOT_TRACKED)
//        {
//            HPEN hJointPen;
//        
//            hJointPen=CreatePen(PS_SOLID,9, g_JointColorTable[i]);
//            hOldObj=SelectObject(m_SkeletonDC,hJointPen);
//
//            MoveToEx( m_SkeletonDC, m_Points[i].x, m_Points[i].y, NULL );
//            LineTo( m_SkeletonDC, m_Points[i].x, m_Points[i].y );
//
//            SelectObject( m_SkeletonDC, hOldObj );
//            DeleteObject(hJointPen);
//        }
//    }
//
//    return;
//
//}
//
//
//
//
//void NUI_Controller::Nui_DoDoubleBuffer(HWND hWnd,HDC hDC)
//{
//    RECT rct;
//    GetClientRect(hWnd, &rct);
//    int width = rct.right;
//    int height = rct.bottom;
//
//    HDC hdc = GetDC( hWnd );
//
//    BitBlt( hdc, 0, 0, width, height, hDC, 0, 0, SRCCOPY );
//
//    ReleaseDC( hWnd, hdc );
//
//}
//



void NUI_Controller::Nui_GotSkeletonAlert( )
{
    NUI_SKELETON_FRAME SkeletonFrame;

    bool bFoundSkeleton = false;

    if( SUCCEEDED(m_pNuiInstance->NuiSkeletonGetNextFrame( 0, &SkeletonFrame )) )
    {
        for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
        {
            if( SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED )
            {
                bFoundSkeleton = true;
            }
        }
    }

    // no skeletons!
    //
    if( !bFoundSkeleton )
    {
        return;
    }

    // smooth out the skeleton data
    m_pNuiInstance->NuiTransformSmooth(&SkeletonFrame,NULL);

    // we found a skeleton, re-start the timer
    //m_bScreenBlanked = false;
    //m_LastSkeletonFoundTime = -1;

    // draw each skeleton color according to the slot within they are found.
    //
    //bool bBlank = true;
    //for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
    //{
    //    // Show skeleton only if it is tracked, and the center-shoulder joint is at least inferred.

    //    if( SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED &&
    //        SkeletonFrame.SkeletonData[i].eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_SHOULDER_CENTER] != NUI_SKELETON_POSITION_NOT_TRACKED)
    //    {
    //        //Nui_DrawSkeleton( bBlank, &SkeletonFrame.SkeletonData[i], GetDlgItem( m_hWnd, IDC_SKELETALVIEW ), i );
			
    //        bBlank = false;
    //    }
    //}

	if (NUI_SKELETON_COUNT > 1)
	{
		for (  int i = 0; i < NUI_SKELETON_POSITION_COUNT; i++)
		{
			m_Points[i]=SkeletonFrame.SkeletonData[0].SkeletonPositions[i];
		}
		NuiSkeletonCalculateBoneOrientations(SkeletonFrame.SkeletonData,m_Orientations);
		mSkeletonUpdated=true;
	}

	
    //Nui_DoDoubleBuffer(GetDlgItem(m_hWnd,IDC_SKELETALVIEW), m_SkeletonDC);
}



//
//
RGBQUAD NUI_Controller::Nui_ShortToQuad_Depth( USHORT s )
{
    bool hasPlayerData = HasSkeletalEngine(m_pNuiInstance);
    USHORT RealDepth = hasPlayerData ? (s & 0xfff8) >> 3 : s & 0xffff;
    USHORT Player = hasPlayerData ? s & 7 : 0;

    // transform 13-bit depth information into an 8-bit intensity appropriate
    // for display (we disregard information in most significant bit)
    BYTE l = 255 - (BYTE)(256*RealDepth/0x0fff);

    RGBQUAD q;
    q.rgbRed = q.rgbBlue = q.rgbGreen = 0;

    switch( Player )
    {
    case 0:
        q.rgbRed = l / 2;
        q.rgbBlue = l / 2;
        q.rgbGreen = l / 2;
        break;
    case 1:
        q.rgbRed = l;
        break;
    case 2:
        q.rgbGreen = l;
        break;
    case 3:
        q.rgbRed = l / 4;
        q.rgbGreen = l;
        q.rgbBlue = l;
        break;
    case 4:
        q.rgbRed = l;
        q.rgbGreen = l;
        q.rgbBlue = l / 4;
        break;
    case 5:
        q.rgbRed = l;
        q.rgbGreen = l / 4;
        q.rgbBlue = l;
        break;
    case 6:
        q.rgbRed = l / 2;
        q.rgbGreen = l / 2;
        q.rgbBlue = l;
        break;
    case 7:
        q.rgbRed = 255 - ( l / 2 );
        q.rgbGreen = 255 - ( l / 2 );
        q.rgbBlue = 255 - ( l / 2 );
    }
	q.rgbReserved=255;
    return q;
}
