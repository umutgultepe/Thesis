#include "StdAfx.h"
#pragma once
class NUI_Controller
{
public:
    NUI_Controller();
    ~NUI_Controller();
    bool                 Nui_Init();
	void				 Nui_Shutdown();
//    HRESULT                 Nui_Init(int index);
//    HRESULT                 Nui_Init(OLECHAR *instanceName);

    void                Nui_GotDepthAlert( );
    void                Nui_GotVideoAlert( );
    void                Nui_GotSkeletonAlert( );
    void                Nui_Zero();
	void				bufferSkeletonData( NUI_SKELETON_DATA * pSkel);
//    void                    Nui_BlankSkeletonScreen( HWND hWnd );
//    void                    Nui_DoDoubleBuffer(HWND hWnd,HDC hDC);
//    void                    Nui_DrawSkeleton( bool bBlank, NUI_SKELETON_DATA * pSkel, HWND hWnd, int WhichSkeletonColor );
//    void                    Nui_DrawSkeletonSegment( NUI_SKELETON_DATA * pSkel, int numJoints, ... );
//
    RGBQUAD                 Nui_ShortToQuad_Depth( USHORT s );
//
//    static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
//    LRESULT CALLBACK        WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
//    static void CALLBACK    Nui_StatusProcThunk(const NuiStatusData *pStatusData);
//    void CALLBACK           Nui_StatusProc(const NuiStatusData *pStatusData);
//
//private:
//    bool m_fUpdatingUi;
//    void UpdateComboBox();
//
//	CRITICAL_SECTION        m_critSecUi; // Gate UI operations on the background thread.
    static DWORD WINAPI     Nui_ProcessThread(LPVOID pParam);
    DWORD WINAPI            Nui_ProcessThread();
    INuiSensor*           m_pNuiInstance;
    BSTR                    m_instanceId;
	bool	mSkeletonUpdated;
//
//    // thread handling
    HANDLE        m_hThNuiProcess;
    HANDLE        m_hEvNuiProcessStop;
//
    HANDLE        m_hNextDepthFrameEvent;
    HANDLE        m_hNextVideoFrameEvent;
    HANDLE        m_hNextSkeletonEvent;
    HANDLE        m_pDepthStreamHandle;
    HANDLE        m_pVideoStreamHandle;
//    HFONT         m_hFontFPS;
//    HPEN          m_Pen[6];
//    HDC           m_SkeletonDC;
//    HBITMAP       m_SkeletonBMP;
//    HGDIOBJ       m_SkeletonOldObj;
//    int           m_PensTotal;
    NUI_Vector4         m_Points[NUI_SKELETON_POSITION_COUNT];
//    RGBQUAD       m_rgbWk[640*480];
//    int           m_LastSkeletonFoundTime;
//    bool          m_bScreenBlanked;
//    int           m_FramesTotal;
//    int           m_LastFPStime;
//    int           m_LastFramesTotal;
};
