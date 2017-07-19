// main.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "LibRTMP.h"
#include <iostream>
#include <string>

TCHAR* rtmp_url = TEXT("rtmp://txy.live-send.acg.tv/live-txy/?streamname=live_6235300_5849362&key=10fc91777ea6f60a940ab13fab01c7f5");
int VIDEO_COUNT = 20;

TCHAR* string_to_tchar(std::string str)
{
    TCHAR *tchar = new TCHAR[str.size() + 1];
    tchar[str.size()] = 0;
    //As much as we'd love to, we can't use memcpy() because
    //sizeof(TCHAR)==sizeof(char) may not be true:
    std::copy(str.begin(), str.end(), tchar);
    return tchar;
}

int main()
{
    LibRTMP* lib = new LibRTMP();
    for (int i = 1; i <= VIDEO_COUNT; i++)
    {
        std::string video_file = "h264/" + std::to_string(i) + ".h264";
        std::cout << "Start to publish " << video_file << "..." << std::endl;
        lib->connectRTMPWithH264(rtmp_url, string_to_tchar(video_file));
        lib->close();
    }
    
    std::cout << "Press Enter to Continue" << std::endl;
    std::cin.ignore();

    return 0;
}

