#pragma once
#include <string> 
#include <iostream>
#include <sstream>
#include <windows.h>

class LibFFmpeg
{
public:
    LibFFmpeg(char* path, char* ffpath);
    ~LibFFmpeg();

    void WriteFrame(unsigned char* data, int size);
    void Close();

private:
    std::string command;
    HANDLE childInReadHandle;
    HANDLE childInWriteHandle;
    bool CreateCapturePipe();
    bool CreateCaptureProcess();
};

