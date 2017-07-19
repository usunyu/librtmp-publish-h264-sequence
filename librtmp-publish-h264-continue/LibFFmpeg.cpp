#include "stdafx.h"
#include "LibFFmpeg.h"

// ffmpeg -r 30 -f h264 -i 1.h264 -s 1280x720 -preset ultrafast -pix_fmt yuv420p -vcodec libx264 -crf 30 -framerate 25 -tune zerolatency -f flv 1.flv
// ffmpeg -r 30 -f h264 -i 1.h264 -preset ultrafast -pix_fmt yuv420p -vcodec libx264 -crf 30 -framerate 25 -tune zerolatency -f flv 1.flv
// ffmpeg -r 30 -f h264 -i 1.h264 -preset ultrafast -c copy -crf 30 -framerate 25 -tune zerolatency -f flv 1.flv
// cat 1.h264 | ffmpeg -r 30 -f h264 -i - -y -preset ultrafast -c copy -crf 30 -framerate 25 -tune zerolatency -f flv 1.flv

LibFFmpeg::LibFFmpeg(char* path, char* ffpath)
{
    // start ffmpeg telling it to expect raw rgb frames
    // -i - tells it to read frames from stdin
    std::stringstream cmd;
    cmd << '"' << ffpath << '"' << " "
        << "-r 30 "
        << "-f h264 "
        //    << "-i - -threads 0 -y "
        //<< "-s 1280x720 "
        << "-i - -y "
        << "-preset ultrafast " // Encoding speed: ultrafast, superfast,0 veryfast, faster, fast, medium, slow, slower, veryslow
        << "-pix_fmt yuv420p "
        << "-vcodec libx264 "
        << "-crf 30 " // codec & quality. Range is logarithmic 0 (lossless) to 51 (worst quality). Default is 23.
        //<< "-vf vflip "
        << "-framerate 25 "
        << "-tune zerolatency "
        //    << "-x264opts vbv-maxrate=2000:vbv-bufsize=100:intra-refresh=1:slice-max-size=1500:keyint=30:ref=1"
        << "-f flv "
        << '"' << path << '"';

    command = cmd.str();
    //ffmpeg = _popen(cmd.str().c_str(), "wb");

    if (!CreateCapturePipe())
    {
        std::cerr << "LibFFmpeg() - CreateCapturePipe failed!" << std::endl;
    }
    if (!CreateCaptureProcess())
    {
        std::cerr << "LibFFmpeg() - CreateCaptureProcess failed!" << std::endl;
    }
}

LibFFmpeg::~LibFFmpeg()
{
    CloseHandle(childInReadHandle);
}

bool LibFFmpeg::CreateCapturePipe()
{
    SECURITY_ATTRIBUTES saAttr;
    // Set the bInheritHandle flag so pipe handles are inherited. 
    saAttr.nLength = sizeof(SECURITY_ATTRIBUTES);
    saAttr.bInheritHandle = TRUE;
    saAttr.lpSecurityDescriptor = NULL;

    // Create a pipe for the child process's STDIN. 
    if (!CreatePipe(&childInReadHandle, &childInWriteHandle, &saAttr, 0))
    {
        std::cerr << "CreateCapturePipe() - CreatePipe failed!" << std::endl;
        return false;
    }

    // Ensure the write handle to the pipe for STDIN is not inherited. 
    if (!SetHandleInformation(childInWriteHandle, HANDLE_FLAG_INHERIT, 0))
    {
        std::cerr << "CreateCapturePipe() - SetHandleInformation failed!" << std::endl;
        return false;
    }
    return true;
}

bool LibFFmpeg::CreateCaptureProcess()
{
    TCHAR* cmdline = new TCHAR[command.size() + 1];
    cmdline[command.size()] = 0;
    std::copy(command.begin(), command.end(), cmdline);

    PROCESS_INFORMATION procInfo;
    STARTUPINFO startInfo;
    // Set up members of the PROCESS_INFORMATION structure.
    ZeroMemory(&procInfo, sizeof(PROCESS_INFORMATION));
    // Set up members of the STARTUPINFO structure. 
    // This structure specifies the STDIN and STDOUT handles for redirection.
    ZeroMemory(&startInfo, sizeof(STARTUPINFO));
    startInfo.cb = sizeof(STARTUPINFO);
    // Redirect child process's stdin to child_in_read_handle.
    startInfo.hStdInput = childInReadHandle;
    startInfo.dwFlags |= STARTF_USESTDHANDLES;

    // Create the child process.
    BOOL success = CreateProcess(
        NULL,
        cmdline,			    // command line 
        NULL,                   // process security attributes
        NULL,                   // primary thread security attributes
        TRUE,                   // handles are inherited
        CREATE_NO_WINDOW,       // creation flags
        NULL,                   // use parent's environment
        NULL,                   // use parent's current directory
        &startInfo,             // STARTUPINFO pointer
        &procInfo               // receives PROCESS_INFORMATION
    );
    // If an error occurs, exit the application. 
    if (!success)
    {
        std::cerr << "CreateCaptureProcess() - CreateProcess failed!" << std::endl;
        return false;
    }
    else
    {
        // Close handles to the child process and its primary thread.
        // Some applications might keep these handles to monitor the status
        // of the child process, for example. 
        CloseHandle(procInfo.hProcess);
        CloseHandle(procInfo.hThread);
    }
    return true;
}

void LibFFmpeg::WriteFrame(unsigned char* data, int size)
{
    DWORD dwWritten;
    if (!WriteFile(childInWriteHandle, data, size, &dwWritten, NULL))
    {
        std::cerr << "WriteFrame() - WriteFile failed!" << std::endl;
    }
    std::cout << "WriteFrame: " << dwWritten << " bytes." << std::endl;
}

void LibFFmpeg::Close()
{
    BOOL success = CloseHandle(childInWriteHandle);

    if (!success)
    {
        std::cerr << "Close() - CloseHandle failed!" << std::endl;
    }
}