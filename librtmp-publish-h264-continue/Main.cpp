// main.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "LibRTMP.h"
#include "LibFFmpeg.h"
#include <iostream> // library that contain basic input/output functions
#include <fstream>  // library that contains file input/output functions
#include <string>

//TCHAR* rtmp_url = TEXT("rtmp://txy.live-send.acg.tv/live-txy/?streamname=live_6235300_5849362&key=10fc91777ea6f60a940ab13fab01c7f5");
char* rtmp_url = "rtmp://txy.live-send.acg.tv/live-txy/?streamname=live_6235300_5849362&key=10fc91777ea6f60a940ab13fab01c7f5";
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

char* read_file_to_array(std::string input, int file_size)
{
    int array_size = 1024 * 100 * 8; // define the size of character array
    char* array_data = new char[array_size]; // allocating an array of 100 kB
    int position = 0; //this will be used incremently to fill characters in the array 

    std::ifstream fin(input); //opening an input stream for file
    // checking whether file could be opened or not. If file does not exist or don't have read permissions, file
    // stream could not be opened.
    if (!fin.is_open()) // file could not be opened
    {
        std::cerr << "File could not be opened." << std::endl;
        return nullptr;
    }
    // file opened successfully so we are here
    //std::cout << "File opened successfully. Reading data from file into array..." << std::endl;
    // this loop run until end of file (eof) does not occur
    while (position < file_size && position < array_size)
    {
        // reading one character from file to array
        fin.get(array_data[position]);
        position++;
    }
    array_data[position - 1] = '\0'; //placing character array terminating character

    //std::cout << "Displaying Array..." << std::endl;
    //// this loop display all the charaters in array till \0 
    //for (int i = 0; array[i] != '\0'; i++)
    //{
    //    std::cout << array[i];
    //}

    std::cout << "Read file successfully, read " << position << " bytes." << std::endl;
    return array_data;
}

unsigned char* read_file_to_array2(const char* input, int file_size)
{
    FILE * file = fopen(input, "r+");
    unsigned char* data = (unsigned char *) malloc(file_size);
    int bytes_read = fread(data, sizeof(unsigned char), file_size, file);
    fclose(file);

    return data;
}

int get_file_size(std::string input)
{
    std::streampos begin, end;
    std::ifstream myfile(input, std::ios::binary);
    begin = myfile.tellg();
    myfile.seekg(0, std::ios::end);
    end = myfile.tellg();
    myfile.close();
    return end - begin;
}

int get_file_size2(const char* input)
{
    FILE * file = fopen(input, "r+");
    if (file == NULL)
    {
        return 0;
    }
    fseek(file, 0, SEEK_END);
    long int size = ftell(file);
    fclose(file);
    return size;
}

int main()
{
    // Attemp 1: Try publish h264 list in a loop with librtmp.
    // Defect: Require reconnect when process the new video file.
    //LibRTMP* lib = new LibRTMP();
    //for (int i = 1; i <= VIDEO_COUNT; i++)
    //{
    //    std::string video_file = "h264/" + std::to_string(i) + ".h264";
    //    std::cout << "Start to publish " << video_file << "..." << std::endl;
    //    lib->connectRTMPWithH264(rtmp_url, string_to_tchar(video_file));
    //    lib->close();
    //}

    // Attemp 2: Try use namedpipe to send h264 to ffmpeg and publish.
    LibFFmpeg* lib = new LibFFmpeg(rtmp_url, "ffmpeg.exe");
    for (int i = 1; i <= VIDEO_COUNT; i++)
    {
        std::string video_file = "h264/" + std::to_string(i) + ".h264";
        //int size = get_file_size(video_file);
        int size = get_file_size2(video_file.c_str());
        std::cout << "Start to publish " << video_file << ": " << size << " bytes" << "..." << std::endl;
        //char* data = read_file_to_array(video_file, size);
        unsigned char* data = read_file_to_array2(video_file.c_str(), size);
        lib->WriteFrame(data, size);
    }
    lib->Close();

    std::cout << "Press Enter to Continue" << std::endl;
    std::cin.ignore();

    return 0;
}

