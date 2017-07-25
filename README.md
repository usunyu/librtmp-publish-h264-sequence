# librtmp-publish-h264-continue
Exploring publish H264 video file list to RTMP server

#### Attemp 1: Try publish h264 list in a loop with librtmp.
##### Defect: Require reconnect when process the new video file.
```
LibRTMP* lib = new LibRTMP();
for (int i = 1; i <= VIDEO_COUNT; i++)
{
    std::string video_file = "h264/" + std::to_string(i) + ".h264";
    std::cout << "Start to publish " << video_file << "..." << std::endl;
    lib->connectRTMPWithH264(rtmp_url, string_to_tchar(video_file));
    lib->close();
}
```

#### Attemp 2: Try use pipe to send h264 to ffmpeg and publish.
```
LibFFmpeg* lib = new LibFFmpeg(rtmp_url, "ffmpeg.exe");
for (int i = 1; i <= VIDEO_COUNT; i++)
{
    std::string video_file = "h264/" + std::to_string(i) + ".h264";
    int size = get_file_size2(video_file.c_str());
    std::cout << "Start to publish " << video_file << ": " << size << " bytes" << "..." << std::endl;
    unsigned char* data = read_file_to_array2(video_file.c_str(), size);
    lib->WriteFrame(data, size);
}
lib->Close();
```
