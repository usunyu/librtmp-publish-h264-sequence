#include "stdafx.h"
#include "LibRTMP.h"
#include <iostream>
#include <math.h>

/*
 * Constructor.
 */
LibRTMP::LibRTMP()
{
    file_ = NULL;
}

/*
 * Read 1 byte.
 *
 * @param u8 address of data.
 * @oaram fp read file.
 *
 * @return 1 if success, 0 if failed.
 */
int LibRTMP::readU8(uint32_t* u8, FILE* fp)
{
    if (fread(u8, 1, 1, fp) != 1)
        return 0;
    return 1;
}

/*
 * Read 2 byte.
 *
 * @param u16 address of data.
 * @oaram fp read file.
 *
 * @return 1 if success, 0 if failed.
 */
int LibRTMP::readU16(uint32_t* u16, FILE* fp)
{
    if (fread(u16, 2, 1, fp) != 1)
        return 0;
    *u16 = HTON16(*u16);
    return 1;
}

/*
 * Read 3 byte.
 *
 * @param u24 address of data.
 * @oaram fp read file.
 *
 * @return 1 if success, 0 if failed.
 */
int LibRTMP::readU24(uint32_t* u24, FILE* fp)
{
    if (fread(u24, 3, 1, fp) != 1)
        return 0;
    *u24 = HTON24(*u24);
    return 1;
}

/*
 * Read 4 byte.
 *
 * @param u32 address of data.
 * @oaram fp read file.
 *
 * @return 1 if success, 0 if failed.
 */
int LibRTMP::readU32(uint32_t* u32, FILE* fp)
{
    if (fread(u32, 4, 1, fp) != 1)
        return 0;
    *u32 = HTON32(*u32);
    return 1;
}

/*
 * Read 1 byte, and loopback 1 byte at once.
 *
 * @param u8 address of data.
 * @oaram fp read file.
 *
 * @return 1 if success, 0 if failed.
 */
int LibRTMP::peekU8(uint32_t* u8, FILE* fp)
{
    if (fread(u8, 1, 1, fp) != 1)
        return 0;
    fseek(fp, -1, SEEK_CUR);
    return 1;
}

/*
 * Read 4 byte and convert to time format.
 *
 * @param utime address of data.
 * @oaram fp read file.
 *
 * @return 1 if success, 0 if failed.
 */
int LibRTMP::readTime(uint32_t* utime, FILE* fp)
{
    if (fread(utime, 4, 1, fp) != 1)
        return 0;
    *utime = HTONTIME(*utime);
    return 1;
}

/*
 * Init winsock.
 *
 * @return true if success, false if failed.
 */
bool LibRTMP::initSocket()
{
#ifdef WIN32
    WORD version;
    WSADATA wsaData;
    version = MAKEWORD(1, 1);
    return (WSAStartup(version, &wsaData) == 0);
#else     
    return TRUE;
#endif
}

/*
 * Init winsocks.
 *
 * @return true if success, false if failed.
 */
bool LibRTMP::initSockets()
{
#ifdef WIN32
    WORD version;
    WSADATA wsaData;
    version = MAKEWORD(2, 2);
    return (WSAStartup(version, &wsaData) == 0);
#else     
    return TRUE;
#endif
}

/*
 * Read buffer.
 *
 * @param buf the buffer address.
 * @param bufSize the buffer size.
 *
 * @return the buffer size read.
 */
int LibRTMP::readBuffer(unsigned char* buf, int bufSize)
{
    if (!feof(file_))
    {
        int true_size = fread(buf, 1, bufSize, file_);
        return true_size;
    }
    else
    {
        return -1;
    }
}

/*
 * Initial and connect to server with h264 video file.
 *
 * @param url the streaming server address.
 * @param videoFile h264 file for streaming.
 *
 * @return true if success, false if failed.
 */
bool LibRTMP::connectRTMPWithH264(const TCHAR* url, const TCHAR* videoFile)
{
    char videoFileName[_MAX_PATH];
    WideCharToMultiByte(CP_UTF8, 0, videoFile, -1, videoFileName, _MAX_PATH, NULL, NULL);

    char urlName[_MAX_PATH];
    WideCharToMultiByte(CP_UTF8, 0, url, -1, urlName, _MAX_PATH, NULL, NULL);
    file_ = fopen(videoFileName, "rb");
    if (file_ == NULL)
    {
        std::cout << "Read file failed" << std::endl;
    }
    //std::cout << "Read file success" << std::endl;

    headPos_ = 0;
    fileBufferSize_ = BUFFER_SIZE;
    fileBuffer_ = (unsigned char*)malloc(BUFFER_SIZE);
    tmpFileBuffer_ = (unsigned char*)malloc(BUFFER_SIZE);
    initSocket();

    rtmp_ = RTMP_Alloc();
    RTMP_Init(rtmp_);

    // Set server url.
    if (RTMP_SetupURL(rtmp_, (char*)urlName) == FALSE)
    {
        RTMP_Free(rtmp_);
        return false;
    }

    // Set enable write, e.g publish stream. Must called before connect.
    RTMP_EnableWrite(rtmp_);

    // Connect server.
    if (RTMP_Connect(rtmp_, NULL) == FALSE)
    {
        RTMP_Free(rtmp_);
        std::cout << "RTMP_Connect failed" << std::endl;
        return false;
    }
    //std::cout << "RTMP_Connect success" << std::endl;

    int retTest = RTMP_IsConnected(rtmp_);

    // Connect stream.
    if (RTMP_ConnectStream(rtmp_, 0) == FALSE)
    {
        RTMP_Close(rtmp_);
        RTMP_Free(rtmp_);
        std::cout << "RTMP_ConnectStream failed" << std::endl;
        return false;
    }
    //std::cout << "RTMP_ConnectStream success" << std::endl;

    int retTest2 = RTMP_IsConnected(rtmp_);

    //std::cout << "Start to send h264 streaming data..." << std::endl;
    sendH264StreamingData(&LibRTMP::readBuffer);

    return true;
}

/*
 * Read first nal unit from memory.
 *
 * @param nalu nal unit data.
 *
 * @return true if success, false if failed.
 */
int LibRTMP::readFirstNaluFromBuf(NaluUnit &nalu)
{
    int tailPos = headPos_;
    memset(tmpFileBuffer_, 0, BUFFER_SIZE);
    while (headPos_ < fileBufferSize_)
    {
        // Search for nal header.
        if (fileBuffer_[headPos_++] == 0x00 &&
            fileBuffer_[headPos_++] == 0x00)
        {
            if (fileBuffer_[headPos_++] == 0x01)
                goto gotnal_head;
            else
            {
                // Cuz we have done an i++ before,so we need to roll back now.
                headPos_--;
                if (fileBuffer_[headPos_++] == 0x00 &&
                    fileBuffer_[headPos_++] == 0x01)
                    goto gotnal_head;
                else
                    continue;
            }
        }
        else
            continue;

        // Search for nal tail which is also the head of next nal.
        gotnal_head:
        // Normal case: the whole nal is in this m_pFileBuf.
        tailPos = headPos_;
        while (tailPos < fileBufferSize_)
        {
            if (fileBuffer_[tailPos++] == 0x00 &&
                fileBuffer_[tailPos++] == 0x00)
            {
                if (fileBuffer_[tailPos++] == 0x01)
                {
                    nalu.size = (tailPos - 3) - headPos_;
                    break;
                }
                else
                {
                    tailPos--;
                    if (fileBuffer_[tailPos++] == 0x00 &&
                        fileBuffer_[tailPos++] == 0x01)
                    {
                        nalu.size = (tailPos - 4) - headPos_;
                        break;
                    }
                }
            }
        }

        nalu.type = fileBuffer_[headPos_] & 0x1f;
        memcpy(tmpFileBuffer_, fileBuffer_ + headPos_, nalu.size);
        nalu.data = tmpFileBuffer_;
        headPos_ = tailPos;
        return true;
    }
}

/*
 * Read one nal unit from memory.
 *
 * @param nalu nal unit data.
 * @param read_buffer callback function, when need more data, system will call this function to input data.
 *                      @param buf external data to this address.
 *                      @param buf_size external data szie.
 *
 *                      @return data size of read from memory.
 *
 * @return true if success, false if failed.
 */
int LibRTMP::readOneNaluFromBuf(NaluUnit &nalu, int(LibRTMP::*readBuffers)(uint8_t *buf, int buf_size))
{
    int tailPos = headPos_;
    int ret;
    int nalustart; // How many 00 in nal start identifier.
    memset(tmpFileBuffer_, 0, BUFFER_SIZE);
    nalu.size = 0;
    while (1)
    {
        if (headPos_ == NO_MORE_BUFFER_TO_READ)
            return false;
        while (tailPos < fileBufferSize_)
        {
            // Search for nal tail.
            if (fileBuffer_[tailPos++] == 0x00 &&
                fileBuffer_[tailPos++] == 0x00)
            {
                if (fileBuffer_[tailPos++] == 0x01)
                {
                    nalustart = 3;
                    goto gotnal;
                }
                else
                {
                    // Cuz we have done an i++ before, so we need to roll back now.
                    tailPos--;
                    if (fileBuffer_[tailPos++] == 0x00 &&
                        fileBuffer_[tailPos++] == 0x01)
                    {
                        nalustart = 4;
                        goto gotnal;
                    }
                    else
                        continue;
                }
            }
            else
                continue;

            gotnal:
            // Special case: parts of the nal lies in a m_pFileBuf and we have to read from buffer
            // again to get the rest part of this nal.			
            if (headPos_ == GOT_A_NAL_CROSS_BUFFER || headPos_ == GOT_A_NAL_INCLUDE_A_BUFFER)
            {
                nalu.size = nalu.size + tailPos - nalustart;
                if (nalu.size > BUFFER_SIZE)
                {
                    // Save pointer in case realloc fails.
                    prevTmpFileBuffer_ = tmpFileBuffer_;
                    if ((tmpFileBuffer_ = (unsigned char*)realloc(tmpFileBuffer_, nalu.size)) == NULL)
                    {
                        // Free original block.
                        free(prevTmpFileBuffer_);
                        return false;
                    }
                }
                memcpy(tmpFileBuffer_ + nalu.size + nalustart - tailPos, fileBuffer_, tailPos - nalustart);
                nalu.data = tmpFileBuffer_;
                headPos_ = tailPos;
                return true;
            }
            // Normal case: the whole nal is in this m_pFileBuf.
            else
            {
                nalu.type = fileBuffer_[headPos_] & 0x1f;
                nalu.size = tailPos - headPos_ - nalustart;
                if (nalu.type == 0x06)
                {
                    headPos_ = tailPos;
                    continue;
                }
                memcpy(tmpFileBuffer_, fileBuffer_ + headPos_, nalu.size);
                nalu.data = tmpFileBuffer_;
                headPos_ = tailPos;
                return true;
            }
        }

        if (tailPos >= fileBufferSize_ && headPos_ != GOT_A_NAL_CROSS_BUFFER && headPos_ != GOT_A_NAL_INCLUDE_A_BUFFER)
        {
            nalu.size = BUFFER_SIZE - headPos_;
            nalu.type = fileBuffer_[headPos_] & 0x1f;
            memcpy(tmpFileBuffer_, fileBuffer_ + headPos_, nalu.size);

            if ((ret = (this->*readBuffers)(fileBuffer_, fileBufferSize_)) < BUFFER_SIZE)
            {
                memcpy(tmpFileBuffer_ + nalu.size, fileBuffer_, ret);
                nalu.size = nalu.size + ret;
                nalu.data = tmpFileBuffer_;
                headPos_ = NO_MORE_BUFFER_TO_READ;
                return false;
            }
            tailPos = 0;
            headPos_ = GOT_A_NAL_CROSS_BUFFER;
            continue;
        }
        if (headPos_ == GOT_A_NAL_CROSS_BUFFER || headPos_ == GOT_A_NAL_INCLUDE_A_BUFFER)
        {
            nalu.size = BUFFER_SIZE + nalu.size;
            // Save pointer in case realloc fails.
            prevTmpFileBuffer_ = tmpFileBuffer_;
            if ((tmpFileBuffer_ = (unsigned char*)realloc(tmpFileBuffer_, nalu.size)) == NULL)
            {
                // Free original block.
                free(prevTmpFileBuffer_);
                return false;
            }

            memcpy(tmpFileBuffer_ + nalu.size - BUFFER_SIZE, fileBuffer_, BUFFER_SIZE);

            if ((ret = (this->*readBuffers)(fileBuffer_, fileBufferSize_)) < BUFFER_SIZE)
            {
                memcpy(tmpFileBuffer_ + nalu.size, fileBuffer_, ret);
                nalu.size = nalu.size + ret;
                nalu.data = tmpFileBuffer_;
                headPos_ = NO_MORE_BUFFER_TO_READ;
                return false;
            }
            tailPos = 0;
            headPos_ = GOT_A_NAL_INCLUDE_A_BUFFER;
            continue;
        }
    }
    return 0;
}

/*
 * Sned h264 streaming data with RTMP to server.
 *
 * @param nalu nal unit data.
 * @param read_buffer callback function, when need more data, system will call this function to input data.
 *                      @param buf external data to this address.
 *                      @param buf_size external data szie.
 *
 *                      @return data size of read from memory.
 *
 * @return true if success, false if failed.
 */
bool LibRTMP::sendH264StreamingData(int(LibRTMP::*readBuffers)(unsigned char *buf, int buf_size))
{
    int ret;
    uint32_t now, last_update;

    memset(&metaData_, 0, sizeof(RTMPMetaData));
    memset(fileBuffer_, 0, BUFFER_SIZE);
    if ((ret = (this->*readBuffers)(fileBuffer_, fileBufferSize_)) < 0)
    {
        std::cout << "No buffer data available" << std::endl;
        return false;
    }

    NaluUnit naluUnit;
    // Read sps frame.
    readFirstNaluFromBuf(naluUnit);
    metaData_.nSpsLen = naluUnit.size;
    metaData_.Sps = NULL;
    metaData_.Sps = (unsigned char*)malloc(naluUnit.size);
    memcpy(metaData_.Sps, naluUnit.data, naluUnit.size);
    // Read pps frame.
    readOneNaluFromBuf(naluUnit, &LibRTMP::readBuffer);
    metaData_.nPpsLen = naluUnit.size;
    metaData_.Pps = NULL;
    metaData_.Pps = (unsigned char*)malloc(naluUnit.size);
    memcpy(metaData_.Pps, naluUnit.data, naluUnit.size);
    // Decode sps, get image width and height.
    int width = 0, height = 0, fps = 0;
    h264Decode(metaData_.Sps, metaData_.nSpsLen, width, height, fps);

    if (fps)
        metaData_.frameRate = fps;
    else
        metaData_.frameRate = 25;

    unsigned int tick = 0;
    unsigned int tick_gap = 1000 / metaData_.frameRate;
    readOneNaluFromBuf(naluUnit, &LibRTMP::readBuffer);
    int keyframe = (naluUnit.type == 0x05) ? TRUE : FALSE;
    char buffer[50];
    while (sendH264Packet(naluUnit.data, naluUnit.size, keyframe, tick))
    {
        got_sps_pps:
        //if(naluUnit.size==8581)
        sprintf(buffer, "NALU size:%8d\n", naluUnit.size);
        OutputDebugStringA(buffer);
        //printf("NALU size:%8d\n", naluUnit.size);
        last_update = RTMP_GetTime();
        if (!readOneNaluFromBuf(naluUnit, &LibRTMP::readBuffer))
            goto end;
        if (naluUnit.type == 0x07 || naluUnit.type == 0x08)
            goto got_sps_pps;
        keyframe = (naluUnit.type == 0x05) ? TRUE : FALSE;
        tick += tick_gap;
        now = RTMP_GetTime();
        msleep(tick_gap - now + last_update);
    }
    end:
    free(metaData_.Sps);
    free(metaData_.Pps);

    return true;
}

/*
 * Decode sps, get image width and height.
 *
 * @param buf sps content.
 * @param length sps length.
 * @param width image width.
 * @param height image height.
 *
 * @return true if success, false if failed.
 */
bool LibRTMP::h264Decode(BYTE * buf, unsigned int length, int &width, int &height, int &fps)
{
    UINT startBit = 0;
    fps = 0;
    deEmulationPrevention(buf, &length);

    int forbidden_zero_bit = u(1, buf, startBit);
    int nal_ref_idc = u(2, buf, startBit);
    int nal_unit_type = u(5, buf, startBit);
    if (nal_unit_type == 7)
    {
        int profile_idc = u(8, buf, startBit);
        int constraint_set0_flag = u(1, buf, startBit); // (buf[1] & 0x80)>>7;
        int constraint_set1_flag = u(1, buf, startBit); // (buf[1] & 0x40)>>6;
        int constraint_set2_flag = u(1, buf, startBit); // (buf[1] & 0x20)>>5;
        int constraint_set3_flag = u(1, buf, startBit); // (buf[1] & 0x10)>>4;
        int reserved_zero_4bits = u(4, buf, startBit);
        int level_idc = u(8, buf, startBit);

        int seq_parameter_set_id = Ue(buf, length, startBit);

        if (profile_idc == 100 || profile_idc == 110 ||
            profile_idc == 122 || profile_idc == 144)
        {
            int chroma_format_idc = Ue(buf, length, startBit);
            if (chroma_format_idc == 3)
                int residual_colour_transform_flag = u(1, buf, startBit);
            int bit_depth_luma_minus8 = Ue(buf, length, startBit);
            int bit_depth_chroma_minus8 = Ue(buf, length, startBit);
            int qpprime_y_zero_transform_bypass_flag = u(1, buf, startBit);
            int seq_scaling_matrix_present_flag = u(1, buf, startBit);

            int seq_scaling_list_present_flag[8];
            if (seq_scaling_matrix_present_flag)
            {
                for (int i = 0; i < 8; i++)
                {
                    seq_scaling_list_present_flag[i] = u(1, buf, startBit);
                }
            }
        }
        int log2_max_frame_num_minus4 = Ue(buf, length, startBit);
        int pic_order_cnt_type = Ue(buf, length, startBit);
        if (pic_order_cnt_type == 0)
            int log2_max_pic_order_cnt_lsb_minus4 = Ue(buf, length, startBit);
        else if (pic_order_cnt_type == 1)
        {
            int delta_pic_order_always_zero_flag = u(1, buf, startBit);
            int offset_for_non_ref_pic = Se(buf, length, startBit);
            int offset_for_top_to_bottom_field = Se(buf, length, startBit);
            int num_ref_frames_in_pic_order_cnt_cycle = Ue(buf, length, startBit);

            int *offset_for_ref_frame = new int[num_ref_frames_in_pic_order_cnt_cycle];
            for (int i = 0; i < num_ref_frames_in_pic_order_cnt_cycle; i++)
                offset_for_ref_frame[i] = Se(buf, length, startBit);
            delete[] offset_for_ref_frame;
        }
        int num_ref_frames = Ue(buf, length, startBit);
        int gaps_in_frame_num_value_allowed_flag = u(1, buf, startBit);
        int pic_width_in_mbs_minus1 = Ue(buf, length, startBit);
        int pic_height_in_map_units_minus1 = Ue(buf, length, startBit);

        width = (pic_width_in_mbs_minus1 + 1) * 16;
        height = (pic_height_in_map_units_minus1 + 1) * 16;

        int frame_mbs_only_flag = u(1, buf, startBit);
        if (!frame_mbs_only_flag)
            int mb_adaptive_frame_field_flag = u(1, buf, startBit);

        int direct_8x8_inference_flag = u(1, buf, startBit);
        int frame_cropping_flag = u(1, buf, startBit);
        if (frame_cropping_flag)
        {
            int frame_crop_left_offset = Ue(buf, length, startBit);
            int frame_crop_right_offset = Ue(buf, length, startBit);
            int frame_crop_top_offset = Ue(buf, length, startBit);
            int frame_crop_bottom_offset = Ue(buf, length, startBit);
        }
        int vui_parameter_present_flag = u(1, buf, startBit);
        if (vui_parameter_present_flag)
        {
            int aspect_ratio_info_present_flag = u(1, buf, startBit);
            if (aspect_ratio_info_present_flag)
            {
                int aspect_ratio_idc = u(8, buf, startBit);
                if (aspect_ratio_idc == 255)
                {
                    int sar_width = u(16, buf, startBit);
                    int sar_height = u(16, buf, startBit);
                }
            }
            int overscan_info_present_flag = u(1, buf, startBit);
            if (overscan_info_present_flag)
                int overscan_appropriate_flagu = u(1, buf, startBit);
            int video_signal_type_present_flag = u(1, buf, startBit);
            if (video_signal_type_present_flag)
            {
                int video_format = u(3, buf, startBit);
                int video_full_range_flag = u(1, buf, startBit);
                int colour_description_present_flag = u(1, buf, startBit);
                if (colour_description_present_flag)
                {
                    int colour_primaries = u(8, buf, startBit);
                    int transfer_characteristics = u(8, buf, startBit);
                    int matrix_coefficients = u(8, buf, startBit);
                }
            }
            int chroma_loc_info_present_flag = u(1, buf, startBit);
            if (chroma_loc_info_present_flag)
            {
                int chroma_sample_loc_type_top_field = Ue(buf, length, startBit);
                int chroma_sample_loc_type_bottom_field = Ue(buf, length, startBit);
            }
            int timing_info_present_flag = u(1, buf, startBit);
            if (timing_info_present_flag)
            {
                int num_units_in_tick = u(32, buf, startBit);
                int time_scale = u(32, buf, startBit);
                fps = time_scale / (2 * num_units_in_tick);
            }
        }
        return true;
    }
    else
        return false;

}

/*
 * H264 startcode emulation prevention mechanism.
 *
 * @param buf sps content.
 */
void LibRTMP::deEmulationPrevention(BYTE* buf, unsigned int* buf_size)
{
    int i = 0, j = 0;
    BYTE* tmpPtr = NULL;
    unsigned int tmpBufSize = 0;
    int val = 0;

    tmpPtr = buf;
    tmpBufSize = *buf_size;
    for (i = 0; i < (tmpBufSize - 2); i++)
    {
        // Check for 0x000003.
        val = (tmpPtr[i] ^ 0x00) + (tmpPtr[i + 1] ^ 0x00) + (tmpPtr[i + 2] ^ 0x03);
        if (val == 0)
        {
            // Kick out 0x03.
            for (j = i + 2; j < tmpBufSize - 1; j++)
                tmpPtr[j] = tmpPtr[j + 1];

            // And so we should devrease bufsize.
            (*buf_size)--;
        }
    }

    return;
}

DWORD LibRTMP::u(UINT BitCount, BYTE * buf, UINT &startBit)
{
    DWORD dwRet = 0;
    for (UINT i = 0; i < BitCount; i++)
    {
        dwRet <<= 1;
        if (buf[startBit / 8] & (0x80 >> (startBit % 8)))
        {
            dwRet += 1;
        }
        startBit++;
    }
    return dwRet;
}

int LibRTMP::Se(BYTE *pBuff, UINT length, UINT &startBit)
{
    int UeVal = Ue(pBuff, length, startBit);
    double k = UeVal;
    int nValue = ceil(k / 2); // ceil(2)=ceil(1.2)=ceil(1.5)=2.00
    if (UeVal % 2 == 0)
        nValue = -nValue;
    return nValue;
}

UINT LibRTMP::Ue(BYTE *pBuff, UINT length, UINT &startBit)
{
    // Count 0bit num
    UINT zeroNum = 0;
    while (startBit < length * 8)
    {
        if (pBuff[startBit / 8] & (0x80 >> (startBit % 8)))
        {
            break;
        }
        zeroNum++;
        startBit++;
    }
    startBit++;

    // Calculate result
    DWORD dwRet = 0;
    for (UINT i = 0; i < zeroNum; i++)
    {
        dwRet <<= 1;
        if (pBuff[startBit / 8] & (0x80 >> (startBit % 8)))
        {
            dwRet += 1;
        }
        startBit++;
    }
    return (1 << zeroNum) - 1 + dwRet;
}

/*
 * Send h264 frame packet.
 *
 * @param data frame data.
 * @param size frame size.
 * @param isKeyFrame if the frame is key frame.
 * @param timeStamp current frame time stamp.
 *
 * @return true if success, false if failed.
 */
int LibRTMP::sendH264Packet(unsigned char *data, unsigned int size, int isKeyFrame, unsigned int timeStamp)
{
    if (data == NULL && size < 11)
    {
        return false;
    }

    unsigned char *body = (unsigned char*)malloc(size + 9);
    memset(body, 0, size + 9);

    int i = 0;
    if (isKeyFrame)
    {
        body[i++] = 0x17; // 1:Iframe  7:AVC   
        body[i++] = 0x01; // AVC NALU   
        body[i++] = 0x00;
        body[i++] = 0x00;
        body[i++] = 0x00;

        // NALU size   
        body[i++] = size >> 24 & 0xff;
        body[i++] = size >> 16 & 0xff;
        body[i++] = size >> 8 & 0xff;
        body[i++] = size & 0xff;
        // NALU data   
        memcpy(&body[i], data, size);
        sendVideoSpsPps(metaData_.Pps, metaData_.nPpsLen, metaData_.Sps, metaData_.nSpsLen);
    }
    else
    {
        body[i++] = 0x27; // 2:Pframe  7:AVC   
        body[i++] = 0x01; // AVC NALU   
        body[i++] = 0x00;
        body[i++] = 0x00;
        body[i++] = 0x00;

        // NALU size   
        body[i++] = size >> 24 & 0xff;
        body[i++] = size >> 16 & 0xff;
        body[i++] = size >> 8 & 0xff;
        body[i++] = size & 0xff;
        // NALU data   
        memcpy(&body[i], data, size);
    }

    int bRet = sendPacket(RTMP_PACKET_TYPE_VIDEO, body, i + size, timeStamp);

    free(body);

    return bRet;
}

/*
 * Send video sps and pps info.
 *
 * @param pps video pps data.
 * @param pps_len video pps length.
 * @param sps video sps data.
 * @param sps_len video sps length.
 *
 * @return true if success, or false if failed.
 */
int LibRTMP::sendVideoSpsPps(unsigned char *pps, int pps_len, unsigned char * sps, int sps_len)
{
    // rtmp struct
    RTMPPacket * packet = NULL;
    unsigned char * body = NULL;
    int i;
    packet = (RTMPPacket *)malloc(RTMP_HEAD_SIZE + 1024);
    //RTMPPacket_Reset(packet); // Reset packet status.
    memset(packet, 0, RTMP_HEAD_SIZE + 1024);
    packet->m_body = (char *)packet + RTMP_HEAD_SIZE;
    body = (unsigned char *)packet->m_body;
    i = 0;
    body[i++] = 0x17;
    body[i++] = 0x00;

    body[i++] = 0x00;
    body[i++] = 0x00;
    body[i++] = 0x00;

    // AVCDecoderConfigurationRecord
    body[i++] = 0x01;
    body[i++] = sps[1];
    body[i++] = sps[2];
    body[i++] = sps[3];
    body[i++] = 0xff;

    // sps
    body[i++] = 0xe1;
    body[i++] = (sps_len >> 8) & 0xff;
    body[i++] = sps_len & 0xff;
    memcpy(&body[i], sps, sps_len);
    i += sps_len;

    // pps
    body[i++] = 0x01;
    body[i++] = (pps_len >> 8) & 0xff;
    body[i++] = (pps_len) & 0xff;
    memcpy(&body[i], pps, pps_len);
    i += pps_len;

    packet->m_packetType = RTMP_PACKET_TYPE_VIDEO;
    packet->m_nBodySize = i;
    packet->m_nChannel = 0x04;
    packet->m_nTimeStamp = 0;
    packet->m_hasAbsTimestamp = 0;
    packet->m_headerType = RTMP_PACKET_SIZE_MEDIUM;
    packet->m_nInfoField2 = rtmp_->m_stream_id;

    // Send packet.
    int nRet = RTMP_SendPacket(rtmp_, packet, TRUE);
    // Release memory.
    free(packet);
    return nRet;
}

/*
 * Send RTMP packet.
 *
 * @param packetType packet type.
 * @param data packet content.
 * @param size packet size.
 * @param timeStamp current packet time stamp.
 *
 * @return true if success, false if failed.
 */
int LibRTMP::sendPacket(unsigned int packetType, unsigned char* data, unsigned int size, unsigned int timeStamp)
{
    RTMPPacket* packet;
    // Alloc packet memory and initialize.
    packet = (RTMPPacket*)malloc(RTMP_HEAD_SIZE + size);
    memset(packet, 0, RTMP_HEAD_SIZE);
    // Packet body.
    packet->m_body = (char*)packet + RTMP_HEAD_SIZE;
    packet->m_nBodySize = size;
    memcpy(packet->m_body, data, size);
    packet->m_hasAbsTimestamp = 0;
    // Two type available: video and audio.
    packet->m_packetType = packetType;
    packet->m_nInfoField2 = rtmp_->m_stream_id;
    packet->m_nChannel = 0x04;

    packet->m_headerType = RTMP_PACKET_SIZE_LARGE;
    if (RTMP_PACKET_TYPE_AUDIO == packetType && size != 4)
    {
        packet->m_headerType = RTMP_PACKET_SIZE_MEDIUM;
    }
    packet->m_nTimeStamp = timeStamp;
    // Send packet.
    int nRet = 0;
    if (RTMP_IsConnected(rtmp_))
    {
        nRet = RTMP_SendPacket(rtmp_, packet, TRUE);
    }
    // Relase packet.
    free(packet);
    return nRet;
}

/*
 * Disconnect and release resources.
 */
void LibRTMP::close()
{
    if (rtmp_)
    {
        RTMP_Close(rtmp_);
        RTMP_Free(rtmp_);
        rtmp_ = NULL;
    }

    // Clean up sockets.
    WSACleanup();

    if (fileBuffer_ != NULL)
    {
        free(fileBuffer_);
    }
    if (tmpFileBuffer_ != NULL)
    {
        free(tmpFileBuffer_);
    }
}