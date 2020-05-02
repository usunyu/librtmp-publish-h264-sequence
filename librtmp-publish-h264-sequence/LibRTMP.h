#pragma once
#include "rtmp/include/librtmp/rtmp_sys.h"
#include "rtmp/include/librtmp/rtmp.h"
#include "rtmp/include/librtmp/log.h"

// Save nal unit data buffer size
#define BUFFER_SIZE 32768	// 32k
// Flags for searching nal unit
#define GOT_A_NAL_CROSS_BUFFER BUFFER_SIZE + 1
#define GOT_A_NAL_INCLUDE_A_BUFFER BUFFER_SIZE + 2
#define NO_MORE_BUFFER_TO_READ BUFFER_SIZE + 3
// Define packet header size, RTMP_MAX_HEADER_SIZE=18
#define RTMP_HEAD_SIZE   (sizeof(RTMPPacket)+RTMP_MAX_HEADER_SIZE)

#define HTON16(x)  ((x>>8&0xff)|(x<<8&0xff00))
#define HTON24(x)  ((x>>16&0xff)|(x<<16&0xff0000)|(x&0xff00))
#define HTON32(x)  ((x>>24&0xff)|(x>>8&0xff00)|\
    (x<<8&0xff0000)|(x<<24&0xff000000))
#define HTONTIME(x) ((x>>16&0xff)|(x<<16&0xff0000)|(x&0xff00)|(x&0xff000000))

/*
 * RTMPMetadata struct, save RTMP data.
 */
typedef struct _RTMPMetadata
{
    unsigned int    width;
    unsigned int    height;
    unsigned int    frameRate;
    unsigned int    nSpsLen;
    unsigned char*  Sps;
    unsigned int    nPpsLen;
    unsigned char*  Pps;
} RTMPMetaData, *LPRTMPMetaData;

/*
 * NaluUnit struct, save nal unit type, size and data.
 */
typedef struct _NaluUnit
{
    int type;
    int size;
    unsigned char* data;
} NaluUnit;

class LibRTMP
{
public:
    /* Constructor. */
    LibRTMP();
    /* Initial and connect to server with h264 video file. */
    bool connectRTMPWithH264(const TCHAR* url, const TCHAR* videoFile);
    /* Initial and connect to server with flv video file. */
    bool connectRTMPWithFlv(const TCHAR* url, const TCHAR* videoFile);
    /* Disconnect and release resources. */
    void close();

private:
    /* Init winsock. */
    bool initSocket();
    /* Init winsocks. */
    bool initSockets();
    /* Read buffer. */
    int readBuffer(unsigned char* buf, int bufSize);
    /* Read first nal unit from memory. */
    int readFirstNaluFromBuf(NaluUnit &nalu);
    /* Read one nal unit from memory. */
    int readOneNaluFromBuf(NaluUnit &nalu, int(LibRTMP::*readBuffers)(uint8_t* buf, int buf_size));
    /* Decode sps, get image width and height. */
    bool h264Decode(BYTE* buf, unsigned int length, int &width, int &height, int &fps);
    /* Send h264 frame packet. */
    int sendH264Packet(unsigned char* data, unsigned int size, int isKeyFrame, unsigned int timeStamp);
    /* H264 startcode emulation prevention mechanism. */
    void deEmulationPrevention(BYTE* buf, unsigned int* buf_size);
    /* Send video sps and pps info. */
    int sendVideoSpsPps(unsigned char* pps, int pps_len, unsigned char* sps, int sps_len);
    /* Send RTMP packet. */
    int sendPacket(unsigned int packetType, unsigned char* data, unsigned int size, unsigned int timestamp);
    /* Sned h264 streaming data with RTMP to server. */
    bool sendH264StreamingData(int(LibRTMP::*readBuffers)(unsigned char* buf, int buf_size));

    DWORD u(UINT BitCount, BYTE* buf, UINT &startBit);
    int Se(BYTE* pBuff, UINT length, UINT &startBit);
    UINT Ue(BYTE* pBuff, UINT length, UINT &startBit);

    /* Read 1 byte. */
    int LibRTMP::readU8(uint32_t* u8, FILE* fp);
    /* Read 2 byte. */
    int LibRTMP::readU16(uint32_t* u16, FILE* fp);
    /* Read 3 byte. */
    int LibRTMP::readU24(uint32_t* u24, FILE* fp);
    /* Read 4 byte. */
    int LibRTMP::readU32(uint32_t* u32, FILE* fp);
    /* Read 1 byte, and loopback 1 byte at once. */
    int LibRTMP::peekU8(uint32_t* u8, FILE* fp);
    /* Read 4 byte and convert to time format. */
    int LibRTMP::readTime(uint32_t* utime, FILE* fp);

    RTMPMetaData metaData_;

    FILE* file_;
    RTMP* rtmp_;
    unsigned char* fileBuffer_;
    unsigned char* tmpFileBuffer_;
    /* Used for realloc. */
    unsigned char* prevTmpFileBuffer_;

    unsigned int fileBufferSize_;
    unsigned int headPos_;
};