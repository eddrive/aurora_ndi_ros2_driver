#pragma once

#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <math.h>
#include <sstream>
#include <cstdio>
#include <unistd.h>
#include <vector>

typedef unsigned char uchar;

namespace AuroraDriver
{
    struct ToolPose
    {
        double qx;
        double qy;
        double qz;
        double qw;
        double x;
        double y;
        double z;

        ToolPose():
            qx(0), qy(0), qz(0), qw(0), x(0), y(0), z(0)
        {}
        ToolPose(const double& _qx, const double& _qy, const double& _qz, const double& _qw, const double& _x, const double& _y, const double& _z):
            qx(_qx), qy(_qy), qz(_qz), qw(_qw), x(_x), y(_y), z(_z)
        {}
    };

    class ndi_aurora
    {
      private:
        const char* _port;
        termios oldtio;
        struct termios newtio_struct;
        struct termios *newtio;
        int fd;
        int fd_global;
        speed_t baudrate;

        char returnMessage[255];
        char returnMessageCopy[255];
        char buffer[255];
        char *bufptr;
        size_t nmbBytesRead;
        const char *Output;

        char portHandle[2];
        char sign_refPortHandle[2];
        int PHint, PHint2, PHint3;
        std::string sPH, sPHStatus;
        int indDefFile, noPorts2Init, noPorts2Enable, noPortsEnabled;
        int noTxPorts, noTools;
        bool bDuplicate, bAutoConfig;
        char currentPrio;
        std::vector<std::string> activePHs;

        int i, j, k, cv4;

        std::string command;
        std::string message2send, errorMessage, missingMessage, warningMessage, replyOptionStr;
        std::ostringstream tempInt2string;
        std::stringstream tempChar2int;
        std::vector<std::string> toolDefinitions;
        std::string priority, string2convert;
        int intFromChar;
        char char2convert;
        int int2convert;

        char* toolDefinition;
        unsigned char toolDefData[1024];
        int nBytes;
        FILE *romFile;
        char tooldefCommand[256];
        char* romFileName;

        int trackReplyOption;
        std::string measureReplyOption;
        int handleNumber, systemStatus;
        unsigned int replyOptionInt;
        int replyOption, messageLength;
        std::string tempString;
        char tempChar;
        int startIndex;
        int checkReference, realHandleNumber, PHcounter;
        int noSM, outVolInt;

        ToolPose toolPose;
        std::vector<ToolPose> allToolPose;
        std::vector<int> portIndexes;
        std::vector<int> visibleTools;

        int tempQuaternion;
        double quaternion0, quaternionX, quaternionY, quaternionZ;
        double positionX, positionY, positionZ, errorRMS;
        int frameNumber, tempFno;
        std::string newLineCmd;
        bool statusReply;

        const int CLcr, CLcrc, CLnoPH, CLPHno, CLPHstat, CLstatus;
        const int CLquaternion, CLposition, CLerrorRMS, CLlf;
        const int CLportStatus, CLframeNo, CLnoSM;
        const int CLOutVol, CLSM;

        bool sendF, readF, missingF, interpretOK, frameNoF, extraTransfo;

        bool initSerial(const char *serialdev, struct termios *oldtio, int& fd);
        bool resetAurora(const int& fd, struct termios *oldtio);
        bool portHandleStatus(const int& replyOption, char (&returnMessage)[255]);
        bool portHandleFree(char(&portHandle)[2]);
        bool loadToolDefFile(char (&portHandle)[2], char* toolDefinition);
        bool portHandleInit(char(&portHandle)[2]);
        bool defaultToolConfig(char(&portHandle)[2]);
        bool portHandleEnable(char(&portHandle)[2], char priority);
        bool interpretMessage0001(int& startIndex, ToolPose& toolPose, double& errorRMS, int& handleNumber, const bool& statusReply, bool& missingF);
        bool readQuaternion(int& startIndex, double& quaternion);
        bool readPosition(int& startIndex, double& position);
        bool readError(int& startIndex, double& errorRMS);
        bool readPortStatus(int& startIndex, int& handleNumber);
        bool readFrameNumber(int& startIndex, int& frameNumber);
        bool checkNewLine(int& startIndex);
        bool searchNewLine(int& startIndex, const int& messageLength);
        bool int2string(const int& int2convert, std::string& stringFromInt);
        bool char2int(const char& char2convert, int& intFromChar);
        bool char2hex(const char& char2convert, int& intFromChar);
        bool string2int(const std::string& string2convert, unsigned int& intFromChar);
        bool string2hex(const std::string& string2convert, unsigned int& intFromChar);

      public:
        ndi_aurora(const char* port) : 
            _port(port),
            newtio(nullptr),
            fd(0),
            fd_global(0),
            baudrate(B9600),
            bufptr(nullptr),
            nmbBytesRead(0),
            Output(nullptr),
            PHint(0), PHint2(0), PHint3(0),
            indDefFile(-1),
            noPorts2Init(0), noPorts2Enable(0), noPortsEnabled(0),
            noTxPorts(0), noTools(0),
            bDuplicate(false), bAutoConfig(false),
            currentPrio('D'),
            i(0), j(0), k(0), cv4(0),
            errorMessage("ERROR"), missingMessage("MISSING"), warningMessage("WARNING"),
            intFromChar(0), char2convert('\0'), int2convert(0),
            toolDefinition(nullptr), nBytes(0), romFile(nullptr), romFileName(nullptr),
            trackReplyOption(0),
            handleNumber(0), systemStatus(0), replyOptionInt(0),
            replyOption(0), messageLength(0),
            tempChar('\0'), startIndex(0),
            checkReference(0), realHandleNumber(0), PHcounter(0),
            noSM(0), outVolInt(0),
            tempQuaternion(0),
            quaternion0(0.0), quaternionX(0.0), quaternionY(0.0), quaternionZ(0.0),
            positionX(0.0), positionY(0.0), positionZ(0.0), errorRMS(0.0),
            frameNumber(0), tempFno(0),
            statusReply(false),
            CLcr(1), CLcrc(4), CLnoPH(2), CLPHno(2), CLPHstat(3), CLstatus(4),
            CLquaternion(6), CLposition(7), CLerrorRMS(6), CLlf(1),
            CLportStatus(8), CLframeNo(8), CLnoSM(2),
            CLOutVol(0), CLSM(0),
            sendF(true), readF(false), missingF(false), interpretOK(false),
            frameNoF(false), extraTransfo(false)
        {
            memset(returnMessage, 0, sizeof(returnMessage));
            memset(returnMessageCopy, 0, sizeof(returnMessageCopy));
            memset(buffer, 0, sizeof(buffer));
            memset(portHandle, 0, sizeof(portHandle));
            memset(sign_refPortHandle, 0, sizeof(sign_refPortHandle));
            memset(toolDefData, 0, sizeof(toolDefData));
            memset(tooldefCommand, 0, sizeof(tooldefCommand));
            memset(&oldtio, 0, sizeof(oldtio));
            memset(&newtio_struct, 0, sizeof(newtio_struct));
        }

        ~ndi_aurora()
        {
            if (romFile) {
                fclose(romFile);
                romFile = nullptr;
            }
        }

        bool startSerial();
        bool stopSerial();
        bool sendBeep();
        bool sendMessage(const int& fd, const char *msg, const int& len);
        bool readMessage(const int& fd, char (&msg)[255]);
        bool changeBaudRate(const int& BaudRate);
        bool changePCBaudRate(const char *serialdev, const speed_t& PcBaudRate);
        bool initAurora();
        bool stopAurora(const int& fd);
        bool getFd(int& fd);
        bool initToolVector(const int& noTools);
        bool initToolVector();
        bool getNoToolsReady(int& noToolsReady);
        bool initPortHandleWT(const std::vector<std::string>& toolDefinitions,
                             const std::vector<std::string>& toolDefPortHandles,
                             const std::vector<std::string>& defaultToolsConfiguration,
                             const std::string& refPortHandle);
        bool initPortHandleWT(const std::vector<std::string>& toolDefinitions,
                             const std::vector<std::string>& toolDefPortHandles);
        bool startDiagnosticMode();
        bool stopDiagnosticMode();
        bool startTrackingMode(const int& trackReplyOption);
        bool stopTrackingMode();
        bool measureTool(const std::string& measureReplyOption,
                        std::vector<ToolPose>& allToolPose,
                        const bool& statusReply,
                        std::vector<int>& portIndexes,
                        std::vector<int>& visibleTools);
    };
}