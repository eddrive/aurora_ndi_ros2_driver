#include "aurora_ndi_ros2_driver/ndi_aurora_ros2.hpp"
#include <unistd.h>
#include <iostream>
#include <cstring>

#define NO_AURORA_STDOUT

namespace AuroraDriver
{
    bool ndi_aurora::initSerial(const char *serialdev, struct termios *oldtio, int& fd)
    {
        newtio = &newtio_struct;

        fd = open(serialdev, O_RDWR | O_NOCTTY);
        if (fd < 0)
        {
            std::cout << "(ndi_aurora) IOException. Not able to open: " << serialdev << std::endl;
            return false;
        }

        tcgetattr(fd, oldtio); 

        //after power up, the camera will reset to 9600bps
        memset(newtio, 0, sizeof(struct termios));
        newtio->c_cflag = B9600 | CS8 | CLOCAL | CREAD;
        newtio->c_iflag = IGNPAR;
        newtio->c_oflag = 0;

        // set input mode (non-canonical, no echo,...)
        newtio->c_lflag = 0;
        newtio->c_cc[VTIME] = 10;   // inter-character timer unused 
        newtio->c_cc[VMIN] = 255;   // blocking read until 1 chars received 
        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, newtio);

        // set the PC side as well
        close(fd);
        fd = open(serialdev, O_RDWR | O_NOCTTY);
        if (fd < 0) {
            std::cout << "ndi_aurora IOException PC side." << std::endl;
            return false;
        }
        newtio->c_cflag = B9600 | CS8 | CLOCAL | CREAD;
        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, newtio);
        fd_global = fd; //for getFd function
        
        return true;
    }

    bool ndi_aurora::startSerial()
    {
        if (!initSerial(_port, &oldtio, fd)) return false;
        tcsendbreak(fd, 0);
        if (!readMessage(fd, returnMessage)) return false;
        std::cout << "(ndi_aurora) returned message for serial break: " << returnMessage << "\n" << std::endl;

        return true;
    }
    
    bool ndi_aurora::stopSerial()
    {   
        bool ret1 = stopAurora(fd);
        bool ret2 = resetAurora(fd, &oldtio);
        return ret1 && ret2;
    }

    bool ndi_aurora::sendBeep()
    {
        std::string beepmessage = "BEEP 1\r";
        Output = beepmessage.c_str();
        bool sendF = sendMessage(fd, Output, strlen(Output));

        bool readF = readMessage(fd, returnMessage);
        std::cout << "(ndi_aurora) returned message for BEEP: " << returnMessage << "\n" << std::endl;

        if (strncmp(returnMessage, errorMessage.c_str(), errorMessage.length()) == 0)
        {
            std::cout << "(ndi_aurora) ERROR: Aurora BEEP failed: " << returnMessage << "\n" << std::endl;
            return false;
        }
        //return true if send and read were successfull
        return sendF && readF; 
    } 

    bool ndi_aurora::sendMessage(const int& fd, const char *msg, const int& len)
    {
        write(fd, (const void*)msg, len);
        return true;
    }

    bool ndi_aurora::readMessage(const int& fd, char (&msg)[255])
    {
        bufptr = msg;

        while ((nmbBytesRead = read(fd, bufptr, 1)) > 0)
        {
            bufptr += nmbBytesRead;
            if (bufptr[-1] == '\r')
            {
                break;
            }
        }

        *bufptr = '\0';
        return true;
    }

    bool ndi_aurora::changeBaudRate(const int& BaudRate)
    {
        sendF = true;

        switch(BaudRate)
        {
            case 9600:
                message2send = "COMM 00000\r";
                baudrate = B9600;
                break;
            
            case 19200:
                message2send = "COMM 20000\r";
                baudrate = B19200;
                break;

            case 38400:
                message2send = "COMM 30000\r";
                baudrate = B38400;
                break;

            case 57600:
                message2send = "COMM 40000\r";
                baudrate = B57600;
                break;

            case 115200:
                message2send = "COMM 50001\r";
                baudrate = B115200;
                break;

            case 230400:
                message2send = "COMM A0000\r";
                baudrate = B230400;
                break;

            default:
                std::cout << "(ndi_aurora) ERROR: " << BaudRate << " is not a supported baudrate!" << "\n" << std::endl;
                return false;
        }

        Output = message2send.c_str();
        sendF = sendMessage(fd, Output, strlen(Output));
        sendF = sendF && readMessage(fd, returnMessage);

        if (strncmp(returnMessage, errorMessage.c_str(), errorMessage.length()) == 0)
        {
            std::cout << "(ndi_aurora) ERROR: could not change baudrate: " << returnMessage << "\n" << std::endl;
            return false;
        }
        std::cout << "(ndi_aurora) baudrate change request send to camera: successful?: " << sendF << " ,message returned: " << returnMessage << "\n" << std::endl;

        usleep(500000);

        sendF = sendF && changePCBaudRate(_port, baudrate);
        std::cout << "(ndi_aurora) baudrate change request send to PC: successful?: " << sendF << "\n" << std::endl;
 
        return sendF;
    }

    bool ndi_aurora::changePCBaudRate(const char *serialdev, const speed_t& PcBaudRate)
    {
        close(fd);
        fd = open(serialdev, O_RDWR | O_NOCTTY);
        if (fd < 0) {
            std::cout << "ndi_aurora IOException." << std::endl;
            return false;
        }
        newtio->c_cflag = PcBaudRate | CS8 | CLOCAL | CREAD;
        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, newtio);
        return true;
    }

    bool ndi_aurora::initAurora()
    {
        sendF = true;
        message2send = "INIT \r";
        Output = message2send.c_str();
        sendF = sendMessage(fd, Output, strlen(Output));

        usleep(10000);

        sendF = sendF && readMessage(fd, returnMessage);

        if (strncmp(returnMessage, errorMessage.c_str(), errorMessage.length()) == 0)
        {
            std::cout << "(ndi_aurora) ERROR: initialization Aurora failed " << returnMessage << "\n" << std::endl;
            return false;
        }
        std::cout << "(ndi_aurora) initialization of Aurora: successful?: " << sendF << " ,message returned: " << returnMessage << "\n" << std::endl;

        return sendF;
    }

    bool ndi_aurora::resetAurora(const int& fd, struct termios *oldtio)
    {
        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, oldtio);
        close(fd);
        return true;
    }

    bool ndi_aurora::stopAurora(const int& fd)
    {
        (void)fd;
        noPortsEnabled = 0;
        activePHs.clear();
        allToolPose.clear();
        visibleTools.clear();
        return true;
    }
    
    bool ndi_aurora::getFd(int& fd)
    {
        fd = fd_global;
        return true; 
    }

    bool ndi_aurora::getNoToolsReady(int& noToolsReady)
    {
        noToolsReady = noPortsEnabled;
        return true;
    }

    bool ndi_aurora::initToolVector() {
        return initToolVector(noPortsEnabled);
    }

    bool ndi_aurora::initToolVector(const int& noTools)
    {
        allToolPose.reserve(noTools);
        visibleTools.reserve(noTools);
        portIndexes.reserve(noTools);
        return true;
    }

    //returns number of assigned port handles and status, assigns port handle to
    //wired tools, GPIO device or strober
    bool ndi_aurora::portHandleStatus(const int& replyOption, char(&returnMessage)[255])
    {
        sendF = true;
        replyOptionStr = "";
        message2send = "";
        //port handle status request command
        //convert int to string
        sendF = int2string(replyOption, replyOptionStr);
        //compose actual message
        message2send = "PHSR 0" + replyOptionStr + "\r";
        Output = message2send.c_str();
#ifndef NO_AURORA_STDOUT
        std::cout << "(nid_aurora) PHSR msg : " << message2send << std::endl;
#endif
        sendF = sendMessage(fd, Output, strlen(Output));
        sendF = sendF && readMessage(fd, returnMessage);        
        
        //check returnMessage
        if (strncmp(returnMessage, errorMessage.c_str(), errorMessage.length()) == 0)
        { 
            std::cout << "(ndi_aurora) ERROR: port handle status request failed " << returnMessage << "\n" << std::endl;
            return false;
        }
        std::cout << "(ndi_aurora) port handle status request: successful?: " << sendF << " ,message returned: " << returnMessage << "\n" << std::endl;
        
        return sendF;
    }
    
    //releases system resources from an unused port handle
    bool ndi_aurora::portHandleFree(char(&portHandle)[2])
    {
        sendF = true;
        //port handle free command
        //compose actual command
        sprintf(buffer, "PHF %c%c\r", portHandle[0], portHandle[1]);                                                                                      
        Output = &buffer[0];
        sendF = sendMessage(fd, Output, strlen(Output));
        sendF = sendF && readMessage(fd, returnMessage);        
        
        //check returnMessage
        if (strncmp(returnMessage, errorMessage.c_str(), errorMessage.length()) == 0)
        { 
            std::cout << "(ndi_aurora) ERROR: free port handle failed " << returnMessage << "\n" << std::endl;
            return false;
        }
        std::cout << "(ndi_aurora) free port handle request: successful?: " << sendF << " ,message returned: " << returnMessage << "\n" << std::endl;
        
        return sendF;
    }
    
    //load tool definition file
    bool ndi_aurora::loadToolDefFile(char (&portHandle)[2], char* toolDefinition)
    {
        sendF = true;

        //open tool definition file (.rom)
        if (!(romFile = fopen(toolDefinition, "rb"))) //rb=read binary
        {
            std::cout << "(ndi_aurora) ERROR: failed to open tool definintion file (.rom)" << "\n" << std::endl;
            if (romFile) { fclose(romFile); }
            return false;                                                                                                                                               
        } 

        // read the tool definition file
        if ((nBytes = fread(toolDefData, 1, sizeof(toolDefData), romFile)) < 1)
        {
            std::cout << "(ndi_aurora) ERROR: failed to read tool definintion file (.rom)" << "\n" << std::endl;
            if (romFile) { fclose(romFile); }
            return false;                                                                                                                                               
        } 

        //construct the command (port per port)
        for (j = 0; j < nBytes;)
        {
            sprintf(tooldefCommand, "PVWR %c%c%04X", portHandle[0], portHandle[1], j);
            for (i = 0; i < 64; i++, j++) //tooldefinition data = 128 hex characters = 64*2
            {
                sprintf(tooldefCommand + 11 + 2 * i, "%02X", toolDefData[j]); //10 = start message, 2 ascii characters/char
            } 

            sprintf(tooldefCommand + 139, "\r"); //don't forget the cariage return!    
            Output = &tooldefCommand[0];
            sendF = sendMessage(fd, Output, strlen(Output));
            sendF = sendF && readMessage(fd, returnMessage);        
        
            //check returnMessage
            if (strncmp(returnMessage, errorMessage.c_str(), errorMessage.length()) == 0)
            { 
                std::cout << "(ndi_aurora) ERROR: failed to load tool definition file" << returnMessage << "\n" << std::endl;
                return false;
            }
            std::cout << "(ndi_aurora) load tool definition file: successful?: " << sendF << " ,message returned: " << returnMessage << "\n" << std::endl;
        }

        //close the tool definition file
        if (romFile) { fclose(romFile); }
       
        return sendF;
    }

    //initialize port handles
    bool ndi_aurora::portHandleInit(char(&portHandle)[2])
    {
        sendF = true;
        //port handle init command
        //compose actual command

        sprintf(buffer, "PINIT %c%c\r", portHandle[0], portHandle[1]);
        Output = &buffer[0];
        sendF = sendMessage(fd, Output, strlen(Output));
        sendF = sendF && readMessage(fd, returnMessage);        

#ifndef NO_AURORA_STDOUT
        std::cout << "(ndi_aurora) Going to initializate port with command: " << buffer << std::endl;
#endif
        //check returnMessage
        if (strncmp(returnMessage, errorMessage.c_str(), errorMessage.length()) == 0)
        { 
            std::cout << "(ndi_aurora) ERROR: init port handle failed " << returnMessage << "\n" << std::endl;
            return false;
        }
        else if (strncmp(returnMessage, warningMessage.c_str(), warningMessage.length()) == 0)
        {
            std::cout << "(ndi_aurora) WARNING: init port handle warning: " << returnMessage << "\n" << std::endl;
            return false;                                                                                     
        }
        std::cout << "(ndi_aurora) init port handle request: successful?: " << sendF << " ,message returned: " << returnMessage << "\n" << std::endl;
        
        return sendF;
    }

    bool ndi_aurora::defaultToolConfig(char(&portHandle)[2])
    {
        sendF = true;
        //port handle init command
        //compose actual command

        sprintf(buffer, "TTCFG %c%c\r", portHandle[0], portHandle[1]);
        Output = &buffer[0];
        sendF = sendMessage(fd, Output, strlen(Output));
        sendF = sendF && readMessage(fd, returnMessage);

#ifndef NO_AURORA_STDOUT
        std::cout << "(ndi_aurora) Going to load Generic Tool for (dual 5DOF) port with command: " << buffer << std::endl;
#endif
        //check returnMessage
        if (strncmp(returnMessage, errorMessage.c_str(), errorMessage.length()) == 0)
        {
            std::cout << "(ndi_aurora) ERROR: init port handle failed " << returnMessage << "\n" << std::endl;
            return false;
        }
        else if (strncmp(returnMessage, warningMessage.c_str(), warningMessage.length()) == 0)
        {
            std::cout << "(ndi_aurora) WARNING: init port handle warning: " << returnMessage << "\n" << std::endl;
            return false;
        }
        std::cout << "(ndi_aurora) ttcfg port handle request: successful?: " << sendF << " ,message returned: " << returnMessage << "\n" << std::endl;

        return sendF;
    }

    //enable port handles
    bool ndi_aurora::portHandleEnable(char(&portHandle)[2], char priority)
    {
        sendF = true;
        //port handle enable command
        //compose actual command
        
        sprintf(buffer, "PENA %c%c%c\r", portHandle[0], portHandle[1], priority);
        Output = &buffer[0];
        sendF = sendMessage(fd, Output, strlen(Output));
        sendF = sendF && readMessage(fd, returnMessage);

#ifndef NO_AURORA_STDOUT
        std::cout << "(ndi_aurora) Going to enable port with command: " << buffer << std::endl;
#endif
        //check returnMessage
        if (strncmp(returnMessage, errorMessage.c_str(), errorMessage.length()) == 0)
        { 
            std::cout << "(ndi_aurora) ERROR: enable port handle failed " << returnMessage << "\n" << std::endl;
            return false;
        }
        else if (strncmp(returnMessage, warningMessage.c_str(), warningMessage.length()) == 0)
        {
            std::cout << "(ndi_aurora) WARNING: enable port handle warning: " << returnMessage << "\n" << std::endl;
            return false;                                                                                     
        }
        std::cout << "(ndi_aurora) enable port handle request: successful?: " << sendF << " ,message returned: " << returnMessage << "\n" << std::endl;
        
        return sendF;
    }
    
    //initialize and enable port handles for wireless tools
    bool ndi_aurora::initPortHandleWT(const std::vector<std::string>& toolDefinitions, const std::vector<std::string>& toolDefPortHandles) {
        return initPortHandleWT(toolDefinitions, toolDefPortHandles, std::vector<std::string>(), "00");
    }

    //initialize and enable port handles for wireless tools   
    bool ndi_aurora::initPortHandleWT(const std::vector<std::string>& toolDefinitions, const std::vector<std::string>& toolDefPortHandles, const std::vector<std::string>& portHandlesAutoconfig, const std::string& refPortHandle)
    {
        // Validate port handles
        for (size_t idx = 0; idx < toolDefPortHandles.size(); idx++)
            if (2 != toolDefPortHandles.at(idx).length()) {
                std::cout << "(ndi_aurora) Error: Length of the port handles must be 2" << std::endl;
                return false;
            }
        if (2 != refPortHandle.length()) {
            std::cout << "(ndi_aurora) Error: Length of the port handles must be 2" << std::endl;
            return false;
        }
        sign_refPortHandle[0] = refPortHandle.c_str()[0];
        sign_refPortHandle[1] = refPortHandle.c_str()[1];
        
        // Calculate how many ports need ROM files vs autoconfigured
        int num_autoconfig = portHandlesAutoconfig.size();
        int expected_rom_files = toolDefPortHandles.size() - num_autoconfig;
        
        if (toolDefinitions.size() != static_cast<size_t>(expected_rom_files)) {
            std::cout << "(ndi_aurora) ERROR: Number of tool definition files (" << toolDefinitions.size() 
                      << ") doesn't match expected (" << expected_rom_files 
                      << " = total ports " << toolDefPortHandles.size() 
                      << " - autoconfig " << num_autoconfig << ")" << std::endl;
            return false;
        }
        
        std::cout << "(ndi_aurora) Configuration: " << toolDefPortHandles.size() << " total ports, "
                  << toolDefinitions.size() << " with ROM files, "
                  << num_autoconfig << " autoconfigured" << std::endl;

        //initialize
        PHint = 0;
        PHint2 = 0;
        PHint3 = 0;

        usleep(500000);

        if (!portHandleStatus(0, returnMessageCopy)) //0= report port handles that need to be freed
        {
            std::cout << "(ndi_aurora) port handle status request for PH to free failed." << "\n" << std::endl;
            return false;
        }

        //Get port handles that need to be freed
        if (!portHandleStatus(1, returnMessageCopy)) //1= report port handles that need to be freed
        {                                                                                                                            
            std::cout << "(ndi_aurora) port handle status request for PH to free failed." << "\n" << std::endl;
            return false;                                                                                                                                               
        }

        //initialize
        PHint = 0; 
        PHint2 = 0;
        //convert char's to integers to calculate number of port handles
        char2int(returnMessageCopy[0], PHint);
        char2int(returnMessageCopy[1], PHint2);
        //free porthandles if needed to
        for (i = 0; i < (PHint * 16 + PHint2); i++)
        {
            portHandle[0] = returnMessageCopy[2 + i * (CLPHno + CLPHstat)];
            portHandle[1] = returnMessageCopy[3 + i * (CLPHno + CLPHstat)];
            if (!portHandleFree(portHandle)) //free porthandle with this number                                                                          
            {                                                                                                                                                               
                std::cout << "(ndi_aurora) free port handle " << i << " failed." << "\n" << std::endl;
                return false;                                                                                                                                             
            }
        }

        //get port handles that need to be initialized
        if (!portHandleStatus(2, returnMessageCopy)) //2=port handles that need to be initialized
        {
            std::cout << "(ndi_aurora) port handle status request for PH to initialize failed." << "\n" << std::endl;
            return false;
        }

        //initialize
        PHint = 0;
        PHint2 = 0;
        char2int(returnMessageCopy[0], PHint);
        char2int(returnMessageCopy[1], PHint2);
        noTools = 16 * PHint + PHint2;

        std::cout << "(ndi_aurora) Number of ports located: " << noTools << std::endl << "Status of the ports:" << std::endl;
        
        // Helper function to check if port is autoconfigured
        auto isAutoconfig = [&portHandlesAutoconfig](const std::string& port) -> bool {
            for (const auto& ac_port : portHandlesAutoconfig) {
                if (port == ac_port) return true;
            }
            return false;
        };
        
        // Helper function to check if port is in configured list
        auto isConfigured = [&toolDefPortHandles](const std::string& port) -> bool {
            for (const auto& handle : toolDefPortHandles) {
                if (port == handle) return true;
            }
            return false;
        };
        
        // FIRST LOOP: Load ROM files for configured ports (excluding autoconfig)
        int rom_file_index = 0;
        for (k = 0; k < noTools; k++)
        {
            PHint = 0;
            PHint2 = 0;
            PHint3 = 0;
            sPH = std::string(returnMessageCopy + (2 + k * 5), 2);
            sPHStatus = std::string(returnMessageCopy + (4 + k * 5), 3);
            std::cout << "Port: " << k << " with handle: " << sPH << 
                                    "; has status of: " << sPHStatus << std::endl;
            
            // Check if this port is configured
            if (!isConfigured(sPH)) {
                std::cout << "\tSkipping unconfigured port: " << sPH << std::endl;
                continue;
            }
            
            // Check if this port needs autoconfig
            if (isAutoconfig(sPH)) {
                std::cout << "\tPort " << sPH << " will be AUTOCONFIGURED (using internal ROM)" << std::endl;
                // Don't load external ROM file for autoconfigured ports
                continue;
            }
            
            // Regular port with external ROM file
            portHandle[0] = sPH.c_str()[0];
            portHandle[1] = sPH.c_str()[1];

            std::cout << "\tThis port will load Tool Definition File: " << toolDefinitions.at(rom_file_index) << std::endl;

            // Load the tool definition file
            romFileName = const_cast<char*>(toolDefinitions[rom_file_index].c_str());
            std::cout << "[Debug] (ndi_aurora) Trying to open the file: " << romFileName << std::endl;
            if (!loadToolDefFile(portHandle, romFileName))
            {
                std::cout << "(ndi_aurora) loading tool definition file failed." << "\n" << std::endl;
                return false;
            }
            rom_file_index++;
        }

        do {
            // Check for port Handles that have to be initialized
            if (!portHandleStatus(2, returnMessageCopy)) //2=port handles that need to be initialized
            {
                std::cout << "(ndi_aurora) port handle status request for PH to initialize failed." << "\n" << std::endl;
                return false;
            }

            // Initialize ALL configured port handles (including autoconfig)
            PHint = 0;
            PHint2 = 0;
            char2int(returnMessageCopy[0], PHint);
            char2int(returnMessageCopy[1], PHint2);
            noPorts2Init = 16 * PHint + PHint2;
            
            for (k = 0; k < noPorts2Init; k++) {
                sPH = std::string(returnMessageCopy + (2 + k * 5), 2);
                
                // Initialize ALL configured ports (both ROM and autoconfig)
                if (isConfigured(sPH)) {
                    portHandle[0] = sPH.c_str()[0];
                    portHandle[1] = sPH.c_str()[1];
                    if (!portHandleInit(portHandle))
                    {
                        std::cout << "(ndi_aurora) port handle initialization of PH " << sPH << " failed." << "\n" << std::endl;
                        return false;
                    }
                    
                    if (isAutoconfig(sPH)) {
                        std::cout << "(ndi_aurora) Successfully initialized AUTOCONFIGURED port: " << sPH << std::endl;
                    } else {
                        std::cout << "(ndi_aurora) Successfully initialized configured port: " << sPH << std::endl;
                    }
                } else {
                    std::cout << "(ndi_aurora) Skipping initialization of unconfigured port: " << sPH << std::endl;
                }
            }
        } while ((noPorts2Init != 0) && bDuplicate);

        do {
            // Check all the ports to be enabled
            if (!portHandleStatus(3, returnMessageCopy)) // 3 = port handles that need to be enabled
            {
                std::cout << "(ndi_aurora) port handle status request for PH to enable failed." << "\n" << std::endl;
                return false;
            }

            // Enable ALL configured ports (both ROM and autoconfig)
            PHint = 0;
            PHint2 = 0;
            char2int(returnMessageCopy[0], PHint);
            char2int(returnMessageCopy[1], PHint2);
            noPorts2Enable = 16 * PHint + PHint2;
            
            for (k = 0; k < noPorts2Enable; k++) {
                sPH = std::string(returnMessageCopy + (2 + k * 5), 2);
                
                // Enable ALL configured ports
                if (isConfigured(sPH)) {
                    portHandle[0] = sPH.c_str()[0];
                    portHandle[1] = sPH.c_str()[1];
                    
                    if (0 == sPH.compare(0, 2, sign_refPortHandle)) {
                        currentPrio = 'S';
                        std::cout << "(ndi_aurora) Selected port " << sPH << " as Static (ref Tool)" << std::endl;
                    }
                    else {
                        currentPrio = 'D';
                        std::cout << "(ndi_aurora) Selected port " << sPH << " as Dynamic" << std::endl;
                    }
                    
                    if (!portHandleEnable(portHandle, currentPrio))
                    {
                        std::cout << "(ndi_aurora) port handle enable of PH " << sPH << " failed." << "\n" << std::endl;
                        return false;
                    }
                    
                    if (isAutoconfig(sPH)) {
                        std::cout << "(ndi_aurora) Successfully enabled AUTOCONFIGURED port: " << sPH << std::endl;
                    } else {
                        std::cout << "(ndi_aurora) Successfully enabled configured port: " << sPH << std::endl;
                    }
                } else {
                    std::cout << "(ndi_aurora) Skipping enable of unconfigured port: " << sPH << std::endl;
                }
            }
        } while (noPorts2Enable != 0);

        //get port handles that are initialized and enabled
        if (!portHandleStatus(4, returnMessageCopy)) //4=port handles that are initialized and enabled
        {                                                                                                                            
            std::cout << "(ndi_aurora) port handle status request for enabled PH failed." << "\n" << std::endl;
            return false;       
        }
        PHint = 0;
        PHint2 = 0;
        noPortsEnabled = 0;
        char2int(returnMessageCopy[0], PHint);
        char2int(returnMessageCopy[1], PHint2);
        noPortsEnabled = 16 * PHint + PHint2;
        activePHs.clear();
        for (k = 0; k < noPortsEnabled; k++)
            activePHs.push_back(std::string(returnMessageCopy + (2 + k * 5), 2));
        
        std::cout << "(ndi_aurora) Successfully configured " << noPortsEnabled << " port handles" << std::endl;
        return true;
    }


    // PARTE 4

    //start Diagnostic Mode (after system initialization)
    bool ndi_aurora::startDiagnosticMode()
    {
        sendF = true;
        //send start Diagnostic Mode command
        message2send = "DSTART \r";
        Output = message2send.c_str();
        sendF = sendMessage(fd, Output, strlen(Output));
        sendF = sendF && readMessage(fd, returnMessage);        
        
        //check returnMessage
        if (strncmp(returnMessage, errorMessage.c_str(), errorMessage.length()) == 0)
        {
            std::cout << "(ndi_aurora) ERROR: starting diagnostic mode failed " << returnMessage << "\n" << std::endl;
            return false;
        }
        std::cout << "(ndi_aurora) starting diagnostic mode: successful?: " << sendF << " ,message returned: " << returnMessage << "\n" << std::endl;
        
        return sendF;
    }

    //stop Diagnostic Mode (while in Diagnostic mode)
    bool ndi_aurora::stopDiagnosticMode()
    {
        sendF = true;
        //send stop Diagnostic Mode command
        message2send = "DSTOP \r";
        Output = message2send.c_str();
        sendF = sendMessage(fd, Output, strlen(Output));
        sendF = sendF && readMessage(fd, returnMessage);        
        
        //check returnMessage
        if (strncmp(returnMessage, errorMessage.c_str(), errorMessage.length()) == 0)
        {
            std::cout << "(ndi_aurora) ERROR: stopping diagnostic mode failed " << returnMessage << "\n" << std::endl;
            return false;
        }
        std::cout << "(ndi_aurora) stopping diagnostic mode: successful?: " << sendF << " ,message returned: " << returnMessage << "\n" << std::endl;
        
        return sendF;
    }
    
    //start Tracking Mode (after system initialization)
    bool ndi_aurora::startTrackingMode(const int& trackReplyOption)
    {
        sendF = true;
        //send start tracking Mode command
        //compose command
        switch (trackReplyOption)                                                                                                                                                
        {                                                                                                                                                               
            case 0:                                                                                                                                                  
                sprintf(buffer, "TSTART \r");
                break; 

            case 80:
                sprintf(buffer, "TSTART %i\r", trackReplyOption);
                break;

            default:                                                                                                                                                    
                std::cout << "(ndi_aurora) ERROR: " << trackReplyOption << " is not a supported tracking reply option!" << "\n" << std::endl;
                return false;
        }
        Output = &buffer[0];   
        sendF = sendMessage(fd, Output, strlen(Output));
        sendF = sendF && readMessage(fd, returnMessage);        
        
        //check returnMessage
        if (strncmp(returnMessage, errorMessage.c_str(), errorMessage.length()) == 0)
        {
            std::cout << "(ndi_aurora) ERROR: starting tracking mode failed " << returnMessage << "\n" << std::endl;
            return false;
        }
        std::cout << "(ndi_aurora) starting tracking mode: successful?: " << sendF << " ,message returned: " << returnMessage << "\n" << std::endl;
        
        return sendF;
    }

    //stop Tracking Mode (while in Tracking mode)
    bool ndi_aurora::stopTrackingMode()
    {
        sendF = true;
        //send stop Tracking Mode command
        message2send = "TSTOP \r";
        Output = message2send.c_str();
        sendF = sendMessage(fd, Output, strlen(Output));
        sendF = sendF && readMessage(fd, returnMessage);        
        
        //check returnMessage
        if (strncmp(returnMessage, errorMessage.c_str(), errorMessage.length()) == 0)
        {
            std::cout << "(ndi_aurora) ERROR: stopping tracking mode failed " << returnMessage << "\n" << std::endl;
            return false;
        }
        std::cout << "(ndi_aurora) stopping tracking mode: successful?: " << sendF << " ,message returned: " << returnMessage << "\n" << std::endl;
        
        return sendF;
    }

    //measure 3D tool positions - main measurement function
    bool ndi_aurora::measureTool(const std::string& measureReplyOption, std::vector<ToolPose>& allToolPose, 
                                 const bool& statusReply, std::vector<int>& portIndexes, std::vector<int>& visibleTools)
    {           
        //initializations
        interpretOK = true;
        sendF = true;
        memset(returnMessage, '0', 255); //clear char array
        memset(buffer, '0', 255); //clear char array
        PHcounter = 0;
        allToolPose.clear();
        visibleTools.clear();
        portIndexes.clear();

        //measure tool command
        sprintf(buffer, "TX %c%c%c%c\r", *measureReplyOption.c_str(), *(measureReplyOption.c_str() + 1), *(measureReplyOption.c_str() + 2), *(measureReplyOption.c_str() + 3));
        Output = &buffer[0];   
        sendF = sendMessage(fd, Output, strlen(Output));
        sendF = sendF && readMessage(fd, returnMessage);        

        if (statusReply)
        {
#ifndef NO_AURORA_STDOUT
            std::cout << "(ndi_aurora) measure tool: message returned: " << returnMessage << "\n" << std::endl;
#endif
        }

        //check returnMessage - error?
        if (strncmp(returnMessage, errorMessage.c_str(), errorMessage.length()) == 0) 
        {
            std::cout << "(ndi_aurora) ERROR: measure tool failed " << returnMessage << "\n" << std::endl;
            return false;
        }

        //length of returnMessage
        tempString = returnMessage;
        messageLength = tempString.length();

        //system status check
        if (statusReply)
        {    
            char2int(returnMessage[messageLength - CLcr - CLcrc - 1], systemStatus); // bits 0-3 - Less significant word
            if (systemStatus & 0x1)//bit 0
            {                                                                                                      
                std::cout << "(ndi_aurora) system status ERROR: system communication error" << std::endl;
                return false;                                                                                                                                               
            }       
            if (systemStatus & 0x2)//bit 1
            {                                                                                                      
                std::cout << "(ndi_aurora) system status ERROR: reserved (bit 1)" << std::endl;
                return false;                                                                                                                                               
            }       
            if (systemStatus & 0x4)//bit 2
            {                                                                                                      
                std::cout << "(ndi_aurora) system status ERROR: system CRC error" << std::endl;
                return false;                                                                                                                                               
            }       
            if (systemStatus & 0x8)//bit 3
            {                                                                                                      
                std::cout << "(ndi_aurora) system status ERROR: recoverable system processing exception" << std::endl;
                return false;                                                                                                                                               
            }       
            char2int(returnMessage[messageLength - CLcr - CLcrc - 2], systemStatus); // bits 4-7
            if (systemStatus & 0x1)//bit 4
            {                                                                                                      
                std::cout << "(ndi_aurora) system status ERROR: hardware failure" << std::endl;
                return false;                                                                                                                                               
            }       
            if (systemStatus & 0x2)//bit 5
            {                                                                                                      
                std::cout << "(ndi_aurora) system status ERROR: hardware change, Field Generator is disconnected from SCU" << std::endl;
                return false;                                                                                                                                               
            }
            if (systemStatus & 0x4)//bit 6
            {                                                                                                      
                std::cout << "(ndi_aurora) system status ERROR: some port handle has become occupied" << std::endl;
                return false;                                                                                                                                               
            }       
            if (systemStatus & 0x8)//bit 7
            {                                                                                                      
                std::cout << "(ndi_aurora) system status ERROR: some port handle has become unoccupied" << std::endl;
                return false;                                                                                                                                               
            } 
        }

        // Check number of tools
        PHint = PHint2 = 0;
        char2int(returnMessage[0], PHint);
        char2int(returnMessage[1], PHint2);
        noTxPorts = 16 * PHint + PHint2;
        
        //define the startindex of the reply data
        startIndex = CLnoPH; // =>because reply option data starts after noPH and PHno
        string2int(measureReplyOption, replyOptionInt);
        
        //check all options for every handle
        //check all options for every handle
        if (replyOptionInt & 0x800) 
        {
            if (noTxPorts != noPortsEnabled) {
                std::cout << "(ndi_aurora) Error on the number of ports expected to receive: " << noPortsEnabled << ", Received: " << noTxPorts << std::endl;
                return false;
            }
            extraTransfo = true;
        }

        if (noTxPorts < noPortsEnabled) {
            std::cout << "(ndi_aurora) Warning: Received fewer ports than enabled: " << noTxPorts << " < " << noPortsEnabled << std::endl;
        }
        std::cout << "(ndi_aurora) Processing " << noTxPorts << " ports from Aurora response" << std::endl;

        //check all options for every handle
        for (handleNumber = 1; handleNumber <= noTxPorts; handleNumber++) //eg. 2 handles=> handle 1 and handle 2
        {
            //calculate handle number that is examined
            char2int(returnMessage[startIndex], PHint);
            realHandleNumber = PHint * 16;
            char2int(returnMessage[startIndex + 1], PHint);
            realHandleNumber = realHandleNumber + PHint;

            //print handle number that is examined
            if (statusReply)
            {
#ifndef NO_AURORA_STDOUT
                std::cout << "(ndi_aurora) handleNumber = " << realHandleNumber << std::endl;
#endif
            }
            startIndex += CLPHno;
            
            if (replyOptionInt & 0x1)//check if replyoption 0001: transformation data
            { 
                interpretOK = interpretOK && interpretMessage0001(startIndex, toolPose, errorRMS, handleNumber, statusReply, missingF);
            } 

            //check if there is a newline character
            if (checkNewLine(startIndex))
            {
                PHcounter++;
            }
            else
            {
                interpretOK = false;
            }
            startIndex += CLlf; 
            
            //put the measured toolPoses in the vector with the frames
            allToolPose.push_back(toolPose);
            portIndexes.push_back(realHandleNumber);

            //book keeping: which tool is visible?
            if (!missingF)
            {
                visibleTools.push_back(realHandleNumber);
            }

            //put the tool missing flag back to false
            missingF = false;
        }

        if (statusReply)
        {
#ifndef NO_AURORA_STDOUT
            std::cout << "(ndi_aurora) measure tool: successful?: " << sendF << " ,message returned: " << returnMessage << "\n" << std::endl;
#endif
        }
        return sendF && interpretOK;
    }

    //message interpreters
    bool ndi_aurora::interpretMessage0001(int& startIndex, ToolPose& toolPose, double& errorRMS, int& handleNumber, const bool& statusReply, bool& missingF)
    {  
        (void)handleNumber; // suppress unused parameter warning
        readF = true;
        if (strncmp(returnMessage + startIndex, missingMessage.c_str(), missingMessage.length()) == 0)
        {
            startIndex += missingMessage.length();
            missingF = true;
#ifndef NO_AURORA_STDOUT
            std::cout << "(ndi_aurora) tool number " << handleNumber << " went MISSING \n" << std::endl;
#endif
        }
        else
        {
            //rotation
            readF = readF && readQuaternion(startIndex, quaternion0); 
            startIndex += CLquaternion; //6 quaternion message is 6 chars long
            readF = readF && readQuaternion(startIndex, quaternionX); 
            startIndex += CLquaternion; //6 quaternion message is 6 chars long
            readF = readF && readQuaternion(startIndex, quaternionY); 
            startIndex += CLquaternion; //6 quaternion message is 6 chars long
            readF = readF && readQuaternion(startIndex, quaternionZ); 
            startIndex += CLquaternion; //6 quaternion message is 6 chars long

            //translation
            readF = readF && readPosition(startIndex, positionX);
            startIndex += CLposition; //7 => position message is 7 chars long        
            readF = readF && readPosition(startIndex, positionY);
            startIndex += CLposition; //7 => position message is 7 chars long        
            readF = readF && readPosition(startIndex, positionZ);
            startIndex += CLposition; //7 => position message is 7 chars long        

            //errorRMS: RMS of the error on the measurement in mm
            readF = readF && readError(startIndex, errorRMS);
            startIndex += CLerrorRMS; //6 => position message is 6 chars long        

            toolPose.qw = quaternion0;
            toolPose.qx = quaternionX;
            toolPose.qy = quaternionY;
            toolPose.qz = quaternionZ;
            toolPose.x = positionX;
            toolPose.y = positionY;
            toolPose.z = positionZ;
        }

        //port status
        if (statusReply)
        {
#ifndef NO_AURORA_STDOUT
            readF = readF && readPortStatus(startIndex, handleNumber);
#endif
        }
        startIndex += CLportStatus;

        //frame number
        readF = readF && readFrameNumber(startIndex, frameNumber);
        startIndex += CLframeNo;

        return readF;
    }

    // PARTE 5
    //read quaternion component from Aurora message
    bool ndi_aurora::readQuaternion(int& startIndex, double& quaternion)
    {
        //read the first value 
        char2int(returnMessage[startIndex + 1], tempQuaternion);
        quaternion = tempQuaternion;
        //read second value (first value after decimal)
        char2int(returnMessage[startIndex + 2], tempQuaternion);
        quaternion += tempQuaternion / 10.0;
        //read third value (second value after decimal)
        char2int(returnMessage[startIndex + 3], tempQuaternion);
        quaternion += tempQuaternion / 100.0;
        //read fourth value (third value after decimal)
        char2int(returnMessage[startIndex + 4], tempQuaternion);
        quaternion += tempQuaternion / 1000.0;
        //read fifth value (fourth value after decimal)
        char2int(returnMessage[startIndex + 5], tempQuaternion);
        quaternion += tempQuaternion / 10000.0;
        //read the sign
        if (returnMessage[startIndex] == '-')
        {
            quaternion = -quaternion;
        }
        
        return true;
    }

    //read position component from Aurora message
    bool ndi_aurora::readPosition(int& startIndex, double& position)
    {
         // ===== AGGIUNGI QUESTO LOG RAW =====
        char rawPos[8];
        memcpy(rawPos, returnMessage + startIndex, 7);
        rawPos[7] = '\0';
        std::cout << "[RAW POSITION] startIndex=" << startIndex 
                  << " chars: '" << rawPos << "'" << std::endl;
        // ===== FINE LOG =====

        //read the first value 
        char2int(returnMessage[startIndex + 1], tempQuaternion);
        position = tempQuaternion * 1000.0;
        //read second value 
        char2int(returnMessage[startIndex + 2], tempQuaternion);
        position += tempQuaternion * 100.0;
        //read third value 
        char2int(returnMessage[startIndex + 3], tempQuaternion);
        position += tempQuaternion * 10.0;
        //read fourth value 
        char2int(returnMessage[startIndex + 4], tempQuaternion);
        position += tempQuaternion;
        //read fifth value (first value after decimal)
        char2int(returnMessage[startIndex + 5], tempQuaternion);
        position += tempQuaternion / 10.0;
        //read sixth value (2nd value after decimal)
        char2int(returnMessage[startIndex + 6], tempQuaternion);
        position += tempQuaternion / 100.0;
        //read the sign
        if (returnMessage[startIndex] == '-')
        {
            position = -position;
        }
        
        return true;
    }

    //read error RMS from Aurora message
    bool ndi_aurora::readError(int& startIndex, double& errorRMS)
    {
        //read the first value 
        char2int(returnMessage[startIndex + 1], tempQuaternion);
        errorRMS = tempQuaternion;
        //read second value (first value after decimal)
        char2int(returnMessage[startIndex + 2], tempQuaternion);
        errorRMS += tempQuaternion / 10.0;
        //read third value (second value after decimal)
        char2int(returnMessage[startIndex + 3], tempQuaternion);
        errorRMS += tempQuaternion / 100.0;
        //read fourth value (third value after decimal)
        char2int(returnMessage[startIndex + 4], tempQuaternion);
        errorRMS += tempQuaternion / 1000.0;
        //read fifth value (fourth value after decimal)
        char2int(returnMessage[startIndex + 5], tempQuaternion);
        errorRMS += tempQuaternion / 10000.0;
        //read the sign
        if (returnMessage[startIndex] == '-')
        {
            errorRMS = -errorRMS;
        }
        
        return true;
    }
    
    //read and interpret port status information
    bool ndi_aurora::readPortStatus(int& startIndex, int& handleNumber)
    {
        //port status NOTE:return message is LITTLE ENDIAN=>within a byte bit
        //0 is at the right side 
        char2int(returnMessage[startIndex + CLportStatus - 1], systemStatus);// => right char of 8 portstatus chars
        if (systemStatus & 0x1)//bit 0
        {                                                                                                      
            std::cout << "(ndi_aurora) port " << handleNumber << " status: occupied" << std::endl;
        }       
        if (systemStatus & 0x2)//bit 1
        {                                                                                                      
            std::cout << "(ndi_aurora) port " << handleNumber << " status: switch 1 closed /GPIO line 1 active" << std::endl;
        }       
        if (systemStatus & 0x4)//bit 2
        {                                                                                                      
            std::cout << "(ndi_aurora) port " << handleNumber << " status: switch 2 closed/GPIO line 2 active" << std::endl;
        }       
        if (systemStatus & 0x8)//bit 3
        {                                                                                                      
            std::cout << "(ndi_aurora) port " << handleNumber << " status: switch 3 closed/GPIO line 3 active" << std::endl;
        }       
        char2int(returnMessage[startIndex + CLportStatus - 2], systemStatus);// => second from right char of 8 portstatus chars
        if (systemStatus & 0x1)//bit 4
        {                                                                                                      
            std::cout << "(ndi_aurora) port " << handleNumber << " status: initialized" << std::endl;
        }       
        if (systemStatus & 0x2)//bit 5
        {                                                                                                      
            std::cout << "(ndi_aurora) port " << handleNumber << " status: enabled" << std::endl;
        }       
        if (systemStatus & 0x4)//bit 6
        {                                                                                                      
            std::cout << "(ndi_aurora) port " << handleNumber << " status: out of volume" << std::endl;
        }       
        if (systemStatus & 0x8)//bit 7
        {                                                                                                      
            std::cout << "(ndi_aurora) port " << handleNumber << " status: partially out of volume" << std::endl;
        } 
        char2int(returnMessage[startIndex + CLportStatus - 3], systemStatus);// => 3th from right char of 8 portstatus chars
        if (systemStatus & 0x1)//bit 8
        {
            std::cout << "(ndi_aurora) port " << handleNumber << " status: a sensor coil is broken" << std::endl;
        }       
        if (systemStatus & 0x2)//bit 9 // Only Polaris (Reserved Aurora)
        {
            std::cout << "(ndi_aurora) port " << handleNumber << " status: Reserved for Polaris" << std::endl;
        }       
        if (systemStatus & 0x4)//bit 10
        {                                                                                                      
            std::cout << "(ndi_aurora) port " << handleNumber << " status: reserved" << std::endl;
        }       
        if (systemStatus & 0x8)//bit 11
        {                                                                                                      
            std::cout << "(ndi_aurora) port " << handleNumber << " status: reserved" << std::endl;
        }

        char2int(returnMessage[startIndex + CLportStatus - 4], systemStatus);// => 4th from right char of 8 portstatus chars
        if (systemStatus & 0x1)//bit 12
        {                                                                                                      
            std::cout << "(ndi_aurora) port " << handleNumber << " status: processing exception " << std::endl;
        }       
        if (systemStatus & 0x4)//bit 14 // Only Polaris
        {
            std::cout << "(ndi_aurora) port " << handleNumber << " status: Reserved for Polaris" << std::endl;
        }       
        if (systemStatus & 0x8)//bit 15
        {
            std::cout << "(ndi_aurora) port " << handleNumber << " status: Reserved for Polaris" << std::endl;
        } 
        char2int(returnMessage[startIndex + CLportStatus - 5], systemStatus); // bits 16-19 (All reserved)
        char2int(returnMessage[startIndex + CLportStatus - 6], systemStatus); // bits 20-23 (All reserved)
        char2int(returnMessage[startIndex + CLportStatus - 7], systemStatus); // bits 24-27 (All reserved)
        char2int(returnMessage[startIndex + CLportStatus - 8], systemStatus); // bits 28-31 - Most significative word (All reserved)
        
        return true;
    }

    //read frame number from Aurora message
    bool ndi_aurora::readFrameNumber(int& startIndex, int& frameNumber)
    {
        frameNoF = char2int(returnMessage[startIndex], tempFno);
        frameNumber = tempFno;
        for (i = 1; i < CLframeNo; i++)
        {
            frameNoF = frameNoF && char2int(returnMessage[startIndex + i], tempFno);
            frameNumber = frameNumber * 16 + tempFno;
        }
        return true;
    }

    //check for newline character at expected position
    bool ndi_aurora::checkNewLine(int& startIndex)
    {
        newLineCmd = "\n"; 
        if (strncmp(&returnMessage[startIndex], newLineCmd.c_str(), 1) == 0)//returns 0 if equal
        {
            return true;
        }
        std::cout << "(ndi_aurora) no newline character at the expected position" << std::endl;
        return false;
    }

    //search for newline character in message
    bool ndi_aurora::searchNewLine(int& startIndex, const int& messageLength)
    {
        tempChar = returnMessage[startIndex];
        startIndex++;
        checkReference = messageLength - CLcr - CLcrc - CLstatus + 1;//index where the system status begins
        //check if the next character is a endline and if we didn't search to
        //far in the message (=an error occurred)
        while (tempChar != '\n' && startIndex < checkReference)
        {
            tempChar = returnMessage[startIndex];
            startIndex++;
        }
        if (startIndex < checkReference)
        {
            return true;
        }
        std::cout << "(ndi_aurora) ERROR: trying to search for a newline character after system status on the last line" << std::endl;
        return false;
    }

    //convert integer to string
    bool ndi_aurora::int2string(const int& int2convert, std::string& stringFromInt)
    {
        tempInt2string.str("");
        tempInt2string.clear();
        tempInt2string << int2convert; 
        stringFromInt = tempInt2string.str();
        return true;
    }

    //convert hexadecimal character to integer
    bool ndi_aurora::char2int(const char& char2convert, int& intFromChar)
    {
        tempChar2int.str("");
        tempChar2int.clear();
        tempChar2int << std::hex << char2convert;
        tempChar2int >> intFromChar;             
        return true;
    }

    //convert character to hexadecimal integer
    bool ndi_aurora::char2hex(const char& char2convert, int& intFromChar)
    {
        tempChar2int.str("");
        tempChar2int.clear();
        tempChar2int << char2convert;
        tempChar2int >> std::hex >> intFromChar;             
        return true;
    }

    //convert hexadecimal string to unsigned integer
    bool ndi_aurora::string2int(const std::string& string2convert, unsigned int& intFromChar)
    {
        tempChar2int.str("");
        tempChar2int.clear();
        tempChar2int << std::hex << string2convert;
        tempChar2int >> intFromChar;             
        return true;
    }

    //convert string to hexadecimal unsigned integer
    bool ndi_aurora::string2hex(const std::string& string2convert, unsigned int& intFromChar)
    {
        tempChar2int.str("");
        tempChar2int.clear();
        tempChar2int << string2convert;
        tempChar2int >> std::hex >> intFromChar;
        return true;
    }

} // namespace AuroraDriver