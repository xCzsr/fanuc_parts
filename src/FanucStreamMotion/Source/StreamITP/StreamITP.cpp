// 
// StreamITP.cpp : The PC interface test program for Robot's J519 stream motion option
//


#include "stdafx.h"
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <stdlib.h>
#include <stdio.h>
#include <Winsock2.h>
#include <WS2tcpip.h>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include <array>
#include <queue>



using namespace std;

// solve for the __imp__htonl problem
#pragma comment(lib, "Ws2_32.lib")
typedef unsigned __int8 u_byte;    // replace char for clarity.

// define some constants here

const u_short ROBOT_PORT = 60015;
const int MaxAxisNumber = 9;  // data file can have  either 9 axis (or xyzwpr ext) data per each position
const int MinAxisNumber = 6;  // Or can have 6 axis (or xyzwpr) data


// store read in position data from data file.
struct PositionData_T {
	float data[MaxAxisNumber];
};

// Data exchange start packet, send to robot controller
typedef struct StartPacket_T {
	u_long packetType;
	u_long versionNo;
} StartPacket_T;


// Motion command packet send to robot controller
typedef struct CommandPacket_T {
	u_long packetType;
	u_long versionNo;
	u_long sequenceNo;
	u_byte lastData;
	u_byte readIOType;
	u_short readIOIndex;
	u_short readIOMask;
	u_byte dataStyle;
	u_byte writeIOType;
	u_short writeIOIndex;
	u_short writeIOMask;
	u_short writeIOValue;
	u_short unused;
	u_long commandPos[MaxAxisNumber];  // could be either cartesian position or joint angle, based on dataStyle
} CommandPacket_T;

// data exchange complete,s end to robot controller
typedef struct StopPacket_T {
	u_long packetType;
	u_long versionNo;
} StopPacket_T;

// Receive packet from robot controller
typedef struct RobotStatusPacket_T {
	u_long packetType;
	u_long versionNo;
	u_long sequenceNo;
	u_byte status;
	u_byte readIOType;
	u_short readIOIndex;
	u_short readIOMask;
	u_short readIOValue;
	u_long timeStamp;
	float position[MaxAxisNumber];
	u_long jontAngle[MaxAxisNumber];
	float current[MaxAxisNumber];
} RobotStatusPacket_T;

typedef struct RobotThresholdPacket_T {
	u_long packetType;
	u_long versionNo;
	u_long axisNumber;
	u_long thresholdType;
	u_long maxCartesianSpeed; 
	u_long interval;

	float noPayload[20];
	float fullPayload[20];
} RobotThresholdPacket_T;


// Threshold request packet
typedef struct ThresholdPacket_T {
	u_long packetType; /* = 3*/
	u_long versionNo;  /* = 2 */
	u_long axisNumber;  /* from 1-9 */
	u_long thresholdType;  /* 0: velocity, 1: acceleration, 2: Jerk */
} ThresholdPacket_T;

/* ---------------------------------------------------
 *
 -----------------------------------------------------*/
static void InitThresholdPacket(ThresholdPacket_T *packet_p, u_long axisNumber, u_long thresholdType) {
	packet_p->packetType = htonl(3);
	packet_p->versionNo = htonl(1);
	packet_p->axisNumber = htonl(axisNumber);
	packet_p->thresholdType = htonl(thresholdType);
}

static void InitCommandPacket(CommandPacket_T *packet, u_long seqNo, float commandPos[9], u_byte dStyle, u_byte lastD) {
	packet->packetType = htonl(1);
	packet->versionNo = htonl(1);
	packet->sequenceNo = htonl(seqNo);
	packet->lastData = lastD;
	packet->readIOType = 0;
	packet->readIOIndex = htons(0);
	packet->readIOMask = htons(0);
	packet->dataStyle = dStyle;
	packet->writeIOType = 0;
	packet->writeIOIndex = htons(0);
	packet->writeIOMask = htons(0);
	packet->writeIOValue = htons(0);
	packet->unused = htons(0);
	for (int idx = 0; idx < MaxAxisNumber; idx++){
		packet->commandPos[idx] = htonl(*reinterpret_cast<long *> (&commandPos[idx]));
	}
	// printf("seq ID: %d \n", seqNo);
}

static void InitStartPacket(StartPacket_T *packet) {
	packet->packetType = htonl(0L);
	//cout << "StartPacket packtetType (htonl): " << packet->packetType <<endl;
	packet->versionNo = htonl(1L);
	//cout << "StartPacket versionNum (htonl): " << packet->versionNo << endl;
}

static void InitStopPacket(StopPacket_T *packet)
{
	packet->packetType = htonl(2L);
	packet->versionNo = htonl(1L);
}


/*
 * ParseString: split the input string into number of tokens based
 *              on the delimiter.
 */
vector<string> ParseString(string inString, string delimiter) {
	string token;
	vector<string> tokens;
	int pos_start = 0, pos_end;
	int delim_len = delimiter.length();

	while ((pos_end = inString.find(delimiter, pos_start)) != string::npos) {
		token = inString.substr(pos_start, pos_end - pos_start);
		pos_start = pos_end + delim_len;
		tokens.push_back(token);
	}
	tokens.push_back(inString.substr(pos_start));
	return tokens;
}

/*
 * CheckDataFile: Check the data file to see if it has valid data: 
 *    Valid data file: each line has either 6 or 9 position data
 *                     each item on a line is seperated by either a tab or a space.
 */
static int CheckDataFile(string inName, int *lineDataCount, bool *useTab) 
{
	int dataCount;
	int lineCount = 0;  // line count
	ifstream inFile;
	string inLine;
	vector<string> lineString;
	vector<float> angles;

	*useTab = true;  // by default, use tab as delimiter

	inFile.open(inName);
	if (!inFile){
		cout << "Unable to open file: " << inName << endl;
		return 0;
	}
	else { // File exists
		cout << "reading file: " << inName << endl;
		while (!inFile.eof()) {
			getline(inFile, inLine);
			if (lineCount == 0) {
				lineString = ParseString(inLine, ",");
				*lineDataCount = lineString.size();
				cout << "data size: " << *lineDataCount << endl;
				if (*lineDataCount == 0) {
					useTab = false;
					lineString = ParseString(inLine, " ");
					*lineDataCount = lineString.size();
					cout << "data size use space: " << *lineDataCount << endl;
				}

			    if ((*lineDataCount != MaxAxisNumber) && (*lineDataCount != MinAxisNumber)) {
					cout << "Invalid data count" << endl;
					return 0;
				}
				lineCount++;
			} 
			else {
				if (useTab) {
					lineString = ParseString(inLine, ",");
				}
				else {
					lineString = ParseString(inLine, " ");
				}
				dataCount = lineString.size();
				if (dataCount == *lineDataCount){
					lineCount++;
				}
			}
		
			// istringstream ss(inLine);
			// copy(istream_iterator <float>(ss), istream_iterator <float>(), back_inserter(angles));
			//for (size_t idx = 0; idx < angles.size(); idx++) {
				//cout << angles[idx] << " , ";
			//}
			//cout << "\n";
			
		} // end while

		inFile.close();
		return lineCount;
	}
}

/*
 * SetRepresetnation: Check the data representation: either JOINT or CARTESIAN
 *                    default is joint angle.
 */
static bool UseJointRepresentation(string rep)
{
	// convert the string to upper case
	for (size_t idx = 0; idx < rep.size(); idx++){
		rep.at(idx) = toupper(rep.at(idx));
	}

	// compare the string to JOINT/CARTESIAN
	if(rep.compare("CARTESIAN") == 0) {
		return(false);
	}
	else {
		return true;
	}
}

#if 0
static bool CheckThreshold(string thresholdStr)
{
	for (size_t idx = 0; idx < thresholdStr.size(); idx++){
		thresholdStr.at(idx) = toupper(thresholdStr.at(idx));
	}
	if (thresholdStr.compare("THRESHOLD") == 0){
		return(true);
	}
	else {
		return(false);
	}
}
#endif

/* 
 * ReadDataFile: Read in the positions from the data file
 */

static bool ReadDataFile(queue<PositionData_T> *dataQueue, string inName, int dataPerLine, bool tabDelimiter) 
{
	ifstream inFile;
	string inLine;
	vector<string> tokens;
	PositionData_T posData ;
	int axisNum;

	inFile.open(inName);
	if (!inFile){
		cout << "Unable to open file: " << inName << endl;
		return false;
	}

	// initialize the array
	for (int idx = 0; idx < MaxAxisNumber; idx++) {
		posData.data[idx] = 0.0;
	}

	// make sure we won't exceed 9 position data per line
	if (dataPerLine > MaxAxisNumber){
		axisNum = MaxAxisNumber;
	}
	else {
		axisNum = dataPerLine;
	}

	while (!inFile.eof()) {
		getline(inFile, inLine);
	
		if (tabDelimiter == true) {
			tokens = ParseString(inLine, "\t");
		}
		else {
			tokens = ParseString(inLine, " ");
		}
		if (tokens.size() == dataPerLine) {
			for (int idx = 0; idx < axisNum; idx++) {
				posData.data[idx] = (float) stod(tokens[idx]);
			}
			dataQueue->push(posData);
		}

	} // end while

	inFile.close();
	return true;
}

static void DebugQueue(queue<PositionData_T> *posQueue) {
	while (!posQueue->empty()){
		PositionData_T pos = posQueue->front();
		for (int idx = 0; idx < MaxAxisNumber; idx++) {
			cout << pos.data[idx] << " ";
		}
		cout << "\n";
		posQueue->pop();
	}
}

static ULONG32 Swap32(ULONG32 x)
{
	return static_cast<ULONG32> ((x << 24) | ((x << 8) & 0x00FF0000) | ((x >> 8) & 0x0000FF00) | (x >> 24));
}

static float SwapFloat(float x)
{
	union {
		float f;
		ULONG32 u32;
	} swapper;
	swapper.f = x;
	swapper.u32 = Swap32(swapper.u32);
	return swapper.f;
}

static void WriteThresholdData(RobotThresholdPacket_T *velPkt_p,
	                           RobotThresholdPacket_T *accPkt_p,
	                           RobotThresholdPacket_T *jerkPkt_p)
{
	printf(" axis: %1d, max speed; %d\n", ntohl(velPkt_p->axisNumber), ntohl(velPkt_p->maxCartesianSpeed));
	printf("Threshold No Payload limits:  Velocity Acceleration Jerk\n");
	for (int idx = 0; idx < 20; idx++) {
		printf(" %2d, %f %f %f \n", idx + 1,SwapFloat(velPkt_p->noPayload[idx]), SwapFloat(accPkt_p->noPayload[idx]), SwapFloat(jerkPkt_p->noPayload[idx]));
	}
	printf(" Threshold Full Payload limits: Velocity Acceleration Jerk\n");
	for (int idx = 0; idx < 20; idx++) {
		printf(" %2d, %f %f %f\n", idx + 1, SwapFloat(velPkt_p->fullPayload[idx]), SwapFloat(accPkt_p->fullPayload[idx]), SwapFloat(jerkPkt_p->fullPayload[idx]));
	}
}

/* ------------------------------------------------------------------
* Main routine: Read in ITP level robot motion command data and
* move the robot using the stream motion option
* NOTE: This program does not handle I/O setting/reading etc.
--------------------------------------------------------------------- */
int main(int argc, char* argv[])
{
	int lineCount = 0;
	u_byte representation = 1;  // Cartesian position = 0, joint angle = 1;
	u_byte lastData;
	bool doJointAngle = true;
	bool readOK = false;
	bool useTab;
	int lineDataCount;
	int packetStack = 0;
	int startSeqID = 0;
	int seqID;
	int thresholdAxisNumber;

	// socket related 
	struct sockaddr_in robot_addr;
	int addr_len = sizeof(robot_addr);
	int socketID;
	bool doDataExchange = true;
	bool doThreshold = false;

	StartPacket_T startPacket;
	StopPacket_T  stopPacket;
	RobotStatusPacket_T statusPacket;
	RobotThresholdPacket_T jerkThresholdPacket;
	RobotThresholdPacket_T accThresholdPacket;
	RobotThresholdPacket_T velThresholdPacket;
	ThresholdPacket_T thresholdPacket;

	CommandPacket_T commandPacket;

	float curJoint[9];

	// input data
	queue<PositionData_T> posDataQueue;

	/*
	 * Read in the command line arguments:
	 * 1st argument: the data file
	 * 2nd argument: the IP address of the robot controller
	 * 3rd argument: the command data representation: Joint/Cartesian
	 * 4th argument: read the axis threshold data. 1-6 is legal axis value.
	 * 5th argument: use packet stack buffer. (valid value 0-9)
	 */

	if ((argc < 3) || (argc > 6)) {
		cout << " Usage: StreamITP DataFileName RobotIPAddress (Optional: DataRepresentation) (Optional: axis Jerk Threshold axis number (1-6)) (Optional: buffer packet number (1-9))" << endl;
		system("pause");
		return 0;
	}

	// get argument data
	string inName(argv[1]);
	string robotIPAddress(argv[2]);
	if (argc >= 4) { // check the optional argument
		string dataRepresentation(argv[3]);
		if (UseJointRepresentation(dataRepresentation) == false) {
			representation = 0;  // Data file contains Cartesian position data, xyzwpr & ext1-3
		}
		else {
			representation = 1;  // Data file contains joint anlge data, j1-j9
		}
	}
	if (argc >= 5) { // check the optional argument for threshold 
		thresholdAxisNumber = atoi(argv[4]);
		
		if ((thresholdAxisNumber > 0) && (thresholdAxisNumber <= 6)) {
			doThreshold = true;
		}
		else {
			doThreshold = false;
		}
	}

	if (argc == 6) {  // check optional buffer stack size
		// allow user to fill the buffer before start handshecking 
		packetStack = atoi(argv[5]);
		if (packetStack >= 10) {
			packetStack = 9;
		}
		else {
			if (packetStack < 0) {
				packetStack = 0;
			}
		}
		cout << "packet stack size : " << packetStack << endl;
	}

	lineCount = CheckDataFile(inName, &lineDataCount, &useTab);
	if (lineCount == 0) {
		// Error should already posted. Just return
		system("pause");
		return 0;
	}

	if (ReadDataFile(&posDataQueue, inName, lineDataCount, useTab) == false) {
		// Error should have been posted. Just return.
		system("pause");
		return 0;
	}

	cout << "number of lines read: " << lineCount << " queue size: " << posDataQueue.size() << endl;
	// DebugQueue(&posDataQueue);

	// initialize the current joint angle array
	for (int idx = 0; idx < 9; idx++) {
		curJoint[idx] = 0.0f;
	}

	// Now, do data exchange 
	InitStartPacket(&startPacket);
	InitStopPacket(&stopPacket);

	// position data reading OK
	// make connection with robot controller
	WSADATA wsaData;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		cout << "WSAStartup failed: " << iResult << endl;
		system("pause");
		return 0;
	}

	// set up remote robot IP address
	robot_addr.sin_addr.s_addr = inet_addr(argv[2]);
	robot_addr.sin_family = AF_INET;
	robot_addr.sin_port = htons(ROBOT_PORT);

	socketID = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	DWORD timeout = 1000;
	int iError = setsockopt(socketID, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(DWORD));

	// do connect: make the UDP socket more efficient
	connect(socketID, (struct sockaddr *)&robot_addr, sizeof(robot_addr));

	// UDP socket does not need to connect to the server, just send the packets
	// send out the start packet to start data exchange:
	// if (sendto(socketID, (const char *)(&startPacket), sizeof(startPacket), 0, (struct sockaddr *) &robot_addr, sizeof(robot_addr)) == SOCKET_ERROR) {
	if (sendto(socketID, (const char *)(&startPacket), sizeof(startPacket), 0, NULL, 0) == SOCKET_ERROR) {
		cout << "Cannot send start packet" << endl;
		system("pause");
		return 0;
	}

	// get threshold one axis at a time
	// NOTE: set blocking read in recvfrom should work as well.
	if (doThreshold == true) {
		/* get this specific axes vel/acc/jerk treashold */
		InitThresholdPacket(&thresholdPacket, thresholdAxisNumber, 2);
		sendto(socketID, (char *)&thresholdPacket, sizeof(thresholdPacket), 0, NULL, 0);

		bool waitThreshold = true;
		while (waitThreshold) {
			int receiveSize = recvfrom(socketID, (char *)&jerkThresholdPacket, sizeof(jerkThresholdPacket), 0, NULL, 0);
			if (receiveSize == sizeof(jerkThresholdPacket)) {
				/* got the threshold data */
				waitThreshold = false;
			}
		}

		/* get acc trehshold packet */
		InitThresholdPacket(&thresholdPacket, thresholdAxisNumber, 1);
		sendto(socketID, (char *)&thresholdPacket, sizeof(thresholdPacket), 0, NULL, 0);

		waitThreshold = true;
		while (waitThreshold) {
			int receiveSize = recvfrom(socketID, (char *)&accThresholdPacket, sizeof(accThresholdPacket), 0, NULL, 0);
			if (receiveSize == sizeof(accThresholdPacket)) {
				/* got the threshold data */
				waitThreshold = false;
			}
		}

		/* get velocity packet */
		InitThresholdPacket(&thresholdPacket, thresholdAxisNumber, 0);
		sendto(socketID, (char *)&thresholdPacket, sizeof(thresholdPacket), 0, NULL, 0);

		waitThreshold = true;
		while (waitThreshold) {
			int receiveSize = recvfrom(socketID, (char *)&velThresholdPacket, sizeof(velThresholdPacket), 0, NULL, 0);
			if (receiveSize == sizeof(velThresholdPacket)) {
				/* got the threshold data */
				waitThreshold = false;
			}
		}
	}

	bool isRobotReady = false;
	// TODO: add time out !!
	while (!isRobotReady) { // wait for the robot controller be ready
		// int receiveSize = recvfrom(socketID, (char *)&statusPacket, sizeof(statusPacket), 0, (struct sockaddr *)&robot_addr, &addr_len);
		int receiveSize = recvfrom(socketID, (char *)&statusPacket, sizeof(statusPacket), 0, NULL, 0);
		//cout << "Status PacketType (RAW): " << statusPacket.packetType << endl;
		//cout << "Status PacketType (ntohl): " << ntohl(statusPacket.packetType) << endl;
		//
		//cout << "Status VersionNum (RAW): " << statusPacket.versionNo << endl;
		//cout << "Status VersionNum (ntohl): " << ntohl(statusPacket.versionNo) << endl;

		//cout << "Status SeqeunceNum (RAW): " << statusPacket.sequenceNo << endl;
		//cout << "Status SeqeunceNum (ntohl): " << ntohl(statusPacket.sequenceNo) << endl;

		//cout << "Status status (RAW): " << statusPacket.status << endl;
		//cout << "Status status (ntohl): " << ntohl(statusPacket.status) << endl;

		//cout << "Status ReadIO (RAW): " << statusPacket.readIOType << endl;
		//cout << "Status ReadIo (ntohl): " << ntohl(statusPacket.readIOType) << endl;

		//cout << "Status ReadIndex (RAW): " << statusPacket.readIOIndex << endl;
		//cout << "Status ReadIndex (ntohl): " << ntohl(statusPacket.readIOIndex) << endl;

		//cout << "Status ReadIOMask (RAW): " << statusPacket.readIOMask << endl;
		//cout << "Status ReadIOMask (ntohl): " << ntohl(statusPacket.readIOMask) << endl;

		//cout << "Status ReadIOValue (RAW): " << statusPacket.readIOValue << endl;
		//cout << "Status ReadIoValue (ntohl): " << ntohl(statusPacket.readIOValue) << endl;

		//cout << "Status timestamp (RAW): " << statusPacket.timeStamp << endl;
		//cout << "Status timestamp (ntohl): " << ntohl(statusPacket.timeStamp) << endl;

		//cout << "Status ReadIOValue (RAW): " << statusPacket.readIOValue << endl;
		//cout << "Status ReadIoValue (ntohl): " << ntohl(statusPacket.readIOValue) << endl;


		// Received a packet, check to see if robot is ready to receive a command position
		if ((statusPacket.status & 1) > 0) {
			for (int idx = 0; idx < 6; idx++) {
				u_long joint = ntohl(statusPacket.jontAngle[idx]);
				//cout << "Status Joint (RAW): " << statusPacket.jontAngle[idx] << endl;
				//cout << "Status Joint (ntohl): " << joint << endl;
				curJoint[idx] = reinterpret_cast<float &> (joint);
			}
			isRobotReady = true;;
		}
		//if ((statusPacket.status & 1) > 0) {
		//	for (int idx = 0; idx < 6; idx++) {
		//		u_long pos = ntohl(statusPacket.position[idx]);
		//		cout << "Status Joint (RAW): " << statusPacket.position[idx] << endl;
		//		cout << "Status Joint (ntohl): " << pos << endl;
		//		curJoint[idx] = reinterpret_cast<float&> (pos);
		//	}
		//	isRobotReady = true;;
		//}
		//if ((statusPacket.status & 1) > 0) {
		//	for (int idx = 0; idx < 6; idx++) {
		//		u_long curr = ntohl(statusPacket.current[idx]);
		//		cout << "Status Joint (RAW): " << statusPacket.current[idx] << endl;
		//		cout << "Status Joint (ntohl): " << curr << endl;
		//		curJoint[idx] = reinterpret_cast<float&> (curr);
		//	}
		//	isRobotReady = true;;
		//}
	} // end of of waiting for the status bit

	// send number of packet to controller to fill the packet stack in the controller
	if (packetStack > 0) {
		for (int idx = 0; idx < packetStack; idx++) {
			PositionData_T pos = posDataQueue.front();
			if (posDataQueue.size() > 1) {
				lastData = 0;
			}
			else {  // this is the last command data sent to robot controller
				lastData = 1;
			}

			// InitCommandPacket(&commandPacket, htonl(statusPacket.sequenceNo), &(pos.data[0]), representation, lastData);
			InitCommandPacket(&commandPacket, idx + startSeqID, &(curJoint[0]), representation, lastData);
			posDataQueue.pop();

			// send the command packet out
			// sendto(socketID, (char *)&commandPacket, sizeof(commandPacket), 0, (struct sockaddr *) &robot_addr, sizeof(robot_addr));
			sendto(socketID, (char *)&commandPacket, sizeof(commandPacket), 0, NULL, 0);
		}
	}

	doDataExchange = true;
	// Start to send the command packets
	while (!posDataQueue.empty() && doDataExchange) {
		PositionData_T pos = posDataQueue.front();
		if (posDataQueue.size() > 1) {
			lastData = 0;
		}
		else {  // this is the last command data sent to robot controller
			lastData = 1;
		}

		if (packetStack > 0) {
			seqID = ntohl(statusPacket.sequenceNo) + packetStack + startSeqID -1;
		}
		else {
			seqID = ntohl(statusPacket.sequenceNo);
		}

		// InitCommandPacket(&commandPacket, htonl(statusPacket.sequenceNo), &(pos.data[0]), representation, lastData);
		// InitCommandPacket(&commandPacket, htonl(statusPacket.sequenceNo), &(curJoint[0]), representation, lastData);
		InitCommandPacket(&commandPacket, seqID, &(pos.data[0]), representation, lastData);
		posDataQueue.pop();


		// send the command packet out
		// sendto(socketID, (char *)&commandPacket, sizeof(commandPacket), 0, (struct sockaddr *) &robot_addr, sizeof(robot_addr));
		sendto(socketID, (char *)&commandPacket, sizeof(commandPacket), 0, NULL, 0);

		// Wait for the robot status packet */
		// int receiveSize = recvfrom(socketID, (char *)&statusPacket, sizeof(statusPacket), 0, (struct sockaddr *)&robot_addr, &addr_len);
		int receiveSize = recvfrom(socketID, (char *)&statusPacket, sizeof(statusPacket), 0, NULL, 0);
		// check for the status
		if ((statusPacket.status & 5) != 5) {
			cout << "** CONTROLLER ERROR at sequence ID: " << seqID << " **" << endl;
			doDataExchange = false;
		}
		else {
			for (int idx = 0; idx < 6; idx++) {
				u_long joint = ntohl(statusPacket.jontAngle[idx]);
				curJoint[idx] = reinterpret_cast<float &> (joint);
			}
		}
	}

	if (packetStack > 0) {
		// wait for the controller to processed all packets in the buffer before sending the last packet
		Sleep(packetStack * 8);
	}

	// Send the last data
	// sendto(socketID, (const char *)&stopPacket, sizeof(stopPacket), 0, (struct sockaddr *) &robot_addr, sizeof(robot_addr));
	sendto(socketID, (const char *)&stopPacket, sizeof(stopPacket), 0, NULL, 0);
	// clean up
	closesocket(socketID);
	WSACleanup();

	// 
	// cout << "Current Joint Angle: ";
	// for (int idx = 0; idx < 6; idx++) {
	//	printf("%12.6f ", curJoint[idx]);
	// }
	//cout << endl;

	if (doDataExchange != false) {
		cout << "Motion Completed" << endl;
	}

	// print out threshold data, if set.
	if (doThreshold == TRUE) {
		WriteThresholdData(&velThresholdPacket, &accThresholdPacket, &jerkThresholdPacket);
	}
	system("pause");

	return 0;


}

