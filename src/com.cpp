#include "md/global.hpp"
#include "md/main.hpp"
#include "md/com.hpp"
#include "md/robot.hpp"
#include "iostream"

void myAnalyzer_monitor(BYTE byArray[]);


serial::Serial ser;
//
// Get the low and high byte from short
IByte Short2Byte(short sIn)
{
    IByte Ret;

    Ret.byLow = sIn & 0xff; // 16자리 중 뒤에 8자리
    Ret.byHigh = sIn>>8 & 0xff; // 16자리 중 앞에 8자리

    return Ret;
}

// Get the low and high short data from long
shortByte Long4Byte(int longData)
{
    shortByte Ret;

    Ret.by0 = longData & 0xff; // long Data 32자리 중 뒤 8자리
    Ret.by1 = longData>>8 & 0xff; // long Data 32자리 중 9~16 자리
    Ret.by2 = longData>>16 & 0xff; // long Data 32자리 중 17~24 자리
    Ret.by3 = longData>>24 & 0xff; // long Data 32자리 중 25~32 자리

    return Ret;
}



// Make short data from two bytes
int Byte2Short(BYTE byLow, BYTE byHigh)
{
    return (byLow | (int)byHigh<<8);
}

// Make long data from four bytes
int Byte2LInt(BYTE byData1, BYTE byData2, BYTE byData3, BYTE byData4)
{
    return ((int)byData1 | (int)byData2<<8 | (int)byData3<<16 | (int)byData4<<24);
}
//Function for stabilizing position values
int MovingAverage(void)
{
    static BYTE byCnt;

    switch(byCnt){
        case 0:
            Com.lMoving[_X][0]     = Com.lTempPosi[_X];
            Com.lMoving[_Y][0]     = Com.lTempPosi[_Y];
            Com.lMoving[_THETA][0] = Com.sTempTheta;

            Com.lPosi[_X] = Com.lTempPosi[_X];
            Com.lPosi[_Y] = Com.lTempPosi[_Y];
            Com.lPosi[_THETA] = Com.lTempPosi[_THETA];

            byCnt = 1;
            break;
        case 1:
            Com.lMoving[_X][1]     = Com.lTempPosi[_X];
            Com.lMoving[_Y][1]     = Com.lTempPosi[_Y];
            Com.lMoving[_THETA][1] = Com.sTempTheta;

            Com.lPosi[_X] = Com.lTempPosi[_X];
            Com.lPosi[_Y] = Com.lTempPosi[_Y];
            Com.lPosi[_THETA] = Com.lTempPosi[_THETA];
            byCnt = 2;
            break;
        case 2:
            Com.lMoving[_X][2]     = Com.lTempPosi[_X];
            Com.lMoving[_Y][2]     = Com.lTempPosi[_Y];
            Com.lMoving[_THETA][2] = Com.sTempTheta;

            Com.lPosi[_X] = (Com.lMoving[_X][0] + Com.lMoving[_X][1] + Com.lMoving[_X][2])/3;
            Com.lPosi[_Y] = (Com.lMoving[_Y][0] + Com.lMoving[_Y][1] + Com.lMoving[_Y][2])/3;
            Com.sTheta    = (Com.lMoving[_THETA][0] + Com.lMoving[_THETA][1] + Com.lMoving[_THETA][2])/3;

            byCnt = 3;
            break;
        case 3:
            Com.lMoving[_X][0] = Com.lMoving[_X][1];
            Com.lMoving[_X][1] = Com.lMoving[_X][2];

            Com.lMoving[_Y][0] = Com.lMoving[_Y][1];
            Com.lMoving[_Y][1] = Com.lMoving[_Y][2];

            Com.lMoving[_THETA][0] = Com.lMoving[_THETA][1];
            Com.lMoving[_THETA][1] = Com.lMoving[_THETA][2];

            Com.lMoving[_X][2]     = Com.lTempPosi[_X];
            Com.lMoving[_Y][2]     = Com.lTempPosi[_Y];
            Com.lMoving[_THETA][2] = Com.sTempTheta;

            Com.lPosi[_X] = (Com.lMoving[_X][0] + Com.lMoving[_X][1] + (double)Com.lMoving[_X][2])/3;
            Com.lPosi[_Y] = (Com.lMoving[_Y][0] + Com.lMoving[_Y][1] + (double)Com.lMoving[_Y][2])/3;
            Com.sTheta    = (Com.lMoving[_THETA][0] + (double)Com.lMoving[_THETA][1] + Com.lMoving[_THETA][2])/3 ;

            break;
    }
    return SUCCESS;
}

//Initialize serial communication in ROS
int InitSerial(void)
{
    try
    {
        ser.setPort("/dev/ttyUSB_motor"); // port simlink
        ser.setBaudrate(Com.nBaudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1667); //1667 when baud is 57600, 0.6ms
        ser.setTimeout(to);                                        //2857 when baud is 115200, 0.35ms
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    if(ser.isOpen())
        ROS_INFO_STREAM("Serial Port initialized");
    else
        return -1;
}

//Initialize the lower platform posture
int InitSetParam(void)
{
    int nArray[3];

    Log.byID = ID_ALL;
    Com.fgControlstate = ON;

    nArray[0] = Com.nDiameter;
    nArray[1] = Com.nWheelLength;
    nArray[2] = Com.nGearRatio;

    PutMdData(PID_ROBOT_PARAM, Com.nRMID, nArray);

    return SUCCESS;
}
int InitSetSlowStart(void)
{
    int nArray[3];

    Log.byID = ID_ALL;
    Com.fgControlstate = ON;

    nArray[0] = Com.nSlowstart;

    PutMdData(PID_SLOW_START, Com.nRMID, nArray);

    return SUCCESS;
}
int InitSetSlowDown(void)
{
    int nArray[3];

    Log.byID = ID_ALL;
    Com.fgControlstate = ON;

    nArray[0] = Com.nSlowdown;

    PutMdData(PID_SLOW_DOWN, Com.nRMID, nArray);

    return SUCCESS;
}



//for sending the data
int PutMdData(BYTE byPID, BYTE byMID, int nArray[])
{
    IByte iData;
    BYTE byPidDataSize, byDataSize, i, j;
    static BYTE byTempDataSum;

    for(j = 0; j <MAX_PACKET_SIZE; j++) Com.bySndBuf[j] = 0; // 송신 데이터 배열 초기화

    Com.bySndBuf[0] = byMID;
    Com.bySndBuf[1] = Com.nIDPC;
    Com.bySndBuf[2] = 1;
    Com.bySndBuf[3] = byPID;

    switch(byPID){

        case PID_REQ_PID_DATA:  //PID 4 데이터 요청

            byDataSize      = 1;
            byPidDataSize   = 7;
            byTempDataSum   = 0;

            Com.bySndBuf[4] = byDataSize;
            Com.bySndBuf[5] = (BYTE)nArray[0];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;            


        case PID_TQ_OFF:  // PID 5 자연정지명령

            byDataSize      = 1;
            byPidDataSize   = 7;
            byTempDataSum   = 0;

            Com.bySndBuf[4] = byDataSize;
            Com.bySndBuf[5] = (BYTE)nArray[0];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;         


        case PID_COMMAND:

            byDataSize      = 1;
            byPidDataSize   = 7;
            byTempDataSum   = 0;

            Com.bySndBuf[4] = byDataSize;
            Com.bySndBuf[5] = (BYTE)nArray[0];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;


        case PID_POSI_RESET:

            byDataSize    = 1;
            byPidDataSize = 7;
            byTempDataSum = 0;

            Com.bySndBuf[4] = byDataSize;
            Com.bySndBuf[5] = (BYTE)nArray[0];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;


        case PID_SLOW_START:

            byDataSize    = 2;
            byPidDataSize = 8;
            byTempDataSum = 0;

            Com.bySndBuf[4] = byDataSize;
            iData = Short2Byte((short)nArray[0]);
            Com.bySndBuf[5] = iData.byLow;
            Com.bySndBuf[6] = iData.byHigh;

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;

            
        case PID_SLOW_DOWN:

            byDataSize    = 2;
            byPidDataSize = 8;
            byTempDataSum = 0;

            Com.bySndBuf[4] = byDataSize;
            iData = Short2Byte((short)nArray[0]);
            Com.bySndBuf[5] = iData.byLow;
            Com.bySndBuf[6] = iData.byHigh;

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;


        case PID_PNT_VEL_CMD:

            byDataSize    = 7;
            byPidDataSize = 13;
            byTempDataSum = 0;

            Com.bySndBuf[4]  = byDataSize;
            Com.bySndBuf[5]  = ENABLE;
            Com.bySndBuf[6]  = nArray[0];
            Com.bySndBuf[7]  = nArray[1];
            Com.bySndBuf[8]  = ENABLE;
            Com.bySndBuf[9]  = nArray[2];
            Com.bySndBuf[10] = nArray[3];
            Com.bySndBuf[11] = nArray[4];


            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);


            break;


        case PID_ROBOT_PARAM:

            byDataSize       = 6;
            byPidDataSize    = 12;
            byTempDataSum    = 0;

            Com.bySndBuf[4]  = byDataSize;

            iData = Short2Byte((short)nArray[0]);  //diameter
            Com.bySndBuf[5]  = iData.byLow;
            Com.bySndBuf[6]  = iData.byHigh;

            iData = Short2Byte((short)nArray[1]);  //wheellength
            Com.bySndBuf[7]  = iData.byLow;
            Com.bySndBuf[8]  = iData.byHigh;

            iData = Short2Byte((short)nArray[2]);  //reduction ratio
            Com.bySndBuf[9]  = iData.byLow;
            Com.bySndBuf[10] = iData.byHigh;

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;


        case PID_ROBOT_CMD:            //PID_NUMBER -> 247

            byDataSize       = 6;
            byPidDataSize    = 12;
            byTempDataSum    = 0;

            Com.bySndBuf[4]  = byDataSize;

            Com.bySndBuf[5]  = (BYTE)nArray[0];   //kind of control

            iData = Short2Byte((short)nArray[1]); //linear velocity(mm/s)
            Com.bySndBuf[6]  = iData.byLow;
            Com.bySndBuf[7]  = iData.byHigh;

            iData = Short2Byte((short)nArray[2]); //angular velocity(deg/s)
            Com.bySndBuf[8]  = iData.byLow;
            Com.bySndBuf[9]  = iData.byHigh;

            Com.bySndBuf[10] = (BYTE)nArray[3];   //reset encoder value(x, y, theta)

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);
            break;


        case PID_PNT_TQ_OFF:

            byDataSize    = 3;
            byPidDataSize = 9;
            byTempDataSum = 0;

            Com.bySndBuf[4]  = byDataSize;
            Com.bySndBuf[5]  = (BYTE)nArray[0];
            Com.bySndBuf[6]  = (BYTE)nArray[1];
            Com.bySndBuf[7]  = (BYTE)nArray[2];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;


        case PID_PNT_BRAKE:

            byDataSize    = 3;
            byPidDataSize = 9;
            byTempDataSum = 0;

            Com.bySndBuf[4]  = byDataSize;
            Com.bySndBuf[5]  = (BYTE)nArray[0];
            Com.bySndBuf[6]  = (BYTE)nArray[1];
            Com.bySndBuf[7]  = (BYTE)nArray[2];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;


        case PID_TQ_LIMIT_SW_VAL: // PID 187 전류값에 의한 가상 리미트스위치의 구현

            byDataSize    = 2;
            byPidDataSize = 8;
            byTempDataSum = 0;

            Com.bySndBuf[4]  = byDataSize;

            Com.bySndBuf[5]  = (BYTE)nArray[0];
            Com.bySndBuf[6]  = (BYTE)nArray[1];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;


        case PID_VEL_CMD: // PID 130 속도 제어

            byDataSize    = 2;
            byPidDataSize = 8;
            byTempDataSum = 0;

            Com.bySndBuf[4]  = byDataSize;

            Com.bySndBuf[5]  = (BYTE)nArray[0];
            Com.bySndBuf[6]  = (BYTE)nArray[1];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;


        case PID_POSI_VEL_CMD: // 위치 제어 PID 219

            byDataSize    = 6;
            byPidDataSize = 12;
            byTempDataSum = 0;

            Com.bySndBuf[4]  = byDataSize;
            Com.bySndBuf[5]  = (BYTE)nArray[0];
            Com.bySndBuf[6]  = (BYTE)nArray[1];
            Com.bySndBuf[7]  = (BYTE)nArray[2];
            Com.bySndBuf[8]  = (BYTE)nArray[3];
            Com.bySndBuf[9]  = (BYTE)nArray[4];
            Com.bySndBuf[10] = (BYTE)nArray[5];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;

        
        case PID_INC_POSI_VEL_CMD: // 위치 제어 PID 220

            byDataSize    = 6;
            byPidDataSize = 12;
            byTempDataSum = 0;

            Com.bySndBuf[4]  = byDataSize;
            Com.bySndBuf[5]  = (BYTE)nArray[0];
            Com.bySndBuf[6]  = (BYTE)nArray[1];
            Com.bySndBuf[7]  = (BYTE)nArray[2];
            Com.bySndBuf[8]  = (BYTE)nArray[3];
            Com.bySndBuf[9]  = (BYTE)nArray[4];
            Com.bySndBuf[10] = (BYTE)nArray[5];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;

    }


    return SUCCESS;
}


long *GetMdData(BYTE byPID)
{
    long *lArray = (long*)malloc(sizeof(long) * 8);

    switch(byPID) {
        case PID_ROBOT_PARAM:
            lArray[0] = (long)Com.sSetDia;
            lArray[1] = (long)Com.sSetWheelLen;
            lArray[2] = (long)Com.sSetGear;

            break;
        case PID_ROBOT_MONITOR2:
            lArray[0] = (long)Com.sVoltIn;
            lArray[1] = (long)Com.byUS1;
            lArray[2] = (long)Com.byUS2;
            lArray[3] = (long)Com.byUS3;
            lArray[4] = (long)Com.byUS4;
            lArray[5] = (long)Com.byPlatStatus;
            lArray[6] = (long)Com.byDocStatus;
            break;

        case PID_ROBOT_MONITOR:
            lArray[0] = Com.lPosi[_X];
            lArray[1] = Com.lPosi[_Y];
            lArray[2] = Com.sTheta;

            break;
    }
    return lArray;
}

int MdReceiveProc(void) //save the identified serial data to defined variable according to PID NUMBER data
{
    BYTE byRcvRMID, byRcvTMID, byRcvID, byRcvPID, byRcvDataSize;

    byRcvRMID     = Com.byRcvBuf[0];
    byRcvTMID     = Com.byRcvBuf[1];
    byRcvID       = Com.byRcvBuf[2];
    byRcvPID      = Com.byRcvBuf[3];
    byRcvDataSize = Com.byRcvBuf[4];

    switch(byRcvPID){

        case PID_MAIN_DATA: // 193

            std::cout << "analyzing ... ..." << std::endl;

            Com.rpm = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);
            Com.current = Byte2Short(Com.byRcvBuf[7], Com.byRcvBuf[8]);
            // Com.position = Byte2LInt(Com.byRcvBuf[15], Com.byRcvBuf[16], Com.byRcvBuf[17], Com.byRcvBuf[18]); // 위치값을 제대로 읽지 못하는 문제가 있음

            // for(int i = 0; i < 23; i++)
            // {
            //     std::cout << (int)Com.byRcvBuf[i] << " ";
            // }
            // std::cout <<  std::endl;

            // std::cout << "position Data (Byte2Lint) -> " << Byte2LInt(Com.byRcvBuf[15], Com.byRcvBuf[16], Com.byRcvBuf[17], Com.byRcvBuf[18]) << std::endl;
            // Com.position = int(Byte2LInt(Com.byRcvBuf[15], Com.byRcvBuf[16], Com.byRcvBuf[17], Com.byRcvBuf[18]));
            // std::cout << "Com.position : "<< Com.position << std::endl;
            // cout << "Com.positon adress : " << &Com.position << endl;



            // std::cout << "position Data (Byte2Lint) -> " << Byte2LInt(Com.byRcvBuf[15], Com.byRcvBuf[16], Com.byRcvBuf[17], Com.byRcvBuf[18]) << std::endl;
            monitor_position = int(Byte2LInt(Com.byRcvBuf[15], Com.byRcvBuf[16], Com.byRcvBuf[17], Com.byRcvBuf[18]));
            // std::cout << "monitor_position : "<< monitor_position << std::endl;
            


        case PID_MONITOR: //196
            Com.rpm = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);
            Com.current = Byte2Short(Com.byRcvBuf[7], Com.byRcvBuf[8]);
            Com.position = Byte2LInt(Com.byRcvBuf[12], Com.byRcvBuf[13], Com.byRcvBuf[14], Com.byRcvBuf[15]); // 위치값을 제대로 읽지 못하는 문제가 있음


        case PID_POSI_DATA: //197    TMID, 183, ID, 197, 4, D1, D2, D3, D4, CHK
            Com.position = Byte2LInt(Com.byRcvBuf[5], Com.byRcvBuf[6], Com.byRcvBuf[7], Com.byRcvBuf[8]); //Make long data from four bytes




        case PID_VOLT_IN: //143

            Com.sTempVoltIn = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);
            Com.sSumVolt += Com.sTempVoltIn;

            if(++Com.byCntVoltAver == 10)
            {
                Com.byCntVoltAver = 0;
                Com.sVoltIn = (Com.sSumVolt/10) + 7;
                Com.sSumVolt = 0;
            }
            break;

        case PID_PNT_MAIN_DATA: //210
            Com.sMotorRPM[MOT_LEFT]  = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);
            Com.sCurrent[MOT_LEFT]   = Byte2Short(Com.byRcvBuf[7], Com.byRcvBuf[8]);
            Com.byStatus[MOT_LEFT]   = Com.byRcvBuf[9];
            Com.fgAlarm[MOT_LEFT]    = Com.byRcvBuf[9] & BIT0;
            Com.fgCtrlFail[MOT_LEFT] = Com.byRcvBuf[9] & BIT1;
            Com.fgOverVolt[MOT_LEFT] = Com.byRcvBuf[9] & BIT2;
            Com.fgOverTemp[MOT_LEFT] = Com.byRcvBuf[9] & BIT3;
            Com.fgOverLoad[MOT_LEFT] = Com.byRcvBuf[9] & BIT4;
            Com.fgHallFail[MOT_LEFT] = Com.byRcvBuf[9] & BIT5;
            Com.fgInvVel[MOT_LEFT]   = Com.byRcvBuf[9] & BIT6;
            Com.fgStall[MOT_LEFT]    = Com.byRcvBuf[9] & BIT7;

            Com.lMotorPosi[MOT_LEFT] = Byte2LInt(Com.byRcvBuf[10], Com.byRcvBuf[11],
                Com.byRcvBuf[12], Com.byRcvBuf[13]);

            Com.sMotorRPM[MOT_RIGHT]  = Byte2Short(Com.byRcvBuf[14], Com.byRcvBuf[15]);
            Com.sCurrent[MOT_RIGHT]   = Byte2Short(Com.byRcvBuf[16], Com.byRcvBuf[17]);
            Com.byStatus[MOT_RIGHT]   = Com.byRcvBuf[18];
            Com.fgAlarm[MOT_RIGHT]    = Com.byRcvBuf[18] & BIT0;
            Com.fgCtrlFail[MOT_RIGHT] = Com.byRcvBuf[18] & BIT1;
            Com.fgOverVolt[MOT_RIGHT] = Com.byRcvBuf[18] & BIT2;
            Com.fgOverTemp[MOT_RIGHT] = Com.byRcvBuf[18] & BIT3;
            Com.fgOverLoad[MOT_RIGHT] = Com.byRcvBuf[18] & BIT4;
            Com.fgHallFail[MOT_RIGHT] = Com.byRcvBuf[18] & BIT5;
            Com.fgInvVel[MOT_RIGHT]   = Com.byRcvBuf[18] & BIT6;
            Com.fgStall[MOT_RIGHT]    = Com.byRcvBuf[18] & BIT7;

            Com.lMotorPosi[MOT_RIGHT] = Byte2LInt(Com.byRcvBuf[19], Com.byRcvBuf[20],
                    Com.byRcvBuf[21], Com.byRcvBuf[22]);

            //ROS_INFO("%d  %d", Com.sCurrent[MOT_LEFT], Com.sCurrent[MOT_RIGHT]);
            //ROS_INFO("%d, %ld, %ld, %d, %ld, %ld, %d", Com.byStatus[MOT_LEFT], (long)Com.sMotorRPM[MOT_LEFT], Com.lMotorPosi[MOT_LEFT],
            //         Com.byStatus[MOT_RIGHT], (long)Com.sMotorRPM[MOT_RIGHT], Com.lMotorPosi[MOT_RIGHT], Com.fgResetEncoder);


            MotData2RobotPosture(Com.byStatus[MOT_LEFT], (long)Com.sMotorRPM[MOT_LEFT], Com.lMotorPosi[MOT_LEFT],
                                 Com.byStatus[MOT_RIGHT], (long)Com.sMotorRPM[MOT_RIGHT], Com.lMotorPosi[MOT_RIGHT]);

            break;

        case PID_ROBOT_PARAM:  //247
            Com.sSetDia      = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);
            Com.sSetWheelLen = Byte2Short(Com.byRcvBuf[7], Com.byRcvBuf[8]);
            Com.sSetGear     = Byte2Short(Com.byRcvBuf[9], Com.byRcvBuf[10]);
            break;

        case PID_ROBOT_MONITOR2:   //224
            //Com.sVoltIn      = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);
            Com.byUS1        = Com.byRcvBuf[7];
            Com.byUS2        = Com.byRcvBuf[8];
            Com.byUS3        = Com.byRcvBuf[9];
            Com.byUS4        = Com.byRcvBuf[10];
            Com.byPlatStatus = Com.byRcvBuf[11];
            Com.bEmerSW      = Com.byRcvBuf[11] & BIT0;
            Com.bBusy        = (Com.byRcvBuf[11] & BIT1)>>1;
            Com.bBumper1     = (Com.byRcvBuf[11] & BIT3)>>3;
            Com.bBumper2     = (Com.byRcvBuf[11] & BIT4)>>4;
            Com.bBumper3     = (Com.byRcvBuf[11] & BIT5)>>5;
            Com.bBumper4     = (Com.byRcvBuf[11] & BIT6)>>6;
            Com.byDocStatus  = Com.byRcvBuf[12];
            Com.bDocComple   = Com.byRcvBuf[12] & BIT0;
            Com.bChargeState = (Com.byRcvBuf[12] & BIT1)>>1;
            Com.bCharComple  = (Com.byRcvBuf[12] & BIT2)>>2;
            Com.bIr1         = (Com.byRcvBuf[12] & BIT4)>>4;
            Com.bIr2         = (Com.byRcvBuf[12] & BIT5)>>5;
            Com.bIr3         = (Com.byRcvBuf[12] & BIT6)>>6;
            Com.bRccState    = (Com.byRcvBuf[12] & BIT7)>>7;
            break;

        case PID_ROBOT_MONITOR:   //253
            Com.lTempPosi[_X] = Byte2LInt(Com.byRcvBuf[5], Com.byRcvBuf[6], Com.byRcvBuf[7], Com.byRcvBuf[8]);
            Com.lTempPosi[_Y] = Byte2LInt(Com.byRcvBuf[9], Com.byRcvBuf[10], Com.byRcvBuf[11], Com.byRcvBuf[12]);
            Com.sTempTheta    = Byte2Short(Com.byRcvBuf[13], Com.byRcvBuf[14]);
            break;
    }
    return SUCCESS;
}

int AnalyzeReceivedData(BYTE byArray[], BYTE byBufNum) //Analyze the communication data
{
    ros::NodeHandle n;
    //ros::Publisher string_pub;
    ros::Publisher string_pub = n.advertise<std_msgs::String>("string_com_topic", 100);
    ros::Time stamp;


    static BYTE byChkSec;
    static long lExStampSec, lExStampNsec;
    BYTE i, j;

    if(Com.byPacketNum >= MAX_PACKET_SIZE)
    {
        Com.byStep =0;
        return FAIL;
    }

    for(j = 0; j < byBufNum; j++)
    {
        switch(Com.byStep){
            case 0:    //Put the reading machin id after checking the data
                if(byArray[j] == Com.nIDPC)
                {
                    for(i = 0; i < MAX_PACKET_SIZE; i++) Com.byRcvBuf[i] = 0;
                    Com.fgPacketOK  = 0;
                    Com.byPacketNum = 0;
                    Com.byChkComError = 0;

                    Com.byChkSum = byArray[j];
                    Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                    Com.byStep++;

                }
                else
                {
                    Com.byTotalRcvDataNum = 0;
                    Com.byPacketNum = 0;
                    Com.byChkComError++;
                }
                break;
            case 1:    //Put the transmitting machin id after checking the data
                if((byArray[j] == Com.nIDMDUI) || (byArray[j] == Com.nIDMDT))
                {
                    Com.byChkSum += byArray[j];
                    Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                    Com.byStep++;
                    Com.byChkComError = 0;
                }
                else
                {
                    Com.byStep      = 0;
                    Com.fgPacketOK  = 0;
                    Com.byPacketNum = 0;
                    Com.byChkComError++;

                }
                break;

            case 2:    //Check ID
                if(byArray[j] == 1|| byArray[j] == ID_ALL)
                {
                    Com.byChkSum += byArray[j];
                    Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                    Com.byStep++;
                    Com.byChkComError = 0;
                }
                else
                {
                    Com.byStep = 0;
                    Com.byPacketNum = 0;
                    Com.byChkComError++;
                }
                break;
             case 3:    //Put the PID number into the array
                Com.byChkSum += byArray[j];
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                Com.byStep++;
                break;

             case 4:    //Put the DATANUM into the array
                Com.byMaxDataNum = byArray[j];
                Com.byDataNum = 0;
                Com.byChkSum += byArray[j];
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                Com.byStep++;
                break;

             case 5:    //Put the DATA into the array
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                Com.byChkSum += byArray[j];

                if(++Com.byDataNum >= MAX_DATA_SIZE)
                {
                    Com.byStep = 0;
                    Com.byTotalRcvDataNum = 0;
                    break;
                }

                if(Com.byDataNum>= Com.byMaxDataNum) Com.byStep++;
                break;

             case 6:    //Put the check sum after Checking checksum
                Com.byChkSum += byArray[j];
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j];

                if(Com.byChkSum == 0)
                {
                    Com.fgPacketOK   = 1;
                    Com.fgComDataChk = 1;
                    Com.byDataNum    = 0;
                    Com.byMaxDataNum = 0;
                }

                Com.byStep = 0;
                Com.byTotalRcvDataNum = 0;

                break;

            default:
                Com.byStep = 0;
                Com.fgComComple = ON;
                break;
        }

        if(Com.fgPacketOK)
        {
            Com.fgPacketOK   = 0;
            Com.byPacketSize = 0;
            Com.byPacketNum  = 0;


            std_msgs::String com_data_string;
            std::stringstream ss;
            stamp = ros::Time::now();

            if(byChkSec == 0)
            {
                byChkSec = 1;
                lExStampSec = stamp.sec;

            }

            stamp.sec = stamp.sec - lExStampSec;
            stamp.nsec = stamp.nsec / 100000;

            if(stamp.nsec < 10000 && stamp.nsec > 1000) ss << (long)stamp.sec << "." << (long)stamp.nsec;
            else if(stamp.nsec < 1000 && stamp.nsec > 100) ss << (long)stamp.sec << ".0" << (long)stamp.nsec;
            else if(stamp.nsec < 100 && stamp.nsec > 10) ss << (long)stamp.sec << ".00" << (long)stamp.nsec;
            else if(stamp.nsec < 10 && stamp.nsec > 1) ss << (long)stamp.sec << ".000" << (long)stamp.nsec;
            ss << " " << (int)Com.byRcvBuf[0] << " " << (int)Com.byRcvBuf[1] << " " << (int)Com.byRcvBuf[2] << " " << (int)Com.byRcvBuf[3] << " "
                                               << (int)Com.byRcvBuf[4] << " " << (int)Com.byRcvBuf[5] << " " << (int)Com.byRcvBuf[6] << " " << (int)Com.byRcvBuf[7] << " "
                                                 << (int)Com.byRcvBuf[8] << " " << (int)Com.byRcvBuf[9] << " " << (int)Com.byRcvBuf[10] << " " << (int)Com.byRcvBuf[11] << " "
                                                    << (int)Com.byRcvBuf[12] << " " << (int)Com.byRcvBuf[13] << " " << (int)Com.byRcvBuf[14] << " " << (int)Com.byRcvBuf[15] << " "
                                                       << (int)Com.byRcvBuf[16] << " " << (int)Com.byRcvBuf[17] << " " << (int)Com.byRcvBuf[18] << " " << (int)Com.byRcvBuf[19] << " "
                                                         << (int)Com.byRcvBuf[20] << " " << (int)Com.byRcvBuf[21] << " " << (int)Com.byRcvBuf[22] << " " << (int)Com.byRcvBuf[23];

            com_data_string.data = ss.str();
            string_pub.publish(com_data_string);
            lExStampNsec = stamp.nsec;

            MdReceiveProc();                                 //save the identified serial data to defined variable
        }

        if(Com.byChkComError == 10) //while 50ms
        {
            Com.byChkComError = 0;
            Com.byStep = 0;
            Com.byChkSum = 0;
            Com.byMaxDataNum = 0;
            Com.byDataNum = 0;
            for(i = 0; i < MAX_PACKET_SIZE; i++) Com.byRcvBuf[i] = 0;
            j = byBufNum;
        }

    }
    return SUCCESS;
}


int ReceiveDataFromController(void) //Analyze the communication data
{
    BYTE byRcvBuf[250];
    BYTE byBufNumber;

    byBufNumber = ser.available();

    if(byBufNumber != 0)
    {
        motor_connection_flag = ON;

        ser.read(byRcvBuf, byBufNumber);
        
        std::cout << "start analyzing the data" << std::endl;

        AnalyzeReceivedData(byRcvBuf, byBufNumber);

        std::cout << "analyzing fisished" << std::endl;
        
        {
        // ---- 디버깅 ---- //
        // for(int i = 0; i< byBufNumber; i++)
        // {
        //     std::cout << (int)byRcvBuf[i] << " ";
        // }
        // std::cout << std::endl << std::endl;
        }
        


        
        // myAnalyzer_monitor(byRcvBuf);
    }

    return 1;
}

