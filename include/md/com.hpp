#define ID_BLDC_CTRL        1
#define ID_MDUI             2
#define ID_ALL              0xfe

#define PID_REQ_PID_DATA    4
#define PID_TQ_OFF          5
#define PID_BRAKE           6
#define PID_COMMAND         10
#define PID_POSI_RESET      13
#define PID_VEL_CMD         130
#define PID_BAUDRATE        135
#define PID_VOLT_IN         143
#define PID_SLOW_START      153
#define PID_SLOW_DOWN       154
#define PID_PNT_TQ_OFF      174
#define PID_PNT_BRAKE       175
#define PID_TQ_LIMIT_SW_VAL 187
#define PID_MAIN_DATA       193
#define PID_MONITOR         196
#define PID_POSI_DATA       197
#define PID_PNT_VEL_CMD     207
#define PID_PNT_MAIN_DATA   210
#define PID_POSI_VEL_CMD    219
#define PID_INC_POSI_VEL_CMD    220
#define PID_ROBOT_PARAM     247
#define PID_ROBOT_MONITOR   253
#define PID_ROBOT_MONITOR2  224
#define PID_ROBOT_CMD       252

#define MAX_PACKET_SIZE     26
#define MAX_DATA_SIZE       21

#define MOT_LEFT            0
#define MOT_RIGHT           1

#define REQUEST_PNT_MAIN_DATA 2

#define DURATION            0.0001

#define TIME_50MS           1
#define TIME_100MS          2
#define TIME_1S             20
#define TIME_5S             100

#define CHECK_EMERGENCY_SW  0
#define SEND_DATA_AFTER_1S  1

#define CMD_ALARM_RESET     8
typedef struct {
    BYTE bySndBuf[MAX_PACKET_SIZE];
    BYTE byRcvBuf[MAX_PACKET_SIZE];
    BYTE byPacketSize;
    BYTE byPacketNum;
    BYTE byIn, byStep;
    BYTE byChkSend;
    BYTE byChkRcv;
    BYTE fgInIdleLine, fgPacketOK, fgComComple;
    BYTE byTotalRcvDataNum; 
    BYTE fgChk;
    BYTE byChkSum, byMaxDataNum, byDataNum;

    int nIDPC, nIDMDUI, nIDMDT, nRMID;
    int nDiameter, nBaudrate, nWheelLength, nGearRatio, fgDirSign;
    short sSetDia, sSetWheelLen, sSetGear;
    int nCmdSpeed, nCmdAngSpeed;

    short sTempVoltIn, sSumVolt ,sVoltIn;
    BYTE byPlatStatus, bEmerSW, bBusy, bBumper1, bBumper2, bBumper3, bBumper4;
    BYTE byDocStatus, bDocComple, bChargeState, bCharComple, bIr1, bIr2, bIr3, bRccState;

    BYTE byUS1, byUS2, byUS3, byUS4;

    BYTE fgResetOdometry, fgControlstate, fgResetAngle;

    int fgSndOK, nCntDelaySnd;

    long lPosi[2], lTempPosi[2];
    short sTheta, sTempTheta, sExTheta;

    long lMoving[3][3];

    short sMotorRPM[2];
    long lMotorPosi[2];

    WORD sCurrent[2];

    BYTE byStatus[2];
    BYTE fgAlarm[2], fgCtrlFail[2], fgOverVolt[2], fgOverTemp[2];
    BYTE fgOverLoad[2], fgHallFail[2], fgInvVel[2], fgStall[2];

    BYTE byChkComError;
    BYTE fgComDataChk;

    BYTE byCntVoltAver;

    int nHallType, nMaxRPM, nAngResol;
    double dTheta;

    int nSlowstart, nSlowdown;
    BYTE fgInitsetting;

    BYTE fgResetAlarm;

    int position; // motor position
    short current; // motor current
    short rpm; //motor current rpm

}Communication;
extern Communication Com;

typedef struct {
    BYTE byLow;
    BYTE byHigh;
}IByte;

typedef struct {
    BYTE by0;
    BYTE by1;
    BYTE by2;
    BYTE by3;
}shortByte;


extern IByte Short2Byte(short sIn);
extern shortByte Long4Byte(int longData);
extern int Byte2Short(BYTE byLow, BYTE byHigh);
extern int Byte2LInt(BYTE byData1, BYTE byData2, BYTE byData3, BYTE byData4);
extern int MovingAverage(void);

extern int InitSerial(void);
extern int InitSetParam(void);
extern int InitSetSlowStart(void);
extern int InitSetSlowDown(void);
extern int PutMdData(BYTE byPID, BYTE byID, int nArray[]);
extern long *GetMdData(BYTE byPID);
extern int MdReceiveProc(void);
extern int ReceiveDataFromController(void);
extern int AnalyzeReceivedData(BYTE byArray[], BYTE byBufNum);



