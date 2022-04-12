// My Constants
#define max_rpm_posi_cmd 800                       // 위치제어할 때의 rpm
#define half_of_stiringwheel_rotation_range 1000    // 직접 측정해서 설정해줘야 하는 값
#define INC_RELATIVE_POSITION 100                    // incremental command 할 때의 값
#define MAX_STEERING_WHEEL_POSITION 900            // 스티어링휠 최대 위치 크기

///v1.9d modifying code while PNT_TQ_OFF(PNT_BREKE) communication. it request PNT_MAIN_DATA and for 2s delay
///v1.9e adding code reset command after 2s
///v1.9f adding code reset command when RMID is MDT'
///v1.9g modifying 'Md.sCmdAngularVel calculation' in cmd_main.cpp according to "md_node/angleresolution"

#include "md/global.hpp"
#include "md/main.hpp"
#include "md/com.hpp"
#include "md/robot.hpp"

#include "iostream"

#include "md/vel_msg.h"
#include "md/monitor_msg.h"

#include <ros/ros.h>

TableOfRobotControlRegister RB;
Communication Com;
LOG Log;

// 함수 프로토타입 선언
void Monitor_publish(md::monitor_msg &, const ros::Publisher &);
void Position_cmd(int target_position);
void Position_rqs();
void Monitor_rqs();
void Main_data_rqs();
void Tq_free_cmd();
void Current_limit_set(int current_limit);
void Initial_adjustment();
void Velocity_cmd(int rpm);
void Inc_position_cmd(int relative_target_postion);
void Position_reset();
void Brake_cmd();

// My Variables
int monitor_position;             // 위치 모니터 변수
int target_position;              // 위치 명령 변수
bool tq_free_flag = OFF;          // Tq_free Flag , OFF 상태로 초기화
std::string tq_free_status;
bool motor_connection_flag = OFF; // 모터 연결 확인 Flag, OFF 상태로 초기화
bool initialization_flag = OFF;   // 초기화 플래그
bool steering_running_flag = OFF;
int stop_counts = 0;              // stop msg 연속으로 들어온 갯수

//variable declaration
IByte iData;
shortByte i4Data; //4Byte
int nArray[6];    // 위치제어 PID 219 Data Size 6개

//It is a message callback function.
//It is a function that oprates when a topic message named 'vel_topic' is received.
//The input message is to receive the vel_msg message from 'md' package in msg directory

// ---------CallBack 함수들

void velCallBack(const md::vel_msg::ConstPtr &vel)
{
    Com.nCmdSpeed = vel->nLinear;
    Com.nCmdAngSpeed = vel->nAngular;
    Com.fgResetOdometry = vel->byResetOdometry;
    Com.fgResetAngle = vel->byResetAngle;
    Com.fgResetAlarm = vel->byResetAlarm;

    target_position = vel->nPosition;
}

void reset_Callback(const std_msgs::String)
{
    Position_reset();
    ros::Duration(0.01).sleep();
}

void posi_Callback(const std_msgs::Int32 &num)
{
    target_position = num.data;

    if (tq_free_flag == OFF) // tq_free 꺼져있고 초기화 끝난상태
    {
        Position_cmd(target_position);
        ros::Duration(0.02).sleep();
    }
}

void tq_free_Callback(const std_msgs::String &tq_free)
{
    if (tq_free.data == "tq_free ON")
    {
        tq_free_flag = ON;
        tq_free_status = "ON";
        Tq_free_cmd();
    }

    if (tq_free.data == "tq_free OFF")
    {
        tq_free_flag = OFF;
        tq_free_status = "OFF";
        target_position = monitor_position;
        Position_cmd(target_position);
    }

    ros::Duration(0.01).sleep(); // Callback 여러개에서 command 줄 때 delay (spinOnce 밑에 r.sleep() 0.05초 줬음)
}

void current_limit_Callback(const std_msgs::Int32 &current_limit)
{
    Current_limit_set(current_limit.data);

    ros::Duration(0.01).sleep(); // Callback 여러개에서 command 줄 때 delay (spinOnce 밑에 r.sleep() 0.05초 줬음)
}

void steering_wheel_control_Callback(const std_msgs::String &msg)
{
    if (tq_free_flag == ON)
    {
        if (msg.data == "right" && monitor_position > -MAX_STEERING_WHEEL_POSITION)
        {
            Inc_position_cmd(-100);
            // Position_cmd(-300);
            steering_running_flag = ON;
        }

        else if (msg.data == "left" && monitor_position < MAX_STEERING_WHEEL_POSITION)
        {
            Inc_position_cmd(100);
            // Position_cmd(300);
            steering_running_flag = ON;
        }

        else
        {
            if (steering_running_flag == ON)
            {
                Tq_free_cmd();
                cout<<"---------------------Tq_free_cmd----------------------"<<endl;
                // target_position = monitor_position;
                // ros::Duration(0.01).sleep();
                // Position_cmd(target_position);
            }
            steering_running_flag = OFF;
        }
    }

    ros::Duration(0.01).sleep();
}

// ----------------------------------------------- main 함수 --------------------------------------------//

int main(int argc, char **argv)
{
    ros::init(argc, argv, "md_node"); // Node name initialization.
    ros::NodeHandle nh;               // Node handle declaration for communication with ROS system.

    // ros::Subscriber vel_sub = nh.subscribe("vel_topic", 100, velCallBack);                                    //Subscriber declaration.
    ros::Subscriber posi_sub = nh.subscribe("cmd_posi", 1, posi_Callback);                                       // Subscriber declaration.
    ros::Subscriber tq_free_sub = nh.subscribe("tq_free_topic", 1, tq_free_Callback);                            // Subscriber declaration.
    ros::Subscriber current_limit_sub = nh.subscribe("current_limit_topic", 1, current_limit_Callback);          // Subscriber declaration.
    ros::Subscriber reset_sub = nh.subscribe("reset_topic", 1, reset_Callback);                                     // Subscriber declaration.
    ros::Subscriber steering_wheel_keyboard_sub = nh.subscribe("steering_wheel_control_topic", 1, steering_wheel_control_Callback); // Subscriber declaration

    // ros::Publisher monitor_pub = nh.advertise<md::monitor_msg>("monitor_topic", 1); //Publisher declaration.
    ros::Publisher string_pub = nh.advertise<std_msgs::String>("string_com_topic", 1);

    md::monitor_msg monitor; // monitor_msg declares message 'message' as message file.

    // ros::Rate r(10000);                //Set the loop period -> 100us

    ros::Rate r(60); // 60Hz

    static BYTE byCntComStep, byCntCmdVel, fgSendCmdVel, byCntInitStep;
    static BYTE byCnt5000us, byCntCase[10], byFgl, byFglReset, fgInitPosiResetAfter2s, byCntReset;
    static BYTE byCnt, byCntStartDelay;

    Log.fgSet = ON;
    fgSendCmdVel = ON;
    byFgl = OFF;
    Com.fgInitsetting = OFF;

    //Store the value of the parameter in the variable
    {
        nh.getParam("md_node/PC", Com.nIDPC);
        nh.getParam("md_node/MDUI", Com.nIDMDUI);
        nh.getParam("md_node/MDT", Com.nIDMDT);
        nh.getParam("md_node/baudrate", Com.nBaudrate);
        nh.getParam("md_node/diameter", Com.nDiameter);
        nh.getParam("md_node/wheelLength", Com.nWheelLength);
        nh.getParam("md_node/reduction", Com.nGearRatio);
        nh.getParam("md_node/direction", Com.fgDirSign);
        nh.getParam("md_node/halltype", Com.nHallType);
        nh.getParam("md_node/maxrpm", Com.nMaxRPM);
        nh.getParam("md_node/angleresolution", Com.nAngResol);
        nh.getParam("md_node/RMID", Com.nRMID);
        nh.getParam("md_node/slowstart", Com.nSlowstart);
        nh.getParam("md_node/slowdown", Com.nSlowdown);
    }

    InitSerial(); //communication initialization in com.cpp


    // ------------------- tq_free 상태롤 Initial adjustment 명령 전까지 대기 ------------------//
    tq_free_flag = ON;
    tq_free_status = "ON";

    Tq_free_cmd();
    ros::Duration(0.01).sleep();
    Position_reset();

    // while(ros::ok() && initialization_flag == OFF)
    // {
    //     ros::Rate(1).sleep(); // 1초 delay
    //     ros::spinOnce();
    // }


    // --------------------------------------------- main loop 0.1s 주기, 0.05s 마다 콜백 함수 처리 ---------------------------------------------- //

    while (ros::ok())
    {

        ros::spinOnce();

        r.sleep(); 
        r.sleep();

        // Monitor_publish(monitor, monitor_pub); // Position value monitor publishing

        // Position_rqs();
        // // r.sleep(); // 0.025sec delay
        // ReceiveDataFromController();
        // r.sleep();
        // ROS_INFO("position : %d", Com.position);

        std::cout << "main data request" << std::endl;
        Main_data_rqs();
        std::cout << "data receiving ..." << std::endl;

        ReceiveDataFromController();
        ROS_INFO("position : %d, current : %.1f A, RPM : %d, ", monitor_position, Com.current * 0.1, Com.rpm);
        cout << "TQ FREE STATUS : " << tq_free_status << endl << endl;

        // cout << "position :" << monitor_position << ", current : " <<  Com.current << ", RPM : " << Com.rpm << endl;
        // std::cout <<" monitor_position in main.cpp : "<< monitor_position << std::endl;
        // cout << "monitor_position adress in main.cpp : " << &monitor_position << endl;

        r.sleep();

        // ROS_INFO("position : %d, current : %d, RPM : %d", Com.position, Com.current, Com.rpm);

        ros::spinOnce(); // 콜백함수 처리

        // std::cout << "end the loop ..." << std::endl << std::endl;
        r.sleep();

        // if (monitor_position < - MAX_STEERING_WHEEL_POSITION || monitor_position > MAX_STEERING_WHEEL_POSITION)
        // {
        //     Brake_cmd();
        // }

        r.sleep();
        r.sleep(); 
    }
}

// -------------------------- 함수 정의----------------------------//

void Monitor_publish(md::monitor_msg &monitor, const ros::Publisher &monitor_pub)
{
    monitor.position = Com.position;
    monitor.current = Com.current;
    monitor.rpm = Com.rpm;
    monitor_pub.publish(monitor); // cmd_main.cpp에서 콜백함수로 받음
    //ROS_INFO("monitor pub");
}

void Position_cmd(int target_position)
{
    i4Data = Long4Byte(target_position); // 위치값 int -> Byte 4 개로 쪼개기

    nArray[0] = i4Data.by0;
    nArray[1] = i4Data.by1;
    nArray[2] = i4Data.by2;
    nArray[3] = i4Data.by3;

    iData = Short2Byte((short)max_rpm_posi_cmd); // 최대 rpm short -> Byte 2개로 쪼개기

    nArray[4] = iData.byLow;
    nArray[5] = iData.byHigh;

    PutMdData(PID_POSI_VEL_CMD, Com.nRMID, nArray);
}

void Current_limit_set(int current_limit)
{
    iData = Short2Byte((short)current_limit);

    nArray[0] = iData.byLow;
    nArray[1] = iData.byHigh;
    PutMdData(PID_TQ_LIMIT_SW_VAL, Com.nRMID, nArray);
}

void Position_rqs()
{
    nArray[0] = PID_POSI_DATA;
    PutMdData(PID_REQ_PID_DATA, Com.nRMID, nArray); // 특정 PID Data 요청 : 183, TMID, 1, 4, 1, PID, CHK
}

void Monitor_rqs()
{
    nArray[0] = PID_MONITOR;
    PutMdData(PID_REQ_PID_DATA, Com.nRMID, nArray);
}

void Main_data_rqs()
{
    nArray[0] = PID_MAIN_DATA;
    PutMdData(PID_REQ_PID_DATA, Com.nRMID, nArray);
}

void Tq_free_cmd()
{
    nArray[0] = 1;
    PutMdData(PID_TQ_OFF, Com.nRMID, nArray);
}

void Brake_cmd()
{
    nArray[0] = 1;
    PutMdData(PID_BRAKE, Com.nRMID, nArray);
}

// -------------------- Initial adjustment --------------------------- //
void Initial_adjustment()
{
    cout << "\n   --- Motor initial adjustment --- \n"
         << endl;

    ros::Rate r_init(10);

    // 모터 연결 확인
    while (ros::ok())
    {
        static bool print_disconnected_flag = true;

        Main_data_rqs();
        ros::Rate(1).sleep(); // 1초 delay

        ReceiveDataFromController();

        if (motor_connection_flag == ON)
        {
            {
                cout << "\n --- Motor Connected ... ---\n"
                     << endl;
                ros::Duration(2).sleep();

                cout << "\n --- Initial adjustment start ... ---\n"
                     << endl;
                ros::Duration(1).sleep();

                cout << "           --- 3... --- \n"
                     << endl;
                ros::Duration(1).sleep();

                cout << "           --- 2... --- \n"
                     << endl;
                ros::Duration(1).sleep();

                cout << "           --- 1... --- \n"
                     << endl;
                ros::Duration(1).sleep();
            }

            break; // 다음 단계로 이동 (while문 탈출)
        }

        if (print_disconnected_flag == true)
        {
            cout << "   ---- Motor disconnected... ----" << endl;
            print_disconnected_flag = false;
        }
    }

    //rpm 1000으로 회전시킴
    Velocity_cmd(1000);
    r_init.sleep(); // 0.1초 delay

    for (int i; i < 10; i++)
    {
        Main_data_rqs();
        r_init.sleep();
    }

    {
        // Main_data_rqs();
        // r_init.sleep();
        // cout << "Velocity command for 5s \n" << endl;
        // cout << "           --- 5... --- \n" << endl;
        // ReceiveDataFromController();
        // ROS_INFO("position : %d, current : %.1f A, RPM : %d \n", monitor_position, Com.current * 0.1, Com.rpm);

        // Main_data_rqs();
        // ros::Duration(1).sleep();
        // cout << "           --- 4... --- \n" << endl;
        // ReceiveDataFromController();
        // ROS_INFO("position : %d, current : %.1f A, RPM : %d \n", monitor_position, Com.current * 0.1, Com.rpm);

        // Main_data_rqs();
        // ros::Duration(1).sleep();
        // cout << "           --- 3... --- \n" << endl;
        // ReceiveDataFromController();
        // ROS_INFO("position : %d, current : %.1f A, RPM : %d \n", monitor_position, Com.current * 0.1, Com.rpm);

        // Main_data_rqs();
        // ros::Duration(1).sleep();
        // cout << "           --- 2... --- \n" << endl;
        // ReceiveDataFromController();
        // ROS_INFO("position : %d, current : %.1f A, RPM : %d \n", monitor_position, Com.current * 0.1, Com.rpm);

        // Main_data_rqs();
        // ros::Duration(1).sleep();
        // cout << "           --- 1... --- \n" << endl;
        // ReceiveDataFromController();
        // ROS_INFO("position : %d, current : %.1f A, RPM : %d \n", monitor_position, Com.current * 0.1, Com.rpm);

        // ros::Duration(1).sleep();
    }

    ReceiveDataFromController();
    ROS_INFO("position : %d, current : %.1f A, RPM : %d \n", monitor_position, Com.current * 0.1, Com.rpm);

    // 모터가 정지하면 반대 방향으로 ** 만큼 incremental position control
    while (ros::ok())
    {
        Main_data_rqs();
        r_init.sleep(); // 0.1초 delay

        ReceiveDataFromController();
        ROS_INFO("position : %d, current : %.1f A, RPM : %d \n", monitor_position, Com.current * 0.1, Com.rpm);

        if (Com.rpm == 0) // if (Com.rpm == 0), 디버깅용 : if (monitor_position >= 2000)
        {
            Tq_free_cmd();

            cout << "   --- Motor stopped... ---\n"
                 << endl;
            ros::Duration(1).sleep();

            cout << "   --- Going mid position Start... --- \n"
                 << endl;
            ros::Duration(1).sleep();

            cout << "           --- 3... --- \n"
                 << endl;
            ros::Duration(1).sleep();

            cout << "           --- 2... --- \n"
                 << endl;
            ros::Duration(1).sleep();

            cout << "           --- 1... --- \n"
                 << endl;
            ros::Duration(1).sleep();

            break;
        }
    }

    int target_positoin_for_zero = monitor_position - half_of_stiringwheel_rotation_range;

    Position_cmd(target_positoin_for_zero);

    ros::Duration(1).sleep(); // 1s delay for rotating start

    // 모터가 해당 위치에 오면 현재 위치 0으로 설정
    while (ros::ok())
    {
        Main_data_rqs();
        r_init.sleep(); // 0.1초 delay

        ReceiveDataFromController();
        ROS_INFO("position : %d, current : %.1f A, RPM : %d \n", monitor_position, Com.current * 0.1, Com.rpm);

        if (monitor_position == target_positoin_for_zero)
        {
            Position_reset();
            {
                cout << " --- Initial adjustment finished --- \n"
                     << std::endl;

                ros::Duration(1).sleep();
                cout << " --- Motor Control Mode --- \n"
                     << endl;

                ros::Duration(1).sleep();
                cout << "           --- 3... --- \n"
                     << endl;

                ros::Duration(1).sleep();
                cout << "           --- 2... --- \n"
                     << endl;

                ros::Duration(1).sleep();
                cout << "           --- 1... --- \n"
                     << endl;

                ros::Duration(1).sleep();
            }

            break;
        }
    }
}

void Velocity_cmd(int rpm)
{
    iData = Short2Byte((short)rpm);

    nArray[0] = iData.byLow;
    nArray[1] = iData.byHigh;

    PutMdData(PID_VEL_CMD, Com.nRMID, nArray);
}

void Inc_position_cmd(int relative_target_postion)
{
    i4Data = Long4Byte(relative_target_postion); 

    nArray[0] = i4Data.by0;
    nArray[1] = i4Data.by1;
    nArray[2] = i4Data.by2;
    nArray[3] = i4Data.by3;

    iData = Short2Byte((short)max_rpm_posi_cmd); // 최대 rpm short -> Byte 2개로 쪼개기

    nArray[4] = iData.byLow;
    nArray[5] = iData.byHigh;

    PutMdData(PID_INC_POSI_VEL_CMD, Com.nRMID, nArray);
}

void Position_reset()
{
    nArray[0] = 1;
    PutMdData(PID_POSI_RESET, Com.nRMID, nArray);
}