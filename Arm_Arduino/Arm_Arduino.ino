/****************************************************************************
机械臂主控程序
****************************************************************************/

#include <Wire.h>                         //调用库文件Wire库，这个是I2C通信库，是Arduino官方库文件
#include <SoftwareSerial.h>               //调用库文件SoftwareSerial，这个是软串口库，是Arduino官方库文件
#include "Adafruit_MotorShield.h"         //调用库文件Adafruit_MotorShield，这个是舵机扩展板库
#include "Adafruit_MS_PWMServoDriver.h"   //调用库文件Adafruit_MS_PWMServoDriver，这个是舵机扩展板PWM驱动库
#include "PS2X_lib.h"                     //调用库文件PS2X_lib，这个是无线手柄库
#include "BTJoystick.h"                   //调用库文件BTJoystick，这个是蓝牙app摇杆库
//#include "Adafruit_Encoder.h"             //调用库文件

#define PS2_SEL 10  //定义PS2手柄引脚10
#define PS2_CMD 11  //定义PS2手柄引脚11
#define PS2_DAT 12  //定义PS2手柄引脚12
#define PS2_CLK 13  //定义PS2手柄引脚13

#define pressures   true    //启用手柄功能
#define rumble      true    //启用手柄功能
SoftwareSerial  softSerial( 2, 3 );       //蓝牙通讯模块软串口引脚：蓝牙Rx端→arduino3#，蓝牙Tx端→arduino2#
/***********目前，本库不支持热插拔手柄接收器，否则必须重启控制板***********/
PS2X ps2x;                     //定义PS2手柄
BTJoystick ps2x1 (softSerial);  //定义蓝牙手柄

int error = 0;      //定义变量错误
byte type = 0;      //定义变量形式
byte vibrate = 0;   //定义变量震动

Adafruit_MotorShield AFMS = Adafruit_MotorShield();  //定义舵机扩展板主名称
Adafruit_DCMotor *DCMotor1 = AFMS.getMotor(1);  //DC电机
Adafruit_DCMotor *DCMotor2 = AFMS.getMotor(2);  //DC电机
//Adafruit_Encoder Encoder1(1); //创建1号编码器（对于M1电机）

const int SERVOS = 4;       //定义变量SERVOS,表示舵机数，预先赋值4
const int ACC = 10;         //定义变量ACC，用于消除电位器误差
int PIN[SERVOS];            //定义变量数组PIN[SERVOS]，表示舵机PWM引脚号
int value[SERVOS];          //定义变量数组value[SERVOS]，用于存放无线手柄的模拟量值
int Bvalue[SERVOS];         //定义变量数组Bvalue[SERVOS]，用于存放蓝牙app手柄的模拟量值
int idle[SERVOS];           //定义变量空闲
int currentAngle[SERVOS];   //定义变量数组currentAngle[SERVOS]，用于存放当前舵机角度值
int MIN[SERVOS];            //定义变量数组MIN[SERVOS]，用于存放当前舵机的最小角度值
int MAX[SERVOS];            //定义变量数组MAX[SERVOS]，用于存放当前舵机的最大角度值
int INITANGLE[SERVOS];      //定义变量数组INITANGLE[SERVOS]，用于存放当前舵机的初始角度值
int previousAngle[SERVOS];  //定义变量数组previousAngle[SERVOS]，用于存放当前舵机的前角度值
int actionIndex = 0;  //运行步骤指针
int Totalact = 0;  //定义录制总点位数
int Led1 = A0;  //定义RGB指示灯R引脚接模拟A0端口
int Led2 = A1;  //定义RGB指示灯G引脚接模拟A1端口
int Led3 = A2;  //定义RGB指示灯B引脚接模拟A2端口
int Led4 = A3;  //定义RGB指示灯GND引脚接模拟A3端口
int Pressbuf[10];    //定义按键按压buf
int Releasebuf[10];  //定义按键释放buf
int BTenable;  //消除蓝牙未连接的Bug动作异常
int BTbuf;     //消除蓝牙未连接的Bug导致动作异常
int Mode;  //定义变量模式，0：手动模式 1：录制 2：自动播放
unsigned long ms;     //定义毫秒读数
unsigned long t[10];  //定义t用于生成定时器
int autoplaying = false;  //定义自动运行状态
int autotrig =false;  //定义自动播放上升沿
int autotrigbuf =false;  //定义自动播放上升沿buf
int autoindex = 0;  //定义自动播放指针
int indexbuf = 0;  //定义自动播放指针缓存
int const maxAutoActions = 20;  //定义最大自动数据记录值30，由于Uno内存有限，初始值为20点
int actdelay = 300;  //自动播放动作间隔300毫秒，可修改间隔
int Actionmem[maxAutoActions][4] ={{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};  //定义缓存用于录制坐标点
String inputString = "";         //定义PC串口数据
bool stringComplete = false;  //定义串口接收标志
int stringlength = 0;  //定义字符串长度
String PCcmd[20];  //定义PC指令字符
String PCmove[4];  //定义PC模式移动指令
int PCact[20];  //定义PC模式按钮动作
int Serialindex=0;  //定义串口指针
String RcvBuf[4];  //定义接收角度值String
int Rcvint[4];  //定义接收角度值Int

// 声明函数:设置舵机频率
void setServoPulse( uint8_t n, double pulse )  //n:舵机号； pulse：脉冲时间
{
    double pulselength;
    pulselength = 1000000;      // 百万分之一秒
    pulselength /= 50;          //50 Hz
    pulselength /= 4096;        //(12位分辨率)
    pulse *= 1000;              //乘以1000
    pulse /= pulselength;       //除以pulselength变量
    AFMS.setPWM( n, pulse );    //设置PWM值
}

// 声明函数:PWM值转化为角度
void writeServo( uint8_t n, uint8_t angle )  //n:舵机号； angle：舵机需要转到的角度
{
    double pulse;
    angle = (int)angle*0.922222;  //根据舵机批次的不同，此批舵机实际转角与理论输入值存在0.922222的系数关系。如果舵机实际转角与理论输入值一样，则可删除此语句
    pulse = 0.5 + angle / 90.0;  //脉冲时间和角度值的关系公式
    setServoPulse( n, pulse );  //让指定的舵机通过相对应的脉冲时间转到指定的角度
}


void setup()
{
    Serial.begin( 57600 );  //串口通讯波特率57600
    softSerial.begin(9600); //软串口通讯开启波特率9600，此项设置必须与蓝牙模块串口波特率一致
    softSerial.listen();    //开启软串口数据监听
    delay( 2000 );          //延时2秒用于PS2手柄初始化
    Serial.print( "Search Controller.." );  //串口打印“搜索手柄中”
    do
    {
        error = ps2x.config_gamepad( PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble );  //此函数为配置无线手柄的初始化通信能力，如果ok，返回值0
        if( error == 0 )
        {
            Serial.println( "\nConfigured successful " );  //连接成功
            break;  //中断，跳出循环函数do..while
        }
        else  //否则
        {
            Serial.print( "." );  //等待连接，此时每0.1秒串口输出'.'用于提示功能 
            delay( 100 );
        }
    }while( 1 );  //while循环直到手柄连接成功
    type = ps2x.readType();   //此函数判断连接控制端类型，0/1：Wireless DualShock Controller found; 2: Unknown Controller; 
    switch( type )  //根据连接手柄读取模式，串口输出型号
    {
        case 0:
            Serial.println( "Wireless DualShock Controller found " );  //无线手柄
            break;
        case 1:
            Serial.println( "Wireless DualShock Controller found " );  //无线手柄
            break;
        case 2:
            Serial.println( "Unknown Controller type found " );  // 未知控制器
            break;
    }
    ps2x.read_gamepad( true, 200 );  //开机震动0.2秒用于提示手柄连接成功
    delay( 500 );
    pinMode(Led1,OUTPUT);  //定义LED灯引脚为输出
    pinMode(Led2,OUTPUT);  //定义LED灯引脚为输出
    pinMode(Led3,OUTPUT);  //定义LED灯引脚为输出
    pinMode(Led4,OUTPUT);  //定义LED灯引脚为输出
    digitalWrite(Led1,LOW);  //LED灯初始化引脚R
    digitalWrite(Led2,LOW);  //LED灯初始化引脚G
    digitalWrite(Led3,LOW);  //LED灯初始化引脚B
    digitalWrite(Led4,LOW);  //LED灯初始化引脚GND
    int Mode = 0;             //定义模式代码
    int BTenable = false;     //定义蓝牙使能
    int BTbuf = false;        //蓝牙中继
    init_Pins();              //初始化四轴为设定值，所调用的函数处于182行
    
    AFMS.begin( 50 ); //开启机械手库的函数设置
    for ( int i = 0; i < SERVOS; i ++ )  //For循环初始化赋值设定角度（开始将4个舵机回原点）
    {
        value[i] = 0;
        idle[i] = 0;
        previousAngle[i] = INITANGLE[i];
        currentAngle[i] = INITANGLE[i];
        writeServo( PIN[i], INITANGLE[i] );
    }

    // 初始化地盘电机
}

void loop()
{  
        ps2x.read_gamepad( false, vibrate );    //震动
        ps2x1.readCommand();                    //蓝牙手柄读取启用
        vibrate = ps2x.Analog( PSAB_CROSS );    //按下'X'手柄震动功能
           
        Control();            //舵机控制函数，函数所在220行        
        ReadRockerValue();    //读取PS手柄摇杆值，函数所在243行
        ReadBtRockerValue();  //读取蓝牙手柄摇杆值，函数所在252行
        //ShowRockerValue();    //调试用函数，显示摇杆数值，函数所在274行
        BTonline();           //验证蓝牙手柄是否在线，函数所在263行
        ModeChange();         //变更模式，函数所在286行
        ShowAngle();          //串口读取当前四轴角度值(调试用)，函数所在316行
        MemoryAction();       //动作录制，函数所在340行
        Totalactcnt();        //当前记录动作数，函数所在373行
        Autoplay();           //自动播放函数，函数所在380行
        Autodelete();         //删除记录点位，函数所在416行
        PCmode();             //PC控制模式，函数所在439行

        //电机控制
        if (ps2x.Button(PSB_TRIANGLE)) {//前进
          DCMotor1->setSpeed(100);
          DCMotor2->setSpeed(100);
          DCMotor1->run(FORWARD); 
          DCMotor2->run(FORWARD); 
        }else if (ps2x.Button(PSB_SQUARE)) {//左转
          DCMotor1->setSpeed(10);
          DCMotor2->setSpeed(100);
          DCMotor1->run(FORWARD); 
          DCMotor2->run(FORWARD); 
        }else if (ps2x.Button(PSB_CIRCLE)) {//右转
          DCMotor1->setSpeed(100);
          DCMotor2->setSpeed(10);
          DCMotor1->run(FORWARD); 
          DCMotor2->run(FORWARD); 
        }else if(ps2x.Button(PSB_CROSS)){ //停止电机
          DCMotor1->run(BRAKE); 
          DCMotor2->run(BRAKE); 
        }

    delay( 10 );
}

void init_Pins(){  //函数1：舵机初始化参数函数
    /*00:爪头舵机参数*/
    PIN[0] = 0;  //爪头舵机对应舵机板插口号
    MIN[0] = 0;  //爪头舵机最小角度限位
    MAX[0] = 180;  //爪头舵机最大角度限位
    INITANGLE[0] = 80;  //78;  //爪头舵机原点角度
    /*01:小臂舵机参数*/
    PIN[1] = 1;  //小臂舵机对应舵机板插口号
    MIN[1] = 45;  //小臂舵机最小角度限位
    MAX[1] = 105;  //小臂舵机最大角度限位
    INITANGLE[1] = 90;  //小臂舵机原点角度
    /*02:大臂舵机参数*/
    PIN[2] = 2;  //大臂舵机对应舵机板插口号
    MIN[2] = 40;  //大臂舵机最小角度限位
    MAX[2] = 135;  //大臂舵机最大角度限位
    INITANGLE[2] = 90;  //大臂舵机原点角度
    /*03:转台舵机参数*/
    PIN[3] = 3;  //转台舵机对应舵机板插口号
    MIN[3] = 0;  //转台舵机最小角度限位   
    MAX[3] = 180;  //转台舵机最大角度限位 
    INITANGLE[3] =90;  //转台舵机原点角度
}

void Home()  //函数2：按下Left键回原点功能 
{
  if (ps2x.Button(PSB_PAD_LEFT)|| ps2x1.Button(BTB_PAD_LEFT)||(PCact[12]==1))  //按下无线手柄Left键、手机app上位机Left键、串口输入"LEFT"时，回原点
  {
    for ( int i = 0; i < SERVOS; i ++ )  //For循环初始化赋值设定角度
    {
      //如果当前i#舵机的当前角度小于初始值角度，则每隔0.005秒当前角度递增1度，直至当前角度等于初始值角度，退出循环
      while ( currentAngle[i] < INITANGLE[i] )  //如果当前角度小于初始值角度
      {
        currentAngle[i] += 1;  //当前角度以1度递增
        writeServo( PIN[i], currentAngle[i] );  //并让当前i#舵机转到当前角度
        delay(5);  //每隔0.005秒当前角度递增1度，这个参数可以控制舵机的旋转速度
      }
     
     //如果当前i#舵机的当前角度大于初始值角度，则每隔0.005秒当前角度递减1度，直至当前角度等于初始值角度，退出循环
      while ( currentAngle[i] > INITANGLE[i] )  //如果当前角度大于初始值角度
      {
        currentAngle[i] -= 1;  //当前角度以1度递增
        writeServo( PIN[i], currentAngle[i] );  //并让当前i#舵机转到当前角度
        delay(5);  //每隔0.005秒当前角度递增1度，这个参数可以控制舵机的旋转速度
      }
    }
    PCact[12]=0;  //因为在串口输入"LEFT"后，PCact[12]被赋值1，导致舵机转到初始角度，所以，舵机转完后，需将PCact[12]赋0值
  }
}

void Control()  //函数3：舵机控制函数
{
  if(Mode<2)  //非自动播放模式下手动可控
  {
    Home();  //回原点
    for ( int i = 0; i < SERVOS; i ++ )  //4个舵机For循环函数0-3
    {   
      if ( (value[i] > 150)||((BTenable==true)&&(Bvalue[i] > 200))||(PCact[i]==1))  //如果无线手柄摇杆的模拟值>150,或者蓝牙功能可用且App虚拟摇杆的模拟值>200，又或者电脑上位机虚拟摇杆的值为1，则角度递增
      {
        if ( currentAngle[i] < MAX[i] )  //如果当前角度未达到设定最大角度值
        currentAngle[i] += 1;  //当前角度以1度递增
        writeServo( PIN[i], currentAngle[i] );  //写入输出引脚，让当前i#舵机转到当前角度
      }
        else if ((value[i] < 100)||((BTenable==true)&&(Bvalue[i] < 60))||(PCact[i]==2))  //如果无线手柄摇杆的模拟值<100,或者蓝牙功能可用且App虚拟摇杆的模拟值<60，又或者电脑上位机虚拟摇杆的值为2，则角度递减
      {
        if ( currentAngle[i] > MIN[i] )  //如果当前角度未达到设定最小角度值
        currentAngle[i] -= 1;  //当前角度以1度递减
        writeServo( PIN[i], currentAngle[i] );  //写入输出引脚，让当前i#舵机转到当前角度
      }        
     }
   } 
}

void ReadRockerValue()  //函数4：读取无线手柄摇杆值
{
value[0] = ps2x.Analog( PSS_LX );  //读取左摇杆横向模拟量值
value[1] = ps2x.Analog( PSS_RY );  //读取右摇杆纵向模拟量值
value[2] = ps2x.Analog( PSS_LY );  //读取左摇杆纵向模拟量值
value[3] = ps2x.Analog( PSS_RX );  //读取右摇杆横向模拟量值
value[3] = 255-value[3];
}

void ReadBtRockerValue()  //函数5：读取蓝牙摇杆值
{
Bvalue[0] = ps2x1.Analog( PSS_LXX );  //读取左摇杆横向模拟量值
Bvalue[1] = ps2x1.Analog( PSS_RYY );  //读取右摇杆纵向模拟量值
Bvalue[2] = ps2x1.Analog( PSS_LYY );  //读取左摇杆纵向模拟量值
Bvalue[3] = ps2x1.Analog( PSS_RXX );  //读取右摇杆横向模拟量值
Bvalue[1] = 255-Bvalue[1];  //反向处理
Bvalue[2] = 255-Bvalue[2];  //反向处理      
Bvalue[3] = 255-Bvalue[3];  //反向处理
}

void BTonline()  //函数6：检测蓝牙手柄在线
{
  if(BTbuf==false)  //如果蓝牙未连接的Bug导致动作异常
  {
    BTenable = true;  //蓝牙功能可用
    for ( int i = 0; i < 4; i ++ )  //for循环检测蓝牙手柄初始化成功
    {
      if(Bvalue[i]<100||Bvalue[i]>150)  //如果蓝牙App当前虚拟摇杆的值<100或>150
      {BTenable=false;}  //蓝牙功能不可用
      else  //否则
      {BTbuf = true;BTenable=true;}  //蓝牙未连接的Bug导致动作正常，且让蓝牙功能可用
    }
  }
}

void ModeChange()  //函数7：按下UP模式变更
{
  if (ps2x.Button(PSB_PAD_UP)||ps2x1.Button(BTB_PAD_UP)||(PCact[10]==1))  //如果按下无线手柄UP键，或按下蓝牙App的UP键，又或者电脑上位机PCact[10]状态为1
  {
    if(Pressbuf[1] == false)  //如果按键buf为0
    {
      if(Mode<2) //如果模式<2，即为手动控制模式或者动作录制模式
      {Mode = Mode + 1;}  //手动控制模式变为动作录制模式，或者动作录制模式变为自动播放模式
      else  //如果模式为自动播放模式
      {Mode = 0;}  //自动播放模式变为手动控制模式
      PCact[10]=0;  //PCact[10]赋值0
      Serial.print("模式");  //串口监视器中打印"模式"字样
      Serial.print(Mode);  //串口监视器中打印"Mode"字样
      if(Mode==0){Serial.println("手动控制模式");}  //如果是手动控制模式，串口监视器中打印"手动控制模式"字样
      if(Mode==1){Serial.println("动作录制模式");}  //如果是动作录制模式，串口监视器中打印"动作录制模式"字样
      if(Mode==2){Serial.println("自动播放模式");}  //如果是自动播放模式，串口监视器中打印"自动播放模式"字样
      Pressbuf[1] = true;  //按键buf赋值1
      Releasebuf[1] = false;  //Releasebuf赋值0
     }  
  }
  if (!ps2x.Button(PSB_PAD_UP)&&!ps2x1.Button(BTB_PAD_UP))  //如果没有按下无线手柄UP键，并且没有按下蓝牙App的UP键
  {
    if(Releasebuf[1] == false)  //如果Releasebuf值为0
    {
      Pressbuf[1]=false;  //按键buf赋值0
      Releasebuf[1]=true;  //Releasebuf赋值1
    }
  }
  //按照RBG指示灯对颜色显示的引脚设置,001蓝灯 110黄色 100红色 010绿色
  if(Mode==0){digitalWrite(Led1,LOW);digitalWrite(Led2,LOW);digitalWrite(Led3,HIGH);}  //如果模式为手动控制模式，蓝灯亮
  if((Mode==1)&&((!(ps2x.Button(PSB_CIRCLE)&&!ps2x1.Button(BTB_B))&&!(PCact[14]==1))&&(actionIndex<=maxAutoActions))){digitalWrite(Led1,HIGH);digitalWrite(Led2,HIGH);digitalWrite(Led3,LOW);}  //如果模式为动作录制模式，并且无线手柄○按键没有按下，或者蓝牙App B按键没有按下，或者电脑上位机没有使PCact[14]值为1，并且运动步骤数actionIndex<最大可自动运行步骤数的，黄灯亮
  if((Mode==1)&&(ps2x.Button(PSB_CIRCLE)||ps2x1.Button(BTB_B)||(PCact[14]==1))&&(actionIndex<=maxAutoActions)){digitalWrite(Led1,HIGH);digitalWrite(Led2,LOW);digitalWrite(Led3,LOW);}  ////如果模式为动作录制模式，并且无线手柄○按键按下，或者蓝牙App B按键按下，或者电脑上位机使PCact[14]值为1，并且运动步骤数actionIndex<最大可自动运行步骤数的，红灯亮
  if(Mode==2){digitalWrite(Led1,LOW);digitalWrite(Led2,HIGH);digitalWrite(Led3,LOW);}  //如果模式为自动播放模式，绿灯亮     
}

void ShowAngle()  //函数8：按下Down读取当前坐标值
{
  if (ps2x.Button(PSB_PAD_DOWN)||ps2x1.Button(BTB_PAD_DOWN)||(PCact[11]==1))  //如果按下无线手柄DOWN键，或按下蓝牙App的DOWN键，又或者电脑上位机PCact[11]状态为1
  {
      if(Pressbuf[4] == false)  //如果按键buf为0
      {
        Serial.print( "a:" );  //串口监视器中打印"a:"字样
        Serial.print( currentAngle[0] );  //串口监视器中打印当前爪头舵机角度值
        Serial.print( ",b:" );  //串口监视器中打印"b:"字样
        Serial.print( currentAngle[1] );  //串口监视器中打印当前小臂舵机角度值
        Serial.print( ",c:" );  //串口监视器中打印"c:"字样
        Serial.print( currentAngle[2] );  //串口监视器中打印当前大臂舵机角度值
        Serial.print( ",d:" );  //串口监视器中打印"d:"字样
        Serial.print( currentAngle[3] );  //串口监视器中打印当前转台舵机角度值
        Serial.println();  //串口监视器中换行
        PCact[11]=0;  //PCact[11]赋值0
        Pressbuf[4] = true;  //按键buf赋值1
        Releasebuf[4] = false;  //Releasebuf赋值0
      }
  }
  if(!(ps2x.Button(PSB_PAD_DOWN)||ps2x1.Button(BTB_PAD_DOWN)||(PCact[11]==1)))  //如果没有按下无线手柄DOWN键，也没有按下蓝牙App的DOWN键，电脑上位机PCact[11]状态也不为1
  {
    Pressbuf[4] = false;  //按键buf赋值0
    Releasebuf[4] = true;  //Releasebuf赋值1
  }
}

void MemoryAction()  //函数9：自动录制模式,按下○或蓝牙B录制点位
{
   if(Mode==1)  //如果为录制模式
   {
     if (ps2x.Button(PSB_CIRCLE)||ps2x1.Button(BTB_B)||(PCact[14]==1))  //如果按下无线手柄○键，或按下蓝牙App的B键，又或者电脑上位机PCact[14]状态为1
     {
       if(Pressbuf[2] == false)  //如果按键buf为0
       {
         if(actionIndex<=maxAutoActions)  //如果运动步骤数actionIndex<=最大可自动运行步骤数
         {
           Actionmem[actionIndex][0]=currentAngle[0];  //记录爪头当前步数的当前角度值
           Actionmem[actionIndex][1]=currentAngle[1];  //记录小臂当前步数的当前角度值
           Actionmem[actionIndex][2]=currentAngle[2];  //记录大臂当前步数的当前角度值
           Actionmem[actionIndex][3]=currentAngle[3];  //记录转台当前步数的当前角度值
           Serial.print("动作记录");  //串口监视器中打印"动作记录"字样
           Serial.print(actionIndex);
           Serial.print(":");  //串口监视器中打印":"字样
           Serial.print(Actionmem[actionIndex][0]);  //串口监视器中打印爪头当前步数的当前角度值
           Serial.print(",");  //串口监视器中打印","字样
           Serial.print(Actionmem[actionIndex][1]);  //串口监视器中打印小臂当前步数的当前角度值
           Serial.print(",");  //串口监视器中打印","字样
           Serial.print(Actionmem[actionIndex][2]);  //串口监视器中打印大臂当前步数的当前角度值
           Serial.print(",");  //串口监视器中打印","字样
           Serial.println(Actionmem[actionIndex][3]);  //串口监视器中打印转台当前步数的当前角度值
           if(actionIndex<=maxAutoActions)  //如果运动步骤数actionIndex<=最大可自动运行步骤数
           {
             actionIndex++;  //运动步骤数actionIndex自加1
           }
         }
         PCact[14]=0;  //PCact[14]赋值0
         Pressbuf[2] = true;  //按键buf赋值1
         Releasebuf[2] = false;  //Releasebuf赋值0
       }
    }
    if((!ps2x.Button(PSB_CIRCLE))&&(!ps2x1.Button(BTB_B))&&(!(PCact[14]==1)))  //如果没有按下无线手柄○键，或没有按下蓝牙App的B键，又或者电脑上位机PCact[14]状态不为1  
    {
      Pressbuf[2] = false;  //按键buf赋值0
      Releasebuf[2] = true;  //Releasebuf赋值1
    }
  }    
}

void Totalactcnt()  //函数10：记录当前录制总步骤数
{
  for ( int i = 0; i < maxAutoActions; i ++ )  //for循环检测当前步数是否小于最大可自动运行步骤数
  {
    if( Actionmem[i][0]!=0)  //如果当前记录舵机的当前步骤不是0
    {
      Totalact=i;  //总步骤数为当前步数
    }
  }
}

void Autoplay()  //函数11：自动播放模式,按下△或蓝牙A开始自动播放,按下X或蓝牙C结束自动播放
{
  ms = millis();  //从arduino上电到现在返回时间，单位ms
  if(Mode!=2)  //如果不是自动播放模式
  {
    autoplaying=false;  //自动播放禁止
  }
  if(Mode==2&&Totalact!=0)  //如果是自动播放模式，并且总步骤数不为0
  {
    if ((ps2x.Button(PSB_TRIANGLE)||ps2x1.ButtonPressed(BTB_A)||(PCact[13]==1))&&autoplaying==false)  //如果按下无线手柄△键，或按下蓝牙App的A键，又或者电脑上位机PCact[13]状态不为1，并且自动播放禁止
    {
      autoplaying=true;  //此时，将自动播放可用
      Serial.println("自动运行开始");  //串口监视器中打印"自动运行开始"字样
      PCact[13]=0;  //PCact[13]赋值0
    }
    if ((ps2x.Button(PSB_CROSS)||ps2x1.ButtonPressed(BTB_C)||(PCact[15]==1))&&autoplaying==true)  //如果没有按下无线手柄Ⅹ键，或没有按下蓝牙App的C键，又或者电脑上位机PCact[15]状态不为1，并且自动播放可用  
    {
      autoplaying=false;  //此时，将自动播放禁用
      Serial.println("自动运行停止");  //串口监视器中打印"自动运行停止"字样
      PCact[15]=0;  //PCact[15]赋值0
    }
  }
  if(!autoplaying)  //如果自动播放禁用
  {
    autotrigbuf=true;  //自动播放上升沿buf赋值1
    autotrig=false;  //自动播放上升沿赋值0
  }
  if(autoplaying&&autotrigbuf)  //如果自动播放可用，并且自动播放上升沿buf为1
  {
    autotrig=true;  //自动播放上升沿赋值1
    autotrigbuf=false;  //自动播放上升沿buf赋值0
  }
  if(autotrig)  //自动播放上升沿为1
  {
    autoindex=0;  //自动播放步数为0
    autotrig=false;  //自动播放上升沿赋值0
  }
  if(autoplaying)  //如果自动播放可用
  {
    writeServo( PIN[0], Actionmem[autoindex][0] );  //将当前步数的当前角度值，写入爪头舵机，让该舵机转到这个角度
    writeServo( PIN[1], Actionmem[autoindex][1] );  //将当前步数的当前角度值，写入小臂舵机，让该舵机转到这个角度
    writeServo( PIN[2], Actionmem[autoindex][2] );  //将当前步数的当前角度值，写入大臂舵机，让该舵机转到这个角度
    writeServo( PIN[3], Actionmem[autoindex][3] );  //将当前步数的当前角度值，写入转台舵机，让该舵机转到这个角度
    if(autoindex!=indexbuf)  //如果当前步数不等于自动播放指针缓存
    {
      t[0]=ms;  //记录下从Arduino上电到现在的总时间，单位ms
      indexbuf=autoindex;  //把当前步数赋值给自动播放指针缓存
    }  
    t[1]=ms-t[0];  //再将当前从Arduino上电到现在的总时间，减去之前的t]0]时间的差值，即，前一次记录时间到现在的记录的时间的差值时间赋值给t[1]
    if(t[1]>actdelay)  //如果这个t[1]时间>actdelay(自动播放动作间隔300毫秒)时间
    {
      if(autoindex<Totalact+1)  //如果当前步数<总步骤数+1
      {
        autoindex=autoindex+1;  //当前步数自加1
      }
    }
    if(autoindex==Totalact+1)  //如果当前步数=总步骤数+1
    {
      autoindex=0;  //当前步数置0
    }
  }
}

void Autodelete()  //函数12：按下□或蓝牙D,删除记录点位
{
    if(Mode==1)  //如果为录制模式
    {
      if ((ps2x.Button(PSB_SQUARE)||ps2x1.Button(BTB_D)||(PCact[16]==1))&&Totalact!=0)  //如果按下无线手柄□键，或按下蓝牙App的D键，又或者电脑上位机PCact[16]状态不为1，并且总步骤数不为0
      {
         if(Pressbuf[3] == false)  //如果按键buf为0
         {
           for ( int i = 0; i < maxAutoActions; i ++ )  //for循环检测当前步数是否小于最大可自动运行步骤数
           {
             Actionmem[i][0]=0;  //爪头舵机的当前步数当前角度清零
             Actionmem[i][1]=0;  //小臂舵机的当前步数当前角度清零
             Actionmem[i][2]=0;  //大臂舵机的当前步数当前角度清零
             Actionmem[i][3]=0;  //转台舵机的当前步数当前角度清零
           } 
           Totalact=0;  //总步骤数清0
           actionIndex=0;  //当前步数置0
           Serial.println("清空已存点位");  ////串口监视器中打印"清空已存点位"字样
           PCact[16]=0;  //PCact[16]置0
           Pressbuf[3] = true;  //按键buf赋值1
           Releasebuf[3] = false;  //Releasebuf赋值0
         }
      }
      if(!ps2x.Button(PSB_SQUARE)&&!ps2x1.Button(BTB_D))  //如果没有按下无线手柄□键，或没有按下蓝牙App的D键
      {
        Pressbuf[3] = false;  //按键buf赋值0
        Releasebuf[3] = true;  //Releasebuf赋值1
      }
    }
}

void PCmode()  //函数13:PC控制模式通讯
{         
  if (stringComplete)  //如果串口接收标志为1
  {
    if(PCcmd[0]=="M")  //如果PCcmd[0]获得的窗口数据字符M
    {
      for ( int i = 0; i < 4; i ++ )  //for循环
      {
        PCmove[i]=PCcmd[i+1];  //M以后的输入值，一位一位赋值给每个舵机
      }     
    } 
    for ( int i = 0; i < 4; i ++ )  //for循环
    {
      if(PCmove[i]=="0")  //如果当前舵机的输入值为字符0
      {
        PCact[i]=0;  //执行停止
      }
      if(PCmove[i]=="1")  //如果当前舵机的输入值为字符1
      {
        PCact[i]=1;  //执行正转
      }
      if(PCmove[i]=="2")  //如果当前舵机的输入值为字符2
      {
        PCact[i]=2;  //执行反转
      }
     }
    if((PCcmd[0]=="a")&&(stringlength==16))  //如果输入的首字符为a，并且，字符串长度为16个字符
    {
      RcvBuf[0]+=PCcmd[1];  //爪头：角度值RcvBuf（String类型）先自加1到下一位（第2位），然后，字符串第2个字符赋值给它
      RcvBuf[0]+=PCcmd[2];  //爪头：角度值RcvBuf（String类型）先自加1到下一位（第3位），然后，字符串第3个字符赋值给它
      RcvBuf[0]+=PCcmd[3];  //爪头：角度值RcvBuf（String类型）先自加1到下一位（第4位），然后，字符串第4个字符赋值给它
      Rcvint[0]=RcvBuf[0].toInt();  //将爪头舵机字符类型的角度值转化为整型
      //以下同理，将小臂、大臂、转台执行输入的角度值（字符类型）转换为int整型，为每个舵机1个确定角度数值
      RcvBuf[1]+=PCcmd[5];
      RcvBuf[1]+=PCcmd[6];
      RcvBuf[1]+=PCcmd[7];
      Rcvint[1]=RcvBuf[1].toInt();
      RcvBuf[2]+=PCcmd[9];
      RcvBuf[2]+=PCcmd[10];
      RcvBuf[2]+=PCcmd[11];
      Rcvint[2]=RcvBuf[2].toInt();
      RcvBuf[3]+=PCcmd[13];
      RcvBuf[3]+=PCcmd[14];
      RcvBuf[3]+=PCcmd[15];
      Rcvint[3]=RcvBuf[3].toInt();
      for( int i = 0; i < 4; i ++ )  //for循环，将4个舵机的输入角度值与限位值进行比较，为的是输入角度值不超过极限位
      {
        if(Rcvint[i]<MIN[i])  //如果当前舵机输入的角度值<该舵机最小限位值
        {
          Rcvint[i]=MIN[i];  //当前舵机输入角度值就改变为该舵机最小限位值
        }
        if(Rcvint[i]>MAX[i])  //如果当前舵机输入的角度值>该舵机最大限位值
        {
          Rcvint[i]=MAX[i];  //当前舵机输入角度值就改变为该舵机最大限位值
        }
      }
      writeServo( PIN[0], Rcvint[0] );  //将输入的角度值，写入爪头舵机，让爪头舵机转到这个角度
      writeServo( PIN[1], Rcvint[1] );  //将输入的角度值，写入爪头舵机，让小臂舵机转到这个角度
      writeServo( PIN[2], Rcvint[2] );  //将输入的角度值，写入爪头舵机，让大臂舵机转到这个角度
      writeServo( PIN[3], Rcvint[3] );  //将输入的角度值，写入爪头舵机，让转台舵机转到这个角度
      currentAngle[0] = Rcvint[0];  //将爪头的输入角度值作为当前角度
      currentAngle[1] = Rcvint[1];  //将小臂的输入角度值作为当前角度
      currentAngle[2] = Rcvint[2];  //将大臂的输入角度值作为当前角度
      currentAngle[3] = Rcvint[3];  //将转台的输入角度值作为当前角度
      }
    if(inputString=="UP")  //如果输入字符UP
    {
      PCact[10]=1;  //PCact[10]置1，该标志就是为函数7：void ModeChange()，模式变更的条件之一
    }
    if(inputString=="DOWN")  //如果输入字符DOWN
    {
      PCact[11]=1;  //PCact[11]置1，该标志就是为函数8：void ShowAngle()，读取当前坐标值的条件之一
    }
    if(inputString=="LEFT")  //如果输入字符LEFT
    {
      PCact[12]=1;  //PCact[12]置1，该标志就是为函数2：void Home()，回原点功能的条件之一
    }
    if(inputString=="A")  //如果输入字符A
    {
      PCact[13]=1;  //PCact[13]置1，该标志就是为函数11：void Autoplay()，自动播放模式,开始自动播放的条件之一
    }
    if(inputString=="B")  //如果输入字符B
    {
      PCact[14]=1;  //PCact[14]置1，该标志就是为函数9：void MemoryAction()，自动录制模式,录制点位的条件之一
    }
    if(inputString=="C")  //如果输入字符C
    {
      PCact[15]=1;  //PCact[15]置1，该标志就是为函数11：void Autoplay()，自动播放模式,结束自动播放的条件之一
    }
    if(inputString=="D")  //如果输入字符D
    {
      PCact[16]=1;  //PCact[16]置1，该标志就是为函数12：void Autodelete()，删除记录点位的条件之一
    }
    for ( int i = 0; i < 4; i ++ )  //for循环，分别设置4个舵机
    {
      RcvBuf[i]="";  //将空字符赋值给每个舵机，以起到重置的作用
      Rcvint[i]=0;  //将0赋值给每个舵机，以起到重置的作用
    }
    inputString = "";  //将串口数据重置
    stringlength = 0;  //将串口字符长度置0（重置）
    Serialindex =0;  //串口指针指向初始0位置
    stringComplete = false;  //串口接收标志false，表示重置，为循环再判断做准备
  }
}

void serialEvent()  //函数14:串口事件,用于PC模式通讯
{
  while (Serial.available())   //当串口有数据
  {
    char inChar = (char)Serial.read();  //读取串口数据（字符类型）
    PCcmd[Serialindex]=inChar;  //将当前读取到的字符赋值给当前的串口指针对应的数据位的PC指令字符
    if(inChar!='\n')  //如果字符不是回车符
    {
      inputString += inChar;  //将该字符+1位
      stringlength++;  //字符长度也+1位
      Serialindex++;  //该位也+1
    }
    if (inChar == '\n')  //如果字符是回车符
    {
      stringComplete = true;  //表示该字符串符合字符定义规则，将标志符stringComplete置1
     }
  }
}
