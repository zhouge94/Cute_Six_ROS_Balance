#include <iostream>
#include"mraa.hpp"
#include "pthread.h"
#include <unistd.h>
#include"balance/Num.h"
#include"ros/ros.h"
#include"std_msgs/String.h"
int flag_auto=1;
using namespace std;
class my_uart
{
public:
    bool Is_connect;
    my_uart()
    {
        Is_connect=0;
    }
    ~my_uart(){}
    mraa::Uart *dev;

    float RecievedFloatArray[10];

    void Init(const std::string path,int rate)
    {
        try { dev = new mraa::Uart(0);  }
        catch (std::exception& e) {   std::cout << e.what() << ", likely invalid platform config" << std::endl;      }
        try {   dev = new mraa::Uart(path);   }
        catch (std::exception& e)
        {
            std::cout << "Error while setting up raw UART, do you have a uart?" << std::endl;
            std::terminate();
        }
        if (dev->setBaudRate(rate) != mraa::SUCCESS) {  std::cout << "Error setting parity on UART" << std::endl;    }
        if (dev->setMode(8, mraa::UART_PARITY_NONE, 1) != mraa::SUCCESS) {  std::cout << "Error setting parity on UART" << std::endl;        }
        if (dev->setFlowcontrol(false, false) != mraa::SUCCESS) { std::cout << "Error setting flow control UART" << std::endl;
        }
        Is_connect=1;
       //  dev->writeStr("<A>");
    }
    void SendStr(const std::string str)
    {
        if(Is_connect)dev->writeStr(str);
        else std::cout<<"you haven't conect!"<<std::endl;
    }
    void Sendbyte(unsigned char byte)
    {
        char *data=new char;
        *data=byte;
        if(Is_connect)dev->write(data,1);
        else std::cout<<"you haven't conect!"<<std::endl;
    }
    void SendCommend(unsigned char Commend)//have tested
    {
        Sendbyte('<');
        Sendbyte(Commend);
        Sendbyte('>');
    }
    void SendFloat(unsigned char Addr,float FloatData)//have tested
    {
        unsigned char i = 0;
        unsigned char *p = ( unsigned char *)&FloatData;
        unsigned char sum = 0;
        Sendbyte('{');
        Sendbyte(Addr);
        for(i=0;i<4;i++)
       {
            sum = sum+*p;
            Sendbyte(*p++);
        }
        sum  = sum + '{' +Addr;
        Sendbyte(sum);
        Sendbyte('}');
    }
    unsigned char RecievetoFloatArray(unsigned char Byte)
  {
     static unsigned char RecievedByteArray[40];
     static unsigned char sum=0;
     static unsigned char index=0;
     static unsigned char RecievedState = 0;
     static float *p  = (float*)RecievedByteArray;
     static float *q = RecievedFloatArray;
     unsigned char i;
     bool NotTakeData ;
     NotTakeData = true;
      if(RecievedState ==0||RecievedState ==3&&NotTakeData)
      {
         NotTakeData = false;
         if(Byte=='$')
          {
             RecievedState = 1;
             index   = 0;
         }
      }
      if(RecievedState ==1&&NotTakeData)
      {
          NotTakeData = false;
          if(index<40)
          {
              RecievedByteArray[index++] = Byte;
          }
          else
          {
              RecievedState =2;
          }
          sum = sum +Byte;
      }
      if(RecievedState ==2&&NotTakeData)
      {
          NotTakeData = false;
          sum = sum+'$';
          if(sum == Byte)
          {
              for(i=0;i<10;i++)
                 *q++ = *p++;
              RecievedState = 3;
          }

          else
          {
             ; //error
             RecievedState =0;
          }
          p  = (float*)RecievedByteArray;
          q = RecievedFloatArray;
          sum = 0;
      }
      return RecievedState ;
  }
    float GetFloatFromCHx(unsigned char CHx)
    {
        return RecievedFloatArray[CHx];
    }

    void SetYunTai(unsigned char Angle) //have tested
    {
        float PWM_YunTai;
        PWM_YunTai = (float)Angle/180.0*1000+250;
        SendFloat(11,PWM_YunTai);
    }
    void QiLuo12Down(){SendFloat(12,600);}//have tested
    void QiLuo3Down(){SendFloat(13,1000);}//have tested
    void QiLuo4Down(){SendFloat(14,1000);}//have tested
    void QiLuo12Up(){SendFloat(12,1000);}//have tested
    void QiLuo3Up(){SendFloat(13,600);}//have tested
    void QiLuo4Up(){SendFloat(14,600);}//have tested

};

void demo_uart(void)
{
    mraa::Uart * dev;
    try {
        dev = new mraa::Uart(0);
    } catch (std::exception& e) {
        std::cout << e.what() << ", likely invalid platform config" << std::endl;
    }
    try {
        dev = new mraa::Uart("/dev/ttyS5");
    } catch (std::exception& e) {
        std::cout << "Error while setting up raw UART, do you have a uart?" << std::endl;
        std::terminate();
    }
    if (dev->setBaudRate(115200) != mraa::SUCCESS) {
        std::cout << "Error setting parity on UART" << std::endl;
    }
    if (dev->setMode(8, mraa::UART_PARITY_NONE, 1) != mraa::SUCCESS) {
        std::cout << "Error setting parity on UART" << std::endl;
    }    if (dev->setFlowcontrol(false, false) != mraa::SUCCESS) {
        std::cout << "Error setting flow control UART" << std::endl;
    }
    dev->writeStr("<A>");
}

my_uart dev;
void* callback(void* ptr)
{
  static unsigned char RecievedData=0;
  static  char *data=new char;
  static int num;
  data=(char*)&RecievedData;
  while(1)
  {
      while(!dev.dev->dataAvailable(0));
      num = dev.dev->read(data,1);
      //printf("%d ",RecievedData);
      //printf("i have get the  data:%d \n",RecievedData);
     // printf("Recieve state is :%d\n",dev.RecievetoFloatArray(RecievedData));
      if(dev.RecievetoFloatArray(RecievedData)==3)
      {
      //   printf("i have get the ok data \n");
         usleep(50*1000);
      }
  }
}
void control_callback(const balance::Num::ConstPtr& msg)
{
    float xspeed,turnspeed;
    float kx=20;
    float kturn=10;
    static int i=0;

    if(i==1)i=0;
    else   i++;
    if(flag_auto)
    {
     xspeed=(msg->leftspeed+msg->rightspeed)*kx;
     turnspeed=(msg->leftspeed-msg->rightspeed)*kturn;
    //std::cout<<"xspeed:"<<xspeed<<"turn:"<<turnspeed<<std::endl;
  //  dev.SendFloat(16,xspeed);
  //  dev.SendFloat(17,turnspeed);
    if(i==0)
    {
        if(turnspeed>0)dev.SendCommend('d');
        else if(turnspeed<0)dev.SendCommend('a');
        std::cout<<"xspeed:"<<xspeed<<std::endl;
        if(turnspeed<6&&turnspeed>-6)dev.SendCommend(5);
    }
      if(i==1)
      {
        if(xspeed>0)dev.SendCommend('w');
        else if(xspeed<0)dev.SendCommend('s');
        std::cout<<"turn:"<<turnspeed<<std::endl;
       //if(xspeed<6&&xspeed>-6)dev.SendCommend(5);
      }

    }else
    {
        std::cout<<"notic :current mode is manual"<<std::endl;
    }
}
void cmd_callback(const std_msgs::String::ConstPtr& msg)
{
    std::string cmd;
    cmd=msg->data;
    std::cout<<"balance::Get comond:"<<cmd<<std::endl;
}
int flag_get_guicmd;
std::string gui_cmd;
void *callback_get_guicmd(void *ptr)
{
    std::cout<<"Get gui_cmd child begin"<<std::endl;
    char cmd[] = "cat /Robot/cmd/cmd_of_gui ";
    FILE *pp;
    while(1)
   {
    printf("waiting for fifo----");
    pp= popen(cmd, "r");
    if (pp != NULL)
    {
        char tmp[1024] = {0};
        if(fgets(tmp, sizeof(tmp), pp) != NULL)
        {
            flag_get_guicmd=1;
            gui_cmd=tmp;
            std::cout<<gui_cmd<<std::endl;
        }
    }
    if(flag_get_guicmd)
    {
         flag_get_guicmd=0;
         if(gui_cmd.find("noauto")==0)flag_auto=0;
         if(gui_cmd.find("auto")==0)flag_auto=1;
         if(flag_auto)std::cout<<"notice : current mode is auto"<<std::endl;
        else
        {
            std::cout<<"notice : current mode is mannual"<<std::endl;
            if(gui_cmd.find("cmd")==0)
            {
                char cmd[]="<0>";
                const char *data;
                data= gui_cmd.c_str();
                dev.SendCommend(data[4]);
                cmd[1]=data[4];
                std::cout<<cmd<<std::endl;
            }
        }
    }
    usleep(50*1000);
}
}
int main(int argc ,char ** argv)
{
    float count;
    pthread_t pid,pid_cmd;
    ros::init(argc, argv, "balance");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);
    ros::Subscriber sub = n.subscribe("control_speed", 1000, control_callback);
    ros::Subscriber pub_control= n.subscribe("/control_cmd",10000,cmd_callback);
    ros::Publisher pub_speed = n.advertise<balance::Num>("wheel_speed", 1000);
    balance::Num speed;
    dev.Init("/dev/ttyS5",115200);
    pthread_create(&pid, NULL, callback, NULL);
    pthread_create(&pid, NULL, callback_get_guicmd, NULL);

    dev.SendStr("<A>");
    while(ros::ok())
    {
          //printf("i am 0:%f\n",dev.GetFloatFromCHx(0));
           speed.leftspeed=dev.GetFloatFromCHx(2)*(-0.01969);//cm
           speed.rightspeed=dev.GetFloatFromCHx(3)*(-0.01969);//cm
           pub_speed.publish(speed);
           ros::spinOnce();
           loop_rate.sleep();
           count+=(dev.GetFloatFromCHx(2)-dev.GetFloatFromCHx(3));
      //     std::cout<<"count::"<<count<<std::endl;
    }
   //  cout << "Hello World!" << endl;
    return 0;
}

