#include"mraa.hpp"
#include"iostream"
#include"ros/ros.h"
#include"std_msgs/String.h"
#define GETIT  system("echo loadfile /Robot/voice/wav/Begin.wav>/Robot/cmd/Mplayer_cmd")
 mraa::Uart * dev;
 void Init_uart(void)
{
    try {
        dev = new mraa::Uart("/dev/ttyS4");
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
    dev->writeStr("RESET\n");
}
int main(int argc,char** argv)
{
    int i;
    std_msgs::String msg;
    ros::init(argc,argv,"wakeup");
    ros::NodeHandle n;
    ros::Rate loop(10);
    ros::Publisher pub=n.advertise<std_msgs::String>("xfwakeup",1000);
     Init_uart();
     GETIT;
     std::cout<<"uart Init OK"<<std::endl;
     while(ros::ok())
     {
         if(dev->dataAvailable())
         {
             std::string s;
             while(dev->dataAvailable())
             {
                 s+=dev->readStr(1);
             }
             i=s.find("WAKE UP!angle:");
             if(i!=std::string::npos)
             {
                 std::stringstream ss;
                  GETIT;
                 ss <<s.substr(i+14,3);
                 msg.data = ss.str();
                 std::cout<<"get wakesign!"<<i<<">"<<ss.str()<<std::endl;
                 pub.publish(msg);
                 sleep(5);
                dev->writeStr("RESET\n");
             }
         }
         ros::spinOnce();
         loop.sleep();
     }

}
