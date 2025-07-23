#ifndef DWM_COMM_HPP_
#define DWM_COMM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include <serial/serial.h>

class dwmComm : public rclcpp::Node
{
    public : 

        serial::Serial dev;
        std::string dev_port;
        std::string hw_id;

        dwmComm();
    
    private :

        std::string getPort();
        void initSerial(std::string dev_port_);
        
        std::string sendData(const std::string &msg_to_send, bool print_oputput = false);
        void readData();

        int baud_;

        bool is_single_;
        bool is_continuous;

        rclcpp::TimerBase::SharedPtr timer_;

};

#endif