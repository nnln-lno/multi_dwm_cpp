#include "dwm_1001_reader/dwm_comm.hpp"

#include <string>
#include <vector>
#include <iostream>

dwmComm::dwmComm() : Node("dwm_node")
{
    this->declare_parameter("dwm_comm_params.use_port", false);
    bool use_port = this->get_parameter("dwm_comm_params.use_port").as_bool();

    RCLCPP_INFO(this->get_logger(), "Use Port Param : %d", use_port);

    if (use_port)
    {
        this->declare_parameter("dwm_comm_params.port", "default");
        std::string port = this->get_parameter("dwm_comm_params.port").as_string();

        RCLCPP_INFO(this->get_logger(), "Port Provided : %s", port.c_str());
        dev_port = port;
    }

    this->declare_parameter("dwm_comm_params.baudrate", 115200);
    baud_ = this->get_parameter("dwm_comm_params.baudrate").as_int();

    if(dev_port.compare("nan") != 0)
    {
        dwmComm::initSerial(dev_port);
    }
    else
    {
        throw std::runtime_error("Provided Device Port not Founded !!");
    }

    /*
    여기에 Serial 또는 Publisher 토픽 같은거 declare 하는 코드 작성하기
    */
   
    this->declare_parameter("dwm_comm_params.rate_hz", 0);
    uint16_t read_rate = this->get_parameter("dwm_comm_params.rate_hz").as_int();

    auto timer_period_ = int((1.0/read_rate) * 1000); // as millisecond

    RCLCPP_INFO(this->get_logger(), "Timer Period : %d", timer_period_);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_),
                std::bind(&dwmComm::readData, this));

    // dev.write("les");
    // dev.write("\r\r");

    dev.write("si");
    dev.write("\r");
}

void dwmComm::initSerial(std::string dev_port_)
{
    try
    {
        dev.setPort(dev_port_);
        dev.setBaudrate(baud_);

        serial::Timeout to_ = serial::Timeout(200, 200, 0, 200, 0);

        dev.setTimeout(to_);
        dev.open();
    } catch(serial::IOException e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to open port %s", e.what());
    }
}

void dwmComm::readData()
{
    std::string msgs;

    if(dev.available() > 1)
    {    
        msgs = dev.readline();        

        RCLCPP_INFO(this->get_logger(), "Received Message : %s", msgs.c_str());
    }

    dev.flushOutput();
    dev.flushInput();
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dwmComm>());
    rclcpp::shutdown();
    return 0;
}
