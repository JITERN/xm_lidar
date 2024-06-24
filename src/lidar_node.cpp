#include "rclcpp/rclcpp.hpp"
#include <libserial/SerialPort.h>
// #include <libserial/SerialStream.h>
#include <vector>
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
using namespace std::chrono_literals;

bool process_packet(std::string str, std::vector<float>& output){
    std::string temp = "";
    char del = '\t';
    int j=0, base_angle;
    for(int i=0; i<(int)str.size(); i++){
        if(str[i] != del){
            temp += str[i];
        }
        else{
            if (j==0) base_angle = stoi(temp);
            // else if (j==1) speed = stoi(temp);
            else if ((1<j)&&(j<6)){
                output[base_angle+j-2] = stoi(temp)/1000.0;
            }
            j++;
            temp = "";
        }
    }
    if (base_angle >=356) return true;
    else return false;
    // std::cout << output[0] <<std::endl;
}

class LidarNode : public rclcpp::Node
{
  public:

    LibSerial::SerialPort serial_port ;
    
    LidarNode()
    : Node("lidar_node"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    serial_port.Open( "/dev/ttyUSB0" ) ;
    using LibSerial::BaudRate ;
    serial_port.SetBaudRate( BaudRate::BAUD_115200 ) ;
    size_t timeout_milliseconds = 10 ;
    char read_byte;
    std::string raw_data;
    std::vector<float> dist(360);

    using LibSerial::ReadTimeout ;

    while(rclcpp::ok()) {
        try
        {
            serial_port.ReadByte(read_byte, timeout_milliseconds) ;
            // serial_port.read(input_buffer, BUFFER_SIZE );

            // std::cout << read_byte ;//<< std::endl;
            if (read_byte=='\n'){
                // std::cout<< raw_data <<std::endl;
                if(process_packet(raw_data,dist)) {
                    auto msg = sensor_msgs::msg::LaserScan();
                    msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
                    msg.header.frame_id = "lidar_frame";
                    msg.angle_min= -3.14159;
                    msg.angle_max= 3.14159;
                    msg.angle_increment= 1/180.0 * 3.14159;
                    // #msg.time_increment= (60/int(self.data['speed'][-1]))/360
                    // #msg.scan_time= 60/int(self.data['speed'][-1])
                    // msg.time_increment= (rospy.Time.now().to_sec()-self.prev_time)/360
                    // msg.scan_time= (rospy.Time.now().to_sec()-self.prev_time)
                    msg.range_min= 0.1;
                    msg.range_max= 8;
                    msg.ranges = dist;

                    publisher_->publish(msg);
                }
                raw_data = "";
                
                // std::cout<<dist[100]<<std::endl;
                
            }
            else{
                raw_data+=read_byte;
            }
            
        }
        catch (const ReadTimeout&)
        { //
        }
    }
    
    }
    ~LidarNode(){
        serial_port.Close();
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    size_t count_;
};


int main(int argc, char **argv)
{
 
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
