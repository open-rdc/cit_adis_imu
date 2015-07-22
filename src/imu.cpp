#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <tf/transform_broadcaster.h>

#include <boost/bind.hpp>
#include <boost/lambda/lambda.hpp>

#include <sys/time.h>

#include "imu/bool_msg.h"
#include "Comm.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>

#include <stdexcept>

class CheckSumError : public std::logic_error { 
public:
    CheckSumError(const std::string &data) : 
        std::logic_error("Received the Invalid IMU Data: " + data)
    {
    }
};

float deg_to_rad(float deg) {
    return deg / 180.0f * M_PI;
}

template<class T>
bool isInRange(T value, T max, T min){
    return (value >= min) && (value <= max);
}

struct ImuData {
    double angular_deg[3];
    double temperature;
    
    ImuData(){
        for(int i=0; i < 2; i++){
            angular_deg[i] = 0;
        }
        temperature = 0;
    }
};


// 角度ごとにURGのデータを送信するためのサービス
// 要求があったときに，URGのデータを別のトピックに送信するだけ
class IMU {
private:
    ros::Publisher imu_pub_;
    ros::Publisher imu_temp_;
    ros::ServiceServer reset_service_;
    ros::ServiceServer carivrate_service_;
    float geta;
    CComm usb;
    double gyro_unit;
    double acc_unit;
    double init_angle;
    std::string port_name;
    int   baudrate;
    ros::Rate loop_rate;
    int  z_axis_dir_;

    ImuData getImuData(){
        ImuData data;
        char command[2] = {0};
        char command2[50] = {0};
        char temp[6];
        unsigned long int sum = 0;
        
        const float temp_unit = 0.0565;
        
        sprintf(command, "o");
        usb.Send(command, strlen(command));
        ros::Duration(0.03).sleep();
        
        usb.Recv(command2, 50);
        ROS_INFO_STREAM("recv = " << command2);

        memmove(temp,command2,4);
        sum = sum ^ ((short)strtol(temp, NULL, 16));
        data.angular_deg[0] = ((short)strtol(temp, NULL, 16)) * gyro_unit;
        memmove(temp,command2+4,4);
        sum = sum ^ ((short)strtol(temp, NULL, 16));
        data.angular_deg[1] = ((short)strtol(temp, NULL, 16)) * gyro_unit;
        memmove(temp,command2+8,4);
        sum = sum ^ ((short)strtol(temp, NULL, 16));
        data.angular_deg[2] = ((short)strtol(temp, NULL, 16)) * gyro_unit * z_axis_dir_;

        memmove(temp,command2+12,4);
        if(sum != ((short)strtol(temp, NULL, 16))){
            ROS_ERROR_STREAM("Recv Checksum: " << ((short)strtol(temp, NULL, 16)));
            ROS_ERROR_STREAM("Calculate Checksum: " << sum);
            throw CheckSumError(command2);
        }
        //while(data.angular_deg[2] < -180) data.angular_deg[2] += 180;
        //while(data.angular_deg[2] > 180) data.angular_deg[2] -= 180;

        memmove(temp,command2+16,4);
        data.temperature = ((unsigned short int)strtol(temp,NULL,16)) * temp_unit;
        
        return data;
    }

public:
    bool resetCallback(imu::bool_msg::Request  &req, 
            imu::bool_msg::Response &res) 
    {
        char command[2] = {0};
        sprintf(command, "0");
        usb.Send(command, strlen(command));
        sleep(1);
        std::cout << "Gyro 0 Reset" << std::endl;
        return true;
    }
    
    bool caribrateCallback(imu::bool_msg::Request  &req, 
            imu::bool_msg::Response &res) 
    {
        char command[2] = {0};
        std::cout << "Calibration" << std::endl;
        sprintf(command, "a");
        usb.Send(command, strlen(command));
        std::cout << "Caribrate start ";
        for(int i=0; i<8; ++i) {
            std::cout << ". ";
            sleep(1);
        }
        std::cout << "finish." << std::endl;
        return true;
    }

    IMU(ros::NodeHandle node) :
        imu_pub_(node.advertise<sensor_msgs::Imu>("imu", 10)),
        imu_temp_(node.advertise<sensor_msgs::Temperature>("imu_temp",10)),
        reset_service_(node.advertiseService("imu_reset", &IMU::resetCallback, this)), 
        carivrate_service_(node.advertiseService("imu_caribrate", &IMU::caribrateCallback, this)),
        geta(0), gyro_unit(0.00836181640625), acc_unit(0.8), init_angle(0.0),
        port_name("/dev/ttyUSB0"), baudrate(115200), loop_rate(50), z_axis_dir_(-1)
    {
        ros::NodeHandle private_nh("~");
        private_nh.getParam("port_name", port_name);
        private_nh.param<double>("gyro_unit", gyro_unit, gyro_unit);
        private_nh.param<double>("acc_unit", acc_unit, acc_unit);
        private_nh.param<int>("baud_rate", baudrate, baudrate);
        private_nh.param<double>("init_angle", init_angle, init_angle);
        private_nh.param<int>("z_axis_dir", z_axis_dir_, z_axis_dir_);
    }

    bool init() {
        char command[2] = {0};
        char command2[101] = {0};
        //シリアルポートオープン 
        char* s = new char[port_name.length()+1];
        strcpy(s, port_name.c_str()); 
        if (!usb.Open((char*)s, baudrate)) {
            std::cerr << "open error" << std::endl;
            delete [] s;
            return false;
        }
        delete [] s;
        std::cout << "device open success" << std::endl;
        sleep(1);
        //コンフィギュレーション キャリブレーション
        // if(m_calibration == true){
        std::cout << "Calibration" << std::endl;
        sprintf(command, "a");
        usb.Send(command, strlen(command));
        std::cout << "send = " << command << std::endl;
        for(int i=0; i<8; ++i) {
            printf(". ");
            sleep(1);
        }
        // }
        //コンフィギュレーション リセット
        // if(m_reset == true){
        sprintf(command, "0");
        usb.Send(command, strlen(command));        //送信
        std::cout << "send = " << command << std::endl;
        sleep(1);
        std::cout << "Gyro 0 Reset" << std::endl;
        // }
        geta = 0;
        usb.Recv(command2, 100);        //空読み バッファクリアが効かない？
        usb.ClearRecvBuf();                //バッファクリア
        return true;
    }

    void run() {
        double old_angular_z_deg = getImuData().angular_deg[2];
        double angular_z_deg = 0;
        double temperature = 0;
        unsigned short abnormal_count = 0;

        while (ros::ok()) {
            sensor_msgs::Imu output_msg;
            sensor_msgs::Temperature output_temp;
            try{
                ImuData data = getImuData();
                output_msg.header.stamp = ros::Time::now();
                
                //ROS_INFO_STREAM("x_deg = " << data.angular_deg[0]);
                //ROS_INFO_STREAM("y_deg = " << data.angular_deg[1]);
                ROS_INFO_STREAM("z_deg = " << data.angular_deg[2]);
                
                output_temp.temperature = data.temperature;

                if(!isInRange(std::abs(old_angular_z_deg), 180.0, 165.0) && !isInRange(std::abs(data.angular_deg[2]), 180.0, 165.0) &&
                    std::abs(old_angular_z_deg - data.angular_deg[2]) > 40 && abnormal_count < 4){
                    
                    ROS_WARN_STREAM("Angle change too large: z = " << data.angular_deg[2]);
                    ROS_WARN_STREAM("old_z = " << old_angular_z_deg);
                    abnormal_count++;
                    continue;
                }else{
                    abnormal_count = 0;
                    angular_z_deg = data.angular_deg[2];
                }

            }catch(const CheckSumError &e){
                output_msg.header.stamp = ros::Time::now();
                ROS_ERROR_STREAM(e.what());
                continue;
            }
            
            output_msg.orientation = tf::createQuaternionMsgFromYaw(deg_to_rad(angular_z_deg));
                
            imu_pub_.publish(output_msg);
            imu_temp_.publish(output_temp);
            old_angular_z_deg = angular_z_deg;

            ros::spinOnce();
            //loop_rate.sleep();
        }
    }
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "imu");
    ros::NodeHandle node;
    IMU imu(node);
    if(!imu.init()) return 1;
    imu.run();
    return 0;
}
