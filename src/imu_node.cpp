#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <tf/transform_broadcaster.h>

#include <boost/bind.hpp>
#include <boost/lambda/lambda.hpp>

#include <sys/time.h>

#include "Comm.h"

#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>

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

float rad_to_deg(float rad){
    return rad * 180.0f / M_PI;
}

template<class T>
bool isInRange(T value, T max, T min){
    return (value >= min) && (value <= max);
}

struct ImuData {
    double angular_deg[3];
		bool flag;
    
    ImuData(){
        for(int i=0; i < 2; i++){
            angular_deg[i] = 0;
        }
    }
};

// 角度ごとにURGのデータを送信するためのサービス
class IMU {
private:
    ros::Publisher imu_pub_;
    ros::ServiceServer reset_service_;
    ros::ServiceServer carivrate_service_;
    CComm* usb_;
    double gyro_unit_;
    double acc_unit_;
    std::string port_name_, imu_frame_;
    int   baudrate_;
    ros::Rate loop_rate_;
    int  z_axis_dir_;
    int  y_axis_dir_;

    ImuData getImuData(){
        ImuData data;
        char command[2] = {0};
        char command2[50] = {0};
        char temp[6];
        unsigned long int sum = 0;
        
        const float temp_unit = 0.00565;
        
        sprintf(command, "o");
        usb_->Send(command, strlen(command));
        ros::Duration(0.03).sleep();
        
        usb_->Recv(command2, 50);
        ROS_INFO_STREAM("recv = " << command2);

        if (command2[0] == '\0') {
        	data.flag = false;
        	return data;
        }
        data.flag = true;

        memmove(temp,command2,4);
        sum = sum ^ ((short)strtol(temp, NULL, 16));
        data.angular_deg[0] = ((short)strtol(temp, NULL, 16)) * gyro_unit_;
        memmove(temp,command2+4,4);
        sum = sum ^ ((short)strtol(temp, NULL, 16));
        data.angular_deg[1] = std::asin(((short)strtol(temp, NULL, 16)) * acc_unit_) * y_axis_dir_;
        memmove(temp,command2+8,4);
        sum = sum ^ ((short)strtol(temp, NULL, 16));
        data.angular_deg[2] = ((short)strtol(temp, NULL, 16)) * gyro_unit_ * z_axis_dir_;

        memmove(temp,command2+12,4);
        
        if(sum != ((short)strtol(temp, NULL, 16))){
            ROS_ERROR_STREAM("Recv Checksum: " << ((short)strtol(temp, NULL, 16)));
            ROS_ERROR_STREAM("Calculate Checksum: " << sum);
            throw CheckSumError(command2);
        }
        //while(data.angular_deg[2] < -180) data.angular_deg[2] += 180;
        //while(data.angular_deg[2] > 180) data.angular_deg[2] -= 180;

        return data;
    }

public:
    bool resetCallback(std_srvs::Trigger::Request  &req, 
            std_srvs::Trigger::Response &res) 
    {
        char command[2] = {0};
        sprintf(command, "0");
        usb_->Send(command, strlen(command));
        sleep(1);
        std::cout << "Gyro 0 Reset" << std::endl;
        res.success = true;
        res.message = "Successful reset angle, gyro angle be zero right now.";

        return true;
    }
    
    bool caribrateCallback(std_srvs::Trigger::Request  &req, 
            std_srvs::Trigger::Response &res) 
    {
        char command[2] = {0};
        std::cout << "Calibration" << std::endl;
        sprintf(command, "a");
        usb_->Send(command, strlen(command));
        std::cout << "Calibrate start ";
        for(int i=0; i<8; ++i) {
            std::cout << ". ";
            sleep(1);
        }
        
        res.success = true;
        res.message = "ADIS driver is done with calibration.";
        ROS_INFO_STREAM(res.message);

        return true;
    }

    IMU(ros::NodeHandle node) :
        imu_pub_(node.advertise<sensor_msgs::Imu>("imu", 10)),
        reset_service_(node.advertiseService("imu_reset", &IMU::resetCallback, this)), 
        carivrate_service_(node.advertiseService("imu_caribrate", &IMU::caribrateCallback, this)),
        gyro_unit_(0.00836181640625), acc_unit_(0.0008192),
        port_name_("/dev/ttyUSB0"), baudrate_(115200), loop_rate_(50), z_axis_dir_(-1), y_axis_dir_(-1)
    {
        ros::NodeHandle private_nh("~");
        private_nh.getParam("port_name", port_name_);
        private_nh.param<std::string>("imu_frame", imu_frame_, imu_frame_);
        private_nh.param<double>("gyro_unit", gyro_unit_, gyro_unit_);
        private_nh.param<double>("acc_unit", acc_unit_, acc_unit_);
        private_nh.param<int>("baud_rate", baudrate_, baudrate_);
        private_nh.param<int>("z_axis_dir", z_axis_dir_, z_axis_dir_);
        private_nh.param<int>("y_axis_dir", y_axis_dir_, y_axis_dir_);

        usb_ = new CComm(port_name_, baudrate_);
    }

    bool init() {
        char command[2] = {0};
        char command2[101] = {0};
        //シリアルポートオープン 
        if (!usb_->Open()) {
            std::cerr << "open error" << std::endl;
            return false;
        }
        std::cout << "device open success" << std::endl;
        sleep(1);
        //コンフィギュレーション キャリブレーション
        // if(m_calibration == true){
        std::cout << "Calibration" << std::endl;
        sprintf(command, "a");
        usb_->Send(command, strlen(command));
        std::cout << "send = " << command << std::endl;
        for(int i=0; i<8; ++i) {
            printf(". ");
            sleep(1);
        }
        // }
        //コンフィギュレーション リセット
        // if(m_reset == true){
        sprintf(command, "0");
        usb_->Send(command, strlen(command));        //送信
        std::cout << "send = " << command << std::endl;
        sleep(1);
        std::cout << "Gyro 0 Reset" << std::endl;
        // }
        usb_->Recv(command2, 100);        //空読み バッファクリアが効かない？
        usb_->ClearRecvBuf();                //バッファクリア
        return true;
    }

    void run() {
        double old_angular_z_deg = getImuData().angular_deg[2];
        double angular_z_deg = 0;
        unsigned short abnormal_count = 0;

        while (ros::ok()) {
            tf::Quaternion q;
            sensor_msgs::Imu output_msg;
            try{
                ImuData data = getImuData();
                if (data.flag == false){
                    usb_->Close();
                    if(!usb_->Open()) {
                        std::cerr << "reconnecting" << std::endl;
                    }
                }else{
                    output_msg.header.frame_id = imu_frame_;
                    output_msg.header.stamp = ros::Time::now();
                      
                    // ROS_INFO_STREAM("x_deg = " << data.angular_deg[0]);
                    ROS_INFO_STREAM("y_deg = " << rad_to_deg(data.angular_deg[1]));
                    ROS_INFO_STREAM("z_deg = " << data.angular_deg[2]);
                      
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
                    
                    q = tf::createQuaternionFromRPY(data.angular_deg[1], 0.0, deg_to_rad(angular_z_deg));
                    tf::quaternionTFToMsg(q, output_msg.orientation);
                    imu_pub_.publish(output_msg);
                    old_angular_z_deg = angular_z_deg;

                }
            }catch(const CheckSumError &e){
                output_msg.header.stamp = ros::Time::now();
                ROS_ERROR_STREAM(e.what());
                continue;
            }
            
            ros::spinOnce();
            //loop_rate.sleep();
        }
    }
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle node;
    IMU imu(node);
    if(!imu.init()) return 1;
    imu.run();
    
    return 0;
}
