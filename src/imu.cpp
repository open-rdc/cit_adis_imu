#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <tf/transform_broadcaster.h>

#include <boost/bind.hpp>
#include <boost/lambda/lambda.hpp>

#include <sys/time.h>

#include "imu/bool_msg.h"
#include "Comm.h"

#include <sensor_msgs/Imu.h>

float deg_to_rad(float deg) {
    return deg / 180.0f * M_PI;
}

template<class T>
bool isInRange(T value, T max, T min){
    return (value >= min) && (value <= max);
}

struct ImuData {
    double angular_deg[3];
    double linear_acc[3];
    double angular_vel[3];
    double temp;

    ImuData() : 
        temp(0)
    {
        for(int i=0; i < 2; i++){
            angular_deg[i] = 0;
            linear_acc[i] = 0;
            angular_vel[i] = 0;
        }
    }
};


// 角度ごとにURGのデータを送信するためのサービス
// 要求があったときに，URGのデータを別のトピックに送信するだけ
class IMU {
private:
    ros::Publisher imu_pub_;
    ros::Publisher imu_pub_inc;
    ros::ServiceServer reset_service_;
    ros::ServiceServer carivrate_service_;
    float geta;
    CComm usb;
    double gyro_unit;
    double delta_ang_unit;
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
        const float temp_unit = 0.00565;
        
        sprintf(command, "o");
        usb.Send(command, strlen(command));
        //usleep(30000);
        ros::Duration(0.03).sleep();
        
        usb.Recv(command2, 50);
        ROS_INFO_STREAM("recv = " << command2);

        memmove(temp,command2,4);
        data.angular_deg[0] = ((short)strtol(temp, NULL, 16)) * delta_ang_unit;
        memmove(temp,command2+4,4);
        data.angular_deg[1] = ((short)strtol(temp, NULL, 16)) * delta_ang_unit;
        memmove(temp,command2+8,4);
        data.angular_deg[2] = ((short)strtol(temp, NULL, 16)) * delta_ang_unit * z_axis_dir_;

        memmove(temp,command2+12,4);
        data.linear_acc[0] = ((short)strtol(temp, NULL, 16)) * acc_unit;
        memmove(temp,command2+16,4);
        data.linear_acc[1] = ((short)strtol(temp, NULL, 16)) * acc_unit;
        memmove(temp,command2+20,4);
        data.linear_acc[2] = init_angle + ((short)strtol(temp, NULL, 16)) * acc_unit ;

        memmove(temp,command2+24,4);
        data.angular_vel[0] = ((short)strtol(temp, NULL, 16)) * gyro_unit;
        memmove(temp,command2+28,4);
        data.angular_vel[1] = ((short)strtol(temp, NULL, 16)) * gyro_unit;
        memmove(temp,command2+32,4);
        data.angular_vel[2] = ((short)strtol(temp, NULL, 16)) * gyro_unit * z_axis_dir_;
        
        memmove(temp,command2+36,4);
        data.temp = ((short)strtol(temp, NULL, 16)) * temp_unit + 25.0;
        
        //while(data.angular_deg[2] < -180) data.angular_deg[2] += 180;
        //while(data.angular_deg[2] > 180) data.angular_deg[2] -= 180;

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
        imu_pub_inc(node.advertise<sensor_msgs::Imu>("imu_inc",10)),
        reset_service_(node.advertiseService("imu_reset", &IMU::resetCallback, this)), 
        carivrate_service_(node.advertiseService("imu_caribrate", &IMU::caribrateCallback, this)),
        geta(0), gyro_unit(0.020001220703125),delta_ang_unit(0.00836181640625), acc_unit(0.8), init_angle(0.0),
        port_name("/dev/ttyUSB0"), baudrate(115200), loop_rate(50), z_axis_dir_(-1)
    {
        ros::NodeHandle private_nh("~");
        private_nh.getParam("port_name", port_name);
        private_nh.param<double>("delta_ang_unit", delta_ang_unit, delta_ang_unit);
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
        double angular_integral = 0;
        unsigned short abnormal_count = 0;
        double time_sec,old_time_sec;
        time_sec = ros::Time::now().toSec();
        old_time_sec = ros::Time::now().toSec();

        while (ros::ok()) {
            sensor_msgs::Imu output_msg;
            sensor_msgs::Imu output_msg_inc;
            old_time_sec = ros::Time::now().toSec();
            ImuData data = getImuData();
            output_msg.header.stamp = ros::Time::now();
            output_msg_inc.header.stamp = ros::Time::now();
            time_sec = ros::Time::now().toSec();
         
            angular_integral += getImuData().angular_vel[2] * (time_sec - old_time_sec);

            //ROS_INFO_STREAM("x_deg = " << data.angular_deg[0]);
            //ROS_INFO_STREAM("y_deg = " << data.angular_deg[1]);
            ROS_INFO_STREAM("z_deg = " << data.angular_deg[2]);
            ROS_INFO_STREAM("z_deg_int = " << angular_integral);
            
            if(!isInRange(std::abs(old_angular_z_deg), 180.0, 165.0) && !isInRange(std::abs(data.angular_deg[2]), 180.0, 165.0) &&
                std::abs(old_angular_z_deg - data.angular_deg[2]) > 40 && abnormal_count < 4){
                
                ROS_WARN_STREAM("Angle change too large: z = " << data.angular_deg[2]);
                ROS_WARN_STREAM("old_z = " << old_angular_z_deg);
                data.angular_deg[2] = old_angular_z_deg;
                abnormal_count++;
            }else{
                abnormal_count = 0;
            }

            output_msg_inc.orientation = tf::createQuaternionMsgFromYaw(deg_to_rad(angular_integral));
            output_msg.orientation = tf::createQuaternionMsgFromYaw(deg_to_rad(data.angular_deg[2]));
            output_msg.linear_acceleration.x = data.linear_acc[0];
            output_msg.linear_acceleration.y = data.linear_acc[1];
            output_msg.linear_acceleration.z = data.linear_acc[2];
            output_msg.angular_velocity.x = data.angular_vel[0];
            output_msg.angular_velocity.y = data.angular_vel[1];
            output_msg.angular_velocity.z = data.angular_vel[2];
    
            //ROS_INFO_STREAM("temp = " << data.temp);
            
            imu_pub_inc.publish(output_msg_inc);
            imu_pub_.publish(output_msg);
            old_angular_z_deg = data.angular_deg[2];

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
