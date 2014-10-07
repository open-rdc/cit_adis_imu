#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <tf/transform_broadcaster.h>

#include <boost/bind.hpp>
#include <boost/lambda/lambda.hpp>

#include <sys/time.h>

#include "imu/bool_msg.h"
#include "Comm.h"

#include <sensor_msgs/Imu.h>

inline float deg_to_rad(float deg) {
	// 180度リミッター（Z軸のみ）
	return deg/180.0f*M_PI;
}
// 角度ごとにURGのデータを送信するためのサービス
// 要求があったときに，URGのデータを別のトピックに送信するだけ
class IMU {
	private :
		ros::Publisher imu_pub_;
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

	public:
		bool resetCallback(imu::bool_msg::Request  &req, 
				imu::bool_msg::Response &res) 
		{ 
			char command[2] = {0};
			sprintf(command, "0");
			usb.Send(command, strlen(command));			//送信
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
			imu_pub_(node.advertise<sensor_msgs::Imu>("imu", 1000)),
			reset_service_(node.advertiseService("imu_reset", &IMU::resetCallback, this)), 
			carivrate_service_(node.advertiseService("imu_caribrate", &IMU::caribrateCallback, this)),
			geta(0), gyro_unit(0.00836181640625), acc_unit(0.8), init_angle(0.0),
			port_name("/dev/ttyUSB0"), baudrate(115200), loop_rate(1000), z_axis_dir_(-1)
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
			usb.Send(command, strlen(command));			//送信
			std::cout << "send = " << command << std::endl;
			sleep(1);
			std::cout << "Gyro 0 Reset" << std::endl;
			// }
			geta = 0;
			usb.Recv(command2, 100);	//空読み バッファクリアが効かない？
			usb.ClearRecvBuf();		//バッファクリア
			return true;
		}
		void run() {
			while (ros::ok()) {
				char command[2] = {0};
				char command2[50] = {0};
				char temp[6];
				const float temp_unit = 0.00565;
				float tempdata;
				//コマンド送信
				//oコマンドでジャイロ3軸、加速度3軸、温度が出力される
				sprintf(command, "o");
				usb.Send(command, strlen(command));			//送信
				usleep(30000);
				//結果受信
				usb.Recv(command2, 50);			//受信
				std::cout << "recv = " << command2 << std::endl;
				geometry_msgs::Twist output_data;
				//ジャイロ読み込み
				memmove(temp,command2,4);
				output_data.angular.x = ((short)strtol(temp, NULL, 16)) * gyro_unit;
				memmove(temp,command2+4,4);
				output_data.angular.y = ((short)strtol(temp, NULL, 16)) * gyro_unit;
				memmove(temp,command2+8,4);
				output_data.angular.z = ((short)strtol(temp, NULL, 16)) * gyro_unit * z_axis_dir_;
				//加速度読み込み
				memmove(temp,command2+12,4);
				output_data.linear.x = ((short)strtol(temp, NULL, 16)) * acc_unit;
				memmove(temp,command2+16,4);
				output_data.linear.y = ((short)strtol(temp, NULL, 16)) * acc_unit;
				memmove(temp,command2+20,4);
				output_data.linear.z = init_angle + ((short)strtol(temp, NULL, 16)) * acc_unit ;
				// 180度リミッター（Z軸のみ）
				while (output_data.angular.z < -180) 
					output_data.angular.z += 180;
				while (output_data.angular.z > 180) 
					output_data.angular.z -= 180;
				//温度読み込み
				memmove(temp,command2+24,4);
				tempdata = ((short)strtol(temp, NULL, 16)) * temp_unit + 25.0;
				// 出力データの表示
				std::cout << "ang_x = " << output_data.angular.x
					<< "ang_y = " << output_data.angular.y
					<< "ang_z = " << output_data.angular.z
					<< "ang_z_orig = " << output_data.angular.z  << std::endl;
				std::cout << "acc_x = " << output_data.linear.x
					<< "acc_y = " << output_data.linear.y
					<< "acc_z = " << output_data.linear.z << std::endl;
				std::cout << "temp  = " << tempdata << std::endl;
				output_data.angular.x = deg_to_rad(output_data.angular.x);
				output_data.angular.y = deg_to_rad(output_data.angular.y);
				output_data.angular.z = deg_to_rad(output_data.angular.z);
				
                sensor_msgs::Imu output_data_imu;
                output_data_imu.header.stamp = ros::Time::now();
                tf::quaternionTFToMsg(tf::Quaternion(
                                        output_data.angular.x, 
                                        output_data.angular.y, 
                                        output_data.angular.z
                                     ), 
                                     output_data_imu.orientation
                );

                output_data_imu.angular_velocity.x = output_data.angular.x;
                output_data_imu.angular_velocity.y = output_data.angular.y;
                output_data_imu.angular_velocity.z = output_data.angular.z;
                
                output_data_imu.linear_acceleration.x = output_data.linear.x;
                output_data_imu.linear_acceleration.y = output_data.linear.y;
                output_data_imu.linear_acceleration.z = output_data.linear.z;

				imu_pub_.publish(output_data_imu);
				
                ros::spinOnce();
				loop_rate.sleep();
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
