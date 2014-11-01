#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

void imu_cb(const sensor_msgs::Imu &msg){
    static tf::TransformBroadcaster tf_br;
    tf::Transform imu_tran;
    imu_tran.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    imu_tran.setRotation(tf::Quaternion(msg.orientation.x,
                                        msg.orientation.y,
                                        msg.orientation.z,
                                        msg.orientation.w));

    tf_br.sendTransform(tf::StampedTransform(imu_tran, ros::Time::now(), "base_link", "imu_link"));

}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "adis_tf_broadcaster");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("imu", 10, &imu_cb);

    ros::spin();
    return 0;

}

