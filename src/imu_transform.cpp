#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include <tf2/convert.h>

class ImuTransform : public rclcpp::Node
{
public:
	explicit ImuTransform() : Node("imu_transform")
	{
		_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu_transform/imu", 10);

		// QoS profile, PX4 specific
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		_fc_imu_sub = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
			[this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {

				auto fc_imu_acc = tf2::Vector3();
				fc_imu_acc.setX(msg->accelerometer_m_s2[0]);
				fc_imu_acc.setY(msg->accelerometer_m_s2[1]);
				fc_imu_acc.setZ(msg->accelerometer_m_s2[2]);

				auto fc_imu_gyro = tf2::Vector3();
				fc_imu_gyro.setX(msg->gyro_rad[0]);
				fc_imu_gyro.setY(msg->gyro_rad[1]);
				fc_imu_gyro.setZ(msg->gyro_rad[2]);

				// Perform rotation
				auto rotation_matrix = tf2::Matrix3x3(
											0, -1,  0,
											0,  0, -1,
											1,  0,  0);

				auto accel = tf2::Transform(rotation_matrix)(fc_imu_acc);
				auto gyro = tf2::Transform(rotation_matrix)(fc_imu_gyro);

				auto imu_msg = sensor_msgs::msg::Imu();
				imu_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
				imu_msg.linear_acceleration.x = accel[0];
				imu_msg.linear_acceleration.y = accel[1];
				imu_msg.linear_acceleration.z = accel[2];
				imu_msg.angular_velocity.x = gyro[0];
				imu_msg.angular_velocity.y = gyro[1];
				imu_msg.angular_velocity.z = gyro[2];
				imu_msg.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
				imu_msg.linear_acceleration_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
				imu_msg.angular_velocity_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
				this->_imu_pub->publish(imu_msg);
			});
	}

private:
	rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr _fc_imu_sub;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_pub;
};

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImuTransform>());
	rclcpp::shutdown();
	return 0;
}
