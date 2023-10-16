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
		// IMU publication
		_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/jake/imu", 10);

		// FC IMU subscription
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		_fc_imu_sub = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
		[this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
			// std::cout << "RECEIVED SENSOR COMBINED DATA"   << std::endl;

			// Publish IMU topic
			auto imu_msg = sensor_msgs::msg::Imu();

			auto imu_acc = tf2::Vector3();
			imu_acc.setX(msg->accelerometer_m_s2[0]);
			imu_acc.setY(msg->accelerometer_m_s2[1]);
			imu_acc.setZ(msg->accelerometer_m_s2[2]);

			// Perform rotation
			auto rotation_matrix = tf2::Matrix3x3(
										0, -1, 0,
										0, 0, -1,
										1, 0, 0);

			tf2::Vector3 rotated_vector = tf2::Transform(rotation_matrix)(imu_acc);

			// TODO: how the fuck do I do this?
			// imu_msg.linear_acceleration = tf2::toMsg(rotated_vector);
			// tf2::toMsg(rotated_vector, imu_msg.linear_acceleration);

			imu_msg.linear_acceleration.x = rotated_vector[0];
			imu_msg.linear_acceleration.y = rotated_vector[1];
			imu_msg.linear_acceleration.z = rotated_vector[2];

			this->_imu_pub->publish(imu_msg);
		});
	}

private:
	// Subscriptions
	rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr _fc_imu_sub;
	// Publications
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_pub;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting imu_transform node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImuTransform>());

	rclcpp::shutdown();
	return 0;
}
