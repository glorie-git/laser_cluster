#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>

class LaserScanToMarker : public rclcpp::Node {
public:
    LaserScanToMarker() : Node("laser_scan_to_marker") {
        // Subscribe to laser scan data
        laser_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanToMarker::laserScanCallback, this, std::placeholders::_1));

        // Create publisher for visualization markers
        marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lidar_circles", 10);

        pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Initialize object size as zero
        object_size_ = 0.0;

        vel_dir_pub = this->create_publisher<std_msgs::msg::String>("/vel_dir", 1);

        object_detected_ = this->create_publisher<std_msgs::msg::Bool>("/object_detected", 1);

        flag.data = false;
    }

private:
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        bool object_detected = false;
        flag.data = false;
        printf("size of ranges: %d\n", scan_msg->ranges.size());

        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            if (std::isinf(scan_msg->ranges[i] )) {
                // do nothing
            } else {
                // Convert polar coordinates to Cartesian for each point
                double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                double angle_deg = angle*180/(22/7);
                if (angle_deg < 47.0 && angle_deg > -47.0) {
                    // geometry_msgs::msg::Point point;
                    // point.x = scan_msg->ranges[i] * std::cos(angle);
                    // point.y = scan_msg->ranges[i] * std::sin(angle);
                    // point.z = 0.0; // Assuming a 2D laser scan
                    printf("range: %f angle: %f index: %d\n", scan_msg->ranges[i], angle_deg, i);

                    if (scan_msg->ranges[i] < threshold_distance ){
                        // printf("Object detected in range: %f\n", scan_msg->ranges[i] );
                        // object_detected = true;
                        flag.data = true;
                        if (angle_deg < 0.0) {
                            vel_dir_.data = "left";
                            vel_dir_pub->publish(vel_dir_);

                        } else if (angle_deg > 0.0){
                            vel_dir_.data = "right";
                            vel_dir_pub->publish(vel_dir_);
                        }
                        // object_detected_->publish(flag);
                        break;
                    }
                }
                // marker.points.push_back(point);
            }
        }

        // if (object_detected) {
        object_detected_->publish(flag);
        // } else {
        //     object_detected_->publish(flag);
        // }
        // if (object_detected != flag.data){
        //     flag.data = object_detected;
        //     object_detected_->publish(flag);
        // }
        printf("***********************************\n");

    }

    // Function to set flag to stop robot
    void stopRobot() {
        RCLCPP_INFO(this->get_logger(), "Stopping robot!");
        std_msgs::msg::Bool flag;
        flag.data = true;
        object_detected_->publish(flag);
    }

    // Function to simulate moving the robot away from the object based on the detected object's angle
    void moveAwayFromObject(double angle_to_object) {
        // Simulated behavior: Move the robot in the opposite direction of the detected object
        // Calculate the opposite direction angle
        double opposite_angle = angle_to_object + M_PI; // Opposite direction angle (180 degrees from the object)
        int direction = 1;

        RCLCPP_INFO(this->get_logger(), "Opposite angle: %f", opposite_angle);

        // Convert the angle to linear x and y velocities
        // double linear_vel = 0.1; // Set your desired linear velocity
        double angular_vel = 0.2; // Set your desired angular velocity

        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0; // linear_vel * cos(opposite_angle);
        cmd_vel.linear.y = 0.0; // linear_vel * sin(opposite_angle);
        cmd_vel.angular.z = angular_vel * direction;

        // Publish the updated cmd_vel to move the robot
        // (Assuming you have a publisher for robot velocity commands)
        pub_cmd_vel->publish(cmd_vel);
    }

    const double threshold_distance = 0.50; // Modify this as needed
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr object_detected_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vel_dir_pub;

    double object_size_; // Holds the detected object size (diameter)
    const double safety_factor = 1.0; // Adjust safety factor for desired distance from object
    std_msgs::msg::Bool flag;
    std_msgs::msg::String vel_dir_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScanToMarker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
