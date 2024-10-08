#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <cmath>

class Marker : public rclcpp::Node {
public:
    Marker() : Node("marker") {
        RCLCPP_INFO(this->get_logger(), "Marker Node initialized.");

        // Subscribe to laser scan and odometry data
        laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Marker::handleLaserScan, this, std::placeholders::_1)
        );

        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&Marker::handleOdometry, this, std::placeholders::_1)
        );

        // Publisher for visualization markers
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("cylinder_visualization_marker", 10);
    }

private:
    sensor_msgs::msg::LaserScan latest_laser_scan_;
    nav_msgs::msg::Odometry latest_odometry_;

    // Callback for handling laser scan data
    void handleLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_laser_scan_ = *msg;

        std::vector<geometry_msgs::msg::Point> detected_cylinders;
        detectCylinders(detected_cylinders);

        if (!detected_cylinders.empty()) {
            RCLCPP_INFO(this->get_logger(), "Detected cylinder at (%f, %f)", detected_cylinders[0].x, detected_cylinders[0].y);
            for (const auto& cylinder : detected_cylinders) {
                geometry_msgs::msg::Point world_position = convertToWorldCoordinates(cylinder);
                publishMarker(world_position);
            }
        }
    }

    // Function to detect cylinders from the laser scan
    void detectCylinders(std::vector<geometry_msgs::msg::Point>& detected_cylinders) {
        detected_cylinders.clear();
        const double distance_threshold = 0.9;
        const double target_diameter = 0.30;
        const double diameter_tolerance = 0.05;

        std::vector<std::vector<geometry_msgs::msg::Point>> segments;
        std::vector<geometry_msgs::msg::Point> current_segment;

        // Segment laser scan points based on proximity
        for (size_t i = 0; i < latest_laser_scan_.ranges.size(); ++i) {
            double range = latest_laser_scan_.ranges[i];
            if (!std::isinf(range) && !std::isnan(range) && range < latest_laser_scan_.range_max) {
                geometry_msgs::msg::Point point = convertToCartesian(i);
                if (current_segment.empty()) {
                    current_segment.push_back(point);
                } else {
                    const auto& last_point = current_segment.back();
                    double distance = std::hypot(point.x - last_point.x, point.y - last_point.y);
                    if (distance < distance_threshold) {
                        current_segment.push_back(point);
                    } else {
                        if (current_segment.size() > 1) {
                            segments.push_back(current_segment);
                        }
                        current_segment.clear();
                        current_segment.push_back(point);
                    }
                }
            }
        }
        if (current_segment.size() > 1) {
            segments.push_back(current_segment);
        }

        // Analyze segments to identify cylinders
        for (const auto& segment : segments) {
            if (segment.size() < 4) continue; // Skip small segments

            const auto& first_point = segment.front();
            const auto& last_point = segment.back();
            double segment_length = std::hypot(last_point.x - first_point.x, last_point.y - first_point.y);

            // Check if the segment resembles a cylinder
            if (std::abs(segment_length - target_diameter) <= diameter_tolerance) {
                geometry_msgs::msg::Point center;
                center.x = (first_point.x + last_point.x) / 2.0;
                center.y = (first_point.y + last_point.y) / 2.0;
                detected_cylinders.push_back(center);
            }
        }
    }

    // Convert polar coordinates to Cartesian
    geometry_msgs::msg::Point convertToCartesian(size_t index) {
        float angle = latest_laser_scan_.angle_min + latest_laser_scan_.angle_increment * index;
        float range = latest_laser_scan_.ranges[index];

        geometry_msgs::msg::Point cartesian_point;
        cartesian_point.x = static_cast<double>(range * cos(angle));
        cartesian_point.y = static_cast<double>(range * sin(angle));
        return cartesian_point;
    }

    // Callback for handling odometry data
    void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odometry_ = *msg;
    }

    // Transform local coordinates to world coordinates
    geometry_msgs::msg::Point convertToWorldCoordinates(const geometry_msgs::msg::Point& local_point) {
        geometry_msgs::msg::Point world_point;

        double robot_x = latest_odometry_.pose.pose.position.x;
        double robot_y = latest_odometry_.pose.pose.position.y;

        tf2::Quaternion orientation(
            latest_odometry_.pose.pose.orientation.x,
            latest_odometry_.pose.pose.orientation.y,
            latest_odometry_.pose.pose.orientation.z,
            latest_odometry_.pose.pose.orientation.w
        );
        tf2::Matrix3x3 rotation_matrix(orientation);
        double roll, pitch, yaw;
        rotation_matrix.getRPY(roll, pitch, yaw);

        // Transform local coordinates to world frame
        world_point.x = robot_x + local_point.x * cos(yaw) - local_point.y * sin(yaw);
        world_point.y = robot_y + local_point.x * sin(yaw) + local_point.y * cos(yaw);
        world_point.z = 0.0;

        return world_point;
    }


    // Publish marker for detected cylinder
    void publishMarker(const geometry_msgs::msg::Point& cylinder_position) {

        visualization_msgs::msg::Marker cyl_mk;

        //We need to set the frame
        // Set the frame ID and time stamp.
        cyl_mk.header.frame_id = "map";
        cyl_mk.header.stamp = this->get_clock()->now();
        //We set lifetime (it will dissapear in this many seconds)
        cyl_mk.lifetime = rclcpp::Duration(1000,0); //zero is forever

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        cyl_mk.ns = "cyl"; //This is namespace, markers can be in diofferent namespace
        cyl_mk.id = 0; 

        // The marker type
        cyl_mk.type = visualization_msgs::msg::Marker::CYLINDER;

        // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
        cyl_mk.action = visualization_msgs::msg::Marker::ADD;

        // have to transfrom to world frame
        cyl_mk.pose.position.x = cylinder_position.x + latest_odometry_.pose.pose.position.x;
        cyl_mk.pose.position.y = cylinder_position.y + latest_odometry_.pose.pose.position.y;
        cyl_mk.pose.position.z = cylinder_position.z;


        //Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
        cyl_mk.pose.orientation.x = 0.0;
        cyl_mk.pose.orientation.y = 0.0;
        cyl_mk.pose.orientation.z = 0.0;
        cyl_mk.pose.orientation.w = 1.0;


        // Set the scale of the marker -- 1m side
        cyl_mk.scale.x = 0.2;
        cyl_mk.scale.y = 0.2;
        cyl_mk.scale.z = 0.5;

        //Let's send a marker with color (green for reachable, red for now)
        std_msgs::msg::ColorRGBA cyl_color;
        cyl_color.a=0.5;//a is alpha - transparency 0.5 is 50%;
        cyl_color.r=0;
        cyl_color.g=0;
        cyl_color.b=250.0/255.0;

        cyl_mk.color = cyl_color;
        
        marker_publisher_->publish(cyl_mk);

    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Marker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}