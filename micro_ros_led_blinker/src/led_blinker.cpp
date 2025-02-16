#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>

class LEDBlinker : public rclcpp::Node {
public:
    LEDBlinker() : Node("led_blinker"), led_state_(false) {
        // Publisher for LED status (Bool)
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("led_status", 10);

        // Publisher for LED Marker (for RViz)
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("led_marker", 10);

        // Timer to toggle LED every second
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&LEDBlinker::toggle_led, this));
    }

private:
    void toggle_led() {
        // Publish Bool message for LED status
        auto led_message = std_msgs::msg::Bool();
        led_message.data = !led_state_;
        publisher_->publish(led_message);
        led_state_ = led_message.data;

        // Publish Marker message for RViz visualization
        auto marker_msg = visualization_msgs::msg::Marker();
        marker_msg.header.frame_id = "base_link";
        marker_msg.header.stamp = this->now();
        marker_msg.ns = "led_visualization";
        marker_msg.id = 0;
        marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_msg.pose.position.x = 0.0;
        marker_msg.pose.position.y = 0.0;
        marker_msg.pose.position.z = 0.0;
        marker_msg.scale.x = 0.2;
        marker_msg.scale.y = 0.2;
        marker_msg.scale.z = 0.2;

        // Change color based on LED state
        if (led_state_) {
            marker_msg.color.r = 0.0;
            marker_msg.color.g = 1.0;  // Green (ON)
            marker_msg.color.b = 0.0;
        } else {
            marker_msg.color.r = 1.0;  // Red (OFF)
            marker_msg.color.g = 0.0;
            marker_msg.color.b = 0.0;
        }
        marker_msg.color.a = 1.0;  // Full opacity

        marker_publisher_->publish(marker_msg);

        // Print LED status in terminal
        RCLCPP_INFO(this->get_logger(), "LED State: %s", led_state_ ? "ON" : "OFF");
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool led_state_;
};

// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LEDBlinker>());
    rclcpp::shutdown();
    return 0;
}
