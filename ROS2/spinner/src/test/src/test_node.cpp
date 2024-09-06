#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <messages/msg/axis_state.hpp>

rclcpp::Node::SharedPtr nodeHandle;

int main(int argc, char **argv){
        rclcpp::init(argc, argv);
        nodeHandle = rclcpp::Node::make_shared("test");
        auto joystickAxisPublisher = nodeHandle->create_publisher<messages::msg::AxisState>("joystick_axis", 1);
        int count = -200;

        rclcpp::Rate rate(20);
        while(rclcpp::ok()){
                rclcpp::spin_some(nodeHandle);
                rate.sleep();
                float countFloat = count / 100.0;
                RCLCPP_INFO(nodeHandle->get_logger(), "Publishing %f", countFloat);
                messages::msg::AxisState axisState;
                axisState.axis = 1;
                axisState.state = countFloat;
                joystickAxisPublisher->publish(axisState);
                count += 1;
        }
}