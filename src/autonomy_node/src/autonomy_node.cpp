#include <rclcpp/rclcpp.hpp>

class AutonomyNode : public rclcpp::Node {
    public:
        AutonomyNode() : Node("autnomy_node") {
            detection_publsiher_ = this->create_publisher<>("/drone/detected", 10)

        }



    private:
        void detect() {
            return 0
        }

        rclcpp::Publisher<> detecion_publsiher_
}