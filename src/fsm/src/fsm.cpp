#include <chrono>
#include <thread>
#include <map>
#include <iostream>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono;
using namespace std::this_thread;

void make_transitions();
void change_state(std::string);
void print_options();

std::map<std::pair<std::string, std::string>, std::string> mp;
std::string currentState;
int option;

class MySubscriber : public rclcpp::Node {
public:
    MySubscriber() : Node("my_subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "my_topic", 10, std::bind(&MySubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
        std::string condition;
        condition.append("condition" + msg->data);
        change_state(condition);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

void make_transitions() {
    std::cout << "hey" << std::endl;
    mp.insert(std::make_pair(std::make_pair("Idle", "condition1"), "Walk"));
    mp.insert(std::make_pair(std::make_pair("Idle", "condition2"), "Shoot(W)"));
    mp.insert(std::make_pair(std::make_pair("Idle", "condition3"), "Shoot(H)"));
    mp.insert(std::make_pair(std::make_pair("Idle", "condition4"), "Right"));
    mp.insert(std::make_pair(std::make_pair("Idle", "condition5"), "Left"));

    mp.insert(std::make_pair(std::make_pair("Walk", "condition1"), "Run"));
    mp.insert(std::make_pair(std::make_pair("Walk", "condition2"), "Right"));
    mp.insert(std::make_pair(std::make_pair("Walk", "condition3"), "Left"));
    mp.insert(std::make_pair(std::make_pair("Walk", "condition4"), "Shoot(W)"));
    mp.insert(std::make_pair(std::make_pair("Walk", "condition5"), "Idle"));

    mp.insert(std::make_pair(std::make_pair("Run", "condition1"), "Walk"));
    mp.insert(std::make_pair(std::make_pair("Run", "condition2"), "Right"));
    mp.insert(std::make_pair(std::make_pair("Run", "condition3"), "Left"));

    mp.insert(std::make_pair(std::make_pair("Shoot(W)", "condition1"), "Idle"));
    mp.insert(std::make_pair(std::make_pair("Shoot(H)", "condition2"), "Idle"));

    mp.insert(std::make_pair(std::make_pair("Right", "condition1"), "Idle"));
    mp.insert(std::make_pair(std::make_pair("Right", "condition2"), "Walk"));
    mp.insert(std::make_pair(std::make_pair("Left", "condition3"), "Idle"));
    mp.insert(std::make_pair(std::make_pair("Left", "condition4"), "Walk"));

    currentState = "Idle";
}

void print_options() {
    int i = 1;
    std::cout << "Possible Transitions:" << std::endl;
    for(auto itr = mp.begin(); itr != mp.end(); itr++) {
        if(itr->first.first == currentState) {
            std::cout << i << ". " << itr->first.first << " --[" << itr->first.second << "]-> " << itr->second << std::endl;;
            i++;
        }
    }
    std::cout << "\ncurrent state: " << currentState << std::endl;
}

void change_state(std::string condition) {
    if(mp.count({currentState, condition}) != 0) {
        std::cout << "changing state ...\n" << std::endl;
        sleep_for(milliseconds(1000));
        currentState = mp[{currentState, condition}];
        print_options();
    }
}

int main(int argc, char *argv[]) {
    make_transitions();
    print_options();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MySubscriber>());
    rclcpp::shutdown();
    return 0;
}
