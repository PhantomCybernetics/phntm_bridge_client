#pragma once

#include "config.hpp"

#include <gpiod.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class StatusLED {
    public:
        StatusLED(gpiod::line line);
        StatusLED(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher);

        void on();
        void off();
        void once(float on_sec = 0.002f);
        void interval(float on_sec, float interval_sec);
        void fastPulse();
    
        void clear();

        enum Mode {
            UNUSED,
            ROS,
            GPIO
        };
        enum Position {
            CONN,
            DATA
        };
        enum State {
            ON,
            OFF,
            BLINK_ONCE,
            BLINKING
        };
    
    private:
        Mode mode;

        gpiod::line line;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher;
        std_msgs::msg::Bool msg_on;
        std_msgs::msg::Bool msg_off;

        State state;
        float on_sec;
        float interval_sec;
        float last_on_time;
        float last_off_time;

        void setState(bool state);
};

class StatusLEDs {

    public:
        static void Init(std::shared_ptr<BridgeConfig> config, std::shared_ptr<rclcpp::Node> node);
        static void Clear();
        static StatusLEDs * GetInstance() { return instance; }
        std::shared_ptr<StatusLED> conn;
        std::shared_ptr<StatusLED> data;

    private:
        StatusLEDs() {};
        static StatusLEDs* instance;
        std::shared_ptr<rclcpp::Node> publisher_node;
};

// convenience
class ConnLED {
    public:
        static void On(); // connected to Cloud Bridge
        static void FastPulse(); // trying to connect
        static void Off(); // clear 
        
};
class DataLED {
    public:
        static void Once(); // message sent via webrtc
        static void Off(); // clear 
};