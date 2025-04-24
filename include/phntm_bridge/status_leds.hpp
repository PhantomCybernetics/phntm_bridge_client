#pragma once

#include "config.hpp"

#include <gpiod.hpp>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace phntm {

    class StatusLED {
        public:
            StatusLED(gpiod::line line);
            StatusLED(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher);

            void on();
            void off();
            void once(int on_ms = 2);
            void interval(int on_ms, int interval_ms);
            void fastPulse();

            void update();
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
            State state;
            float on_ms;
            float interval_ms;

            bool current_state;
            std::chrono::steady_clock::time_point last_on_time;
            std::chrono::steady_clock::time_point last_off_time;

            gpiod::line line;

            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher;
            std_msgs::msg::Bool msg_on;
            std_msgs::msg::Bool msg_off;

            void setState(bool state);
    };

    class StatusLEDs {

        public:
            static void init(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<BridgeConfig> config);
            static void clear();
            static StatusLEDs * getInstance() { return instance; }
            std::shared_ptr<StatusLED> conn;
            std::shared_ptr<StatusLED> data;
            

        private:
            StatusLEDs() {};
            static StatusLEDs* instance;
            std::vector<std::shared_ptr<StatusLED>> leds;

            bool loop_running;
            std::thread loop_thread;
            void loop();
    };

    // convenience
    class ConnLED {
        public:
            static void on(); // connected to Cloud Bridge
            static void fastPulse(); // trying to connect
            static void off(); // clear 
            
    };
    class DataLED {
        public:
            static void once(); // message sent via webrtc
            static void off(); // clear 
    };

}