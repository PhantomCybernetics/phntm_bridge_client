#include "phntm_bridge/status_leds.hpp"
#include "phntm_bridge/const.hpp"
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <stdexcept>
#include <string>

StatusLED::StatusLED (gpiod::line line) { 
    this->line = line;
    this->mode = Mode::GPIO;

    gpiod::line_request line_request;
    line_request.request_type = gpiod::line_request::DIRECTION_OUTPUT;
    this->line.request(line_request);
    this->state = State::OFF;
    this->setState(false);
}

StatusLED::StatusLED(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher) {
    this->publisher = publisher;
    this->mode = Mode::ROS;
    this->state = State::OFF;

    this->msg_on.data = true;
    this->msg_off.data = false;

    this->setState(false);    
}

void StatusLED::setState(bool state) {
    if (this->mode == Mode::GPIO) {
        this->line.set_value(state ? 1 : 0);
    } else if (this->mode == Mode::ROS) {
        publisher->publish(state ? this->msg_on : this->msg_off);
    }
}

void StatusLED::on() {
    this->state = State::ON;
}

void StatusLED::off() {
    this->state = State::OFF;
}

void StatusLED::once(float on_sec) {
    this->state = State::BLINK_ONCE;
    this->on_sec = on_sec;
}

void StatusLED::interval(float on_sec, float interval_sec) {
    this->state = State::BLINKING;
    this->on_sec = on_sec;
    this->interval_sec = interval_sec;
}

void StatusLED::fastPulse() {
    this->interval(0.01f, 0.5f);
}

void StatusLED::clear() {
    this->setState(false);
    if (this->mode == Mode::GPIO) {
        this->line.release();
    }
}

StatusLEDs* StatusLEDs::instance = nullptr;

void StatusLEDs::Init(std::shared_ptr<BridgeConfig> config, std::shared_ptr<rclcpp::Node> node) {

    if (instance == nullptr) {
        instance = new StatusLEDs();
    }

    instance->publisher_node = node;

    if (config->conn_led_pin > -1 || config->data_led_pin > -1) {
        if (config->conn_led_gpio_chip.empty()) {
            std::cerr << RED << "conn_led_gpio_chip not set" << CLR << std::endl;
            return;
        }

        static gpiod::chip chip (config->conn_led_gpio_chip);
        if (!chip) {
            throw std::runtime_error("Failed to open GPIO chip " + config->conn_led_gpio_chip);
        }
        if (config->conn_led_pin > -1) {
            std::cout << "Connection LED uses pin " << config->conn_led_pin << std::endl;
            try {
                auto line = chip.get_line((uint) config->conn_led_pin);
                instance->conn = std::make_shared<StatusLED>(line);
            } catch (const std::out_of_range& e) {
                throw std::runtime_error("Failed to open GPIO line " + std::to_string(config->conn_led_pin));
            }
        }
        if (config->data_led_pin > -1) {
            std::cout << "Data LED uses pin " << config->conn_led_pin << std::endl;
            try {
                auto line = chip.get_line((uint) config->data_led_pin);
                instance->data = std::make_shared<StatusLED>(line);
            } catch (const std::out_of_range& e) {
                throw std::runtime_error("Failed to open GPIO line " + std::to_string(config->data_led_pin));
            }
        }
    } else if (!config->conn_led_topic.empty() || !config->data_led_topic.empty()) {

        instance->publisher_node = node;
        rclcpp::QoS qos(1);
        if (!config->conn_led_topic.empty()) {
            std::cout << "Connection LED uses topic " << config->conn_led_topic << std::endl;
            auto publisher = node->create_publisher<std_msgs::msg::Bool>(config->conn_led_topic, qos);
            instance->conn = std::make_shared<StatusLED>(publisher);
        }
        if (!config->data_led_topic.empty()) {
            std::cout << "Data LED uses topic " << config->data_led_topic << std::endl;
            auto publisher = node->create_publisher<std_msgs::msg::Bool>(config->data_led_topic, qos);
            instance->data = std::make_shared<StatusLED>(publisher);
        }
    }
}

void StatusLEDs::Clear() {
    instance->conn->clear();
    instance->data->clear();
    if (instance->publisher_node) {
        rclcpp::spin_some(instance->publisher_node);
    }
    instance = nullptr;
}

// convenience 
void ConnLED::On() {
    StatusLEDs::GetInstance()->conn->on();
}
void ConnLED::FastPulse() {
    StatusLEDs::GetInstance()->conn->fastPulse();
}
void ConnLED::Off() {
    StatusLEDs::GetInstance()->conn->off();
}
void DataLED::Once() {
    StatusLEDs::GetInstance()->data->once();
}
void DataLED::Off() {
    StatusLEDs::GetInstance()->data->off();
}