#include "phntm_bridge/status_leds.hpp"
#include "phntm_bridge/const.hpp"
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <stdexcept>
#include <string>

namespace phntm {

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

    void StatusLED::on() {
        this->state = State::ON;
    }

    void StatusLED::off() {
        this->state = State::OFF;
    }

    void StatusLED::once(uint on_ms) {
        this->state = State::BLINK_ONCE;
        this->on_ms = on_ms;
    }

    void StatusLED::interval(uint on_ms, uint interval_ms) {
        this->state = State::BLINKING;
        this->on_ms = on_ms;
        this->interval_ms = interval_ms;
    }

    void StatusLED::fastPulse() {
        this->interval(10, 500);
    }

    void StatusLED::update() {
        auto now = std::chrono::steady_clock::now();
        // auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        switch (this->state) {

            case StatusLED::State::ON:
                if (this->current_state != true || std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_on_time).count() > 1000)
                    this->setState(true);
                break;

            case StatusLED::State::OFF:
                if (this->current_state != false || std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_off_time).count() > 1000)
                    this->setState(false);
                break;

            case StatusLED::State::BLINK_ONCE:
                if (this->current_state != true && std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_off_time).count() > this->min_off_ms) {
                    this->setState(true);
                } else if (this->current_state == true && std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_on_time).count() > this->on_ms) {
                    this->off();
                    this->setState(false);
                }
                break;

            case StatusLED::State::BLINKING:
                if (this->current_state != true) {
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_off_time).count() > (this->interval_ms - this->on_ms)) {
                        this->setState(true);
                    }
                } else if (this->current_state == true && std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_on_time).count() > this->on_ms) {
                    this->setState(false);
                }

                break;

            default:
                break;
        }
    }

    void StatusLED::setState(bool state) {
        if (this->mode == Mode::GPIO) {
            this->line.set_value(state ? 1 : 0);
        } else if (this->mode == Mode::ROS && rclcpp::ok()) {
            publisher->publish(state ? this->msg_on : this->msg_off);
        }
        
        this->current_state = state;
        if (state) {
            this->last_on_time = std::chrono::steady_clock::now();
        } else {
            this->last_off_time = std::chrono::steady_clock::now();
        }
    }

    void StatusLED::clear() {
        this->setState(false);
        if (this->mode == Mode::GPIO) {
            this->line.release();
        }
    }

    StatusLEDs* StatusLEDs::instance = nullptr;

    void StatusLEDs::init(std::shared_ptr<PhntmBridge> node, std::shared_ptr<BridgeConfig> config) {

        if (instance == nullptr) {
            instance = new StatusLEDs();
        }

        if (config->conn_led_pin > -1 || config->data_led_pin > -1) {
            if (config->conn_led_gpio_chip.empty()) {
                log("conn_led_gpio_chip not set in the config file!", true);
                return;
            }

            static gpiod::chip chip (config->conn_led_gpio_chip);
            if (!chip) {
                throw std::runtime_error("Failed to open GPIO chip " + config->conn_led_gpio_chip);
            }
            if (config->conn_led_pin > -1) {
                log("Connection LED uses pin " + std::to_string(config->conn_led_pin));
                try {
                    auto line = chip.get_line((uint) config->conn_led_pin);
                    instance->conn = std::make_shared<StatusLED>(line);
                    instance->leds.push_back(instance->conn);
                } catch (const std::out_of_range& e) {
                    throw std::runtime_error("Failed to open GPIO line " + std::to_string(config->conn_led_pin));
                }
            }
            if (config->data_led_pin > -1) {
                log("Data LED uses pin " + std::to_string(config->data_led_pin));
                try {
                    auto line = chip.get_line((uint) config->data_led_pin);
                    instance->data = std::make_shared<StatusLED>(line);
                    instance->leds.push_back(instance->data);
                } catch (const std::out_of_range& e) {
                    throw std::runtime_error("Failed to open GPIO line " + std::to_string(config->data_led_pin));
                }
            }
        } else if (!config->conn_led_topic.empty() || !config->data_led_topic.empty()) {

            if (!config->conn_led_topic.empty()) {
                rclcpp::QoS qos = node->loadTopicQoSConfig(config->conn_led_topic);
                log("Connection LED uses topic " + config->conn_led_topic);
                auto publisher = node->create_publisher<std_msgs::msg::Bool>(config->conn_led_topic, qos);
                instance->conn = std::make_shared<StatusLED>(publisher);
                instance->leds.push_back(instance->conn);
            }
            if (!config->data_led_topic.empty()) {
                rclcpp::QoS qos = node->loadTopicQoSConfig(config->data_led_topic);
                log("Data LED uses topic " + config->data_led_topic);
                auto publisher = node->create_publisher<std_msgs::msg::Bool>(config->data_led_topic, qos);
                instance->data = std::make_shared<StatusLED>(publisher);
                instance->leds.push_back(instance->data);
            }
        }

        if (instance->data != nullptr || instance->conn != nullptr) {
            instance->loop_running = true;
            instance->loop_thread = std::thread(&StatusLEDs::loop, instance);
            instance->loop_thread.detach();
        }
    }

    void StatusLEDs::clear() {
        instance->loop_running = false;
        // instance->loop_thread.join();
        instance->conn->clear();
        instance->data->clear();
        instance = nullptr;
    }

    void StatusLEDs::loop() {
        log("Status LEDs loop running");
        while (this->loop_running) {
            for (auto & led : this->leds) {
            led->update();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1ms loop
        }
        log("Status LEDs loop finished");
    }

    // convenience 
    void ConnLED::on() {
        auto inst = StatusLEDs::getInstance();
        if (inst == nullptr) return;
        if (inst->conn == nullptr) return;
        inst->conn->on();
    }
    void ConnLED::fastPulse() {
        auto inst = StatusLEDs::getInstance();
        if (inst == nullptr) return;
        if (inst->conn == nullptr) return;
        inst->conn->fastPulse();
    }
    void ConnLED::off() {
        auto inst = StatusLEDs::getInstance();
        if (inst == nullptr) return;
        if (inst->conn == nullptr) return;
        inst->conn->off();
    }

    void DataLED::once() {
        auto inst = StatusLEDs::getInstance();
        if (inst == nullptr) return;
        if (inst->data == nullptr) return;
        inst->data->once();
    }
    void DataLED::off() {
        auto inst = StatusLEDs::getInstance();
        if (inst == nullptr) return;
        if (inst->data == nullptr) return;
        inst->data->off();
    }
    
}