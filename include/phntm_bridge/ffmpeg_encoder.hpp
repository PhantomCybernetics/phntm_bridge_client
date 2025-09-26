#pragma once

#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "phntm_bridge/lib.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <std_msgs/msg/header.hpp>
#include <ffmpeg_image_transport_msgs/msg/detail/ffmpeg_packet__struct.hpp>

#include "rclcpp/rclcpp.hpp"
#include <thread>

namespace phntm {
    class FFmpegEncoder {
    public:
        using PacketCallback = std::function<void(std::shared_ptr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> frame)>;

        FFmpegEncoder(int width, int height, const std::string src_encoding, std::string frame_id,
                      std::string topic, int depth_colormap, double depth_max_sensor_value,
                      std::shared_ptr<rclcpp::Node> node,
                      std::string& hw_device, int thread_count, int gop_size, int bit_rate,
                      PacketCallback callback = nullptr);
        ~FFmpegEncoder();
        
        void encodeFrame(const std::shared_ptr<sensor_msgs::msg::Image> msg);
        bool checkCompatibility(const int width, const int height, const std::string & src_encoding) {
            return width == this->width && height == this->height
                  && strToLower(src_encoding) == this->src_encoding;
        };

    private:
        int width, height;
        const int fps = 30;
        std::string src_encoding;
        int64_t pts_counter = 0;
    
        std::string frame_id, topic;

        int depth_colormap; // used to colorize mono images
        double depth_max_sensor_value; // used to normalize raw sensor data
        std::shared_ptr<rclcpp::Node> node;

        PacketCallback packet_callback;

        bool running = false;

        std::thread scaler_thread;
        std::thread encoder_thread;
        std::condition_variable encoder_cv;

        AVFormatContext* fmt_ctx = nullptr;
        AVCodec* codec = nullptr;
        AVCodecContext* codec_ctx = nullptr;
        std::vector<AVFrame*> sw_frame_buffers;
        std::vector<AVFrame*> hw_frame_buffers;
        uint num_frame_buffers = 16;
        uint current_frame_buffer = 0;
        SwsContext* sws_ctx = nullptr;
        
        AVBufferRef* hw_device_ctx = nullptr;
        enum AVHWDeviceType hw_device_type = AV_HWDEVICE_TYPE_NONE;
        
        void sendFrameToEncoder(AVFrame* frame,  std_msgs::msg::Header header);
        void scalerWorker();
        void encoderWorker();
        void flush();

        struct EncoderRequest {
            AVFrame* frame;
            std_msgs::msg::Header header;
        };

        std::queue<std::shared_ptr<sensor_msgs::msg::Image>> scaler_queue;
        std::condition_variable scaler_cv;
        std::mutex scaler_mutex;

        std::queue<EncoderRequest> encoder_queue;
        std::mutex encoder_mutex;


        std::string toString() { return "Enc " + this->topic; };

        static std::vector<AVCodecID> encoder_input_logged;
    };

}