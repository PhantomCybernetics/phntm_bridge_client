#pragma once

#include <opencv2/opencv.hpp>

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

        FFmpegEncoder(int width, int height, const std::string src_encoding, AVPixelFormat opencv_format, std::string frame_id, std::string topic, std::shared_ptr<rclcpp::Node> node, std::string& hw_device, int thread_count, int gop_size, int bit_rate, PacketCallback callback = nullptr);
        ~FFmpegEncoder();
        
        void encodeFrame(const cv::Mat& raw_frame, std_msgs::msg::Header header, bool debug_log);
        bool checkCompatibility(const int width, const int height, const std::string & src_encoding) { return width == this->width && height == this->height && src_encoding == this->src_encoding; };

    private:
        int width, height;
        const int fps = 30;
        std::string src_encoding;
        int64_t pts_counter = 0;
        PacketCallback packet_callback;

        bool running = false;

        std::thread encoder_thread;
        std::condition_variable encoder_cv;

        AVFormatContext* fmt_ctx = nullptr;
        AVCodec* codec = nullptr;
        // AVStream* stream = nullptr;
        AVCodecContext* codec_ctx = nullptr;
        AVFrame* frame = nullptr;
        std::queue<AVFrame*> queue;
        std::mutex mutex;
        SwsContext* sws_ctx = nullptr;
        
        AVBufferRef* hw_device_ctx = nullptr;
        enum AVHWDeviceType hw_device_type = AV_HWDEVICE_TYPE_NONE;

        void sendFrameToEncoder(AVFrame* frame, bool debug_log);
        void encoderWorker();
        void flush();

        std::string frame_id, topic;
        std::shared_ptr<rclcpp::Node> node;

        std::string toString() { return "Enc " + this->topic; };
    };

}