#include "phntm_bridge/ffmpeg_encoder.hpp"
#include "phntm_bridge/lib.hpp"
#include "phntm_bridge/const.hpp"

#include <cstddef>
#include <functional>
#include <libavcodec/avcodec.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <string>

namespace phntm {

   FFmpegEncoder::FFmpegEncoder(int width, int height, std::string src_encoding, AVPixelFormat opencv_format, std::string frame_id, std::string topic, std::shared_ptr<rclcpp::Node> node, std::string& hw_device, int thread_count, int gop_size, int bit_rate, PacketCallback callback)
        : width(width), height(height), src_encoding(src_encoding), packet_callback(callback), frame_id(frame_id), topic(topic), node(node) {

        // Initialize FFmpeg
        avformat_network_init();
        
        // Find hardware device type
        if (hw_device == "cuda") {
            hw_device_type = av_hwdevice_find_type_by_name("cuda");
        } else if (hw_device == "vaapi") {
            hw_device_type = av_hwdevice_find_type_by_name("vaapi");
        } else {
            hw_device_type = AV_HWDEVICE_TYPE_NONE;
        }
        
        // Create hardware device context if needed
        if (hw_device_type != AV_HWDEVICE_TYPE_NONE) {
            if (av_hwdevice_ctx_create(&hw_device_ctx, hw_device_type, nullptr, nullptr, 0) < 0) {
                throw std::runtime_error("["+this->toString()+"] Failed to create hardware device context for " + hw_device);
            }
        }
        
        // Find the H.264 encoder
        const AVCodec* codec = nullptr;
        if (hw_device_type != AV_HWDEVICE_TYPE_NONE) {
            codec = avcodec_find_encoder_by_name("h264_nvenc"); // NVIDIA
            if (!codec) codec = avcodec_find_encoder_by_name("h264_vaapi"); // Intel
            if (!codec) codec = avcodec_find_encoder_by_name("h264_amf"); // AMD
        }
        
        if (!codec) {
            codec = avcodec_find_encoder(AV_CODEC_ID_H264); // Fallback to software
            //log("["+this->toString()+"] Warning: Software h.264 encoding selected for " + topic+", this is rather slow and expensive");
            RCLCPP_WARN(this->node->get_logger(), "[%s] Software h.264 encoding selected for %s, this is rather slow and expensive", this->toString().c_str(),  this->topic.c_str());
        }
        
        if (!codec) {
            throw std::runtime_error("["+this->toString()+"] H.264 encoder not found for " + topic);
        }
        
        // Set up codec context
        this->codec_ctx = avcodec_alloc_context3(codec);
        if (!this->codec_ctx) {
            throw std::runtime_error("["+this->toString()+"] Could not allocate codec context");
        }
        
        this->codec_ctx->width = width;
        this->codec_ctx->height = height;
        this->codec_ctx->time_base = (AVRational){1, fps}; // t
        this->codec_ctx->framerate = (AVRational){fps, 1};
        this->codec_ctx->pix_fmt = AV_PIX_FMT_NV12; // this is input to the codec (output of sws_scale)
        this->codec_ctx->gop_size = gop_size; // 60
        this->codec_ctx->max_b_frames = 0;
        this->codec_ctx->thread_count = thread_count;
        this->codec_ctx->bit_rate = bit_rate; // 512 * 1024 * 8; // 0.5 MB/s
        
        this->codec_ctx->flags &= ~AV_CODEC_FLAG_GLOBAL_HEADER;
        this->codec_ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;     // For real-time
        // this->codec_ctx->flags2 |= AV_CODEC_FLAG2_FAST;        // Faster encoding

        if (hw_device_ctx) {
            this->codec_ctx->hw_device_ctx = hw_device_ctx;
        }
        
        // Set encoder options
        av_opt_set(this->codec_ctx->priv_data, "preset", "fast", 0);
        if (hw_device_type == AV_HWDEVICE_TYPE_NONE)
            av_opt_set(this->codec_ctx->priv_data, "tune", "zerolatency", 0);
        av_opt_set(this->codec_ctx->priv_data, "profile", "high", 0);
        
        // Open codec
        if (avcodec_open2(this->codec_ctx, codec, nullptr) < 0) {
            throw std::runtime_error("["+this->toString()+"] Could not open codec for " + topic);
        }
        
        // Allocate frame
        frame = av_frame_alloc();
        if (!frame) {
            throw std::runtime_error("["+this->toString()+"] Could not allocate frame for "+topic);
        }
        
        frame->format = this->codec_ctx->pix_fmt;
        frame->width = width;
        frame->height = height;
        
        if (av_frame_get_buffer(frame, 0) < 0) {
            throw std::runtime_error("["+this->toString()+"] Could not allocate frame data for " + topic);
        }
        
        // Initialize color conversion context
        sws_ctx = sws_getContext(width, height, opencv_format, // OpenCV format
                                width, height, this->codec_ctx->pix_fmt, // codec input
                                SWS_POINT, nullptr, nullptr, nullptr);

        this->running = true;
        this->encoder_thread = std::thread(&FFmpegEncoder::encoderWorker, this);
        this->encoder_thread.detach();
    }

    void FFmpegEncoder::encodeFrame(const cv::Mat& raw_frame, std_msgs::msg::Header header, bool debug_log) {
    
        if (raw_frame.empty()) {
            throw std::invalid_argument("["+this->toString()+"] Empty frame provided");
        }

        if (!this->running)
            return;

        const uint8_t* src_data[] = { raw_frame.data };
        int src_linesize[] = { static_cast<int>(raw_frame.step) };
        
        // sw scaling here - expensive
        sws_scale(sws_ctx, src_data, src_linesize, 0, height, 
                frame->data, frame->linesize);

        frame->pts = convertToRtpTimestamp(header.stamp.sec, header.stamp.nanosec);

        // Send for encoding
        this->sendFrameToEncoder(frame, debug_log);

    }

    void FFmpegEncoder::sendFrameToEncoder(AVFrame* frame, bool debug_log) {
        
        std::lock_guard<std::mutex> queue_lock(this->mutex);
        this->queue.push(frame);
        this->encoder_cv.notify_one();
    }

    void FFmpegEncoder::flush() {
        sendFrameToEncoder(nullptr, false);  // Flush the encoder
    }

    void FFmpegEncoder::encoderWorker() {

        log("["+this->toString()+"] FFmpegEncoder worker runnig...");

        while (this->running) {

            auto pkt = av_packet_alloc();
            if (!pkt) {
                throw std::runtime_error("["+this->toString()+"] Could not allocate packet");
            }

            std::unique_lock<std::mutex> queue_lock(this->mutex);
            this->encoder_cv.wait(queue_lock, [this] { return !this->queue.empty() || !this->running; });

            if (this->queue.empty()) 
                break;

            auto frame = this->queue.front();
            this->queue.pop();

            queue_lock.unlock();

            int ret = avcodec_send_frame(this->codec_ctx, frame);
            if (ret < 0) {
                throw std::runtime_error("["+this->toString()+"] Error sending frame to encoder");
            }

            if (frame == nullptr) { // flushed
                this->running = false;
                break;
            }
                
            // Convert from OpenCV BGR to encoder's format
            ret = avcodec_receive_packet(this->codec_ctx, pkt);

            if (ret == AVERROR(EAGAIN)) { // The encoder needs more input frames before it can output a packet
                // No more packets will be produced (flush completed)
                // log("["+this->toString()+"] Error during receiving frame - EAGAIN");
                av_packet_free(&pkt);
                continue;
            } else if (ret == AVERROR_EOF) {
                log("["+this->toString()+"] Error during receiving frame - AVERROR_EOF");
                av_packet_free(&pkt);
                this->running = false;
                break;
            } else if (ret < 0) {
                log("["+this->toString()+"] Error during receiving frame, " + std::to_string(ret), true);
                av_packet_free(&pkt);
                this->running = false;
                break;
            }

            if (packet_callback) {
                auto frame = std::make_shared<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>();
                frame->header = std_msgs::msg::Header();
                frame->header.frame_id = this->frame_id;
                frame->header.stamp = node->now();
                frame->encoding = "h.264";
                frame->width = width;
                frame->height = height;
                frame->flags = pkt->flags;
                frame->is_bigendian = false;
                frame->pts = frame->pts; //calculated from the initial header stamp
                // frame->data.resize(pkt->size);
                frame->data.assign(pkt->data, pkt->data + pkt->size);
                packet_callback(frame);
            }
            
            av_packet_unref(pkt);

        }    
        log("["+this->toString()+"] FFmpegEncoder worker finished.");
    }

    FFmpegEncoder::~FFmpegEncoder() {

        log("["+this->toString()+"] Destroying encoder...");

        this->flush(); // flush encoder, kills the thread when complete

        while (this->running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        log("["+this->toString()+"] Claning up...");
        // Cleanup
        if (this->frame)
            av_frame_free(&this->frame);
        if (this->codec_ctx) {
            this->codec_ctx->hw_device_ctx = nullptr; //remove this before deallocating
            avcodec_free_context(&this->codec_ctx);
        }
        if (hw_device_ctx)
            av_buffer_unref(&hw_device_ctx);
        if (fmt_ctx)
            avformat_free_context(fmt_ctx);
        if (sws_ctx)
            sws_freeContext(sws_ctx);

        if (this->node.get() != nullptr)
            this->node.reset();

        log("["+this->toString()+"] Cleanup done.");
    }

    

}