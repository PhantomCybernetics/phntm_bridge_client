#include "phntm_bridge/ffmpeg_encoder.hpp"
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

    std::vector<AVCodecID> FFmpegEncoder::encoder_input_logged;

    FFmpegEncoder::FFmpegEncoder(int width, int height, std::string src_encoding, std::string frame_id,
                                 std::string topic, int depth_colormap, double depth_max_sensor_value,
                                 std::shared_ptr<rclcpp::Node> node,
                                 std::string& hw_device, int thread_count, int gop_size, int bit_rate, PacketCallback callback)
        : width(width), height(height), src_encoding(strToLower(src_encoding)), frame_id(frame_id),
          topic(topic), depth_colormap(depth_colormap), depth_max_sensor_value(depth_max_sensor_value), node(node), packet_callback(callback) {

        AVPixelFormat src_opencv_format;
        if (this->src_encoding == "rgb8") {
            src_opencv_format = AV_PIX_FMT_RGB24;
        } else if (this->src_encoding == "bgr8") {
            src_opencv_format = AV_PIX_FMT_BGR24;
        } else if (this->src_encoding == "mono8" || this->src_encoding == "8uc1") {
            src_opencv_format = AV_PIX_FMT_GRAY8;
        } else if (this->src_encoding == "16uc1" || this->src_encoding == "mono16") {
            src_opencv_format = AV_PIX_FMT_RGB24;
        } else if (this->src_encoding == "32uc1") {
            src_opencv_format = AV_PIX_FMT_RGB24;
        } else if (this->src_encoding == "32fc1") {
            src_opencv_format = AV_PIX_FMT_RGB24;
        } else {
            throw std::runtime_error("["+this->toString()+"] Received frame with unsupported encoding: " + this->src_encoding);
        }

        // Initialize FFmpeg
        avformat_network_init();
        
        // Find hardware device type
        if (hw_device == "cuda") {
            this->hw_device_type = av_hwdevice_find_type_by_name("cuda");
        } else if (hw_device == "vaapi") {
            this->hw_device_type = av_hwdevice_find_type_by_name("vaapi");
        } else {
            this->hw_device_type = AV_HWDEVICE_TYPE_NONE;
        }
        
        // Create hardware device context if needed
        if (this->hw_device_type != AV_HWDEVICE_TYPE_NONE) {
            if (av_hwdevice_ctx_create(&hw_device_ctx, hw_device_type, nullptr, nullptr, 0) < 0) {
                throw std::runtime_error("["+this->toString()+"] Failed to create hardware device context for " + hw_device);
            }
        }
        
        // Find the H.264 encoder
        const AVCodec* codec = nullptr;
        if (this->hw_device_type != AV_HWDEVICE_TYPE_NONE) {
            if (hw_device == "cuda") {
                RCLCPP_INFO(this->node->get_logger(), "[%s] Setting codec to cuda", this->toString().c_str());
                codec = avcodec_find_encoder_by_name("h264_nvenc"); // NVIDIA
            } else if (hw_device == "vaapi") {
                RCLCPP_INFO(this->node->get_logger(), "[%s] Setting codec to h264_amf", this->toString().c_str());
                codec = avcodec_find_encoder_by_name("h264_amf"); // try AMD
                if (!codec) {
                    RCLCPP_INFO(this->node->get_logger(), "[%s] Setting codec to h264_vaapi", this->toString().c_str());
                    codec = avcodec_find_encoder_by_name("h264_vaapi"); // Intel
                }
            }
        }
        
        if (!codec) {
            codec = avcodec_find_encoder(AV_CODEC_ID_H264); // Fallback to software
            //log("["+this->toString()+"] Warning: Software h.264 encoding selected for " + topic+", this is rather slow and expensive");
            RCLCPP_WARN(this->node->get_logger(), "[%s] Software h.264 encoding selected for %s, this is rather slow and expensive", this->toString().c_str(),  this->topic.c_str());
        }
        
        if (!codec) {
            throw std::runtime_error("[" + this->toString() + "] H.264 encoder not found for " + topic);
        }
        
        // output supported input pixel formats for each codec
        if (std::find(FFmpegEncoder::encoder_input_logged.begin(), FFmpegEncoder::encoder_input_logged.end(), codec->id) == FFmpegEncoder::encoder_input_logged.end()) {

            FFmpegEncoder::encoder_input_logged.push_back(codec->id); //only once
            if (codec->pix_fmts) {
                const enum AVPixelFormat *p = codec->pix_fmts;
                while (*p != AV_PIX_FMT_NONE) {
                    RCLCPP_INFO(this->node->get_logger(), "[AVCodec %s] Supported input pixel format: %s", codec->name, av_get_pix_fmt_name(*p));
                    // Optionally, use av_get_pix_fmt_name(*p) to print the name
                    p++;
                }
            } else {
                RCLCPP_WARN(this->node->get_logger(), "[AVCodec %s] No supported input pixel formats detected!", codec->name);
            }
        }
        RCLCPP_INFO(this->node->get_logger(), "[%s] OpenCV conversion format for sw-scaling: %s", this->toString().c_str(), av_get_pix_fmt_name(src_opencv_format));

        // Set up codec context
        this->codec_ctx = avcodec_alloc_context3(codec);
        if (!this->codec_ctx) {
            throw std::runtime_error("[" + this->toString() + "] Could not allocate codec context");
        }
        
        this->codec_ctx->width = width;
        this->codec_ctx->height = height;
        this->codec_ctx->time_base = {1, fps}; // t
        this->codec_ctx->framerate = {fps, 1};
        this->codec_ctx->pix_fmt = this->hw_device_type == AV_HWDEVICE_TYPE_VAAPI ? AV_PIX_FMT_VAAPI : AV_PIX_FMT_NV12; // this is input to the codec (output of sws_scale)
        this->codec_ctx->gop_size = gop_size; // 60
        this->codec_ctx->max_b_frames = 0;
        this->codec_ctx->thread_count = thread_count;
        this->codec_ctx->bit_rate = bit_rate; // 512 * 1024 * 8; // 0.5 MB/s
        
        this->codec_ctx->flags &= ~AV_CODEC_FLAG_GLOBAL_HEADER;
        this->codec_ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;     // For real-time
        // this->codec_ctx->flags2 |= AV_CODEC_FLAG2_FAST;        // Faster encoding
        
        // Set encoder options
        av_opt_set(this->codec_ctx->priv_data, "preset", "fast", 0);
        if (this->hw_device_type == AV_HWDEVICE_TYPE_NONE)
            av_opt_set(this->codec_ctx->priv_data, "tune", "zerolatency", 0);
        av_opt_set(this->codec_ctx->priv_data, "profile", "high", 0);
                
        // Initialize color conversion context
        this->sws_ctx = sws_getContext(width, height,
                                src_opencv_format, // OpenCV format
                                width, height,
                                AV_PIX_FMT_NV12, // codec input
                                SWS_POINT,
                                nullptr, nullptr,
                                nullptr);

        // allocate sw frames pool
        for (uint i = 0; i < this->num_frame_buffers; i++) {
            auto sw_frame = av_frame_alloc();
            if (!sw_frame) {
                throw std::runtime_error("["+this->toString()+"] Could not allocate sw frame #" + std::to_string(i) + " for "+topic);
            }
            sw_frame->format = AV_PIX_FMT_NV12; //this->codec_ctx->pix_fmt;
            sw_frame->width = width;
            sw_frame->height = height;
            if (av_frame_get_buffer(sw_frame, 0) < 0) {
                throw std::runtime_error("["+this->toString()+"] Could not allocate sw frame #" + std::to_string(i) + " data for " + topic);
            }
            this->sw_frame_buffers.push_back(sw_frame);
        }

        // create hardware device context
        if (this->hw_device_type != AV_HWDEVICE_TYPE_NONE) {
            if (this->hw_device_type == AV_HWDEVICE_TYPE_VAAPI) {
                // TODO: Customize/detect device
                RCLCPP_INFO(this->node->get_logger(), "[AVCodec] Making hw device ctx for VAAPI");
                if (av_hwdevice_ctx_create(&this->hw_device_ctx, hw_device_type, "/dev/dri/renderD128", nullptr, 0) < 0) {
                   throw std::runtime_error("["+this->toString()+"] Failed to create hardware device context for " + hw_device);
                }
                this->codec_ctx->hw_device_ctx = av_buffer_ref(this->hw_device_ctx);
            } else {
                RCLCPP_INFO(this->node->get_logger(), "[AVCodec] Making hw device ctx");
                if (av_hwdevice_ctx_create(&this->hw_device_ctx, hw_device_type, nullptr, nullptr, 0) < 0) {
                   throw std::runtime_error("["+this->toString()+"] Failed to create hardware device context for " + hw_device);
                }
                this->codec_ctx->hw_device_ctx = av_buffer_ref(this->hw_device_ctx);
            }
        }

        if (this->hw_device_type == AV_HWDEVICE_TYPE_VAAPI) {
            RCLCPP_INFO(this->node->get_logger(), "[AVCodec %s] Making hw frames ctx", codec->name);
            
            int err = 0;
            AVBufferRef *hw_frames_ref;
            AVHWFramesContext *hw_frames_ctx = NULL;

            if (!(hw_frames_ref = av_hwframe_ctx_alloc(hw_device_ctx))) {
                throw std::runtime_error("Failed to create VAAPI frame context.");
            }
            hw_frames_ctx = (AVHWFramesContext *)(hw_frames_ref->data);
            hw_frames_ctx->format = AV_PIX_FMT_VAAPI;       // Hardware pixel format
            hw_frames_ctx->sw_format = AV_PIX_FMT_NV12;     // SW pixel format to upload from
            hw_frames_ctx->width = width;
            hw_frames_ctx->height = height;
            hw_frames_ctx->initial_pool_size = this->num_frame_buffers;

            if ((err = av_hwframe_ctx_init(hw_frames_ref)) < 0) {
                av_buffer_unref(&hw_frames_ref);
                throw std::runtime_error("Failed to initialize VAAPI frame context. Error code: " + std::to_string(err));
            }

            this->codec_ctx->hw_frames_ctx = av_buffer_ref(hw_frames_ref);
            if (!this->codec_ctx->hw_frames_ctx) {
                av_buffer_unref(&hw_frames_ref);
                throw std::runtime_error("Failed to allocate codec_ctx->hw_frames_ctx");
            }

            av_buffer_unref(&hw_frames_ref);

            // allocate hw frames pool
            for (uint i = 0; i < this->num_frame_buffers; i++) {

                AVFrame * hw_frame;
                
                if (!(hw_frame = av_frame_alloc())) {
                    err = AVERROR(ENOMEM);
                    throw std::runtime_error("Error allocating hw frame " + std::to_string(i));
                }
                if ((err = av_hwframe_get_buffer(this->codec_ctx->hw_frames_ctx, hw_frame, 0)) < 0) {
                    throw std::runtime_error("Error getting buffer for hw frame " + std::to_string(i));
                }
                if (!hw_frame->hw_frames_ctx) {
                    throw std::runtime_error("Error checking allocated hw frame " + std::to_string(i));
                }

                this->hw_frame_buffers.push_back(hw_frame);
            }
        }
        
        // open codec
        if (avcodec_open2(this->codec_ctx, codec, nullptr) < 0) {
            throw std::runtime_error("["+this->toString()+"] Could not open codec for " + topic);
        }

        // lanuch workers
        this->running = true;
        this->scaler_thread = std::thread(&FFmpegEncoder::scalerWorker, this);
        this->scaler_thread.detach();
        this->encoder_thread = std::thread(&FFmpegEncoder::encoderWorker, this);
        this->encoder_thread.detach();
    }

    void FFmpegEncoder::encodeFrame(const std::shared_ptr<sensor_msgs::msg::Image> msg) {
    
        // if (raw_frame.empty()) {
        //     throw std::invalid_argument("["+this->toString()+"] Empty frame provided");
        // }

        if (!this->running)
            return;

        // auto req = ScalerRequest { raw_frame, header, im_ref};
        {
            std::lock_guard<std::mutex> queue_lock(this->scaler_mutex);
            this->scaler_queue.push(msg);
            this->scaler_cv.notify_one();
        }
    }

    void FFmpegEncoder::sendFrameToEncoder(AVFrame* input_frame, std_msgs::msg::Header header) {
        std::lock_guard<std::mutex> queue_lock(this->encoder_mutex);
        EncoderRequest req;
        req.frame = input_frame;
        req.header = header;
        this->encoder_queue.push(req);
        this->encoder_cv.notify_one();
    }

    void FFmpegEncoder::flush() { 
       std::lock_guard<std::mutex> queue_lock(this->encoder_mutex);
        EncoderRequest req;
        req.frame = nullptr; //flush the encoder
        this->encoder_queue.push(req);
        this->encoder_cv.notify_one();
    }

    void FFmpegEncoder::scalerWorker() {
         std::cout << "["+this->toString()+"] FFmpegEncoder scaler runnig..." << std::endl;

        int err;
        while (this->running) {

            std::unique_lock<std::mutex> scaler_lock(this->scaler_mutex);
            this->scaler_cv.wait(scaler_lock, [this] { return !this->scaler_queue.empty() || !this->running; });

            if (this->scaler_queue.empty() || !this->running) 
                break;
            
            std::shared_ptr<sensor_msgs::msg::Image> msg;
            while (!this->scaler_queue.empty()) {
                msg = this->scaler_queue.front();
                this->scaler_queue.pop();
            }

            scaler_lock.unlock();

            cv::Mat raw_frame;
            if (this->src_encoding == "rgb8") {
                raw_frame = cv::Mat(msg->height, msg->width, CV_8UC3, msg->data.data());
            } else if (this->src_encoding == "bgr8") {
                raw_frame = cv::Mat(msg->height, msg->width, CV_8UC3, msg->data.data());
            } else if (this->src_encoding == "mono8" || this->src_encoding == "8uc1") {
                raw_frame = cv::Mat(msg->height, msg->width, CV_8UC1, msg->data.data());
            } else if (this->src_encoding == "16uc1" || this->src_encoding == "mono16") {
                cv::Mat mono16(msg->height, msg->width, CV_16UC1, msg->data.data());
                cv::Mat mono8;
                mono16.convertTo(mono8, CV_8UC1, 255.0 / this->depth_max_sensor_value); // Convert to 8-bit (0-255 range)
                cv::applyColorMap(mono8, raw_frame, this->depth_colormap); // Apply color map
            } else if (this->src_encoding == "32uc1") {
                cv::Mat mono32(msg->height, msg->width, CV_32SC1, msg->data.data());
                cv::Mat mono8;
                mono32.convertTo(mono8, CV_8UC1, 255.0 / this->depth_max_sensor_value); // Convert to 8-bit (0-255 range)
                cv::applyColorMap(mono8, raw_frame, this->depth_colormap); // Apply color map
            } else if (this->src_encoding == "32fc1") {
                cv::Mat mono32(msg->height, msg->width, CV_32FC1, msg->data.data());
                cv::Mat mono8;
                mono32.convertTo(mono8, CV_8UC1, 255.0 / this->depth_max_sensor_value); // Convert to 8-bit (0-255 range)
                cv::applyColorMap(mono8, raw_frame, this->depth_colormap); // Apply color map
            }

            AVFrame* sw_frame = this->sw_frame_buffers[this->current_frame_buffer];
            AVFrame* hw_frame;
            if (this->hw_device_type == AV_HWDEVICE_TYPE_VAAPI) {
                hw_frame = this->hw_frame_buffers[this->current_frame_buffer];
            }
            
            this->current_frame_buffer++;
            if (this->current_frame_buffer == this->num_frame_buffers) {
                this->current_frame_buffer = 0;
            }
            const uint8_t* src_data[] = { raw_frame.data };
            int src_linesize[] = { static_cast<int>(raw_frame.step) };

            // sw scaling & copy here - expensive
            sws_scale(this->sws_ctx, src_data,
                    src_linesize, 0, this->height, 
                    sw_frame->data, sw_frame->linesize);

            // Send for encoding
            if (this->hw_device_type == AV_HWDEVICE_TYPE_VAAPI) {
                if ((err = av_hwframe_transfer_data(hw_frame, sw_frame, 0)) < 0) {
                    throw std::runtime_error("Error while transferring frame data to surface. Error code: " + std::to_string(err));
                }
                // hw_frame->pts = convertToRtpTimestamp(im->header.stamp.sec, im->header.stamp.nanosec);
                this->sendFrameToEncoder(hw_frame, msg->header);
            } else {
                // sw_frame->pts = convertToRtpTimestamp(im->header.stamp.sec, im->header.stamp.nanosec);
                this->sendFrameToEncoder(sw_frame, msg->header);
            }
        }

        std::cout << "["+this->toString()+"] FFmpegEncoder scaler stopped..." << std::endl;
    }

    void FFmpegEncoder::encoderWorker() {

        log("["+this->toString()+"] FFmpegEncoder worker runnig...");

        while (this->running) {

            auto pkt = av_packet_alloc();
            if (!pkt) {
                throw std::runtime_error("["+this->toString()+"] Could not allocate packet");
            }

            std::unique_lock<std::mutex> queue_lock(this->encoder_mutex);
            this->encoder_cv.wait(queue_lock, [this] { return !this->encoder_queue.empty() || !this->running; });

            if (this->encoder_queue.empty()) 
                break;

            EncoderRequest req;
            while (!this->encoder_queue.empty()) {
                req = this->encoder_queue.front();
                this->encoder_queue.pop();
            }

            queue_lock.unlock();

            int ret = avcodec_send_frame(this->codec_ctx, req.frame);
            if (ret < 0) {
                throw std::runtime_error("["+this->toString()+"] Error sending frame to encoder");
            }

            if (req.frame == nullptr) { // flushed
                this->running = false;
                this->scaler_cv.notify_one(); // clear the scaler too
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
                frame->header.stamp.sec = req.header.stamp.sec;
                frame->header.stamp.nanosec = req.header.stamp.nanosec;
                frame->encoding = "h.264";
                frame->width = width;
                frame->height = height;
                frame->flags = pkt->flags;
                frame->is_bigendian = false;
                frame->pts = 0; // header stamp is used
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
        for (uint i = 0; i < this->num_frame_buffers; i++) {
            if (this->sw_frame_buffers[i])
                av_frame_free(&this->sw_frame_buffers[i]);
            if (this->hw_device_type == AV_HWDEVICE_TYPE_VAAPI && this->hw_frame_buffers[i]) {
                av_frame_free(&this->hw_frame_buffers[i]);
            }
        }
        this->sw_frame_buffers.clear();
        this->hw_frame_buffers.clear();
        
        if (this->codec_ctx) {
            this->codec_ctx->hw_device_ctx = nullptr; //remove this before deallocating
            avcodec_free_context(&this->codec_ctx);
        }
        if (this->hw_device_ctx)
            av_buffer_unref(&this->hw_device_ctx);
        if (this->fmt_ctx)
            avformat_free_context(this->fmt_ctx);
        if (this->sws_ctx)
            sws_freeContext(this->sws_ctx);

        // if (this->node.get() != nullptr)
        //     this->node.reset();

        log("["+this->toString()+"] Cleanup done.");
    }
}