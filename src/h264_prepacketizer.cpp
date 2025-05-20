/**
 * Copyright (c) 2020 Filip Klembara (in2core)
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "phntm_bridge/h264_prepacketizer.hpp"
#include "phntm_bridge/lib.hpp"

#include "impl/internals.hpp"

#include <algorithm>
#include <cassert>

#ifdef _WIN32
#include <winsock2.h>
#else
#include <arpa/inet.h>
#endif

namespace phntm {

	H264PrePacketizer::H264PrePacketizer(Separator separator,
										std::shared_ptr<rtc::RtpPacketizationConfig> rtpConfig,
										size_t maxFragmentSize)
		: rtc::RtpPacketizer(rtpConfig), mSeparator(separator), mMaxFragmentSize(maxFragmentSize) {}

	std::vector<rtc::binary> H264PrePacketizer::fragment(rtc::binary data) {
		return rtc::NalUnit::GenerateFragments(splitFrame(data), mMaxFragmentSize);
	}

	std::vector<rtc::NalUnit> H264PrePacketizer::splitFrame(const rtc::binary &frame) {
		std::vector<rtc::NalUnit> nalus;
		if (mSeparator == Separator::Length) {
			size_t index = 0;
			while (index < frame.size()) {
				assert(index + 4 < frame.size());
				if (index + 4 >= frame.size()) {
					log("Invalid NAL Unit data (incomplete length), ignoring!", true);
					break;
				}
				uint32_t length;
				std::memcpy(&length, frame.data() + index, sizeof(uint32_t));
				length = ntohl(length);
				auto naluStartIndex = index + 4;
				auto naluEndIndex = naluStartIndex + length;

				assert(naluEndIndex <= frame.size());
				if (naluEndIndex > frame.size()) {
					log("Invalid NAL Unit data (incomplete unit), ignoring!", true);
					break;
				}
				auto begin = frame.begin() + naluStartIndex;
				auto end = frame.begin() + naluEndIndex;
				nalus.emplace_back(begin, end);
				index = naluEndIndex;
			}
		} else {
			rtc::NalUnitStartSequenceMatch match = rtc::NUSM_noMatch;
			size_t index = 0;
			while (index < frame.size()) {
				match = rtc::NalUnit::StartSequenceMatchSucc(match, frame[index++], mSeparator);
				if (match == rtc::NUSM_longMatch || match == rtc::NUSM_shortMatch) {
					match = rtc::NUSM_noMatch;
					break;
				}
			}

			size_t naluStartIndex = index;

			while (index < frame.size()) {
				match = rtc::NalUnit::StartSequenceMatchSucc(match, frame[index], mSeparator);
				if (match == rtc::NUSM_longMatch || match == rtc::NUSM_shortMatch) {
					auto sequenceLength = match == rtc::NUSM_longMatch ? 4 : 3;
					size_t naluEndIndex = index - sequenceLength;
					match = rtc::NUSM_noMatch;
					auto begin = frame.begin() + naluStartIndex;
					auto end = frame.begin() + naluEndIndex + 1;
					nalus.emplace_back(begin, end);
					naluStartIndex = index + 1;
				}
				index++;
			}
			auto begin = frame.begin() + naluStartIndex;
			auto end = frame.end();
			nalus.emplace_back(begin, end);
		}
		return nalus;
	}

	void H264PrePacketizer::outgoing(rtc::message_vector &messages,
                             [[maybe_unused]] const rtc::message_callback &send) {
		rtc::message_vector result;
		for (const auto &message : messages) {
			if (const auto &frameInfo = message->frameInfo) {
				if (frameInfo->payloadType && frameInfo->payloadType != rtpConfig->payloadType)
					continue;

				if (frameInfo->timestampSeconds)
					rtpConfig->timestamp =
						rtpConfig->startTimestamp +
						rtpConfig->secondsToTimestamp(
							std::chrono::duration<double>(*frameInfo->timestampSeconds).count());
				else
					rtpConfig->timestamp = frameInfo->timestamp;
			}

			auto payloads = fragment(std::move(*message));
			if (payloads.size() > 0) {
				for (size_t i = 0; i < payloads.size() - 1; i++)
					result.push_back(packetize(payloads[i], false));

				result.push_back(packetize(payloads[payloads.size() - 1], true));
			}
		}

		messages.swap(result);
	}

} // namespace phntm
