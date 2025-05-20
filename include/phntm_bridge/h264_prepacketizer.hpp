/**
 * Copyright (c) 2020 Filip Klembara (in2core)
 * Copyright (c) 2023 Paul-Louis Ageneau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "rtc/nalunit.hpp"
#include "rtc/rtppacketizer.hpp"

namespace phntm {

/// RTP packetization for H264
class H264PrePacketizer final : public rtc::RtpPacketizer {
public:
	using Separator = rtc::NalUnit::Separator;

	inline static const uint32_t ClockRate = VideoClockRate;
	void outgoing(rtc::message_vector &messages, const rtc::message_callback &send) override;
	
	std::vector<rtc::binary> fragment(rtc::binary data) override;

	/// Constructs h264 payload packetizer with given RTP configuration.
	/// @note RTP configuration is used in packetization process which may change some configuration
	/// properties such as sequence number.
	/// @param separator NAL unit separator
	/// @param rtpConfig RTP configuration
	/// @param maxFragmentSize maximum size of one NALU fragment
	H264PrePacketizer(Separator separator, std::shared_ptr<rtc::RtpPacketizationConfig> rtpConfig,
	                  size_t maxFragmentSize = DefaultMaxFragmentSize);

private:
	std::vector<rtc::NalUnit> splitFrame(const rtc::binary &frame);

	const Separator mSeparator;
	const size_t mMaxFragmentSize;
};
} // namespace phntm

