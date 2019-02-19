/*
*  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
*
*  Use of this source code is governed by a BSD-style license
*  that can be found in the LICENSE file in the root of the source
*  tree. An additional intellectual property rights grant can be found
*  in the file PATENTS.  All contributing project authors may
*  be found in the AUTHORS file in the root of the source tree.
*/

#include <vector>
#include <utility>

#include "api/video_codecs/sdp_video_format.h"
#include "media/base/h264_profile_level_id.h"
#include "modules/video_coding/codecs/vp8/include/vp8.h"
#include "modules/video_coding/codecs/vp9/include/vp9.h"
#include "modules/video_coding/codecs/h264/include/h264.h"
#include "rtc_base/logging.h"
#include "third_party/winuwp_h264/H264Encoder/H264Encoder.h"
#include "third_party/winuwp_h264/H264Decoder/H264Decoder.h"
#include "third_party/winuwp_h264/winuwp_h264_factory.h"


namespace webrtc {

namespace {

SdpVideoFormat CreateH264Format(H264::Profile profile,
                                H264::Level level,
                                const std::string& packetization_mode) {
  const absl::optional<std::string> profile_string =
      H264::ProfileLevelIdToString(H264::ProfileLevelId(profile, level));
  RTC_CHECK(profile_string);
  return SdpVideoFormat(
      cricket::kH264CodecName,
      {{cricket::kH264FmtpProfileLevelId, *profile_string},
       {cricket::kH264FmtpLevelAsymmetryAllowed, "1"},
       {cricket::kH264FmtpPacketizationMode, packetization_mode}});
}

std::vector<SdpVideoFormat> SupportedFormats() {
  std::vector<SdpVideoFormat> supported_codecs;
  supported_codecs.push_back(SdpVideoFormat(cricket::kVp8CodecName));
  for (const webrtc::SdpVideoFormat& format : webrtc::SupportedVP9Codecs())
    supported_codecs.push_back(format);
  supported_codecs.push_back(
      CreateH264Format(H264::kProfileBaseline, H264::kLevel3_1, "1"));
  supported_codecs.push_back(
      CreateH264Format(H264::kProfileBaseline, H264::kLevel3_1, "0"));
  supported_codecs.push_back(CreateH264Format(H264::kProfileConstrainedBaseline,
                                              H264::kLevel3_1, "1"));
  supported_codecs.push_back(CreateH264Format(H264::kProfileConstrainedBaseline,
                                              H264::kLevel3_1, "0"));
  return supported_codecs;
}

}  // namespace

std::vector<SdpVideoFormat> WinUWPH264EncoderFactory::GetSupportedFormats()
    const {
  return SupportedFormats();
}

VideoEncoderFactory::CodecInfo WinUWPH264EncoderFactory::QueryVideoEncoder(
    const SdpVideoFormat& format) const {
  CodecInfo info;
  info.is_hardware_accelerated = false;
  info.has_internal_source = false;
  return info;
}

std::unique_ptr<VideoEncoder> WinUWPH264EncoderFactory::CreateVideoEncoder(
    const SdpVideoFormat& format) {
  if (cricket::CodecNamesEq(format.name, cricket::kVp8CodecName))
    return VP8Encoder::Create();
  if (cricket::CodecNamesEq(format.name, cricket::kVp9CodecName))
    return VP9Encoder::Create(cricket::VideoCodec(format));
  if (cricket::CodecNamesEq(format.name, cricket::kH264CodecName))
    return std::make_unique<WinUWPH264EncoderImpl>();
  RTC_LOG(LS_ERROR) << "Trying to created encoder of unsupported format "
                    << format.name;
  return nullptr;
}

std::vector<SdpVideoFormat> WinUWPH264DecoderFactory::GetSupportedFormats()
    const {
  return SupportedFormats();
}

std::unique_ptr<VideoDecoder> WinUWPH264DecoderFactory::CreateVideoDecoder(
    const SdpVideoFormat& format) {
  if (cricket::CodecNamesEq(format.name, cricket::kVp8CodecName))
    return VP8Decoder::Create();
  if (cricket::CodecNamesEq(format.name, cricket::kVp9CodecName))
    return VP9Decoder::Create();
  if (cricket::CodecNamesEq(format.name, cricket::kH264CodecName))
    return std::make_unique<WinUWPH264DecoderImpl>();

  RTC_NOTREACHED();
  return nullptr;
}

}  // namespace webrtc
