/*
*  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
*
*  Use of this source code is governed by a BSD-style license
*  that can be found in the LICENSE file in the root of the source
*  tree. An additional intellectual property rights grant can be found
*  in the file PATENTS.  All contributing project authors may
*  be found in the AUTHORS file in the root of the source tree.
*/

#include "third_party/winuwp_h264/H264Decoder/H264Decoder.h"

#include <Windows.h>
#include <stdlib.h>
#include <ppltasks.h>
#include <mfapi.h>
#include <robuffer.h>
#include <wrl.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <wrl\implements.h>
#include <iomanip>
#include "../Utils/Utils.h"
#include "libyuv/convert.h"
#include "rtc_base/logging.h"
#include "rtc_base/checks.h"
#include "common_video/include/video_frame_buffer.h"
#include "modules/video_coding/include/video_codec_interface.h"

#pragma comment(lib, "mfreadwrite")
#pragma comment(lib, "mfplat")
#pragma comment(lib, "mfuuid.lib")

namespace webrtc {

//////////////////////////////////////////
// H264 WinUWP Decoder Implementation
//////////////////////////////////////////

WinUWPH264DecoderImpl::WinUWPH264DecoderImpl()
  : width_(0),
  height_(0),
  decodeCompleteCallback_(nullptr) {
}

WinUWPH264DecoderImpl::~WinUWPH264DecoderImpl() {
  OutputDebugString(L"WinUWPH264DecoderImpl::~WinUWPH264DecoderImpl()\n");
  Release();
}

int WinUWPH264DecoderImpl::InitDecode(const VideoCodec* inst,
  int number_of_cores) {
  RTC_LOG(LS_INFO) << "WinUWPH264DecoderImpl::InitDecode()\n";
  RTC_LOG(LS_INFO) << "Kevin!\n";

  // TODO: may need to do Windows::Foundation::Initialize or CoInitializeEx first
  // TODO: assuming this is called only at begin streaming since it's probably expensive.
  HRESULT hr = CoInitializeEx(0, COINIT_APARTMENTTHREADED); // TODO: uninitialize when finished
  if (SUCCEEDED(hr))
    hr = MFStartup(MF_VERSION, 0);

  if (SUCCEEDED(hr))
    hr = CoCreateInstance(CLSID_MSH264DecoderMFT, nullptr, CLSCTX_INPROC_SERVER,
      IID_IUnknown, (void**)&m_spDecoder);

  if (FAILED(hr))
	  // TODO: log hr
    return WEBRTC_VIDEO_CODEC_ERROR;

  //**********************************************************************
  // Create input media type
  //**********************************************************************
  ULONG frameRateNumerator = 30; /* inst->maxFramerate ? */
  ULONG frameRateDenominator = 1;
  ULONG imageWidth = inst->width;
  ULONG imageHeight = inst->height;

  // TODO: these "SUCCEEDED" macros will all fall through if failure. Ensure this is expected paradigm.
  ComPtr<IMFMediaType> spInputMedia;
  hr = MFCreateMediaType(&spInputMedia);
  if (SUCCEEDED(hr))
    hr = spInputMedia->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
  if (SUCCEEDED(hr))
    hr = spInputMedia->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_H264);
  if (SUCCEEDED(hr))
    hr = MFSetAttributeRatio(spInputMedia.Get(), MF_MT_FRAME_RATE, frameRateNumerator,
                             frameRateDenominator);
  if (SUCCEEDED(hr))
    hr = MFSetAttributeSize(spInputMedia.Get(), MF_MT_FRAME_SIZE, imageWidth,
                            imageHeight);
  if (SUCCEEDED(hr))
    hr = MFSetAttributeRatio(spInputMedia.Get(), MF_MT_PIXEL_ASPECT_RATIO, 1, 1);

  // Register the input type with the decoder
  if (SUCCEEDED(hr))
    hr = m_spDecoder->SetInputType(0, spInputMedia.Get(), 0);

  //**********************************************************************
  // Create output media type
  //**********************************************************************

  // TODO: here, we query the list of supported output formats after providing the input stream above
  //       this is a temporary check. Remove this unless it's possible for h264 decoder impl to not support I420 on some devices.
  int type = 0;
  while (true) {
    ComPtr<IMFMediaType> spMediaType;
    hr = m_spDecoder->GetOutputAvailableType(0, type, &spMediaType);
    if (hr == MF_E_NO_MORE_TYPES)
      return E_FAIL;
    GUID mediatype;
    if (SUCCEEDED(hr)) {
      hr = spMediaType->GetGUID(MF_MT_SUBTYPE, &mediatype);
      if (SUCCEEDED(hr)) {
        if (mediatype == MFVideoFormat_I420) {
          break;
        }
      }
    }
    type++;
  }

  ComPtr<IMFMediaType> spOutputMedia;
  if (SUCCEEDED(hr))
    hr = MFCreateMediaType(&spOutputMedia);
  if (SUCCEEDED(hr))
    hr = spOutputMedia->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
  if (SUCCEEDED(hr))
    hr = spOutputMedia->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_I420);
  if (SUCCEEDED(hr))
    hr = MFSetAttributeRatio(spOutputMedia.Get(), MF_MT_FRAME_RATE,
                             frameRateNumerator, frameRateDenominator);
  if (SUCCEEDED(hr))
    hr = MFSetAttributeSize(spOutputMedia.Get(), MF_MT_FRAME_SIZE, imageWidth,
                            imageHeight);

  //**********************************************************************
  // Assign media types to decoder
  //**********************************************************************
  if (SUCCEEDED(hr))
    hr = m_spDecoder->SetOutputType(0, spOutputMedia.Get(), 0);

  DWORD status;
  if (SUCCEEDED(hr))
    hr = m_spDecoder->GetInputStatus(0, &status);

  if (SUCCEEDED(hr)) {
    if (MFT_INPUT_STATUS_ACCEPT_DATA != status)
      return WEBRTC_VIDEO_CODEC_ERROR;  // H.264 decoder MFT is not accepting
                                        // data
  }

  if (SUCCEEDED(hr))
    hr = m_spDecoder->ProcessMessage(MFT_MESSAGE_COMMAND_FLUSH, NULL);
  if (SUCCEEDED(hr))
    hr = m_spDecoder->ProcessMessage(MFT_MESSAGE_NOTIFY_BEGIN_STREAMING, NULL);
  if (SUCCEEDED(hr))
    hr = m_spDecoder->ProcessMessage(MFT_MESSAGE_NOTIFY_START_OF_STREAM, NULL);
  
  if (SUCCEEDED(hr))
    return WEBRTC_VIDEO_CODEC_OK;

  return WEBRTC_VIDEO_CODEC_ERROR;
}

ComPtr<IMFSample> FromEncodedImage(const EncodedImage& input_image) {
  HRESULT hr = S_OK;

  ComPtr<IMFSample> sample;
  hr = MFCreateSample(&sample);

  ComPtr<IMFMediaBuffer> mediaBuffer;
  ON_SUCCEEDED(MFCreateMemoryBuffer((DWORD)input_image._length, &mediaBuffer));

  BYTE* destBuffer = NULL;
  if (SUCCEEDED(hr)) {
    DWORD cbMaxLength;
    DWORD cbCurrentLength;
    hr = mediaBuffer->Lock(&destBuffer, &cbMaxLength, &cbCurrentLength);
  }

  if (SUCCEEDED(hr)) {
    memcpy(destBuffer, input_image._buffer, input_image._length);
  }

  ON_SUCCEEDED(mediaBuffer->SetCurrentLength((DWORD)input_image._length));
  ON_SUCCEEDED(mediaBuffer->Unlock());

  ON_SUCCEEDED(sample->AddBuffer(mediaBuffer.Get()));

  return sample;
}

// Used to store an encoded H264 sample in a VideoFrame
class H264NativeHandleBuffer : public NativeHandleBuffer {
public:
  H264NativeHandleBuffer(ComPtr<IMFSample> sample, int width, int height)
    : NativeHandleBuffer(sample.Get(), width, height)
    , _sample(sample) {
  }

  virtual ~H264NativeHandleBuffer() {
  }

  rtc::scoped_refptr<I420BufferInterface> ToI420() override {
    return nullptr;
  }

private:
  ComPtr<IMFSample> _sample;
};

int WinUWPH264DecoderImpl::FlushFrames() {

  HRESULT hr;
  DWORD outputStatus;

  while (SUCCEEDED(hr = m_spDecoder->GetOutputStatus(&outputStatus)) &&
         outputStatus == MFT_OUTPUT_STATUS_SAMPLE_READY)
  {
    //**********************************************************************
    // Get needed size of our output buffer
    //**********************************************************************
    MFT_OUTPUT_STREAM_INFO strmInfo;
    HRESULT hr = m_spDecoder->GetOutputStreamInfo(0, &strmInfo);
    if (FAILED(hr))
      return WEBRTC_VIDEO_CODEC_ERROR;
    _ASSERT(!(strmInfo.dwFlags & MFT_OUTPUT_STREAM_PROVIDES_SAMPLES));
    _ASSERT(!(strmInfo.dwFlags & MFT_OUTPUT_STREAM_CAN_PROVIDE_SAMPLES));

    //**********************************************************************
    // Create output sample
    //**********************************************************************
    // TODO: can we avoid allocating buffers by reusing those of the same size?
    // Likely only if buffer lifetime ends after ProcessOutput returns.
    ComPtr<IMFMediaBuffer> spOutBuffer;
    hr = MFCreateMemoryBuffer(strmInfo.cbSize, &spOutBuffer);
    if (FAILED(hr))
      return WEBRTC_VIDEO_CODEC_ERROR;

    ComPtr<IMFSample> spOutSample;
    hr = MFCreateSample(&spOutSample);
    if (FAILED(hr))
      return WEBRTC_VIDEO_CODEC_ERROR;

    hr = spOutSample->AddBuffer(spOutBuffer.Get());
    if (FAILED(hr))
      return WEBRTC_VIDEO_CODEC_ERROR;

    //**********************************************************************
    // Create output buffer
    //**********************************************************************
    MFT_OUTPUT_DATA_BUFFER outputDataBuffer;
    outputDataBuffer.dwStatus = 0;
    outputDataBuffer.dwStreamID = 0;
    outputDataBuffer.pEvents = nullptr;
    outputDataBuffer.pSample = spOutSample.Get();

	// Invoke the Media Foundation decoder
    DWORD status;
    hr = m_spDecoder->ProcessOutput(0, 1, &outputDataBuffer, &status);

	// TODO: if we need to detect stream format changes, read MF_E_TRANSFORM_STREAM_CHANGE here and adjust
    if (status == MF_E_TRANSFORM_NEED_MORE_INPUT) {
	  // Ideally, this won't happen (since that'd mean we allocated buffers unnecessarily as per GetOutputStatus())
	  // but there's nothing wrong (we're just out of data), so we return OK.
      return WEBRTC_VIDEO_CODEC_OK;
	}

	// TODO: we need to unpack decoded frame and provide it to the webrtc callback
  }

  if (FAILED(hr))
    return WEBRTC_VIDEO_CODEC_ERROR;
}

int WinUWPH264DecoderImpl::Decode(const EncodedImage& input_image,
  bool missing_frames,
  const CodecSpecificInfo* codec_specific_info,
  int64_t render_time_ms) {

  DWORD inputStatus;
  HRESULT hr = m_spDecoder->GetInputStatus(0, &inputStatus);

  if (FAILED(hr)) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (inputStatus != MFT_INPUT_STATUS_ACCEPT_DATA) {
	// Process output until the stream can accept the new frame.
    int res = FlushFrames();
    if (res != WEBRTC_VIDEO_CODEC_OK)
      return res;
  }

  // TODO: replace below
  auto sample = FromEncodedImage(input_image);

  if (sample != nullptr) {
    rtc::scoped_refptr<VideoFrameBuffer> buffer(new rtc::RefCountedObject<H264NativeHandleBuffer>(
      sample, width_, height_));
    VideoFrame decodedFrame(buffer, input_image.Timestamp(), render_time_ms, kVideoRotation_0);
    decodedFrame.set_ntp_time_ms(input_image.ntp_time_ms_);



    rtc::CritScope lock(&crit_);

    if (decodeCompleteCallback_ != nullptr) {
      decodeCompleteCallback_->Decoded(decodedFrame);
    }
  }
  return WEBRTC_VIDEO_CODEC_OK;
}

int WinUWPH264DecoderImpl::RegisterDecodeCompleteCallback(
  DecodedImageCallback* callback) {
  rtc::CritScope lock(&crit_);
  decodeCompleteCallback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int WinUWPH264DecoderImpl::Release() {
	// TODO: uninit com and media foundation stuff
  OutputDebugString(L"WinUWPH264DecoderImpl::Release()\n");
  return WEBRTC_VIDEO_CODEC_OK;
}

void WinUWPH264DecoderImpl::UpdateVideoFrameDimensions(const EncodedImage& input_image)
{
  auto w = input_image._encodedWidth;
  auto h = input_image._encodedHeight;

  if (input_image._frameType == FrameType::kVideoFrameKey && w > 0 && h > 0)
  {
    width_ = w;
    height_ = h;
  }
}

const char* WinUWPH264DecoderImpl::ImplementationName() const {
  return "H264_MediaFoundation";
}

}  // namespace webrtc
