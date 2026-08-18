#include <chrono>
#include <cstdint>
#include <memory>

#include <rtc/rtc.hpp>

int main() {
    rtc::Configuration configuration;
    auto peer = std::make_shared<rtc::PeerConnection>(configuration);

    rtc::Description::Video video("right_eye", rtc::Description::Direction::SendOnly);
    constexpr std::uint8_t kPayloadType = 102U;
    constexpr std::uint32_t kSsrc = 42U;
    video.addH264Codec(kPayloadType);
    video.addSSRC(kSsrc, "astrabot-video", "right_eye");
    auto track = peer->addTrack(video);
    auto packetization = std::make_shared<rtc::RtpPacketizationConfig>(kSsrc, "astrabot-video", kPayloadType,
                                                                       rtc::H264RtpPacketizer::ClockRate);
    auto packetizer = std::make_shared<rtc::H264RtpPacketizer>(rtc::NalUnit::Separator::StartSequence, packetization);
    track->setMediaHandler(packetizer);

    rtc::DataChannelInit channel_init;
    channel_init.reliability.unordered = true;
    channel_init.reliability.maxPacketLifeTime = std::chrono::milliseconds(20);
    auto channel = peer->createDataChannel("astrabot.rtc.tech-gate.v1", channel_init);
    const bool api_ready = track != nullptr && channel != nullptr;

    peer->resetCallbacks();
    peer->close();
    return api_ready ? 0 : 1;
}
