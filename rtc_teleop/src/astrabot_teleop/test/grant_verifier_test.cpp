// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "astrabot_teleop/grant/grant_verifier.h"
#include "astrabot_teleop/session/session_registry.h"

#if ASTRABOT_TELEOP_HAS_OPENSSL
#include <openssl/evp.h>
#endif

namespace astrabot::teleop {
namespace {

#if ASTRABOT_TELEOP_HAS_OPENSSL
struct PkeyDeleter {
    void operator()(EVP_PKEY *key) const {
        EVP_PKEY_free(key);
    }
};

struct PkeyContextDeleter {
    void operator()(EVP_PKEY_CTX *context) const {
        EVP_PKEY_CTX_free(context);
    }
};

struct MdContextDeleter {
    void operator()(EVP_MD_CTX *context) const {
        EVP_MD_CTX_free(context);
    }
};

std::string base64UrlEncode(const std::uint8_t *data, const std::size_t size) {
    static constexpr char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
    std::string encoded;
    encoded.reserve((size * 4U + 2U) / 3U);
    std::uint32_t accumulator = 0;
    int bits = 0;
    for (std::size_t index = 0; index < size; ++index) {
        accumulator = (accumulator << 8U) | data[index];
        bits += 8;
        while (bits >= 6) {
            bits -= 6;
            encoded.push_back(kAlphabet[(accumulator >> static_cast<unsigned int>(bits)) & 0x3FU]);
        }
    }
    if (bits > 0) {
        encoded.push_back(kAlphabet[(accumulator << static_cast<unsigned int>(6 - bits)) & 0x3FU]);
    }
    return encoded;
}

std::string standardBase64Encode(const std::uint8_t *data, const std::size_t size) {
    static constexpr char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string encoded;
    encoded.reserve(((size + 2U) / 3U) * 4U);
    for (std::size_t index = 0; index < size; index += 3U) {
        const std::uint32_t first = data[index];
        const std::uint32_t second = index + 1U < size ? data[index + 1U] : 0U;
        const std::uint32_t third = index + 2U < size ? data[index + 2U] : 0U;
        const std::uint32_t combined = (first << 16U) | (second << 8U) | third;
        encoded.push_back(kAlphabet[(combined >> 18U) & 0x3FU]);
        encoded.push_back(kAlphabet[(combined >> 12U) & 0x3FU]);
        encoded.push_back(index + 1U < size ? kAlphabet[(combined >> 6U) & 0x3FU] : '=');
        encoded.push_back(index + 2U < size ? kAlphabet[combined & 0x3FU] : '=');
    }
    return encoded;
}

class TestSigner {
  public:
    static std::unique_ptr<TestSigner> create() {
        std::unique_ptr<EVP_PKEY_CTX, PkeyContextDeleter> context(EVP_PKEY_CTX_new_id(EVP_PKEY_ED25519, nullptr));
        if (!context || EVP_PKEY_keygen_init(context.get()) != 1) {
            return nullptr;
        }
        EVP_PKEY *raw_key = nullptr;
        if (EVP_PKEY_keygen(context.get(), &raw_key) != 1 || raw_key == nullptr) {
            return nullptr;
        }
        return std::unique_ptr<TestSigner>(new TestSigner(raw_key));
    }

    std::string publicKeyBase64() const {
        std::vector<std::uint8_t> public_key(32U);
        std::size_t size = public_key.size();
        if (EVP_PKEY_get_raw_public_key(key_.get(), public_key.data(), &size) != 1) {
            return {};
        }
        public_key.resize(size);
        return standardBase64Encode(public_key.data(), public_key.size());
    }

    std::string signToken(const std::string &payload_json) const {
        const std::string payload_segment =
            base64UrlEncode(reinterpret_cast<const std::uint8_t *>(payload_json.data()), payload_json.size());
        std::unique_ptr<EVP_MD_CTX, MdContextDeleter> context(EVP_MD_CTX_new());
        if (!context || EVP_DigestSignInit(context.get(), nullptr, nullptr, nullptr, key_.get()) != 1) {
            return {};
        }
        std::vector<std::uint8_t> signature(64U);
        std::size_t signature_size = signature.size();
        if (EVP_DigestSign(context.get(), signature.data(), &signature_size,
                           reinterpret_cast<const std::uint8_t *>(payload_json.data()), payload_json.size()) != 1) {
            return {};
        }
        signature.resize(signature_size);
        return payload_segment + "." + base64UrlEncode(signature.data(), signature.size());
    }

  private:
    explicit TestSigner(EVP_PKEY *key) : key_(key) {}
    std::unique_ptr<EVP_PKEY, PkeyDeleter> key_;
};

std::string payload(const std::string &key_id, const std::string &nonce, const std::uint64_t expires_at) {
    return "{\"version\":1,\"key_id\":\"" + key_id +
           "\",\"session_id\":\"session-1\",\"run_id\":\"run-1\",\"user_id\":\"user-1\","
           "\"device_id\":\"robot-001\",\"resource_id\":\"thor\",\"expires_at\":" +
           std::to_string(expires_at) + ",\"nonce\":\"" + nonce + "\"}";
}

std::string payloadWithoutRun(const std::string &key_id, const std::string &nonce, const std::uint64_t expires_at) {
    return "{\"version\":1,\"key_id\":\"" + key_id +
           "\",\"session_id\":\"session-1\",\"run_id\":\"\",\"user_id\":\"user-1\","
           "\"device_id\":\"robot-001\",\"resource_id\":\"thor\",\"expires_at\":" +
           std::to_string(expires_at) + ",\"nonce\":\"" + nonce + "\"}";
}

TEST(GrantVerifierTest, RejectsEverySecondConsumptionOfTheSameNonce) {
    const auto signer = TestSigner::create();
    ASSERT_NE(signer, nullptr);
    GrantVerifier verifier;
    ASSERT_TRUE(verifier.initialize({GrantPublicKeyConfig{"key-1", signer->publicKeyBase64()}}).ok());
    const ExpectedGrantBinding expected{"session-1", "run-1", "robot-001", "thor"};
    const std::string token = signer->signToken(payload("key-1", "nonce-1", 1060U));
    auto verified = verifier.verifyAndConsume(token, expected, 1000U);
    ASSERT_TRUE(verified.ok()) << verified.status().message();
    EXPECT_EQ(verified.value().key_id, "key-1");
    EXPECT_EQ(verifier.verifyAndConsume(token, expected, 1000U).status().code(), ErrorCode::kConflict);
    const std::string nonce_collision = signer->signToken(payload("key-1", "nonce-1", 1070U));
    EXPECT_EQ(verifier.verifyAndConsume(nonce_collision, expected, 1000U).status().code(), ErrorCode::kConflict);
}

TEST(GrantVerifierTest, AcceptsEmptyRunClaimOnlyWhenExpectedBindingIsAlsoEmpty) {
    const auto signer = TestSigner::create();
    ASSERT_NE(signer, nullptr);
    GrantVerifier verifier;
    ASSERT_TRUE(verifier.initialize({GrantPublicKeyConfig{"key-1", signer->publicKeyBase64()}}).ok());
    const std::string token = signer->signToken(payloadWithoutRun("key-1", "nonce-runless", 1060U));
    const ExpectedGrantBinding runless{"session-1", "", "robot-001", "thor"};
    auto verified = verifier.verifyAndConsume(token, runless, 1000U);
    ASSERT_TRUE(verified.ok()) << verified.status().message();
    EXPECT_TRUE(verified.value().run_id.empty());

    GrantVerifier mismatched_verifier;
    ASSERT_TRUE(mismatched_verifier.initialize({GrantPublicKeyConfig{"key-1", signer->publicKeyBase64()}}).ok());
    const ExpectedGrantBinding bound{"session-1", "run-1", "robot-001", "thor"};
    EXPECT_EQ(
        mismatched_verifier
            .verifyAndConsume(signer->signToken(payloadWithoutRun("key-1", "nonce-mismatch", 1060U)), bound, 1000U)
            .status()
            .code(),
        ErrorCode::kUnauthorized);
}

TEST(GrantVerifierTest, ExpiredConsumedNonceCannotBeReissuedWithLaterExpiry) {
    const auto signer = TestSigner::create();
    ASSERT_NE(signer, nullptr);
    GrantVerifier verifier;
    ASSERT_TRUE(verifier.initialize({GrantPublicKeyConfig{"key-1", signer->publicKeyBase64()}}).ok());
    const ExpectedGrantBinding expected{"session-1", "run-1", "robot-001", "thor"};
    ASSERT_TRUE(
        verifier.verifyAndConsume(signer->signToken(payload("key-1", "nonce-never-forget", 1060U)), expected, 1000U)
            .ok());

    const std::string reissued = signer->signToken(payload("key-1", "nonce-never-forget", 1120U));
    EXPECT_EQ(verifier.verifyAndConsume(reissued, expected, 1061U).status().code(), ErrorCode::kConflict);
}

TEST(GrantVerifierTest, FullReplayCacheFailsClosedWithoutEviction) {
    const auto signer = TestSigner::create();
    ASSERT_NE(signer, nullptr);
    GrantVerifier verifier(1U);
    ASSERT_TRUE(verifier.initialize({GrantPublicKeyConfig{"key-1", signer->publicKeyBase64()}}).ok());
    const ExpectedGrantBinding expected{"session-1", "run-1", "robot-001", "thor"};
    ASSERT_TRUE(
        verifier.verifyAndConsume(signer->signToken(payload("key-1", "nonce-first", 1060U)), expected, 1000U).ok());

    EXPECT_EQ(verifier.verifyAndConsume(signer->signToken(payload("key-1", "nonce-second", 1120U)), expected, 1061U)
                  .status()
                  .code(),
              ErrorCode::kResourceExhausted);
}

TEST(GrantVerifierTest, ConsumedGrantCannotBeReusedAfterWriterSessionCloses) {
    const auto signer = TestSigner::create();
    ASSERT_NE(signer, nullptr);
    GrantVerifier verifier;
    ASSERT_TRUE(verifier.initialize({GrantPublicKeyConfig{"key-1", signer->publicKeyBase64()}}).ok());
    const ExpectedGrantBinding expected{"session-1", "run-1", "robot-001", "thor"};
    const std::string token = signer->signToken(payload("key-1", "nonce-closed-session", 1060U));
    ASSERT_TRUE(verifier.verifyAndConsume(token, expected, 1000U).ok());
    const auto fingerprint = GrantVerifier::tokenFingerprint(token);
    ASSERT_TRUE(fingerprint.ok());

    SessionRegistry registry;
    ASSERT_TRUE(registry
                    .authorize(SessionBinding{"session-1", "peer-1", "run-1", "thor", "astrabot.teleop", 1000U,
                                              fingerprint.value(), false},
                               false)
                    .ok());
    ASSERT_TRUE(registry.close("session-1").ok());

    EXPECT_EQ(verifier.verifyAndConsume(token, expected, 1000U).status().code(), ErrorCode::kConflict);
}

TEST(GrantVerifierTest, RejectsExpiredUnknownKeyAndBindingMismatch) {
    const auto signer = TestSigner::create();
    ASSERT_NE(signer, nullptr);
    GrantVerifier verifier;
    ASSERT_TRUE(verifier.initialize({GrantPublicKeyConfig{"key-1", signer->publicKeyBase64()}}).ok());
    const ExpectedGrantBinding expected{"session-1", "run-1", "robot-001", "thor"};
    EXPECT_EQ(verifier.verifyAndConsume(signer->signToken(payload("key-1", "nonce-expired", 999U)), expected, 1000U)
                  .status()
                  .code(),
              ErrorCode::kDeadlineExceeded);
    EXPECT_EQ(verifier.verifyAndConsume(signer->signToken(payload("unknown", "nonce-unknown", 1060U)), expected, 1000U)
                  .status()
                  .code(),
              ErrorCode::kUnauthorized);
    const ExpectedGrantBinding wrong{"session-2", "run-1", "robot-001", "thor"};
    EXPECT_EQ(verifier.verifyAndConsume(signer->signToken(payload("key-1", "nonce-wrong", 1060U)), wrong, 1000U)
                  .status()
                  .code(),
              ErrorCode::kUnauthorized);
}
#else
TEST(GrantVerifierTest, FailsClosedWithoutOpenSsl) {
    GrantVerifier verifier;
    EXPECT_EQ(verifier.initialize({GrantPublicKeyConfig{"key-1", "unused"}}).code(), ErrorCode::kUnavailable);
    EXPECT_EQ(verifier.verifyAndConsume("payload.signature", ExpectedGrantBinding{}, 1000U).status().code(),
              ErrorCode::kUnavailable);
}
#endif

}  // namespace
}  // namespace astrabot::teleop
