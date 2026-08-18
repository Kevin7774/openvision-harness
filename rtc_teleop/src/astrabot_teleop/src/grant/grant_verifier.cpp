// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/grant/grant_verifier.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#if ASTRABOT_TELEOP_HAS_OPENSSL
#include <openssl/bio.h>
#include <openssl/evp.h>
#include <openssl/pem.h>
#endif

namespace astrabot::teleop {
namespace {

constexpr std::size_t kEd25519PublicKeyBytes = 32;
constexpr std::size_t kEd25519SignatureBytes = 64;
constexpr std::size_t kMaxClaimStringBytes = 256;
constexpr std::size_t kMaxNonceBytes = 128;
constexpr std::size_t kMaxGrantTokenBytes = 64U * 1024U;

Result<std::vector<std::uint8_t>> decodeBase64Url(const std::string_view input) {
    if (input.empty() || input.size() % 4U == 1U) {
        return Result<std::vector<std::uint8_t>>::failure(
            Status::error(ErrorCode::kInvalidArgument, "grant contains invalid base64url"));
    }

    std::vector<std::uint8_t> output;
    output.reserve((input.size() * 3U) / 4U + 2U);
    std::uint32_t accumulator = 0;
    int bit_count = 0;
    for (const char character : input) {
        int value = -1;
        if (character >= 'A' && character <= 'Z') {
            value = character - 'A';
        } else if (character >= 'a' && character <= 'z') {
            value = character - 'a' + 26;
        } else if (character >= '0' && character <= '9') {
            value = character - '0' + 52;
        } else if (character == '-') {
            value = 62;
        } else if (character == '_') {
            value = 63;
        }
        if (value < 0) {
            return Result<std::vector<std::uint8_t>>::failure(
                Status::error(ErrorCode::kInvalidArgument, "grant contains invalid base64url character"));
        }
        accumulator = (accumulator << 6U) | static_cast<std::uint32_t>(value);
        bit_count += 6;
        if (bit_count >= 8) {
            bit_count -= 8;
            output.push_back(static_cast<std::uint8_t>((accumulator >> static_cast<unsigned int>(bit_count)) & 0xFFU));
        }
    }
    if (bit_count > 0 && (accumulator & ((1U << static_cast<unsigned int>(bit_count)) - 1U)) != 0U) {
        return Result<std::vector<std::uint8_t>>::failure(
            Status::error(ErrorCode::kInvalidArgument, "grant base64url has non-zero trailing bits"));
    }
    return Result<std::vector<std::uint8_t>>::success(std::move(output));
}

bool readStringClaim(const nlohmann::json &json, const char *name, std::string *value, const std::size_t max_size) {
    const auto iterator = json.find(name);
    if (iterator == json.end() || !iterator->is_string()) {
        return false;
    }
    const auto *string_value = iterator->get_ptr<const nlohmann::json::string_t *>();
    if (string_value == nullptr || string_value->empty() || string_value->size() > max_size) {
        return false;
    }
    *value = *string_value;
    return true;
}

bool readRunClaim(const nlohmann::json &json, std::string *value) {
    const auto iterator = json.find("run_id");
    if (iterator == json.end() || !iterator->is_string()) {
        return false;
    }
    const auto *string_value = iterator->get_ptr<const nlohmann::json::string_t *>();
    if (string_value == nullptr || string_value->size() > kMaxClaimStringBytes) {
        return false;
    }
    *value = *string_value;
    return true;
}

bool readUnsignedClaim(const nlohmann::json &json, const char *name, std::uint64_t *value) {
    const auto iterator = json.find(name);
    if (iterator == json.end()) {
        return false;
    }
    if (iterator->is_number_unsigned()) {
        const auto *number = iterator->get_ptr<const nlohmann::json::number_unsigned_t *>();
        if (number == nullptr) {
            return false;
        }
        *value = *number;
        return true;
    }
    if (iterator->is_number_integer()) {
        const auto *number = iterator->get_ptr<const nlohmann::json::number_integer_t *>();
        if (number == nullptr || *number < 0) {
            return false;
        }
        *value = static_cast<std::uint64_t>(*number);
        return true;
    }
    return false;
}

Result<GrantClaims> parseClaims(const std::vector<std::uint8_t> &payload) {
    const auto json = nlohmann::json::parse(payload.begin(), payload.end(), nullptr, false);
    if (json.is_discarded() || !json.is_object()) {
        return Result<GrantClaims>::failure(
            Status::error(ErrorCode::kInvalidArgument, "grant payload JSON is invalid"));
    }

    GrantClaims claims;
    std::uint64_t version = 0;
    if (!readUnsignedClaim(json, "version", &version) || version != 1U) {
        return Result<GrantClaims>::failure(Status::error(ErrorCode::kInvalidArgument, "grant version is unsupported"));
    }
    claims.version = static_cast<std::uint32_t>(version);
    if (!readStringClaim(json, "key_id", &claims.key_id, kMaxClaimStringBytes) ||
        !readStringClaim(json, "session_id", &claims.session_id, kMaxClaimStringBytes) ||
        !readRunClaim(json, &claims.run_id) ||
        !readStringClaim(json, "user_id", &claims.user_id, kMaxClaimStringBytes) ||
        !readStringClaim(json, "device_id", &claims.device_id, kMaxClaimStringBytes) ||
        !readStringClaim(json, "resource_id", &claims.resource_id, kMaxClaimStringBytes) ||
        !readUnsignedClaim(json, "expires_at", &claims.expires_at_epoch_sec) ||
        !readStringClaim(json, "nonce", &claims.nonce, kMaxNonceBytes)) {
        return Result<GrantClaims>::failure(
            Status::error(ErrorCode::kInvalidArgument, "grant payload is missing a required claim"));
    }
    return Result<GrantClaims>::success(std::move(claims));
}

#if ASTRABOT_TELEOP_HAS_OPENSSL
Result<std::array<std::uint8_t, 32>> tokenSha256(const std::string &token) {
    if (token.empty() || token.size() > kMaxGrantTokenBytes) {
        return Result<std::array<std::uint8_t, 32>>::failure(
            Status::error(ErrorCode::kInvalidArgument, "grant token size is invalid"));
    }
    std::array<std::uint8_t, 32> digest{};
    unsigned int digest_size = 0U;
    if (EVP_Digest(token.data(), token.size(), digest.data(), &digest_size, EVP_sha256(), nullptr) != 1 ||
        digest_size != digest.size()) {
        return Result<std::array<std::uint8_t, 32>>::failure(
            Status::error(ErrorCode::kUnavailable, "failed to hash verified grant"));
    }
    return Result<std::array<std::uint8_t, 32>>::success(digest);
}

std::string digestHex(const std::array<std::uint8_t, 32> &digest) {
    static constexpr char kHex[] = "0123456789abcdef";
    std::string output;
    output.reserve(digest.size() * 2U);
    for (const std::uint8_t byte : digest) {
        output.push_back(kHex[(byte >> 4U) & 0x0FU]);
        output.push_back(kHex[byte & 0x0FU]);
    }
    return output;
}

struct PkeyDeleter {
    void operator()(EVP_PKEY *key) const {
        EVP_PKEY_free(key);
    }
};

struct BioDeleter {
    void operator()(BIO *bio) const {
        BIO_free(bio);
    }
};

struct MdContextDeleter {
    void operator()(EVP_MD_CTX *context) const {
        EVP_MD_CTX_free(context);
    }
};

using UniquePkey = std::unique_ptr<EVP_PKEY, PkeyDeleter>;

Result<std::vector<std::uint8_t>> decodeStandardBase64(std::string input) {
    input.erase(std::remove_if(input.begin(), input.end(),
                               [](const unsigned char character) { return std::isspace(character) != 0; }),
                input.end());
    if (input.empty() || input.size() % 4U != 0U) {
        return Result<std::vector<std::uint8_t>>::failure(
            Status::error(ErrorCode::kInvalidArgument, "grant public key base64 is invalid"));
    }
    std::vector<std::uint8_t> decoded((input.size() / 4U) * 3U);
    const int decoded_size = EVP_DecodeBlock(decoded.data(), reinterpret_cast<const unsigned char *>(input.data()),
                                             static_cast<int>(input.size()));
    if (decoded_size < 0) {
        return Result<std::vector<std::uint8_t>>::failure(
            Status::error(ErrorCode::kInvalidArgument, "grant public key base64 decode failed"));
    }
    std::size_t padding = 0;
    if (!input.empty() && input.back() == '=') {
        ++padding;
    }
    if (input.size() > 1U && input[input.size() - 2U] == '=') {
        ++padding;
    }
    decoded.resize(static_cast<std::size_t>(decoded_size) - padding);
    return Result<std::vector<std::uint8_t>>::success(std::move(decoded));
}

Result<UniquePkey> parsePublicKey(const std::string &encoded) {
    if (encoded.find("-----BEGIN PUBLIC KEY-----") != std::string::npos) {
        std::unique_ptr<BIO, BioDeleter> bio(BIO_new_mem_buf(encoded.data(), static_cast<int>(encoded.size())));
        if (!bio) {
            return Result<UniquePkey>::failure(
                Status::error(ErrorCode::kUnavailable, "failed to allocate public key parser"));
        }
        UniquePkey key(PEM_read_bio_PUBKEY(bio.get(), nullptr, nullptr, nullptr));
        if (!key || EVP_PKEY_base_id(key.get()) != EVP_PKEY_ED25519) {
            return Result<UniquePkey>::failure(
                Status::error(ErrorCode::kInvalidArgument, "grant public key PEM is not Ed25519"));
        }
        return Result<UniquePkey>::success(std::move(key));
    }

    std::string base64 = encoded;
    constexpr std::string_view kPrefix = "base64:";
    if (base64.compare(0, kPrefix.size(), kPrefix) == 0) {
        base64.erase(0, kPrefix.size());
    }
    auto decoded = decodeStandardBase64(std::move(base64));
    if (!decoded.ok() || decoded.value().size() != kEd25519PublicKeyBytes) {
        return Result<UniquePkey>::failure(
            Status::error(ErrorCode::kInvalidArgument, "grant raw public key must contain 32 Ed25519 bytes"));
    }
    UniquePkey key(
        EVP_PKEY_new_raw_public_key(EVP_PKEY_ED25519, nullptr, decoded.value().data(), decoded.value().size()));
    if (!key) {
        return Result<UniquePkey>::failure(
            Status::error(ErrorCode::kUnavailable, "failed to construct Ed25519 public key"));
    }
    return Result<UniquePkey>::success(std::move(key));
}
#endif

}  // namespace

class GrantVerifier::Impl {
  public:
    explicit Impl(const std::size_t max_nonce_entries) : max_nonce_entries_(max_nonce_entries) {}

#if ASTRABOT_TELEOP_HAS_OPENSSL
    std::unordered_map<std::string, UniquePkey> public_keys_;
#endif
    std::size_t max_nonce_entries_{0};
    std::mutex nonce_mutex_;
#if ASTRABOT_TELEOP_HAS_OPENSSL
    std::unordered_set<std::string> consumed_nonces_;
#endif
    bool initialized_{false};
};

GrantVerifier::GrantVerifier(const std::size_t max_nonce_entries) : impl_(std::make_unique<Impl>(max_nonce_entries)) {}

GrantVerifier::~GrantVerifier() = default;

Result<std::string> GrantVerifier::tokenFingerprint(const std::string &token) {
#if !ASTRABOT_TELEOP_HAS_OPENSSL
    static_cast<void>(token);
    return Result<std::string>::failure(
        Status::error(ErrorCode::kUnavailable, "OpenSSL SHA-256 support is unavailable"));
#else
    auto digest = tokenSha256(token);
    if (!digest.ok()) {
        return Result<std::string>::failure(digest.status());
    }
    return Result<std::string>::success(digestHex(digest.value()));
#endif
}

Status GrantVerifier::initialize(const std::vector<GrantPublicKeyConfig> &public_keys) {
    impl_->initialized_ = false;
#if !ASTRABOT_TELEOP_HAS_OPENSSL
    static_cast<void>(public_keys);
    return Status::error(ErrorCode::kUnavailable, "OpenSSL Ed25519 support is unavailable; grants fail closed");
#else
    if (public_keys.empty()) {
        return Status::error(ErrorCode::kUnavailable, "no Teleop grant public key is configured");
    }

    std::unordered_map<std::string, UniquePkey> parsed_keys;
    for (const auto &config : public_keys) {
        if (config.key_id.empty() || config.public_key.empty()) {
            return Status::error(ErrorCode::kInvalidArgument, "grant public key configuration is incomplete");
        }
        if (parsed_keys.find(config.key_id) != parsed_keys.end()) {
            return Status::error(ErrorCode::kConflict, "duplicate Teleop grant key id");
        }
        auto parsed = parsePublicKey(config.public_key);
        if (!parsed.ok()) {
            return parsed.status();
        }
        parsed_keys.emplace(config.key_id, parsed.takeValue());
    }
    impl_->public_keys_ = std::move(parsed_keys);
    impl_->initialized_ = true;
    return Status::success();
#endif
}

Result<GrantClaims> GrantVerifier::verifyAndConsume(const std::string &token, const ExpectedGrantBinding &expected,
                                                    const std::uint64_t system_now_sec) {
#if !ASTRABOT_TELEOP_HAS_OPENSSL
    static_cast<void>(token);
    static_cast<void>(expected);
    static_cast<void>(system_now_sec);
    return Result<GrantClaims>::failure(
        Status::error(ErrorCode::kUnavailable, "OpenSSL Ed25519 support is unavailable; grant rejected"));
#else
    if (!impl_->initialized_) {
        return Result<GrantClaims>::failure(
            Status::error(ErrorCode::kUnavailable, "Teleop grant verifier is not initialized"));
    }
    const auto separator = token.find('.');
    if (separator == std::string::npos || separator == 0U || separator + 1U >= token.size() ||
        token.find('.', separator + 1U) != std::string::npos) {
        return Result<GrantClaims>::failure(Status::error(ErrorCode::kInvalidArgument, "grant wire format is invalid"));
    }

    const std::string_view payload_segment(token.data(), separator);
    const std::string_view signature_segment(token.data() + separator + 1U, token.size() - separator - 1U);
    auto payload = decodeBase64Url(payload_segment);
    auto signature = decodeBase64Url(signature_segment);
    if (!payload.ok()) {
        return Result<GrantClaims>::failure(payload.status());
    }
    if (!signature.ok() || signature.value().size() != kEd25519SignatureBytes) {
        return Result<GrantClaims>::failure(
            Status::error(ErrorCode::kInvalidArgument, "grant Ed25519 signature encoding is invalid"));
    }
    auto claims_result = parseClaims(payload.value());
    if (!claims_result.ok()) {
        return claims_result;
    }
    GrantClaims claims = claims_result.takeValue();
    const auto key_iterator = impl_->public_keys_.find(claims.key_id);
    if (key_iterator == impl_->public_keys_.end()) {
        return Result<GrantClaims>::failure(
            Status::error(ErrorCode::kUnauthorized, "grant references an unknown key id"));
    }

    std::unique_ptr<EVP_MD_CTX, MdContextDeleter> context(EVP_MD_CTX_new());
    if (!context || EVP_DigestVerifyInit(context.get(), nullptr, nullptr, nullptr, key_iterator->second.get()) != 1 ||
        EVP_DigestVerify(context.get(), signature.value().data(), signature.value().size(), payload.value().data(),
                         payload.value().size()) != 1) {
        return Result<GrantClaims>::failure(
            Status::error(ErrorCode::kUnauthorized, "grant signature verification failed"));
    }
    if (claims.expires_at_epoch_sec <= system_now_sec) {
        return Result<GrantClaims>::failure(Status::error(ErrorCode::kDeadlineExceeded, "grant has expired"));
    }
    if (claims.session_id != expected.session_id || claims.run_id != expected.run_id ||
        claims.device_id != expected.device_id || claims.resource_id != expected.resource_id) {
        return Result<GrantClaims>::failure(
            Status::error(ErrorCode::kUnauthorized, "grant binding does not match the RTC session"));
    }

    std::lock_guard<std::mutex> lock(impl_->nonce_mutex_);
    if (impl_->consumed_nonces_.find(claims.nonce) != impl_->consumed_nonces_.end()) {
        return Result<GrantClaims>::failure(Status::error(ErrorCode::kConflict, "grant nonce replay detected"));
    }
    if (impl_->consumed_nonces_.size() >= impl_->max_nonce_entries_) {
        return Result<GrantClaims>::failure(
            Status::error(ErrorCode::kResourceExhausted, "grant nonce replay cache is full"));
    }
    impl_->consumed_nonces_.emplace(claims.nonce);
    return Result<GrantClaims>::success(std::move(claims));
#endif
}

}  // namespace astrabot::teleop
