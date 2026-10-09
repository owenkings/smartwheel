#include "wc_slam/accepted_fingerprint.hpp"

#include <openssl/evp.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <cstring>
#include <memory>
#include <stdexcept>

namespace wc_slam {

AcceptedFingerprint accepted_fingerprint(
    const wc_interfaces::msg::AcceptedBundle &message,
    const rcl_allocator_t &allocator) {
  rclcpp::Serialization<wc_interfaces::msg::AcceptedBundle> serializer;
  rclcpp::SerializedMessage serialized(allocator);
  // First pass asks the actual installed type support for its required storage.
  // It is not hashed: that buffer can contain uninitialized CDR padding.
  serializer.serialize_message(&message, &serialized);
  auto &buffer = serialized.get_rcl_serialized_message();
  const auto capacity = buffer.buffer_capacity;
  const auto length = buffer.buffer_length;
  const auto address = buffer.buffer;
  if (!address || !length || length > capacity)
    throw std::runtime_error("invalid AcceptedBundle serialization storage");
  std::memset(address, 0, capacity);
  buffer.buffer_length = 0;
  serializer.serialize_message(&message, &serialized);
  // A replacement buffer could reintroduce uninitialized padding. Refuse to
  // compare uncertain fingerprints instead of accidentally accepting a retry.
  if (buffer.buffer != address || buffer.buffer_capacity != capacity ||
      buffer.buffer_length != length)
    throw std::runtime_error("AcceptedBundle serialization changed storage after zeroing");

  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> digest(
      EVP_MD_CTX_new(), &EVP_MD_CTX_free);
  AcceptedFingerprint result{};
  unsigned int result_length = 0;
  if (!digest || EVP_DigestInit_ex(digest.get(), EVP_sha256(), nullptr) != 1 ||
      EVP_DigestUpdate(digest.get(), buffer.buffer, length) != 1 ||
      EVP_DigestFinal_ex(digest.get(), result.data(), &result_length) != 1 ||
      result_length != result.size())
    throw std::runtime_error("cannot fingerprint complete AcceptedBundle");
  return result;
}

}  // namespace wc_slam
