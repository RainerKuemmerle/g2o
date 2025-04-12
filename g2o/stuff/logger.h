// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_LOGGER_H
#define G2O_LOGGER_H

#include "g2o/config.h"
#include "g2o/stuff/g2o_stuff_api.h"

#ifdef G2O_HAVE_LOGGING
#include <spdlog/spdlog.h>

#include <memory>

namespace g2o {
namespace internal {

/**
 * @brief Interface class to the underlying logging library
 */
class G2O_STUFF_API LoggerInterface {
 public:
  LoggerInterface();

  ~LoggerInterface();

  LoggerInterface(LoggerInterface&) = delete;
  void operator=(LoggerInterface&) = delete;

  spdlog::logger& console() { return *console_; }

 protected:
  std::shared_ptr<spdlog::logger> console_;
};

/**
 * @brief Singleton wrapper class
 *
 * @tparam T type for which to provide a single
 */
template <class T>
class Singleton {
 public:
  static T& get() {
    static auto instance_ = std::make_unique<T>();
    return *instance_;
  }
  constexpr Singleton(const Singleton&) = delete;
  constexpr Singleton& operator=(const Singleton&) = delete;
};

}  // namespace internal

using Logger = internal::Singleton<internal::LoggerInterface>;

}  // namespace g2o

// TODO(Rainer): Switch to std::source_location with c++20
#define G2O_TRACE(...)                                      \
  g2o::Logger::get().console().log(                         \
      spdlog::source_loc(__FILE__, __LINE__, __FUNCTION__), \
      spdlog::level::trace, __VA_ARGS__)
#define G2O_DEBUG(...)                                      \
  g2o::Logger::get().console().log(                         \
      spdlog::source_loc(__FILE__, __LINE__, __FUNCTION__), \
      spdlog::level::debug, __VA_ARGS__)
#define G2O_INFO(...)                                       \
  g2o::Logger::get().console().log(                         \
      spdlog::source_loc(__FILE__, __LINE__, __FUNCTION__), \
      spdlog::level::info, __VA_ARGS__)
#define G2O_WARN(...)                                       \
  g2o::Logger::get().console().log(                         \
      spdlog::source_loc(__FILE__, __LINE__, __FUNCTION__), \
      spdlog::level::warn, __VA_ARGS__)
#define G2O_ERROR(...)                                      \
  g2o::Logger::get().console().log(                         \
      spdlog::source_loc(__FILE__, __LINE__, __FUNCTION__), \
      spdlog::level::err, __VA_ARGS__)
#define G2O_CRITICAL(...)                                   \
  g2o::Logger::get().console().log(                         \
      spdlog::source_loc(__FILE__, __LINE__, __FUNCTION__), \
      spdlog::level::critical, __VA_ARGS__)

#else

#define G2O_TRACE(...) (void)0
#define G2O_DEBUG(...) (void)0
#define G2O_INFO(...) (void)0
#define G2O_WARN(...) (void)0
#define G2O_ERROR(...) (void)0
#define G2O_CRITICAL(...) (void)0

#endif  // G2O_HAVE_LOGGING

namespace g2o::logging {
enum class Level { kTrace, kDebug, kInfo, kWarn, kError, kOff };
/**
 * @brief Set the Log Level
 *
 * @param level control which severity of log will be print
 */
void setLevel(Level level);
}  // namespace g2o::logging

#endif
