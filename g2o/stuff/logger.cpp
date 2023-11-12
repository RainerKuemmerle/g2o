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

#include "logger.h"

#include <string_view>

#include "g2o/config.h"

#ifdef G2O_HAVE_LOGGING
#include <spdlog/cfg/env.h>
#include <spdlog/common.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <cassert>

namespace g2o::internal {

constexpr std::string_view kLoggerName = "g2o";

LoggerInterface::LoggerInterface() {
  spdlog::cfg::load_env_levels();
  console_ = spdlog::get(std::string(kLoggerName));
  if (console_) return;
  console_ = spdlog::stdout_color_mt(std::string(kLoggerName));
  console_->set_pattern("%+");
}

LoggerInterface::~LoggerInterface() { spdlog::drop(std::string(kLoggerName)); }

}  // namespace g2o::internal
#endif

namespace g2o::logging {

void setLevel(Level level) {
#ifdef G2O_HAVE_LOGGING
  auto toSpdLogLevel = [](Level level) {
    switch (level) {
      case Level::kTrace:
        return spdlog::level::trace;
      case Level::kDebug:
        return spdlog::level::debug;
      case Level::kInfo:
        return spdlog::level::info;
      case Level::kWarn:
        return spdlog::level::warn;
      case Level::kError:
        return spdlog::level::err;
      case Level::kOff:
        return spdlog::level::off;
    }
    assert(false && "Unexpected level passed to the function");
    return spdlog::level::off;
  };
  Logger::get().console().set_level(toSpdLogLevel(level));
#else
  (void)level;
#endif
}

}  // namespace g2o::logging
