//============================================================================//
//----------------------- logutils/logutils.hpp ----------------*- C++ -*-----//
//============================================================================//
// BSD 3-Clause License
//
// Copyright (C) 2020 Integrated Solutions for Systems, Inc
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//----------------------------------------------------------------------------//
/// \file
/// \brief    This file contains the declaration of the logging utilities.
/// \details  Basic declarations for logging information using function pointers
///           are contained here for consistency.
/// \author   William Travis <william.travis@is4s.com>
/// \date     December 15, 2015
//============================================================================//

#ifndef LOGUTILS__LOGUTILS_HPP
#define LOGUTILS__LOGUTILS_HPP

#include <functional>
#include <iostream>
#include <string>

namespace logutils
{
/// \brief Enumeration to define logging levels
enum class LogLevel
{
  Error  = 1,
  Warn   = 2,
  Info   = 3,
  Debug  = 4,
  Debug2 = 5,
  Debug3 = 6
};

typedef std::function<void(const std::string&, const LogLevel&)> LogCallback;

/// \brief Prints the log message including log level decorator to a stream
///
/// \param stream [out] An output stream to write the log to
/// \param msg    [in]  String containing log message
/// \param level  [in]  Log severity level
void printLogToStream(std::ostream&      stream,
                      const std::string& msg,
                      const LogLevel&    level);

/// \brief Prints the log message including log level decorator to stdout
///
/// \param msg    [in]  String containing log message
/// \param level  [in]  Log severity level
void printLogToStdOut(const std::string& msg, const LogLevel& level);
}  // namespace logutils

#endif
