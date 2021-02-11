//============================================================================//
//-------------------------- src/logutils.cpp ------------------*- C++ -*-----//
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
/// \author   William Travis <william.travis@is4s.com>
/// \date     December 15, 2015
//============================================================================//

#include "logutils/logutils.hpp"

void logutils::printLogToStdOut(const std::string& msg, const LogLevel& level)
{
  // std::ostream stream = level ==logutils::LogLevel::Error
  //                      ? std::cerr
  //                      : std::cout;
  //
  // printLogToStream(stream, msg, level);

  if (level == logutils::LogLevel::Error)
  {
    printLogToStream(std::cerr, msg, level);
  }
  else
  {
    printLogToStream(std::cout, msg, level);
  }
}

void logutils::printLogToStream(std::ostream&      stream,
                                const std::string& msg,
                                const LogLevel&    level)
{
  switch (level)
  {
    case logutils::LogLevel::Debug3:
    {
      stream << "DEBUG3: ";
      break;
    }
    case logutils::LogLevel::Debug2:
    {
      stream << "DEBUG2: ";
      break;
    }
    case logutils::LogLevel::Debug:
    {
      stream << "DEBUG: ";
      break;
    }
    case logutils::LogLevel::Info:
    {
      stream << "INFO: ";
      break;
    }
    case logutils::LogLevel::Warn:
    {
      stream << "WARN: ";
      break;
    }
    case logutils::LogLevel::Error:
    {
      stream << "ERROR: ";
      break;
    }
    default:
    {
      break;
    }
  }

  stream << msg << std::endl;
}
