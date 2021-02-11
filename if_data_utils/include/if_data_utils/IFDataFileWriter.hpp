//============================================================================//
//------------------- if_data_utils/IFDataFileWriter.hpp -------*- C++ -*-----//
//============================================================================//
// BSD 3-Clause License
//
// Copyright (C) 2019 Integrated Solutions for Systems, Inc
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
//===----------------------------------------------------------------------===//
///
/// \file
/// \brief    This file contains the declaration of the IFDataFileWriter class.
/// \details
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     August 29, 2019
///
//===----------------------------------------------------------------------===//
#ifndef IF_DATA_UTILS__IF_DATA_FILE_WRITER_HPP
#define IF_DATA_UTILS__IF_DATA_FILE_WRITER_HPP

// only define the file functions on unix platforms
// this effectively renders the class useless on a win platform
#ifndef _WIN32

#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <memory>
#include <sstream>
#include <vector>

#include "logutils/logutils.hpp"

namespace if_data_utils
{
using write_element = std::vector<char>;

/// \brief A class object for writing IF data to files
template <typename samp_type>
class IFDataFileWriter
{
public:
  /// \brief Constructor for the file writer class
  ///
  /// \param writeBufferSize The size of each buffer element in bytes
  IFDataFileWriter(
    const ssize_t&                writeBufferSize = 4096,
    const logutils::LogCallback& log             = logutils::printLogToStdOut);

  bool createFile(const std::string& filename);

  size_t getWriteBufferSize() { return writeBufferSize_; };

  bool writeSamplesToFile(const write_element& writeBuffer);

  size_t getTotalBytesWritten() { return totalBytesWritten_; };

private:
  ssize_t writeBufferSize_;

  int fileDescriptor_;

  // local storage of the log callback
  logutils::LogCallback log_;

  size_t totalBytesWritten_;
};

template <typename samp_type>
IFDataFileWriter<samp_type>::IFDataFileWriter(const ssize_t& writeBufferSize,
                                              const logutils::LogCallback& log)
  : writeBufferSize_(writeBufferSize), log_(log), totalBytesWritten_(0)
{
  const size_t samps_per_element = writeBufferSize / sizeof(samp_type);
  if (writeBufferSize % sizeof(samp_type) != 0)
  {
    std::stringstream errStr;
    errStr << "Element size " << writeBufferSize
           << " not an integer # of samples";
    log_(errStr.str(), logutils::LogLevel::Error);
  }

  std::stringstream sizeMsg;
  sizeMsg << "Elements are " << writeBufferSize << " bytes, "
          << samps_per_element << " samples/element";
  log_(sizeMsg.str(), logutils::LogLevel::Info);
}

template <class samp_type>
bool IFDataFileWriter<samp_type>::createFile(const std::string& filename)
{
  if ((fileDescriptor_ = open(filename.c_str(),
                              O_WRONLY | O_CREAT | O_TRUNC | O_NONBLOCK,
                              S_IRWXU | S_IRWXG | S_IRWXO)) < 0)
  {
    std::stringstream errStr;
    errStr << "File open failed: " << filename.c_str();
    log_(errStr.str(), logutils::LogLevel::Error);
    std::cerr << std::endl;

    return false;
  }
  return true;
}

template <class samp_type>
bool IFDataFileWriter<samp_type>::writeSamplesToFile(
  const write_element& writeBuffer)
{
  auto bytes_written =
    write(fileDescriptor_, (void*)(&writeBuffer), writeBufferSize_);

  totalBytesWritten_ += bytes_written;

  if (bytes_written != writeBufferSize_)
  {
    std::stringstream errStr;
    if (bytes_written < 0)
    {
      errStr << "Problem writing: " << strerror(errno);
    }
    else
    {
      errStr << "Wrote " << bytes_written << "/" << (int)writeBufferSize_
             << " bytes";
    }
    log_(errStr.str(), logutils::LogLevel::Error);
    return false;
  }
  return true;
}
}  // end namespace if_data_utils
#endif
#endif