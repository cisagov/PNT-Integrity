//============================================================================//
//------------------- if_data_utils/IFDataFileReader.hpp -------*- C++ -*-----//
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
/// \brief    This file contains the declaration of the IFDataFileReader class.
/// \details
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     September 11, 2019
///
//===----------------------------------------------------------------------===//
#ifndef IF_DATA_UTILS__IF_DATA_FILE_READER_HPP
#define IF_DATA_UTILS__IF_DATA_FILE_READER_HPP

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
using read_element = std::vector<char>;

/// \brief A class object for writing IF data to files
template <typename samp_type>
class IFDataFileReader
{
public:
  /// \brief Constructor for the file writer class
  ///
  /// \param writeBufferSize The size of each buffer element in bytes
  IFDataFileReader(
    const ssize_t&                readBufferSize = 4096,
    const logutils::LogCallback& log            = logutils::printLogToStdOut);

  ~IFDataFileReader() { closeFile(); }

  bool openFile(const std::string& filename);

  void closeFile()
  {
    if (fileDescriptor_)
    {
      close(fileDescriptor_);
      log_("File closed.", logutils::LogLevel::Info);
    }
  }

  bool readSamplesFromFile(read_element& readBuffer);

  size_t getReadBufferSize() { return readBufferSize_; };

  bool skip(const size_t& bytesToSkip)
  {
    if (fileDescriptor_ == 0)
    {
      log_("IFDataFileReader::skip(): No file open.",
           logutils::LogLevel::Error);
      return false;
    }

    if (lseek(fileDescriptor_, bytesToSkip, SEEK_CUR) >= 0)
    {
      return true;
    }
    else
    {
      // TODO: Parse return error functions
      return false;
    }
  }

  void setLogHandler(const logutils::LogCallback& log) { log_ = log; }

  void setReadBufferSize(const size_t& buffSize) { readBufferSize_ = buffSize; }

private:
  ssize_t readBufferSize_;

  int fileDescriptor_;

  // local storage of the log callback
  logutils::LogCallback log_;
};

template <typename samp_type>
IFDataFileReader<samp_type>::IFDataFileReader(const ssize_t& readBufferSize,
                                              const logutils::LogCallback& log)
  : readBufferSize_(readBufferSize), fileDescriptor_(0) , log_(log)
{
  const size_t samps_per_element = readBufferSize / sizeof(samp_type);
  if (readBufferSize % sizeof(samp_type) != 0)
  {
    std::stringstream errStr;
    errStr << "Element size " << readBufferSize
           << " not an integer # of samples";
    log_(errStr.str(), logutils::LogLevel::Error);
  }

  std::stringstream sizeMsg;
  sizeMsg << "Elements are " << readBufferSize << " bytes, "
          << samps_per_element << " samples/element";
  log_(sizeMsg.str(), logutils::LogLevel::Info);
}

template <class samp_type>
bool IFDataFileReader<samp_type>::openFile(const std::string& filename)
{
  if ((fileDescriptor_ = open(filename.c_str(),
                              O_RDONLY | O_NONBLOCK,
                              S_IRWXU | S_IRWXG | S_IRWXO)) < 0)
  {
    std::stringstream errStr;
    errStr << "File open failed: " << filename.c_str();
    log_(errStr.str(), logutils::LogLevel::Error);
    std::cerr << std::endl;

    return false;
  }
  std::stringstream msg;
  msg << "Successfully opened: " << filename;
  log_(msg.str(), logutils::LogLevel::Info);
  return true;
}

template <class samp_type>
bool IFDataFileReader<samp_type>::readSamplesFromFile(read_element& readBuffer)
{
  auto bytes_read =
    read(fileDescriptor_, (void*)(&readBuffer), readBufferSize_);

  if (bytes_read != readBufferSize_)
  {
    std::stringstream errStr;
    if (bytes_read < 0)
    {
      errStr << "Problem reading: " << strerror(errno);
    }
    else
    {
      errStr << "Read " << bytes_read << "/" << (int)readBufferSize_
             << " bytes";
    }
    log_(errStr.str(), logutils::LogLevel::Error);
    return false;
  }
  return true;
}

}  // namespace if_data_utils
#endif
#endif