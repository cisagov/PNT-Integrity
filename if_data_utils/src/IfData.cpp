//============================================================================//
//------------------------ if_data_utils/IfData.cpp ------------*- C++ -*-----//
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
// This file contains the declaration of the IfData class.
//
// David Hodo <david.hodo@is4s.com>
// May 27, 2015
//
//===----------------------------------------------------------------------===//
#include <iostream>
#include <utility>

#include "if_data_utils/IfData.hpp"

namespace if_data_utils
{
void convertSampleToSigned(uint8_t byte, int8_t samples[4])
{
  uint8_t mask = 0x03;
  for (int ii = 0; ii < 4; ii++)
  {
    uint8_t sample = (byte & mask) >> (ii * 2);
    mask           = mask << 2;

    // convert to signed integer
    if (sample == 1)
      samples[ii] = 3;
    else if (sample == 0)
      samples[ii] = 1;
    else if (sample == 2)
      samples[ii] = -1;
    else
      samples[ii] = -3;
  }
}

IfData::IfData()
  : sampleCount_(0), filesize_(0), samplesPerRead_(0), bytesPerRead_(0)
{
}

IfData::~IfData()
{
  close();
}

bool IfData::open(Settings settings)
{
  settings_ = std::move(settings);

  try
  {
    // log_("Opening IF data file.", logutils::LogLevel::Info);

    std::stringstream settingsOut;
    settingsOut << "Settings:" << std::endl;
    settingsOut << "Sampling Freq: " << settings_.samplingFreq << std::endl;
    ;
    settingsOut << "Complex: " << settings_.complex << std::endl;
    settingsOut << "Bytes per sample: " << settings_.bytesPerSample
                << std::endl;
    settingsOut << "Data file: " << settings_.dataFilename << std::endl;
    settingsOut << "Sample count offset: " << settings_.sampleCountOffset
                << std::endl;

    // log_(settingsOut.str(), logutils::LogLevel::Info);

    samplesFile_.open(settings_.dataFilename);

    if (!samplesFile_.is_open())
    {
      // log_("Failed to open IF data file: " + settings_.dataFilename,
      // logutils::LogLevel::Error);
      return false;
    }

    sampleCount_ = settings_.sampleCountOffset;

    // get file length
    samplesFile_.seekg(0, samplesFile_.end);
    filesize_ = samplesFile_.tellg();
    samplesFile_.seekg(0, samplesFile_.beg);
    std::stringstream sizeOut;
    sizeOut << "Filesize: " << static_cast<double>(filesize_) / 1e6 << " MB";
    // log_(sizeOut.str(), logutils::LogLevel::Info);
  }
  catch (std::exception& e)
  {
    std::stringstream out;
    out << "Error opening IF data file: " << e.what();
    // log_(out.str(), logutils::LogLevel::Error);
    return false;
  }

  return true;
}

void IfData::openOutFile(const std::string& metadataFilename)
{
  outFile_.open(metadataFilename, std::ios::out | std::ios::binary);
}

void IfData::writeSamples(Eigen::RowVectorXi& samplesEigen,
                          std::size_t         sampleSize)
{
  for (int ii = 0; ii < samplesEigen.size(); ii++)
  {
    outFile_.write((char*)&samplesEigen[ii], sampleSize);
  }
}

void IfData::close()
{
  if (samplesFile_.is_open())
  {
    samplesFile_.close();
  }
}

uint64_t IfData::getSamples(Eigen::VectorXf& samplesEigen)
{
  size_t numSamples       = samplesEigen.size();  // number of samples to read
  size_t samplesFromQueue = 0;  // samples to read from the leftover queue
  size_t bytesFromFile    = 0;  // bytes to read from the file
  size_t samplesRead      = 0;  // samples read so far

  uint64_t initialSampleCount = sampleCount_;

  // determine number of bytes to read from queue and file
  if (leftoverSamples_.size() > numSamples)
  {
    samplesFromQueue = numSamples;
    bytesFromFile    = 0;
  }
  else
  {
    samplesFromQueue = leftoverSamples_.size();
    bytesFromFile =
      ceil((numSamples - leftoverSamples_.size()) * settings_.bytesPerSample);
  }

  // read any data from the leftover buffer before reading from the file
  for (size_t ii = 0; ii < samplesFromQueue; ii++)
  {
    samplesEigen[samplesRead++] = leftoverSamples_.front();
    leftoverSamples_.pop();
    sampleCount_++;
    // std::cout << "Reading buffered sample." << std::endl;
  }

  // see if we also need to read data from the file
  if (bytesFromFile == 0)
  {
    return initialSampleCount;
  }

  char* readData = new char[bytesFromFile];
  try
  {
    uint16_t temp = 0;
    samplesFile_.read(readData, bytesFromFile);
    // convert samples to vector of floats
    for (size_t ii = 0; ii < bytesFromFile; ii++)
    {
      int     samplesPerByte = 1 / settings_.bytesPerSample;
      int8_t* samples        = new int8_t[samplesPerByte];
      if (samplesPerByte == 4)
      {
        // std::cout << "Byte read: 0x" << std::hex << (int)readData[ii] <<
        // std::endl;
        if_data_utils::convertSampleToSigned(readData[ii], samples);
        for (size_t jj = 0; jj < 4; jj++)
        {
          // see if we've read all the samples that we've been asked for
          // if not, read another sample
          // if so, store the remaining data in the buffer for next time
          if (samplesRead < numSamples)
          {
            samplesEigen[samplesRead++] = (float)samples[jj];
            sampleCount_++;
            // std::cout << "Count: " << std::dec << sampleCount_ << " Sample: "
            // << (float)samples[jj] << std::endl;
          }
          else
          {
            leftoverSamples_.push((float)samples[jj]);
            // std::cout << "Buffering sample." << std::endl;
          }
        }
      }
      else if (settings_.bytesPerSample == 2)
      {
        int idx = ii % 2;
        if (idx == 0)
        {
          // first byte
          temp = readData[ii];
        }
        else
        {
          // second byte
          temp                        = (temp << 8) | readData[ii];
          samplesEigen[samplesRead++] = (float)temp;
          sampleCount_++;
        }
      }
      else
      {
        // see if we've read all the samples that we've been asked for
        // if so, store the remaining data in the buffer for next time
        if (samplesRead < numSamples)
        {
          samplesEigen[samplesRead++] = (float)readData[ii];
          sampleCount_++;
        }
        else
        {
          std::cout << "Error: we read too much data!" << std::endl;
        }
      }
      delete[] samples;
    }
  }
  catch (std::exception& e)
  {
    std::cout << "Error getting IF sample: " << e.what() << std::endl;
  }
  delete[] readData;

  return sampleCount_;
}

/// Moves the file pointer to the given sample count value
bool IfData::setSampleCount(uint64_t sampleCount)
{
  //    size_t offset = sampleCount - sampleCount_;
  samplesFile_.seekg(sampleCount, samplesFile_.beg);
  sampleCount_ = sampleCount;
  return !samplesFile_.eof();
}

bool IfData::skipSamples(size_t numSamples)
{
  size_t bytesToSkip = numSamples * settings_.bytesPerSample;
  samplesFile_.seekg(bytesToSkip, std::ios_base::cur);
  sampleCount_ += numSamples;
  return !samplesFile_.eof();
}

}  // namespace if_data_utils
