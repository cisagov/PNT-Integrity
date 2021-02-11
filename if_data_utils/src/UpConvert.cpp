//============================================================================//
//---------------------- if_data_utils/UpConvert.cpp -----------*- C++ -*-----//
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
// This file contains souce code for the Upconvert class.
//
// Joshua Starling <josh.starling@is4s.com>
// April 27, 2018
//
//===----------------------------------------------------------------------===//

#include "if_data_utils/UpConvert.hpp"
#include <iostream>
#include <utility>

namespace if_data_utils
{
// Class constructor
UpConvert::UpConvert()
{
  // Declare block size
  blkSize_ = 20000;

  // Generate mixing sine and cosine wave at creation of class object
  mixSine_.reserve(blkSize_);
  mixCos_.reserve(blkSize_);
  for (int ii = 0; ii < blkSize_; ii = ii + 4)
  {
    mixSine_.push_back(0);
    mixSine_.push_back(1);
    mixSine_.push_back(0);
    mixSine_.push_back(-1);

    mixCos_.push_back(1);
    mixCos_.push_back(0);
    mixCos_.push_back(-1);
    mixCos_.push_back(0);
  }
}
// Class deconstructor
UpConvert::~UpConvert()
{
}

bool UpConvert::upconvertFile(std::string outFile)
{
  // Make sure input file is complex
  if (!fileIn_.isComplex)
  {
    std::cout << "Input file must be complex!!! Exiting" << std::endl;
    return false;
  }

  ////////////////////
  /// Upconvert File///
  ////////////////////
  // Determine size of input and output signal vectors
  int dataSize = blkSize_ * (int)fileIn_.samplePerMeas;

  // Open Files
  openFile();
  ifDataOut_.openOutFile(std::move(outFile));

  // Declare I/O variables
  Eigen::VectorXf    dataFile(dataSize);
  Eigen::RowVectorXi dataOutput(blkSize_);

  // Determine if upconvert should be ran until end of file or set end time
  if (ifData_.getFilesize() < fileIn_.event.endTime * fileIn_.sampFreq)
  {
    numOfLoops_ = int(ifData_.getFilesize() - ifData_.getSampleCount()) /
                  (blkSize_ * fileIn_.samplePerMeas * fileIn_.bytesPerSample);
    std::cout << "File size smaller" << numOfLoops_ << std::endl;
  }
  else
  {
    numOfLoops_ = int(fileIn_.event.endTime * fileIn_.sampFreq *
                        fileIn_.samplePerMeas * fileIn_.bytesPerSample -
                      ifData_.getSampleCount()) /
                  (blkSize_ * fileIn_.samplePerMeas * fileIn_.bytesPerSample);
    std::cout << "end smaller " << numOfLoops_ << std::endl;
  }

  // Calculate the number of blocks required to process the entire file

  for (int jj = 0; jj < numOfLoops_; jj++)
  {
    // Read data files
    ifData_.getSamples(dataFile);

    std::vector<float>::iterator itSine = mixSine_.begin();
    std::vector<float>::iterator itCos  = mixCos_.begin();
    int                          outIdx = 0;  // index to write to output file
    for (int kk = 0; kk < blkSize_ * fileIn_.samplePerMeas;
         kk     = kk + 2, outIdx++)
    {
      // Sort into I and Q signal and combine signals
      if (fileIn_.dataFormat == "sc8")
      {
        dataOutput[outIdx] =
          (int8_t)((dataFile[kk] * (*itCos) - dataFile[kk + 1] * (*itSine)));
        // std::cout << dataOutput[outIdx] << " ";
      }
      else if (fileIn_.dataFormat == "sc16")
      {
        dataOutput[outIdx] = (int16_t)(
          (dataFile[kk] * (*itCos) - dataFile[kk + 1] * (*itSine)) / 3.);
      }

      // Iterate mixer signal iterators
      itCos++;
      itSine++;
    }
    // Write combined signal to output file
    ifDataOut_.writeSamples(dataOutput, sizeof(char) * fileIn_.bytesPerSample);
  }
  ifDataOut_.close();
  ifData_.close();
  return true;
}

void UpConvert::addFile(dataFile file)
{
  // Based on dataFormat, determine bytes per sample and max value
  if (file.dataFormat == "sc8")
  {
    file.bytesPerSample = 1;
    file.maxValue       = 127;
  }
  else if (file.dataFormat == "sc16")
  {
    file.bytesPerSample = 2;
    file.maxValue       = 32768;
  }
  else if (file.dataFormat == "sc2")
  {
    file.bytesPerSample = 0.25;
    file.maxValue       = 2;
  }
  if (file.isComplex)
  {
    file.samplePerMeas = 2;
  }
  else
  {
    file.samplePerMeas = 1;
  }

  fileIn_ = file;
}

void UpConvert::openFile()
{
  // Create setting file required by IF data class
  if_data_utils::Settings ifSettings;
  ifSettings.samplingFreq          = fileIn_.sampFreq;
  ifSettings.intermediateFrequency = fileIn_.ifFreq;
  ifSettings.bytesPerSample        = fileIn_.bytesPerSample;
  ifSettings.complex               = fileIn_.isComplex;
  ifSettings.sampleCountOffset     = (uint64_t)fileIn_.skipSeconds *
                                 ifSettings.samplingFreq *
                                 fileIn_.samplePerMeas * fileIn_.bytesPerSample;
  ifSettings.dataFilename = fileIn_.fileName;

  // Open file and move to correct position in file
  ifData_.open(ifSettings);
  ifData_.setSampleCount(ifSettings.sampleCountOffset);
}

void UpConvert::readFile(Eigen::VectorXf& samplesEigen)
{
  ifData_.getSamples(samplesEigen);
}
}  // namespace if_data_utils
