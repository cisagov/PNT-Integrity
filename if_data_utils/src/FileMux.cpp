//============================================================================//
//------------------------ if_data_utils/FileMux.cpp -----------*- C++ -*-----//
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
//
//
// This file contains the declaration of the FileMux class.
// This class contains methods used in the implementation of
// combining two IF files together to simulate a GPS attack
// Joshua Starling <josh.starling@is4s.com>
// 0.1
// April 27, 2018
//
//===----------------------------------------------------------------------===//

#include <iostream>
#include <utility>

#include "if_data_utils/FileMux.hpp"
#include "math.h"

namespace if_data_utils
{
FileMux::FileMux()
{
  numberOfFiles_ = 0;
  blkSize_       = 2000;
}

FileMux::~FileMux()
{
}

void FileMux::addFile(dataFile file)
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

  // Add file to vector of files
  files_.push_back(file);
  numberOfFiles_++;
}

bool FileMux::validateDataFiles()
{
  // Verify that aux data types match reference data type
  if (files_[0].bytesPerSample != files_[1].bytesPerSample)
  {
    std::cout << "ERROR: Number of bytes per sample does not match!!! Exiting"
              << std::endl;
    return false;
  }
  if (files_[0].samplePerMeas != files_[1].samplePerMeas)
  {
    std::cout << "ERROR: Files data types(real/complex) do not match!!! Exiting"
              << std::endl;
    return false;
  }
  if (files_[0].sampFreq != files_[1].sampFreq)
  {
    std::cout << "ERROR: Sample frequencies do not match!!! Exiting"
              << std::endl;
    return false;
  }
  if (files_[0].ifFreq != files_[1].ifFreq)
  {
    std::cout << "ERROR: IF frequencies do not match!!! Exiting" << std::endl;
    return false;
  }
  return true;
}

void FileMux::createEventProfile(int idx)
{
  std::cout << idx << std::endl;
  eventActivationBlk_.push_back(
    floor(files_[idx].event.startTime / (blkSize_ / files_[0].sampFreq)));

  eventDeactivationBlk_.push_back(
    floor(files_[idx].event.endTime / (blkSize_ / files_[0].sampFreq)));

  float deltaPower = files_[idx].event.endPower - files_[idx].event.startPower;
  float numbBlksActive =
    eventDeactivationBlk_.at(idx - 1) - eventActivationBlk_.at(idx - 1);
  eventDeltaPowerFrac_.push_back(deltaPower / numbBlksActive);
  eventPower_.push_back(files_[idx].event.startPower / 100.);
}

bool FileMux::checkIfEventActive(int blk, int idx)
{
  if (blk == eventActivationBlk_.at(idx))
  {
    // Open aux data file if event occurs this block
    openFile(idx + 1);
    return true;
  }
  else if (blk > eventActivationBlk_.at(idx) &&
           blk <= eventDeactivationBlk_.at(idx))
  {
    // Determine new event power level
    eventPower_.at(idx) =
      eventPower_.at(idx) + eventDeltaPowerFrac_.at(idx) / 100.;
    return true;
  }
  else
  {
    return false;
  }
}

bool FileMux::combineFiles(std::string outFile)
{
  // If files do not match data types return false.
  if (!validateDataFiles())
  {
    return false;
  }

  // Determine max value of all files such that combine file does not saturate
  determineMax();

  ///////////////////
  /// Combine Files///
  ///////////////////
  // Determine size of input and output signal vectors
  int                dataSize = blkSize_ * (int)files_[0].samplePerMeas;
  Eigen::VectorXf    dataFile1(dataSize);
  Eigen::VectorXf    dataFile2(dataSize);
  Eigen::VectorXf    dataFile3(dataSize);
  Eigen::RowVectorXi dataOutput(dataSize);
  openFile(0);
  // Open reference files
  ifDataOut_.openOutFile(std::move(outFile));

  // Create event profile of when aux files are activated and desired power
  for (int ii = 1; ii < numberOfFiles_; ii++)
  {
    FileMux::createEventProfile(ii);
  }

  std::vector<float> active;
  for (int jj = 0; jj < numOfLoops_; jj++)
  {
    for (int ii = 0; ii < numberOfFiles_ - 1; ii++)
    {
      active.push_back(checkIfEventActive(jj, ii));
    }

    // Read data files
    readFile(0, dataFile1);
    readFile(1, dataFile2);
    if (numberOfFiles_ == 3)
    {
      readFile(2, dataFile3);
    }
    float tempDataOut = NAN;
    for (int kk = 0; kk < blkSize_ * files_[0].samplePerMeas; kk++)
    {
      tempDataOut =
        dataFile1[kk] + dataFile2[kk] * active.at(0) * eventPower_.at(0);
      if (numberOfFiles_ == 3)
      {
        tempDataOut =
          tempDataOut + dataFile3[kk] * active.at(1) * eventPower_.at(1);
      }
      // Add files together using appropriate data type
      if (files_[0].dataFormat == "sc8")
      {
        dataOutput[kk] = (uint8_t)(tempDataOut * sumScale_);
      }
      else if (files_[0].dataFormat == "sc16")
      {
        dataOutput[kk] = (uint16_t)(dataOutput[kk] * sumScale_);
      }
    }
    // Write combined signal to output file
    ifDataOut_.writeSamples(dataOutput,
                            sizeof(char) * files_[0].bytesPerSample);
    active.clear();
  }

  // Close all files
  ifData0_.close();
  ifData1_.close();
  if (numberOfFiles_ == 3)
  {
    ifData2_.close();
  }
  return true;
}

void FileMux::determineMax()
{
  // Determine size of signal vectors
  int             dataSize = blkSize_ * (int)files_[0].samplePerMeas;
  Eigen::VectorXf dataEigen(dataSize);

  // Determine max for each input file
  for (int ii = 0; ii < numberOfFiles_; ii++)
  {
    fileMaxs_.push_back(0);

    // Open file
    openFile(ii);

    // Determine number of required loops per given block size
    if (ii == 0)
    {
      // Determine if upconvert should be ran until end of file or set end time
      if (ifData0_.getFilesize() < files_[0].event.endTime * files_[0].sampFreq)
      {
        numOfLoops_ =
          int(ifData0_.getFilesize() - ifData0_.getSampleCount()) /
          (blkSize_ * files_[0].samplePerMeas * files_[0].bytesPerSample);
        std::cout << "File size smaller" << numOfLoops_ << std::endl;
      }
      else
      {
        numOfLoops_ =
          int(files_[0].event.endTime * files_[0].sampFreq *
                files_[0].samplePerMeas * files_[0].bytesPerSample -
              ifData0_.getSampleCount()) /
          (blkSize_ * files_[0].samplePerMeas * files_[0].bytesPerSample);
        std::cout << "end smaller " << numOfLoops_ << std::endl;
      }
    }

    for (int jj = 0; jj < numOfLoops_; jj++)
    {
      // Read data
      readFile(ii, dataEigen);
      for (int kk = 0; kk < blkSize_; kk++)
      {
        // Check for new max values, replace if new max found
        if (dataEigen[kk] > fileMaxs_[ii])
        {
          fileMaxs_[ii] = (float)dataEigen[kk];
        }
      }
    }

    // Close file
    if (ii == 0)
    {
      ifData0_.close();
    }
    else if (ii == 1)
    {
      ifData1_.close();
    }
    else
    {
      ifData2_.close();
    }
  }

  // Sum max values to determine largest maximum value,
  // scale per data type max value
  float maxCombValue = 0;
  for (std::vector<float>::iterator it = fileMaxs_.begin();
       it != fileMaxs_.end();
       ++it)
    maxCombValue += *it;
  sumScale_ = files_[0].maxValue / maxCombValue;
}

void FileMux::openFile(int idx)
{
  // Create setting file required by IF data class
  if_data_utils::Settings ifSettings;
  ifSettings.samplingFreq          = files_[idx].sampFreq;
  ifSettings.intermediateFrequency = files_[idx].ifFreq;
  ifSettings.bytesPerSample        = files_[idx].bytesPerSample;
  ifSettings.complex               = files_[idx].isComplex;
  ifSettings.sampleCountOffset =
    (uint64_t)(files_[idx].skipSeconds * ifSettings.samplingFreq *
               files_[idx].samplePerMeas);
  ifSettings.dataFilename = files_[idx].fileName;

  // Open file and move to correct position in file
  if (idx == 0)
  {
    ifData0_.open(ifSettings);
    ifData0_.setSampleCount(ifSettings.sampleCountOffset);
  }
  else if (idx == 1)
  {
    ifData1_.open(ifSettings);
    ifData1_.setSampleCount(ifSettings.sampleCountOffset);
  }
  else
  {
    ifData2_.open(ifSettings);
    ifData2_.setSampleCount(ifSettings.sampleCountOffset);
  }
}

void FileMux::readFile(int idx, Eigen::VectorXf& samplesEigen)
{
  if (idx == 0)
  {
    ifData0_.getSamples(samplesEigen);
  }
  else if (idx == 1)
  {
    ifData1_.getSamples(samplesEigen);
  }
  else
  {
    ifData2_.getSamples(samplesEigen);
  }
}

}  // namespace if_data_utils
