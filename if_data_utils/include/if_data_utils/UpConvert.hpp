//============================================================================//
//--------------------- if_data_utils/UpConvert.hpp ------------*- C++ -*-----//
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
/// \brief    This file contains the declaration of the UpConvert class.
/// \details  This class contains methods used in the implementation to
///           convert a baseband Complex (IQ) data file to a real non-zero
///           IF signal file. IF frequency will be a quarter of
///           the sampling frequency
/// \author   Joshua Starling <josh.starling@is4s.com>
/// \version  0.1
/// \date     April 27, 2018
///
//===----------------------------------------------------------------------===//
#ifndef IF_DATA_UTILS_UPCONVERT_HPP
#define IF_DATA_UTILS_UPCONVERT_HPP

#include <vector>

#include "if_data_utils/IfData.hpp"
#include "if_data_utils/dataFileStructures.hpp"

namespace if_data_utils
{
class UpConvert
{
public:
  /// Basic constructor
  UpConvert();

  /// Basic deconstructor
  ~UpConvert();

  /// \Brief Adds files to list of data files to be combined
  ///
  /// \param dataFile structure that describes recorded data and
  ///  description of event
  void addFile(dataFile file);

  /// \Brief Combines data files to generate a signle attack file
  ///
  /// \param name of desired output file
  bool upconvertFile(std::string outFile);

  //////////////////////////////////
  /////////Setter Functions/////////
  //////////////////////////////////
  /// \Brief Sets number of samples to be read per block
  void setBlkSize(int blkSize) { blkSize_ = blkSize; }

  //////////////////////////////////
  /////////Getter Functions/////////
  //////////////////////////////////
  /// \Brief Gets number of samples to be read per block
  int getBlkSize() { return blkSize_; }

  /// \Brief Gets file duration in number of samples
  float getFileDurationSamp() { return fileDurationSamp_; }

private:
  /// \Brief Calls proper IF Data class object to read data file
  void readFile(Eigen::VectorXf& samplesEigen);

  /// \Brief Opens proper IF Data class object to open data file
  void openFile();

  /// IF data class objects
  if_data_utils::IfData ifData_;
  if_data_utils::IfData ifDataOut_;

  int                blkSize_;
  int                numOfLoops_{};
  float              fileDurationSamp_{};
  dataFile           fileIn_;
  std::vector<float> mixSine_;
  std::vector<float> mixCos_;
};
}  // namespace if_data_utils

#endif
