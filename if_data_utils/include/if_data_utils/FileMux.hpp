//============================================================================//
//------------------------ if_data_utils/FileMux.hpp -----------*- C++ -*-----//
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
/// \brief    This file contains the declaration of the FileMux class.
/// \details  This class contains methods used in the implementation of
///           combining two IF files together to simulate a GPS attack
/// \author   Joshua Starling <josh.starling@is4s.com>
/// \version  0.1
/// \date     April 27, 2018
///
//===----------------------------------------------------------------------===//
#ifndef IF_DATA_UTILS_FILEMUX_HPP
#define IF_DATA_UTILS_FILEMUX_HPP

#include <vector>
#include "IfData.hpp"
#include "dataFileStructures.hpp"

namespace if_data_utils
{
class FileMux
{
public:
  /// Basic constructor
  FileMux();

  /// Basic deconstructor
  ~FileMux();

  /// \Brief Adds files to list of data files to be combined
  ///
  /// \param dataFile structure that describes recorded data and
  ///  description of event
  void addFile(dataFile file);

  /// \Brief Combines data files to generate a signle attack file
  ///
  /// \param name of desired output file
  bool combineFiles(std::string outFile);

  //////////////////////////////////
  /////////Setter Functions/////////
  //////////////////////////////////
  /// \Brief Sets number of samples to be read per block
  void setBlkSize(int blkSize) { blkSize_ = blkSize; }

  /// \Brief Sets number of files to be combined
  void setNumberOfFiles(int numberOfFiles) { numberOfFiles_ = numberOfFiles; }

  //////////////////////////////////
  /////////Getter Functions/////////
  //////////////////////////////////
  /// \Brief Gets number of samples to be read per block
  int getBlkSize() { return blkSize_; }

  /// \Brief Gets file duration in number of samples
  float getFileDurationSamp() { return fileDurationSamp_; }

private:
  /// \Brief Determines the maximum value of all data files and sets
  /// scale value so that combination of signals does not saturate
  void determineMax();

  /// \Brief Validates that all data files are of similar record
  /// values, e.g. same sampling frequencies, IF values, and data formats
  bool validateDataFiles();

  /// \Brief Determines at which block an auxilary file will be activated
  /// and power level at start of event
  void createEventProfile(int idx);

  /// \Brief Checks if current block features an event
  bool checkIfEventActive(int blk, int idx);

  /// \Brief Calls proper IF Data class object to read data file
  void readFile(int idx, Eigen::VectorXf& samplesEigen);

  /// \Brief Opens proper IF Data class object to open data file
  void openFile(int idx);

  /// Private class variables

  /// IF data class objects
  if_data_utils::IfData ifData0_;
  if_data_utils::IfData ifData1_;
  if_data_utils::IfData ifData2_;
  if_data_utils::IfData ifDataOut_;

  /// Event Variables
  std::vector<float> eventActivationBlk_;
  std::vector<float> eventDeactivationBlk_;
  std::vector<float> eventDeltaPowerFrac_;
  std::vector<float> eventPower_;

  /// File Variables
  int                   numberOfFiles_;
  std::vector<dataFile> files_;
  std::vector<float>    fileMaxs_;
  float                 sumScale_{};
  int                   blkSize_;
  int                   numOfLoops_{};
  float                 fileDurationSamp_{};
};
}  // namespace if_data_utils

#endif
