//============================================================================//
//------------------------ if_data_utils/IfData.hpp ------------*- C++ -*-----//
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
/// \brief    This file contains the declaration of the IfData class.
/// \details  This class provides an object for accessing IF data files
/// \author   David Hodo <david.hodo@is4s.com>
/// \date     May 27, 2015
///
//===----------------------------------------------------------------------===//
#ifndef IF_DATA_UTILS_IFDATA_HPP
#define IF_DATA_UTILS_IFDATA_HPP

#include <Eigen/Dense>
#include <fstream>
#include <queue>
#include <string>

namespace if_data_utils
{
// Convert a 4 sample byte from the MAX2769 front end into four signed values
void convertSampleToSigned(uint8_t byte, int8_t samples[4]);

/// Structure to define settings
struct Settings
{
  float    samplingFreq{};           //!< Number of samples per second [Hz]
  float    intermediateFrequency{};  //!< Intermediate frequency [hz]
  float    bytesPerSample{};     //!< Number of bytes per data samples (I & Q)
  bool     complex{};            //!< True if I and Q data is present
  uint64_t sampleCountOffset{};  //!< Sample counter value for the first sample
  std::string dataFilename;
};

/// \brief Class representation for a IfData object
class IfData
{
public:
  /// \brief Constructs the IFData object
  IfData();

  ~IfData();

  bool open(Settings settings);

  void openOutFile(const std::string& metadataFilename);
  void writeSamples(Eigen::RowVectorXi& samplesEigen, std::size_t sampleSize);
  void close();

  /// Reads numSamples samples from the file and store them in the passed in
  /// Eigen vector object (samplesEigen).  Returns the sample count value for
  /// the first sample in the vector
  uint64_t getSamples(Eigen::VectorXf& samplesEigen);

  uint64_t getComplexSamples(Eigen::VectorXf samplesEigen);

  /// Gets the sample count for the next sample to be read from the file
  uint64_t getSampleCount() { return sampleCount_; }

  /// Moves the file pointer to the given sample count value
  bool setSampleCount(uint64_t sampleCount);

  size_t getFilePosition() { return samplesFile_.tellg(); }

  bool skipSamples(size_t numSamples);

  bool isOpen() { return samplesFile_.is_open(); }

  bool isComplex() { return settings_.complex; }

  size_t getFilesize() { return filesize_; }

  size_t getFilesizeInSamples() { return filesize_ / settings_.bytesPerSample; }

  bool isEof() { return samplesFile_.eof(); }

  Settings getSettings() { return settings_; }

  /// \Brief Indicates how much of the data file has been read
  /// \returns the percentage read for the currently open file [0, 1]
  double getPercentRead();

private:
  Settings settings_;

  // Sample count value for the next sample to be read from the file
  // (or the first sample stored in leftover samples)
  uint64_t sampleCount_;

  std::ifstream samplesFile_;

  std::ofstream outFile_;

  size_t filesize_;

  size_t samplesPerRead_;
  size_t bytesPerRead_;

  // Samples read from the file during the last call to getSamples but
  // not returned in the Eigen vector
  std::queue<float> leftoverSamples_;
};
}  // namespace if_data_utils
#endif
