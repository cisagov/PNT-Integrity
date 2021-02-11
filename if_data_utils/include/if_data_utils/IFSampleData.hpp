//============================================================================//
//------------------- if_data_utils/IFSampleData.hpp -----------*- C++ -*-----//
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
//----------------------------------------------------------------------------//
/// \file
/// \brief    IFSampleData class defined for the a set of IF samples
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     September 30, 2019
//============================================================================//
#ifndef IF_DATA_UTILS__IF_SAMPLE_DATA_HPP
#define IF_DATA_UTILS__IF_SAMPLE_DATA_HPP

#include <complex>
#include <vector>

namespace if_data_utils
{
using SamplesType = std::vector<std::complex<double>>;

using IFSampleSC8  = std::complex<std::int8_t>;
using IFSampleSC16 = std::complex<std::int16_t>;

//==============================================================================
enum class IFSampleType
{
  SC8 = 0,  // Complex 8-bit I and Q (2 Bytes per sample)
  SC16,     // Complex 16-bit I and Q (4 Bytes per sample)
  // sc2, // Complex 2-bit I and Q (2 samples per byte)
  FC32,  // Complex float, 32-bit I and Q (8 Bytes per sample)
  FC64   // Complex double, 64-bit I and Q (16 Bytes per sample)

};

/// \brief A structure for representing a digital IF data sample
///
/// The structure is capable of representing zero-IF, complex data
/// (Baseband I/Q) or non-zero IF samples (I/Q or I only)
struct IFSampleHeader
{
  /// The number of samples in the set
  size_t numSamples_;

  IFSampleType sampleType_;

  /// The sampling frequency used to collect the sample(s)
  double fs_;

  /// The intermediate frequency
  double if_;

  /// Flag to indicate wheter data is standard or complex
  bool complex_;

  /// Number of bytes per sample
  /// Set to 0 if < 1.
  uint16_t bytesPerSample_;

  /// Number of samples per byte. Used if sample is less than 8-bit.
  /// Set to 0 if > 1
  uint16_t samplesPerByte_;

  IFSampleHeader(){};
  /// \brief Sample header constructor
  ///
  /// Constructor for the IF sample hanlder structure. The constructor
  /// auto-populates the complex,bytesPerSample_, and samplesPerByte_
  /// fields based on the provided sample type
  ///
  /// \param numSamples The number of samples in the packet
  /// \param sampleType The data type of the samples
  /// \param intermediateFreq The IF at which the samples were taken
  /// \param samplingFreq The sampling rate (Samples / sec) of the
  ///                     sample data
  IFSampleHeader(const size_t&       numSamples,
                 const IFSampleType& sampleType,
                 const double&       intermediateFreq,
                 const double&       samplingFreq)
    : numSamples_(numSamples)
    , sampleType_(sampleType)
    , fs_(samplingFreq)
    , if_(intermediateFreq)
  {
    switch (sampleType_)
    {
      case IFSampleType::SC8:
        bytesPerSample_ = 2;
        complex_        = true;
        break;

      case IFSampleType::SC16:
        bytesPerSample_ = 4;
        complex_        = true;
        break;

      default:
        // Throw error message?
        break;
    }
  }
};

template <typename samp_type>
class IFSampleData
{
public:
  /// \brief Constructor for the sample data structure
  IFSampleData(){};

  IFSampleData(const IFSampleHeader& header);

  size_t getNumberOfSamples() const { return data_.size(); }

  size_t getNumberOfBytes() const { return totalNumBytes_; }

  samp_type* getBufferPtr() const { return &(data_[0]); }

  double getSampleRate() const { return header_.fs_; }

  IFSampleHeader getHeader() const { return header_; }

  void setHeader(const IFSampleHeader& header)
  {
    header_ = header;
    init();
  }

  // void getSampleSubset(std::vector<samp_type> &subset,
  //                      size_t index1,
  //                      size_t index2);

private:
  /// The packet header
  IFSampleHeader header_;

  void init()
  {
    data_.resize(header_.numSamples_);
    totalNumBytes_ = header_.numSamples_ * header_.bytesPerSample_;
  }

  size_t totalNumBytes_;

  mutable std::vector<samp_type> data_;
};

template <typename samp_type>
IFSampleData<samp_type>::IFSampleData(const IFSampleHeader& header)
  : header_(header)
{
  init();
}

// template <typename samp_type>
// void IFSampleData<samp_type>::getSampleSubset(std::vector<samp_type>& subset,
//                                         size_t index1,
//                                         size_t index2)
// {

// }
}  // namespace if_data_utils
#endif