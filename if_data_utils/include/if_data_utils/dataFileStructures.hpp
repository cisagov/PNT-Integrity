//============================================================================//
//----------------- if_data_utils/dataFileStructures.hpp -------*- C++ -*-----//
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
/// \brief    This file contains IF data file stuctures.
/// \details  This class contains methods used in the implementation of
///           combining two IF files together to simulate a GPS attack
/// \author   Joshua Starling <josh.starling@is4s.com>
/// \version  0.1
/// \date     April 27, 2018
///
//===----------------------------------------------------------------------===//
#ifndef IF_DATA_UTILS_DATAFILESTRUCTURES_HPP
#define IF_DATA_UTILS_DATAFILESTRUCTURES_HPP

#include <string>

struct eventData
{
  double startTime;
  double endTime;
  double startPower;
  double endPower;
  double startBlock;
  double endBlock;
};

struct dataFile
{
  std::string fileName;
  std::string dataFormat;
  double      ifFreq{};
  double      sampFreq{};
  bool        isComplex{};
  float       bytesPerSample{};
  float       samplePerMeas{};
  eventData   event{};
  double      skipSeconds{};
  float       maxValue{};
};

#endif
