//============================================================================//
//------------------------ pnt_integrity/repoTestApp.cpp -------*- C++ -*-----//
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
//
//  Test application for the AcquisitionCheck class
//  Josh Clanton <josh.clanton@is4s.com>
//  September 30, 2019
//============================================================================//

#include "if_data_utils/IFDataFileReader.hpp"
#include "logutils/logutils.hpp"
#include "pnt_integrity/AcquisitionCheck.hpp"

#include <complex>
#include <vector>

using namespace if_data_utils;
using namespace logutils;

static bool stop_signal_called = false;
void        sig_int_handler(int)
{
  stop_signal_called = true;
}

int main(int /*argc*/, char** /*argv*/)
{
  // create the check instance
  pnt_integrity::AcquisitionCheck acqCheck;

  // data file for processing
  std::string ifDataFile = "if_data.bin";

  // create the sample header, use 0 for num samples until it is
  // computed below
  IFSampleHeader sampleHeader(0, IFSampleType::SC8, 0.0, 25e6);

  // define the integration period
  double integrationPeriod = 1e-3;
  // the number of periods required for the check
  size_t numIntPeriods = 2;
  // compute the number of samples we are going to read inand store in the
  // header
  sampleHeader.numSamples_ =
    numIntPeriods * integrationPeriod * sampleHeader.fs_;

  // build the data structure
  IFSampleData<std::complex<int8_t> > sampleData(sampleHeader);

  std::stringstream sizeMsg;
  sizeMsg << "Analysis data chunks: " << std::endl;
  sizeMsg << "Num integration periods: " << numIntPeriods << std::endl;
  sizeMsg << "Num samples: " << sampleData.getNumberOfSamples() << std::endl;
  sizeMsg << "Num bytes: " << sampleData.getNumberOfBytes();
  printLogToStdOut(sizeMsg.str(), LogLevel::Info);

  // define the sample buffer to hold the data to be read from the file
  //   std::vector<char> sampleBuffer(readBufferSize);

  // create a pointer to the data type that the file reader is expecting
  if_data_utils::read_element* readElementPtr =
    (if_data_utils::read_element*)(sampleData.getBufferPtr());

  // create the file reader object
  if_data_utils::IFDataFileReader<std::complex<int8_t> > fileReader(
    sampleData.getNumberOfBytes(), logutils::printLogToStdOut);

  // open the IF data file
  fileReader.openFile(ifDataFile);

  // read samples from the file
  fileReader.readSamplesFromFile(*readElementPtr);

  // pass the data to the check
  acqCheck.processIFSampleData(sampleData);

  return 0;
}