//============================================================================//
//------------------------ if_data_utils/test.cpp -----------*- C++ -*-----//
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
#include <iostream>
#include <vector>

#include "if_data_utils/FileMux.hpp"
#include "if_data_utils/UpConvert.hpp"
#include "if_data_utils/dataFileStructures.hpp"

int main()
{
  // Make data and event structure for files
  dataFile liveSky;
  liveSky.fileName         = "liveSky.bin";
  liveSky.dataFormat       = "sc8";
  liveSky.sampFreq         = 5000000;
  liveSky.ifFreq           = 0;
  liveSky.isComplex        = true;
  liveSky.skipSeconds      = 0;
  liveSky.event.startTime  = 0;
  liveSky.event.endTime    = 1;
  liveSky.event.startPower = 100;
  liveSky.event.endPower   = 100;

  dataFile jammer;
  jammer.fileName         = "jammer.bin";
  jammer.dataFormat       = "sc8";
  jammer.sampFreq         = 20000;
  jammer.ifFreq           = 0;
  jammer.isComplex        = true;
  jammer.skipSeconds      = 0;
  jammer.event.startTime  = 10;
  jammer.event.endTime    = 20;
  jammer.event.startPower = 100;
  jammer.event.endPower   = 0;

  // if_data_utils::FileMux fileMux;
  // fileMux.addFile(liveSky);
  // fileMux.addFile(jammer);

  // std::vector<dataFile> dataFiles;

  // dataFiles.push_back(liveSky);
  // dataFiles.push_back(jammer);

  // std::string outfile = "combine.bin";
  // fileMux.combineFiles(outfile);

  // std::cout << "Combined data files!!!" << std::endl;
  if_data_utils::UpConvert upconvert;
  upconvert.addFile(liveSky);
  std::string outfile = "test.bin";
  upconvert.upconvertFile(outfile);
  return 0;
}
