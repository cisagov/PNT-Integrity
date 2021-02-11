//============================================================================//
//------------------------ if_data_utils/combineFiles.cpp -----------*- C++
//-*-----//
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

#include "if_data_utils/FileMux.hpp"
#include "if_data_utils/IniReader.hpp"
#include "if_data_utils/UpConvert.hpp"
#include "if_data_utils/dataFileStructures.hpp"

#include <iostream>
#include <vector>

int main(int argc, char* argv[])
{
  if (argc > 5)
  {
    std::cout << "Too many input arguments" << std::endl;
    return 0;
  }
  if (argc < 4)
  {
    std::cout << "Not enough input arguments" << std::endl;
    return 0;
  }
  // Make data and event structure for files
  std::string liveSkyFile = argv[1];
  INIReader   reader(liveSkyFile);
  if (reader.ParseError() < 0)
  {
    std::cout << "Can't load metadata file: " << liveSkyFile << std::endl;
    return 0;
  }
  dataFile liveSky;

  liveSky.fileName   = reader.Get("ifDataFile", "fileName", "");
  liveSky.dataFormat = reader.Get("ifDataFile", "data_format", "");
  liveSky.sampFreq =
    reader.GetInteger("ifDataFile", "sampling_frequency", 16368000);
  liveSky.ifFreq          = reader.GetInteger("ifDataFile", "if_frequency", 0);
  liveSky.isComplex       = reader.GetBoolean("ifDataFile", "isComplex", false);
  liveSky.skipSeconds     = reader.GetInteger("ifDataFile", "skip_seconds", 0);
  liveSky.event.startTime = reader.GetInteger("ifDataFile", "start_time", 0);
  liveSky.event.endTime = reader.GetInteger("ifDataFile", "end_time", 1000000);
  liveSky.event.startPower =
    reader.GetInteger("ifDataFile", "start_power", 100);
  liveSky.event.endPower = reader.GetInteger("ifDataFile", "end_power", 100);

  if_data_utils::FileMux fileMux;
  fileMux.addFile(liveSky);
  int numberOfAuxFiles = argc - 3;
  for (int ii = 2; ii < (2 + numberOfAuxFiles); ii++)
  {
    std::string jammerFile = argv[ii];
    INIReader   reader2(jammerFile);
    if (reader2.ParseError() < 0)
    {
      std::cout << "Can't load metadata file: " << liveSkyFile << std::endl;
      return 0;
    }
    dataFile jammer;

    jammer.fileName   = reader2.Get("ifDataFile", "fileName", "");
    jammer.dataFormat = reader2.Get("ifDataFile", "data_format", "");
    jammer.sampFreq =
      reader2.GetInteger("ifDataFile", "sampling_frequency", 16368000);
    jammer.ifFreq      = reader2.GetInteger("ifDataFile", "if_frequency", 0);
    jammer.isComplex   = reader2.GetBoolean("ifDataFile", "isComplex", false);
    jammer.skipSeconds = reader2.GetInteger("ifDataFile", "skip_seconds", 0);
    jammer.event.startTime = reader2.GetInteger("ifDataFile", "start_time", 0);
    jammer.event.endTime =
      reader2.GetInteger("ifDataFile", "end_time", 1000000);
    jammer.event.startPower =
      reader2.GetInteger("ifDataFile", "start_power", 100);
    jammer.event.endPower = reader2.GetInteger("ifDataFile", "end_power", 100);

    fileMux.addFile(jammer);
  }

  std::string outfile = argv[argc - 1];
  fileMux.combineFiles(outfile);

  return 0;
}
