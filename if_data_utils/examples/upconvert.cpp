//============================================================================//
//------------------------ if_data_utils/upconvert.cpp -----------*- C++
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
#include "if_data_utils/UpConvert.hpp"
#include "if_data_utils/IniReader.hpp"
#include "if_data_utils/dataFileStructures.hpp"

#include <iostream>
#include <vector>

int main(int argc, char* argv[])
{
  if (argc > 3)
  {
    std::cout << "Too many input arguments" << std::endl;
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
  dataFile ifFile;

  ifFile.fileName   = reader.Get("ifDataFile", "fileName", "");
  ifFile.dataFormat = reader.Get("ifDataFile", "data_format", "");
  ifFile.sampFreq =
    reader.GetInteger("ifDataFile", "sampling_frequency", 16368000);
  ifFile.ifFreq          = reader.GetInteger("ifDataFile", "if_frequency", 0);
  ifFile.isComplex       = reader.GetBoolean("ifDataFile", "isComplex", false);
  ifFile.skipSeconds     = reader.GetInteger("ifDataFile", "skip_seconds", 0);
  ifFile.event.startTime = reader.GetInteger("ifDataFile", "start_time", 0);
  ifFile.event.endTime   = reader.GetInteger("ifDataFile", "end_time", 1000000);
  ifFile.event.startPower = reader.GetInteger("ifDataFile", "start_power", 100);
  ifFile.event.endPower   = reader.GetInteger("ifDataFile", "end_power", 100);

  // Upconvert Signal
  if_data_utils::UpConvert upconvert;
  upconvert.addFile(ifFile);
  std::string outfile = argv[2];
  upconvert.upconvertFile(outfile);
  return 0;
}
