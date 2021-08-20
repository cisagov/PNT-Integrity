//============================================================================//
//----------------- pnt_integrity/GPSNavDataCommon.hpp ---------*- C++ -*-----//
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
/// \brief    Common structures and functions used in processing GPS LNAV data
/// \author   David Hodo <david.hodo@is4s.com>
/// \date     April 2021
//============================================================================//
#include "pnt_integrity/GPSNavDataCommon.hpp"

#include <cstring>
#include <iomanip>
#include <sstream>

namespace pnt_integrity
{
// ------------------------------------------------------------------
/*!
 Convert a hex string to a block of data
 */
void fromHex(const std::string& in,  //!< Input hex string
             void* const        data)       //!< Data store
{
  size_t         length   = in.length();
  unsigned char* byteData = reinterpret_cast<unsigned char*>(data);

  std::stringstream hexStringStream;
  hexStringStream >> std::hex;

  for (size_t strIndex = 0, dataIndex = 0; strIndex < length; ++dataIndex)
  {
    // Read out and convert the string two characters at a time
    const char tmpStr[3] = {in[strIndex++], in[strIndex++], 0};

    // Reset and fill the string stream
    hexStringStream.clear();
    hexStringStream.str(tmpStr);

    // Do the conversion
    int tmpValue = 0;
    hexStringStream >> tmpValue;
    byteData[dataIndex] = static_cast<unsigned char>(tmpValue);
  }

}  // fromHex

// ------------------------------------------------------------------
/*!
 Convert a block of data to a hex string
 */
void toHex(unsigned char* const byteData,    //!< Data to convert
           const size_t         dataLength,  //!< Length of the data to convert
           std::string&         dest)                //!< Destination string
{
  std::stringstream hexStringStream;

  hexStringStream << std::hex << std::setfill('0');
  for (size_t index = 0; index < dataLength; ++index)
    hexStringStream << std::setw(2) << static_cast<int>(byteData[index]);
  dest = hexStringStream.str();
}

//-----------------------------------------------------------------------------
void convertSubframeFrom10To30Word(const uint32_t (&sfIn)[10],
                                   uint8_t (&sfOut)[30])
{
  // Store subframe data into 30 byte array of 8 bits to remove zero pads
  memset(sfOut, 0, sizeof(sfOut));  // Set sf to 0

  for (size_t kk = 0; kk < 10; ++kk)
  {
    size_t jj     = kk * 3;
    sfOut[jj + 2] = (uint8_t)sfIn[kk];
    sfOut[jj + 1] = (uint8_t)(sfIn[kk] >> 8);
    sfOut[jj + 0] = (uint8_t)(sfIn[kk] >> 16);
  }
}

//-----------------------------------------------------------------------------
void convertSubframeFrom30To10Word(const uint8_t (&sfIn)[30],
                                   uint32_t (&sfOut)[10])
{
  for (size_t ww = 0; ww < 10; ++ww)
  {
    size_t jj = ww * 3;

    sfOut[ww] = 0;
    sfOut[ww] |= ((uint32_t)(sfIn[jj])) << 16;
    sfOut[ww] |= ((uint32_t)(sfIn[jj + 1])) << 8;
    sfOut[ww] |= ((uint32_t)(sfIn[jj + 2]));
  }
}

//-----------------------------------------------------------------------------
void removeSubframeParity(const uint32_t (&subframeWordsIn)[10],
                          uint32_t (&subframeWordsOut)[10])
{
  for (size_t kk = 0; kk < 10; ++kk)
  {
    subframeWordsOut[kk] = (subframeWordsIn[kk] >> 6);
  }
}

//------------------------------------------------------------------------------
uint16_t parseSubframeID(const uint8_t (&subframe)[30])
{
  uint16_t sfid;

  parseSubframeID(subframe, sfid);

  return (sfid);
}

//------------------------------------------------------------------------------
void parseSubframeID(const uint8_t (&subframe)[30], uint16_t& subframeID)
{
  subframeID = ((uint16_t)((subframe[5] >> 2) & 0x7));
}

//------------------------------------------------------------------------------
void parseTimeOfWeek(const uint8_t (&subframe)[30], double& tow)
{
  // Time of week
  uint32_t tmp32 = 0;
  tmp32 |= subframe[3] << 11;
  tmp32 |= subframe[4] << 3;
  tmp32 |= (subframe[5] & 0x80) >> 5;
  tow = (tmp32 * 3) / 2.0;
}

//------------------------------------------------------------------------------
double parseTimeOfWeek(const uint8_t (&subframe)[30])
{
  double tow;

  parseTimeOfWeek(subframe, tow);

  return (tow);
}

}  // namespace pnt_integrity