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

#ifndef PNT_INTEGRITY__GPS_NAV_DATA_COMMON_HPP
#define PNT_INTEGRITY__GPS_NAV_DATA_COMMON_HPP

#include <cstdint>
#include <string>

namespace pnt_integrity
{
/// PI as defined in IS-GPS-200 (30.3.3.1.3)
const double gpsPi = 3.1415926535898;
/// 2 * PI as defined in IS-GPS-200 (convenience constant)
const double twoGpsPi = 2.0 * gpsPi;
/// Speed of light as defined in IS-GPS-200 (20.3.4.3) [m/s]
const double speedOfLight = 2.99792458e8;
/// Earth gravitational constant as defined in IS-GPS-200 (Tbl. 30-II) [m^3/s^2]
const double gpsGM = 3.986005e14;
/// Flattening constant as defined in IS-GPS-200 (20.3.3.3.3) [sec/meter^0.5]
const double gpsF = -4.442807633e-10;
/// Earth rotation rate about ECEF Z-axis (little omega), as defined in
/// IS-GPS-200 (Table 20-IV) [rad/s]
const double gpsEarthRotationRate = 7.2921151467e-5;  // rad/s
/// Number of GPS seconds in a week
const double secondsInWeek     = 604800.0;
/// Number of GPS seconds in a half week
const double secondsInHalfWeek = secondsInWeek / 2.0;

/// Enumeration to define the 5 bit satellite signal health
/// field given in bits 18 to 22 of subframe 1 and in the bottom 5 bits
/// of the 8 bit satellite health field in Almanac subframes 4 and 5
/// Defined in paragraph 20.3.3.5.1.3 of IS-GPS-200
enum class SVSignalHealth : uint8_t
{
  AllSignalsOk                                    = 0,
  AllSignalsWeak                                  = 1,
  AllSignalsDead                                  = 2,
  AllSignalsHaveNoDataModulation                  = 3,
  L1PSignalWeak                                   = 4,
  L1PSignalDead                                   = 5,
  L1PSignalHasNoDataModulation                    = 6,
  L2PSignalWeak                                   = 7,
  L2PSignalDead                                   = 8,
  L2PSignalHasNoDataModulation                    = 9,
  L1CSignalWeak                                   = 10,
  L1CSignalDead                                   = 11,
  L1CSignalHasNoDataModulation                    = 12,
  L2CSignalWeak                                   = 13,
  L2CSignalDead                                   = 14,
  L2CSignalHasNoDataModulation                    = 15,
  L1AndL2PSignal_Weak                             = 16,
  L1AndL2PSignal_Dead                             = 17,
  L1AndL2PSignal_HasNoDataModulation              = 18,
  L1AndL2CSignal_Weak                             = 19,
  L1AndL2CSignal_Dead                             = 20,
  L1AndL2CSignal_HasNoDataModulation              = 21,
  L1SignalWeak                                    = 22,
  L1SignalDead                                    = 23,
  L1SignalHasNoDataModulation                     = 24,
  L2SignalWeak                                    = 25,
  L2SignalDead                                    = 26,
  L2SignalHasNoDataModulation                     = 27,
  SVIsTemporarilyOutDoNotUse                      = 28,
  SVWillBeTemporarilyOutUseWithCaution            = 29,
  OneOrMoreSignalsDeforedURAStillValid            = 30,
  MoreThanOneCombinationNeededToDescribeAnomalies = 31
};

/// Enumeration to define the relative time between multiple LNAV data sets
enum class NavDataTimeOfArrival
{
  Older,
  Same,
  Newer
};

/// \brief  Convert hexadecimal string to char array
void fromHex(const std::string& in,  //!< Input hex string
             void* const        data);

/// \brief  Convert char array to hexadecimal string
void toHex(unsigned char* const byteData,    //!< Data to convert
           const size_t         dataLength,  //!< Length of the data to convert
           std::string&         dest);

/// \brief  Converts uint32_t[10] subframe to uint8_t[30] array
void convertSubframeFrom10To30Word(const uint32_t (&sfIn)[10],
                                   uint8_t (&sfOut)[30]);

/// \brief  Converts uint8_t[30] subframe to uint32_t[10] array
void convertSubframeFrom30To10Word(const uint8_t (&sfIn)[30],
                                   uint32_t (&sfOut)[10]);

/// \brief  Remove parity bits from subframe
void removeSubframeParity(const uint32_t (&subframeWordsIn)[10],
                          uint32_t (&subframeWordsOut)[10]);

/// \brief Parse a subframe and return its ID number
uint16_t parseSubframeID(const uint8_t (&subframe)[30]);
/// \brief Parse a subframe and return its ID number
void     parseSubframeID(const uint8_t (&subframe)[30], uint16_t& subframeID);
/// \brief Parse a subframe and return the time of week
double parseTimeOfWeek(const uint8_t (&subframe)[30]);
/// \brief Parse a subframe and return the time of week
void   parseTimeOfWeek(const uint8_t (&subframe)[30], double& tow);

}  // namespace pnt_integrity

#endif