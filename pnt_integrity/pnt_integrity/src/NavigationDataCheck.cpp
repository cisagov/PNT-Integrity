//============================================================================//
//-------------- pnt_integrity/NavigationDataCheck.hpp ---------*- C++ -*-----//
//============================================================================//
// BSD 3-Clause License
//
// Copyright (C) 2021 Integrated Solutions for Systems, Inc
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
//  AssuranceCheck class for checking broadcast navigation data
//  David Hodo <david.hodo@is4s.com>
//  May 14, 2021
//============================================================================//

#include "pnt_integrity/NavigationDataCheck.hpp"
#include <algorithm>
#include <iomanip>  // setfill, setw
#include "pnt_integrity/GPSAlmanac.hpp"
#include "pnt_integrity/GPSEphemeris.hpp"
#include "pnt_integrity/GPSNavDataCommon.hpp"

namespace pnt_integrity
{
//==============================================================================
//--------------------------- handleGnssSubframe -------------------------------
//==============================================================================
bool NavigationDataCheck::handleGnssSubframe(
  const data::GNSSSubframe& gnssSubframe)
{
  // Filter out garbage CNAV subframes
  if (*gnssSubframe.subframeData.begin() != 0x8B)
  {
    std::stringstream cnavOut;
    cnavOut << "CNAV data filtered, preamble = 0x" << std::setfill('0')
            << std::uppercase << std::setw(2) << std::hex
            << (int)*gnssSubframe.subframeData.begin();
    logMsg_(cnavOut.str(), logutils::LogLevel::Debug);
    return false;
  }

  if (gnssSubframe.subframeData.size() != 30)
  {
    std::stringstream msg;
    msg << "NavigationDataCheck: Received GNSS Subframe with unexpected length:"
        << gnssSubframe.subframeData.size();
    logMsg_(msg.str(), logutils::LogLevel::Warn);
    return false;
  }

  double newTimestamp =
    (double)gnssSubframe.header.timestampValid.sec +
    (double)(gnssSubframe.header.timestampValid.nanoseconds) / 1e9;

  NavDataCheckDiagnostics diagnostics;

  // Copy into byte array for parsing
  uint8_t subframeBytes[30];
  std::copy(gnssSubframe.subframeData.begin(),
            gnssSubframe.subframeData.end(),
            subframeBytes);

  // parse subframe id
  uint16_t subframeId = parseSubframeID(subframeBytes);

  // extract the subframe tow for the received subframe
  // process is slightly different for different subframes
  // (almanac vs ephemeris)
  int prn = gnssSubframe.prn;

  double   newTowSec     = std::numeric_limits<double>::min();
  uint16_t newWeekNumber = std::numeric_limits<uint16_t>::max();
  if ((subframeId >= 1) && (subframeId <= 3))
  {
    // parse ephemeris
    GpsEphemeris ephemeris;
    if (ephemeris.setSubframe(prn, subframeBytes))
    {
      // we have a valid ephemeris subframe
      diagnostics.dataValid = true;

      std::stringstream validMsg;
      validMsg << "Valid ephem received for prn: " << prn;
      logMsg_(validMsg.str(), logutils::LogLevel::Debug);

      // extract TOW value from the apropriate subframe
      switch (subframeId)
      {
        case 1:
        {
          newTowSec     = ephemeris.getTowSf1();
          newWeekNumber = ephemeris.getWeekNumber();
          break;
        }
        case 2:
        {
          newTowSec = ephemeris.getTowSf2();
          break;
        }
        case 3:
        {
          newTowSec = ephemeris.getTowSf3();
          break;
        }
      }
    }
    else
    {
      // data for this subframe is not valid
      diagnostics.dataValid = false;
      std::stringstream dataMsg;
      dataMsg << getSubframeErrorStr(prn, subframeId, ephemeris);
      logMsg_(dataMsg.str(), logutils::LogLevel::Warn);
      diagnostics.dataValidMsg = dataMsg.str();
    }
  }
  else if ((subframeId >= 4) && (subframeId <= 5))
  {
    // get tow from almanac
    GpsAlmanac almanac;
    if (almanac.setSubframe(prn, subframeBytes))
    {
      // we have a valid prn / svid
      newTowSec = almanac.getTow();

      // now we need to check if there were any subframe faults
      std::string sfErrorStr;
      if (getSubframeErrorStr(prn, subframeId, almanac, sfErrorStr))
      {
        // error was found
        diagnostics.dataValid    = false;
        diagnostics.dataValidMsg = sfErrorStr;
        logMsg_(sfErrorStr, logutils::LogLevel::Warn);
      }
      else
      {
        // no error was found
        std::stringstream validMsg;
        validMsg << "Valid almanac received for prn: " << gnssSubframe.prn;
        logMsg_(validMsg.str(), logutils::LogLevel::Debug);
        diagnostics.dataValid    = true;
        diagnostics.dataValidMsg = "";
      }
    }
    else
    {
      // not an almanac subframe, set tow to nan to skip check below
      newTowSec     = std::numeric_limits<double>::quiet_NaN();
      newWeekNumber = std::numeric_limits<uint16_t>::max();

      // // we have an invalid prn / svid
      // std::stringstream validMsg;
      // validMsg << "Invalid PRN/SVID received for prn: " << gnssSubframe.prn
      //          << " in subframe " << subframeId;
      // logMsg_(validMsg.str(), logutils::LogLevel::Warn);
      // // data is not invalid but we want to show a warning in the diagnostics
      diagnostics.dataValid    = true;
      diagnostics.dataValidMsg = "";
    }
  }
  else
  {
    // not a valid subframeID
    diagnostics.dataValid = false;
    std::stringstream dataMsg;
    dataMsg << "Invalid subframe ID received for PRN " << prn << " ("
            << subframeId << ")";
    diagnostics.dataValidMsg = dataMsg.str();
    logMsg_(dataMsg.str(), logutils::LogLevel::Warn);
  }

  // if we have valid data check the TOW offset from system clock
  // It should remain relatively consistent from subframe to subframe
  // If the offset differs, then flag
  if (diagnostics.dataValid)
  {
    if ((lastTow_ != std::numeric_limits<uint32_t>::max()) &&
        (lastTowTimeStamp_ != std::numeric_limits<double>::quiet_NaN()))
    {
      double timeSinceTowReceived = newTimestamp - lastTowTimeStamp_;
      double newTow               = lastTow_ + timeSinceTowReceived;

      // determine if time of week has rolled over
      long numRollovers = 0;
      if (newTow >= 604800)
      // time of week should have rolled over
      {
        numRollovers = newTow / 604800;
      }

      // check time of week
      if (std::isnan(towOffset_))
      {
        // set the initial offset
        towOffset_           = newTimestamp - newTowSec;
        diagnostics.towValid = true;
      }
      // skip check if newTowSec is nan
      else if (!std::isnan(newTowSec))
      {
        // compare the new offset to the original (adjusting for any rollovers)
        double newOffset = newTimestamp - newTowSec;
        // Calculate Diff between current and last TOW Offset, accounting for
        // possible week rollovers
        double offsetDiff =
          std::abs(newOffset - towOffset_ - numRollovers * 604800);
        if (offsetDiff <= 2.0)
        {
          // offset is acceptable
          diagnostics.towValid = true;
          towOffset_           = newOffset;
        }
        else
        {
          // offset is not acceptable
          diagnostics.towValid = false;
          std::stringstream towMsg;
          towMsg << "PRN " << prn << ", SF" << subframeId
                 << ": TOW offset greater than threshold ("
                 << "New Offset = " << newOffset
                 << ", Old Offset = " << towOffset_ << ", diff = " << offsetDiff
                 << ")";
          diagnostics.towValidMsg = towMsg.str();
          logMsg_(diagnostics.towValidMsg, logutils::LogLevel::Warn);
        }
      }
    }

    // check week number
    if (subframeId == 1)
    {
      if ((lastWeekNumber_ != std::numeric_limits<uint16_t>::max()) &&
          (lastTow_ != std::numeric_limits<uint32_t>::max()) &&
          (lastTowTimeStamp_ != std::numeric_limits<double>::quiet_NaN()))
      {
        // we have received a new week number from subframe 1
        // so we need to check it

        double timeSinceTowReceived = newTimestamp - lastTowTimeStamp_;
        double newTow               = lastTow_ + timeSinceTowReceived;
        int    expectedDelta        = (int)std::floor(newTow / 604800.0);

        // The expected delta is the number of weeks since the last valid
        // week number was received, check for week number rollover
        bool wnro = false;
        if ((lastWeekNumber_ + expectedDelta) > 1023)
        {
          wnro = true;
        }

        // if the new week number is different than the previous
        // make sure the delta is correct (1)

        int weekNumberDelta = newWeekNumber - lastWeekNumber_;

        // adjust the delta if we have rolled over
        if (wnro)
          weekNumberDelta += 1024;

        if (expectedDelta != weekNumberDelta)
        {
          std::stringstream wnMsg;
          wnMsg << "Bad Week Number (" << newWeekNumber
                << "), Expected week number delta " << expectedDelta
                << " but received " << weekNumberDelta;
          diagnostics.wnValid    = false;
          diagnostics.wnValidMsg = wnMsg.str();
          logMsg_(diagnostics.wnValidMsg, logutils::LogLevel::Warn);
        }
      }

      lastWeekNumber_ = newWeekNumber;
    }
    // Store TOW timestamps only from valid ephemeris subframes
    if (((subframeId == 1) || (subframeId == 2) || (subframeId == 3)) &&
        diagnostics.dataValid)
    {
      lastTow_          = newTowSec;
      lastTowTimeStamp_ = newTimestamp;
    }
  }

  // diagnostics_ = diagnostics;
  diagBuffer_.push_back(diagnostics);
  return true;
}

//==============================================================================
//--------------------------------- runCheck -----------------------------------
//==============================================================================
void NavigationDataCheck::runCheckThread()
{
  // Calls runCheck() at 1Hz until exitSignal is called in destructor
  while (futureObj_.wait_for(std::chrono::milliseconds(1000)) ==
         std::future_status::timeout)
  {
    runCheck();
  }
}

bool NavigationDataCheck::runCheck()
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  auto current_time = std::chrono::steady_clock::now();
  auto duration_in_seconds =
    std::chrono::duration<double>(current_time.time_since_epoch());

  double updateTime = duration_in_seconds.count();

  NavDataCheckDiagnostics combinedDiagnostics;
  combinedDiagnostics.dataValid = true;
  combinedDiagnostics.towValid  = true;

  for (auto diagIt = diagBuffer_.begin(); diagIt != diagBuffer_.end(); ++diagIt)
  {
    combinedDiagnostics.dataValid &= diagIt->dataValid;
    combinedDiagnostics.towValid &= diagIt->towValid;
    combinedDiagnostics.wnValid &= diagIt->wnValid;
    if (!diagIt->dataValidMsg.empty())
      combinedDiagnostics.dataValidMsg += (diagIt->dataValidMsg + ". ");
    if (!diagIt->towValidMsg.empty())
      combinedDiagnostics.towValidMsg += (diagIt->towValidMsg + ". ");
    if (!diagIt->wnValidMsg.empty())
      combinedDiagnostics.wnValidMsg += (diagIt->wnValidMsg + ". ");
  }
  diagBuffer_.clear();

  if (!combinedDiagnostics.dataValid || !combinedDiagnostics.towValid ||
      !combinedDiagnostics.wnValid)
  {
    changeAssuranceLevel(updateTime, data::AssuranceLevel::Unassured);
  }
  else
  {
    changeAssuranceLevel(updateTime, data::AssuranceLevel::Assured);
  }

  if (publishDiagnostics_)
  {
    publishDiagnostics_(updateTime, combinedDiagnostics);
  }
  return true;
}

std::string NavigationDataCheck::getSubframeErrorStr(int      prn,
                                                     uint16_t subframeId,
                                                     const GpsEphemeris& ephem)
{
  std::stringstream sfErrorStr;
  sfErrorStr << ", PRN " << prn << " : ";
  switch (subframeId)
  {
    case 1:
    {
      sfErrorStr << getSubframe1ErrorStr(ephem.getSubframe1Faults());
      break;
    }
    case 2:
    {
      sfErrorStr << getSubframe2ErrorStr(ephem.getSubframe2Faults());
      break;
    }
    case 3:
    {
      sfErrorStr << getSubframe3ErrorStr(ephem.getSubframe3Faults());
      break;
    }
  }

  return sfErrorStr.str();
}

std::string NavigationDataCheck::getSubframe1ErrorStr(
  pnt_integrity::Subframe1Fault faults)
{
  std::stringstream faultStr;
  if (faults.towSf1)
    faultStr << "SF1: bad TOW,";
  if (faults.weekNumber)
    faultStr << "SF1: bad week number,";
  if (faults.codeOnL2)
    faultStr << "SF1: bad codeOnL2,";
  if (faults.uraIndex)
    faultStr << "SF1: bad uraIndex,";
  if (faults.svHealth)
    faultStr << "SF1: bad svHealth,";
  if (faults.iodc)
    faultStr << "SF1: bad IODC,";
  if (faults.l2PDataFlag)
    faultStr << "SF1: bad L2P Data Flag,";
  if (faults.groupDelay)
    faultStr << "SF1: bad group delay,";
  if (faults.clockCorrectionTime)
    faultStr << "SF1: bad clockCorrectionTime,";
  if (faults.clockAging3)
    faultStr << "SF1: bad clock aging 3,";
  if (faults.clockAging2)
    faultStr << "SF1: bad clock aging 2,";
  if (faults.clockAging1)
    faultStr << "SF1: bad clock aging 1,";
  return faultStr.str();
}

std::string NavigationDataCheck::getSubframe2ErrorStr(
  pnt_integrity::Subframe2Fault faults)
{
  std::stringstream faultStr;

  if (faults.towSf2)
    faultStr << "SF2: bad TOW,";
  if (faults.iodeSf2)
    faultStr << "SF2: bad IODE,";
  if (faults.sinOrbitRadius)
    faultStr << "SF2: bad sin orbit radius,";
  if (faults.meanMotionDifference)
    faultStr << "SF2: bad mean motion difference,";
  if (faults.meanAnomaly)
    faultStr << "SF2: bad mean anomolay,";
  if (faults.cosLatitude)
    faultStr << "SF2: bad cos latitude,";
  if (faults.eccentricity)
    faultStr << "SF2: bad eccentricity,";
  if (faults.sinLatitude)
    faultStr << "SF2: bad sin latitude,";
  if (faults.sqrtSemiMajorAxis)
    faultStr << "SF2: bad sqrt semi major axis,";
  if (faults.timeOfEphemeris)
    faultStr << "SF2: bad time of ephem,";
  if (faults.fitInterval)
    faultStr << "SF2: bad fit internal,";
  if (faults.ageOfDataOffset)
    faultStr << "SF2: bad age of data offset,";
  return faultStr.str();
}

std::string NavigationDataCheck::getSubframe3ErrorStr(
  pnt_integrity::Subframe3Fault faults)
{
  std::stringstream faultStr;

  if (faults.towSf3)
    faultStr << "SF3: bad TOW, ";
  if (faults.cosInclination)
    faultStr << "SF3: bad cos inclination, ";
  if (faults.rightAscension)
    faultStr << "SF3: bad right ascension, ";
  if (faults.sinInclination)
    faultStr << "SF3: bad sin inclination, ";
  if (faults.inclinationAngle)
    faultStr << "SF3: bad inclination angle, ";
  if (faults.cosOrbitRadius)
    faultStr << "SF3: bad cos orbit radius, ";
  if (faults.argumentOfPerigee)
    faultStr << "SF3: bad argument of perigree, ";
  if (faults.ascensionRate)
    faultStr << "SF3: bad ascension rate, ";
  if (faults.iodeSf3)
    faultStr << "SF3: bad IODE, ";
  if (faults.inclinationRate)
    faultStr << "SF3: bad inclination rate, ";

  return faultStr.str();
}

bool NavigationDataCheck::getSubframeErrorStr(int               prn,
                                              uint16_t          subframeId,
                                              const GpsAlmanac& almanac,
                                              std::string&      errorStr)
{
  bool sfErrorFound = false;

  std::stringstream sfErrorStr;
  sfErrorStr << ", PRN " << prn << " SF " << subframeId;

  AlmanacSubframeFaults faults = almanac.getSubframeFaults();

  if (faults.faultType.prn)
  {
    sfErrorStr << "bad prn,";
    sfErrorFound |= true;
  }
  if (faults.faultType.tow)
  {
    sfErrorStr << "bad tow,";
    sfErrorFound |= true;
  }
  if (faults.faultType.svHealth)
  {
    sfErrorStr << "bad sv health,";
    sfErrorFound |= true;
  }
  if (faults.faultType.eccentricity)
  {
    sfErrorStr << "bad eccentricity,";
    sfErrorFound |= true;
  }
  if (faults.faultType.toa)
  {
    sfErrorStr << "bad toa,";
    sfErrorFound |= true;
  }
  if (faults.faultType.deltaI)
  {
    sfErrorStr << "bad deltaI,";
    sfErrorFound |= true;
  }
  if (faults.faultType.omegaDot)
  {
    sfErrorStr << "bad omegaDot,";
    sfErrorFound |= true;
  }
  if (faults.faultType.sqrtA)
  {
    sfErrorStr << "bad sqrtA,";
    sfErrorFound |= true;
  }
  if (faults.faultType.omega0)
  {
    sfErrorStr << "bad omega0,";
    sfErrorFound |= true;
  }
  if (faults.faultType.omega)
  {
    sfErrorStr << "bad omega,";
    sfErrorFound |= true;
  }
  if (faults.faultType.m0)
  {
    sfErrorStr << "bad m0,";
    sfErrorFound |= true;
  }
  if (faults.faultType.af0)
  {
    sfErrorStr << "bad af0,";
    sfErrorFound |= true;
  }
  if (faults.faultType.af1)
  {
    sfErrorStr << "bad af1,";
    sfErrorFound |= true;
  }
  if (faults.faultType.referenceWeek)
  {
    sfErrorStr << "bad reference week,";
    sfErrorFound |= true;
  }

  errorStr = sfErrorStr.str();

  return sfErrorFound;
}

}  // namespace pnt_integrity
