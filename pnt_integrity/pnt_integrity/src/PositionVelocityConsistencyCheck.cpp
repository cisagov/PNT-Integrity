//============================================================================//
//----- pnt_integrity/PositionVelocityConsistencyCheck.cpp -----*- C++ -*-----//
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
// AssurancCheck class defined for the position velocity consistency check
// \author Josh Clanton <josh.clanton@is4s.com>
// \author John David Sprunger <jss0027@tigermail.auburn.edu>
// \date   September 18, 2019
//============================================================================//
#include "pnt_integrity/PositionVelocityConsistencyCheck.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <utility>

namespace pnt_integrity
{
const double PI = 3.14159265358979323846;

//==============================================================================
//------------------------------ handleGnssObservables -------------------------
//==============================================================================
bool PositionVelocityConsistencyCheck::handlePositionVelocity(
  const data::PositionVelocity& /*posVel*/,
  const bool& localFlag)
{
  // data has already been added to the repo by the integrity monitor
  if (localFlag)
  {
    return runCheck();
  }
  else
  {
    return true;
  }
}

//==============================================================================
//--------------------------------- runCheck -----------------------------------
//==============================================================================
bool PositionVelocityConsistencyCheck::runCheck()
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
  std::vector<TimeEntry> timeEntryVec;
  TimeEntry              newestEntry;
  IntegrityDataRepository::getInstance().getNewestEntry(newestEntry);
  double startTime = newestEntry.timeOfWeek_ - sampleWindow_;
  if (IntegrityDataRepository::getInstance().getNewestEntries(timeEntryVec,
                                                              startTime))
  {
    return posVelCheck(newestEntry.timeOfWeek_, timeEntryVec);
  }
  else
  {
    return false;
  }
}

//==============================================================================
//-------------------------------- posVelCheck ---------------------------------
//==============================================================================
bool PositionVelocityConsistencyCheck::posVelCheck(
  const double&                 checkTime,
  const std::vector<TimeEntry>& timeEntryVec)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
  // if not enough samples
  if (timeEntryVec.size() < 2)
  {
    logMsg_(
      "PosVelCons Check: Unavailable, timeEntryVec.size() < minNumSamples_",
      logutils::LogLevel::Debug);
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Unavailable);
    return false;
  }

  data::PositionVelocity   firstPv, secondPv;
  double                   dt;
  data::GeodeticPosition3d firstPos, firstPosPropd, secondPos;
  double                   firstVel[3];
  double                   percentConfident = 0;
  unsigned int             numExaminedPairs = 0;

  PosVelConsCheckDiagnostics checkData;

  for (auto it = timeEntryVec.begin(); it != (timeEntryVec.end() - 1); ++it)
  {
    it->localData_.getData(firstPv);
    (it + 1)->localData_.getData(secondPv);
    // need good first and second pos and good first vel
    if (firstPv.isPositionValid() && firstPv.isVelocityValid() &&
        secondPv.isPositionValid())
    {
      ++numExaminedPairs;
      firstPos = firstPv.position;
      memcpy(firstVel, firstPv.velocity, 3 * sizeof(double));
      secondPos = secondPv.position;
      // get delta T
      dt = (it + 1)->timeOfWeek_ - it->timeOfWeek_;

      // propagate first position forward based on its velocity
      firstPosPropd = propagatePosition(firstPos, firstVel, dt);

      double firstVelVar = sqrt(pow(firstPv.covariance[3][3], 2) +
                                pow(firstPv.covariance[4][4], 2) +
                                pow(firstPv.covariance[5][5], 2));

      double error  = calculateDistance(secondPos, firstPosPropd);
      double thresh = errorThreshScaleFactor_ * sqrt(firstVelVar);
      checkData.errorVals.push_back(error);
      checkData.errorThresh.push_back(thresh);
      std::stringstream msg;
      msg << "Pos-Vel Consistency Check: error: " << error
          << " (m), thresh: " << thresh;
      logMsg_(msg.str(), logutils::LogLevel::Debug);
      if (!checkDistance(error, thresh))
      {
        ++percentConfident;
      }
    }
  }

  // if not enough examined pairs
  if (numExaminedPairs < 2)
  {
    logMsg_(
      "PosVelCons Check: Unavailable, numExaminedPairs < minNumSamples_ - 1",
      logutils::LogLevel::Debug);
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Unavailable);
    return false;
  }

  percentConfident           = percentConfident / numExaminedPairs;
  double percentNotConfident = 1 - percentConfident;

  logMsg_("PosVelConsCheck: percentNotConfident: " +
            std::to_string(percentNotConfident),
          logutils::LogLevel::Debug);
  if (percentNotConfident > assuranceUnassuredThresh_)
  {
    logMsg_("PosVelCons Check: Unassured", logutils::LogLevel::Debug);
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Unassured);
  }
  else if (percentNotConfident > assuranceInconsistentThresh_)
  {
    logMsg_("PosVelCons Check: Inconsistent", logutils::LogLevel::Debug);
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Inconsistent);
  }
  else
  {
    logMsg_("PosVelCons Check: Assured", logutils::LogLevel::Debug);
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Assured);
  }

  checkData.percentBad         = percentNotConfident;
  checkData.inconsistentThresh = assuranceInconsistentThresh_;
  checkData.unassuredThresh    = assuranceUnassuredThresh_;

  if (publishDiagnostics_)
  {
    // publish the message
    publishDiagnostics_(checkTime, checkData);
  }
  return true;
}  // end posVelCheck

//==============================================================================
//--------------------------- propagatePosition --------------------------------
//==============================================================================
data::GeodeticPosition3d PositionVelocityConsistencyCheck::propagatePosition(
  data::GeodeticPosition3d p,
  double*                  v,
  double                   dt)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
  
  double                   metersPerDegLat = 111111.0;
  double                   metersPerRadLat = metersPerDegLat * 180 / PI;
  double                   metersPerDegLon = metersPerDegLat * cos(p.latitude);
  double                   metersPerRadLon = metersPerDegLon * 180 / PI;
  data::GeodeticPosition3d retPos;

  retPos.latitude  = p.latitude + v[0] * dt / metersPerRadLat;
  retPos.longitude = p.longitude + v[1] * dt / metersPerRadLon;
  retPos.altitude  = p.altitude + v[2] * dt;
  return retPos;
}

}  // namespace pnt_integrity
