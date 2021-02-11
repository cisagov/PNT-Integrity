//============================================================================//
//---------------- pnt_integrity/StaticPositionCheck.cpp -------*- C++ -*-----//
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
// AssurancCheck class defined for the static position check
// Josh Clanton <josh.clanton@is4s.com>
// John David Sprunger <jss0027@tigermail.auburn.edu>
// September 3, 2019
//============================================================================//
#include "pnt_integrity/StaticPositionCheck.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

namespace pnt_integrity
{
//==============================================================================
//------------------------------ handlePositionVelocity ------------------------
//==============================================================================
bool StaticPositionCheck::handlePositionVelocity(
  const data::PositionVelocity& /*posVel*/,
  const bool& localFlag)
{
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
bool StaticPositionCheck::runCheck()
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  TimeEntry timeEntry;
  if (IntegrityDataRepository::getInstance().getNewestEntry(timeEntry))
  {
    // extract the PV data and make sure it's a valid entry and fresh
    data::PositionVelocity pv;
    timeEntry.localData_.getData(pv);
    if (pv.isPositionValid())  // is it valid?
    {
      if (timeEntry.timeOfWeek_ != lastSurveyPointTime_)  // is it fresh?
      {
        // save this data point time
        lastSurveyPointTime_ = timeEntry.timeOfWeek_;
        // if we need to initialize, perform survey
        if (!staticPositionInitialized_)
        {
          positionsToSurvey_.push_back(pv.position);

          if (positionsToSurvey_.size() >= numPositionsForInit_)
          {
            setStaticPosition(computeSurveyedPosition(positionsToSurvey_));
          }
        }
        // we are initialized, run check
        else
        {
          posHistoryCheck(timeEntry);
        }
      }         // end fresh check
    }           // end valid check
  }             // end repo pull success
  return true;  // function runCheck completed
}

//==============================================================================
//----------------------------- setStaticPosition----------------------------
//==============================================================================
void StaticPositionCheck::setStaticPosition(
  const data::GeodeticPosition3d& staticPos)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  staticPosition_ = staticPos;

  staticPositionInitialized_ = true;

  std::stringstream initMsg;
  initMsg << std::setprecision(10)
          << "StaticPositionCheck::runCheck(): Static position initialized to "
          << "lat: " << staticPos.latitude << ", lon: " << staticPos.longitude
          << ", alt: " << staticPos.altitude;

  logMsg_(initMsg.str(), logutils::LogLevel::Info);
}

//==============================================================================
//--------------------------- computeSurveyedPosition---------------------------
//==============================================================================
data::GeodeticPosition3d StaticPositionCheck::computeSurveyedPosition(
  const std::vector<data::GeodeticPosition3d>& positions)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  data::GeodeticPosition3d surveyPos;
  surveyPos.latitude  = 0.0;
  surveyPos.longitude = 0.0;
  surveyPos.altitude  = 0.0;

  std::stringstream surveyMsg;
  surveyMsg << std::setprecision(10);
  surveyMsg << "StaticPositionCheck::computeSurveyedPosition() : averaging ";

  for (auto posIt = positions.begin(); posIt != positions.end(); ++posIt)
  {
    surveyMsg << std::endl
              << "lat: " << posIt->latitude << ", lon: " << posIt->longitude
              << ", alt: " << posIt->altitude;
    surveyPos = addTwoPositions(surveyPos, *posIt);
  }
  surveyPos.latitude  = surveyPos.latitude / positions.size();
  surveyPos.longitude = surveyPos.longitude / positions.size();
  surveyPos.altitude  = surveyPos.altitude / positions.size();

  surveyMsg << std::endl
            << "Surveyed position: lat: " << surveyPos.latitude
            << ", long:" << surveyPos.longitude
            << ", alt: " << surveyPos.altitude;
  logMsg_(surveyMsg.str(), logutils::LogLevel::Debug);

  return surveyPos;
}

//==============================================================================
//----------------------------- posHistoryCheck --------------------------------
//==============================================================================
void StaticPositionCheck::posHistoryCheck(TimeEntry& timeEntry)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  // add the new position to the deque
  data::PositionVelocity pv;
  timeEntry.localData_.getData(pv);
  if (pv.isPositionValid())
  {
    positionsToCheck_.push_back(pv.position);

    if (positionsToCheck_.size() >= checkWindowSize_)
    {
      double percentNearStart =
        percentNearPos(staticPosition_,
                       {positionsToCheck_.begin(), positionsToCheck_.end()},
                       posChangeThresh_);
      double percentNotNearStart = 1 - percentNearStart;

      if (percentNotNearStart >= assuranceUnassuredThresh_)
      {
        changeAssuranceLevel(timeEntry.timeOfWeek_,
                             data::AssuranceLevel::Unassured);
      }
      else if (percentNotNearStart >= assuranceInconsistentThresh_)
      {
        changeAssuranceLevel(timeEntry.timeOfWeek_,
                             data::AssuranceLevel::Inconsistent);
      }
      else
      {
        changeAssuranceLevel(timeEntry.timeOfWeek_,
                             data::AssuranceLevel::Assured);
      }
      if (publishDiagnostics_)
      {
        StaticPosCheckDiagnostics diagnostics;
        diagnostics.staticPosition     = staticPosition_;
        diagnostics.posChangeThresh    = posChangeThresh_;
        diagnostics.inconsistentThresh = assuranceInconsistentThresh_;
        diagnostics.unassuredThresh    = assuranceUnassuredThresh_;
        diagnostics.percentOverThresh  = percentNotNearStart;

        publishDiagnostics_(timeEntry.timeOfWeek_, diagnostics);
      }
    }
    else
    {
      // we don't have enough data so set level to unavailable
      changeAssuranceLevel(timeEntry.timeOfWeek_,
                           data::AssuranceLevel::Unavailable);
    }
  }
  if (positionsToCheck_.size() > checkWindowSize_)
  {
    positionsToCheck_.pop_front();
  }
}

//==============================================================================
//-------------------------------- addTwoPositions -----------------------------
//==============================================================================

data::GeodeticPosition3d StaticPositionCheck::addTwoPositions(
  data::GeodeticPosition3d a,
  data::GeodeticPosition3d b)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  data::GeodeticPosition3d retPos;
  retPos.latitude  = a.latitude + b.latitude;
  retPos.longitude = a.longitude + b.longitude;
  retPos.altitude  = a.altitude + b.altitude;
  return retPos;
}

//==============================================================================
//---------------------------- isWithinThreshold -------------------------------
//==============================================================================
bool StaticPositionCheck::isWithinThreshold(data::GeodeticPosition3d p,
                                            data::GeodeticPosition3d mean,
                                            double                   thresh)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  double pEcef[3];
  p.getECEF(pEcef);
  double meanEcef[3];
  mean.getECEF(meanEcef);
  double dist =
    sqrt(pow(pEcef[0] - meanEcef[0], 2.0) + pow(pEcef[1] - meanEcef[1], 2.0) +
         pow(pEcef[2] - meanEcef[2], 2.0));
  return dist <= thresh;
}

//==============================================================================
//------------------------------ percentNearPos --------------------------------
//==============================================================================
double StaticPositionCheck::percentNearPos(
  const data::GeodeticPosition3d&              center,
  const std::vector<data::GeodeticPosition3d>& positions,
  double                                       thresh)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  double percent = 0.0;
  for (auto it = positions.begin(); it != positions.end(); ++it)
  {
    if (isWithinThreshold(*it, center, thresh))
    {
      percent += 1.0;
    }
  }
  percent = percent / positions.size();
  return percent;
}
}  // namespace pnt_integrity
