//============================================================================//
//---------------- pnt_integrity/PositionJumpCheck.hpp ---------*- C++ -*-----//
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
//  AssurancCheck class defined for the position jump check
//  Will Travis <will.travis@is4s.com>
//  Josh Clanton <josh.clanton@is4s.com>
//  November 27, 2019
//============================================================================//

#include "pnt_integrity/PositionJumpCheck.hpp"

#include <algorithm>

namespace pnt_integrity
{
//==============================================================================
//---------------------- handleEstimatedPositionVelocity -----------------------
//==============================================================================
bool PositionJumpCheck::handleEstimatedPositionVelocity(
  const data::PositionVelocity& pv)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  // if I'm using estimated position velocity, then I need to update the
  // jump bound and save my current estimated positoin
  if (useEstimatedPv_ && (pv.covariance[0][0] != 0.0))
  {
    currentEstimatedPv_ = pv.position;
    currentEstPvSet_    = true;

    geodeticConverter_.initialiseReference(
      pv.position.latitude, pv.position.longitude, pv.position.altitude);

    double covarianceBound =
      posStdDevMultiplier_ * sqrt(pv.covariance[0][0] + pv.covariance[1][1]);

    positionJumpBound_ = std::max(minimumBound_, covarianceBound);
  }
  return true;
}

//==============================================================================
//------------------------------ handlePositionVelocity ------------------------
//==============================================================================
bool PositionJumpCheck::handlePositionVelocity(
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
bool PositionJumpCheck::runCheck()
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  bool                    retVal = false;
  PosJumpCheckDiagnostics diagnostics;

  TimeEntry newestEntry;
  IntegrityDataRepository::getInstance().getNewestEntry(newestEntry);
  data::PositionVelocity posVel;
  newestEntry.localData_.getData(posVel);

  double updateTime = (double)posVel.header.timestampValid.sec +
                      ((double)posVel.header.timestampValid.nanoseconds) / 1e9;

  // if I'm not using distance traveled or estimated pv, them I'm in
  // "platform" mode and need to update the bound with the maximum velocity
  if ((!useDistTraveled_) && (!useEstimatedPv_))
  {
    distanceTraveledReceived_ = true;

    updateBound(updateTime);
  }

  // if we have set a last known good position, then proceed with the check
  // if not, the check is unavailable
  if ((!useEstimatedPv_) && lastKnownGoodSet_ && distanceTraveledReceived_)
  {
    // compute a distance to the last known good position and
    // compare to the bound
    distanceToLastGoodPos_ =
      calculateDistance(posVel.position, lastKnownGoodPosition_);

    if (checkDistance(distanceToLastGoodPos_, positionJumpBound_))
    {
      // position jump is greater than bound
      changeAssuranceLevel(posVel.header.timestampValid.sec,
                           data::AssuranceLevel::Unassured);

      std::stringstream msg;
      msg << "PositionJumpCheck: UNASSURED: distance to last known good "
             "position: "
          << distanceToLastGoodPos_
          << " (m), jump bound: " << positionJumpBound_ << " (m)";
      logMsg_(msg.str(), logutils::LogLevel::Debug);
    }
    else
    {
      // position jump is less than bound
      changeAssuranceLevel(posVel.header.timestampValid.sec,
                           data::AssuranceLevel::Assured);
    }
    diagnostics.distance = distanceToLastGoodPos_;
    diagnostics.bound    = positionJumpBound_;
    retVal               = true;
  }
  else if (useEstimatedPv_ && currentEstPvSet_)
  {
    double north, east, down;
    geodeticConverter_.geodetic2Ned(posVel.position.latitude,
                                    posVel.position.longitude,
                                    posVel.position.altitude,
                                    &north,
                                    &east,
                                    &down);

    double distance = sqrt(north * north + east * east);

    std::stringstream msg;
    msg << "North: " << north << " East: " << east << " Distance: " << distance
        << " Bound: " << positionJumpBound_
        << " Valid Position: " << checkDistance(distance, positionJumpBound_);
    logMsg_(msg.str(), logutils::LogLevel::Debug);

    // double distance = calculateDistance(posVel.position,
    // currentEstimatedPv_);
    if (checkDistance(distance, positionJumpBound_))
    {
      // position jump is greater than bound
      changeAssuranceLevel(posVel.header.timestampValid.sec,
                           data::AssuranceLevel::Unassured);

      std::stringstream msg;
      msg << "PositionJumpCheck: UNASSURED: distance to estimated position "
             "position: "
          << distance << " (m), jump bound: " << positionJumpBound_ << " (m)";
      logMsg_(msg.str(), logutils::LogLevel::Debug);
    }
    else
    {
      // position jump is greater than bound
      changeAssuranceLevel(posVel.header.timestampValid.sec,
                           data::AssuranceLevel::Assured);
    }
    diagnostics.bound    = positionJumpBound_;
    diagnostics.distance = distance;
    retVal               = true;
  }
  else
  {
    changeAssuranceLevel(posVel.header.timestampValid.sec,
                         data::AssuranceLevel::Unavailable);
    diagnostics.bound    = std::numeric_limits<double>::quiet_NaN();
    diagnostics.distance = std::numeric_limits<double>::quiet_NaN();
    retVal               = false;
  }

  if (publishDiagnostics_)
  {
    publishDiagnostics_(updateTime, diagnostics);
  }
  return retVal;
}

//==============================================================================
//---------------------------- setLastGoodPosition -----------------------------
//==============================================================================
void PositionJumpCheck::setLastGoodPosition(
  const double&                   updateTime,
  const data::GeodeticPosition3d& position)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
  AssuranceCheck::setLastGoodPosition(updateTime, position);

  // reset the distance traveled and jump bound
  distanceTraveled_ = 0.0;
  if (useDistTraveled_)
    positionJumpBound_ = minimumBound_;
}

//==============================================================================
//-------------------------------- updateBound ---------------------------------
//==============================================================================
void PositionJumpCheck::updateBound()
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
  if (useDistTraveled_)
  {
    positionJumpBound_ = std::max(minimumBound_, distanceTraveled_);
  }
  else
  {
    logMsg_(
      "PositionJumpCheck::updateBound(): Function can only be called when "
      "using distrance traveled for bound propagation. 'useDistTraveled_' "
      "must be set to True",
      logutils::LogLevel::Error);
  }
}
//==============================================================================
//-------------------------------- updateBound ---------------------------------
//==============================================================================
void PositionJumpCheck::updateBound(const double& updateTime)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
  if (!useDistTraveled_)
  {
    // propagate the bound forward with the maximum velocity
    double dt = updateTime - lastKnownGoodPositionTime_;

    positionJumpBound_ = std::max(minimumBound_, (maximumVelocity_ * dt));
  }
  else
  {
    logMsg_(
      "PositionJumpCheck::updateBound(updateTime): Function can only be "
      "called when not using distrance traveled for bound propagation. "
      "'useDistTraveled_' must be set to False",
      logutils::LogLevel::Error);
  }
}

}  // namespace pnt_integrity
