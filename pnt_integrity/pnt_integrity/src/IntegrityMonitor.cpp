//============================================================================//
//---------------------pnt_integrity/IntegrityMonitor.cpp ------*- C++ -*-----//
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
//  Defines the IntegrityMonitor class in pnt_integrity
//  Josh Clanton <josh.clanton@is4s.com>
//  May 28, 2019
//============================================================================//
#include "pnt_integrity/IntegrityMonitor.hpp"
#include <math.h>
#include <stdio.h> /* printf */

namespace pnt_integrity
{
//==============================================================================
//-------------------------- Constructor / Destructor --------------------------
//==============================================================================
IntegrityMonitor::IntegrityMonitor(const logutils::LogCallback& log)
  : logMsg_(log)
{
  // set the repo's logger to use the integrity monitor's logging
  IntegrityDataRepository::getInstance().setLogMessageHandler(log);
}

//==============================================================================
//----------------------------- registerCheck ----------------------------------
//==============================================================================
bool IntegrityMonitor::registerCheck(const std::string& checkName,
                                     AssuranceCheck*    check)
{
  // bind the check's log handler to the local log handler
  {
    std::lock_guard<std::mutex> lock(monitorMutex_);
    check->setLogMessageHandler(logMsg_);
  }
  // grant exclusive access to checks_ to add the check to the vector
  std::unique_lock<std::shared_timed_mutex> lock(checkMutex_);

  // "register" the check with the integrity monitor
  checks_[checkName] = check;

  return true;
}

//==============================================================================
//-------------------------- handleGNSSObservables -----------------------------
//==============================================================================
void IntegrityMonitor::handleGnssObservables(
  const data::GNSSObservables& gnssObs,
  const bool&                  localFlag)
{
  // add the provided observable to the repos as either a local or remote
  // determined by the provided flag

  double timestampOfValidity;

  if (getRoundedValidTime(gnssObs.header, timestampOfValidity))
  {
    double time = getCorrectedEntryTime(
      timestampOfValidity, gnssObs, localFlag, gnssObs.header.deviceId);

    addDataToRepo(time, gnssObs, localFlag, gnssObs.header.deviceId);

    // grant shared access to the checks_ vector
    std::shared_lock<std::shared_timed_mutex> lock(checkMutex_);

    // loop through all checks and call the handler for this data type
    AssuranceChecks::const_iterator checkIt;
    for (checkIt = checks_.begin(); checkIt != checks_.end(); ++checkIt)
    {
      checkIt->second->handleGnssObservables(gnssObs, time);

      if (checkIt->second->hasMultiPrnSupport())
      {
        setMultiPrnAssuranceData(checkIt->second->getMultiPrnAssuranceData());
      }
    }
  }
  // calculated the total assurance level based on the latest info
  determineAssuranceLevels();
}

//==============================================================================
//-------------------------- handleGnssSubframe -----------------------------
//==============================================================================
void IntegrityMonitor::handleGnssSubframe(const data::GNSSSubframe& gnssObs,
                                          const bool& /*localFlag*/)
{
  // grant shared access to the checks_ vector
  std::shared_lock<std::shared_timed_mutex> lock(checkMutex_);

  // loop through all checks and call the handler for this data type
  AssuranceChecks::const_iterator checkIt;
  for (checkIt = checks_.begin(); checkIt != checks_.end(); ++checkIt)
  {
    checkIt->second->handleGnssSubframe(gnssObs);
  }
  determineAssuranceLevels();
}

//==============================================================================
//-------------------------- handleDistanceTraveled ----------------------------
//==============================================================================
void IntegrityMonitor::handleDistanceTraveled(
  const data::AccumulatedDistranceTraveled& dist)
{
  // grant shared access to the checks_ vector
  std::shared_lock<std::shared_timed_mutex> lock(checkMutex_);

  // loop through all checks and call the handler for this data type
  AssuranceChecks::const_iterator checkIt;
  for (checkIt = checks_.begin(); checkIt != checks_.end(); ++checkIt)
  {
    checkIt->second->handleDistanceTraveled(dist);
  }

  determineAssuranceLevels();
}

//==============================================================================
//-------------------------- handlePositionVelocity ----------------------------
//==============================================================================
void IntegrityMonitor::handlePositionVelocity(
  const data::PositionVelocity& posVel,
  const bool&                   localFlag)
{
  // add the provided data to the repos as either a local or remote
  // determined by the provided flag
  double timestampOfValidity;

  if (getRoundedValidTime(posVel.header, timestampOfValidity))
  {
    addDataToRepo(
      timestampOfValidity, posVel, localFlag, posVel.header.deviceId);

    AssuranceChecks::const_iterator checkIt;
    // grant shared access to the checks_ vector
    {
      std::shared_lock<std::shared_timed_mutex> lock(checkMutex_);

      // loop through all checks and call the handler for this data type
      for (checkIt = checks_.begin(); checkIt != checks_.end(); ++checkIt)
      {
        checkIt->second->handlePositionVelocity(posVel, localFlag);
      }
    }

    // calculated the total assurance level based on the latest info
    determineAssuranceLevels();

    // grant shared access to the checks_ vector
    std::shared_lock<std::shared_timed_mutex> lock(checkMutex_);
    // locking the assuranceState_ and lastKnownGoodPosition_
    std::lock_guard<std::mutex> classLock(monitorMutex_);
    // set the last known good position if the level is assured
    if ((assuranceState_.getAssuranceLevel() ==
         data::AssuranceLevel::Assured) &&
        localFlag)
    {
      setLastKnownGoodPosition(posVel);
    }

    // update each check with the latest position and assurance level
    for (checkIt = checks_.begin(); checkIt != checks_.end(); ++checkIt)
    {
      checkIt->second->setPositionAssurance(
        posVel.header.timestampValid.sec,
        posVel.position,
        assuranceState_.getAssuranceLevel());
    }
  }
}

//==============================================================================
//--------------------- handleEstimatedPositionVelocity ------------------------
//==============================================================================
void IntegrityMonitor::handleEstimatedPositionVelocity(
  const data::PositionVelocity& posVel,
  const bool& /*localFlag*/)
{
  // loop through all checks and call the handler for this data type
  {
    // grant shared access to the checks_ vector
    std::shared_lock<std::shared_timed_mutex> lock(checkMutex_);

    AssuranceChecks::const_iterator checkIt;
    for (checkIt = checks_.begin(); checkIt != checks_.end(); ++checkIt)
    {
      checkIt->second->handleEstimatedPositionVelocity(posVel);
    }
  }
  // calculated the total assurance level based on the latest info
  determineAssuranceLevels();
}

//==============================================================================
//--------------------------- handleMeasuredRange ------------------------------
//==============================================================================
void IntegrityMonitor::handleMeasuredRange(const data::MeasuredRange& range,
                                           const bool&                localFlag)
{
  // add the provided data to the repos as either a local or remote
  // determined by the provided flag
  double timestampOfValidity;

  if (getRoundedValidTime(range.header, timestampOfValidity))
  {
    addDataToRepo(timestampOfValidity, range, localFlag, range.header.deviceId);

    // grant shared access to the checks_ vector
    std::shared_lock<std::shared_timed_mutex> lock(checkMutex_);

    // loop through all checks and call the handler for this data type
    for (auto checkIt = checks_.begin(); checkIt != checks_.end(); ++checkIt)
    {
      checkIt->second->handleMeasuredRange(range);
    }

    determineAssuranceLevels();
  }
}

//==============================================================================
//----------------------------- handleClockOffset ------------------------------
//==============================================================================
void IntegrityMonitor::handleClockOffset(const data::ClockOffset& clockOffset,
                                         const bool&              localFlag)
{
  double timestampOfValidity;

  if (getRoundedValidTime(clockOffset.header, timestampOfValidity))
  {
    addDataToRepo(
      timestampOfValidity, clockOffset, localFlag, clockOffset.header.deviceId);

    // grant shared access to the checks_ vector
    std::shared_lock<std::shared_timed_mutex> lock(checkMutex_);

    // loop through all checks and call the handler for this data type
    for (auto checkIt = checks_.begin(); checkIt != checks_.end(); ++checkIt)
    {
      checkIt->second->handleClockOffset(clockOffset);
    }

    determineAssuranceLevels();
  }
}

//==============================================================================
//--------------------------------- handleAGC ----------------------------------
//==============================================================================
void IntegrityMonitor::handleAGC(const data::AgcValue& agcValue)
{
  // grant shared access to the checks_ vector
  std::shared_lock<std::shared_timed_mutex> lock(checkMutex_);

  for (auto checkIt = checks_.begin(); checkIt != checks_.end(); ++checkIt)
  {
    checkIt->second->handleAGC(agcValue);
  }

  determineAssuranceLevels();
}
//==============================================================================
//-------------------------- determineAssuranceLevels -------------------------
//==============================================================================
void IntegrityMonitor::determineAssuranceLevels()
{
  // grant this thread shared read access to checks_
  std::shared_lock<std::shared_timed_mutex> sharedLock(checkMutex_);

  // compute a scale factor to normalize the weights
  double                          weightScaleFactor = 0.0;
  AssuranceChecks::const_iterator checkIt;
  for (checkIt = checks_.begin(); checkIt != checks_.end(); ++checkIt)
  {
    if (checkIt->second->isCheckUsed())
    {
      weightScaleFactor += checkIt->second->getWeight();
    }
  }

  // grant this thread exclusive access to clear checksUsed_
  // and access other private data
  std::lock_guard<std::mutex> lock(monitorMutex_);
  checksUsed_.clear();

  if (weightScaleFactor > 0)
  {
    // loop through all checks and call the handler for this data type
    double cumulativeAssuranceValue = 0;
    for (checkIt = checks_.begin(); checkIt != checks_.end(); ++checkIt)
    {
      if (checkIt->second->isCheckUsed())
      {
        double       checkVal = checkIt->second->getAssuranceValue();
        const double weightedVal =
          checkVal * (checkIt->second->getWeight() / weightScaleFactor);

        cumulativeAssuranceValue += weightedVal;
        checksUsed_.push_back(checkIt->second->getName());
      }
    }
    // now set the overall state with
    assuranceState_.setWithValue(cumulativeAssuranceValue);
  }
  else
  {
    // all the checks are unavailable
    assuranceState_.setWithLevel(data::AssuranceLevel::Unavailable);
  }
}

//==============================================================================
//---------------------------- getAssuranceReports -----------------------------
//==============================================================================
data::AssuranceReports IntegrityMonitor::getAssuranceReports()
{
  // grant this thread shared read access to checks_
  std::shared_lock<std::shared_timed_mutex> sharedLock(checkMutex_);

  data::AssuranceReports newReports;
  for (auto checkIt = checks_.begin(); checkIt != checks_.end(); ++checkIt)
  {
    // create a new state for the check
    data::AssuranceState checkState;
    // populate the check state with the level from the check
    checkState.setWithLevel(checkIt->second->getAssuranceLevel());
    // populate the weight used for the level calculation
    checkState.setWeight(checkIt->second->getWeight());
    // populate the check name
    checkState.setName(checkIt->second->getName());
    // add the state to the report structure
    newReports.addReport(checkState);
  }
  return newReports;
}

void IntegrityMonitor::reset()
{
  IntegrityDataRepository::getInstance().clearEntries();

  for (auto check : checks_)
  {
    check.second->reset();
  }

  prnAssuranceLevels_.clear();
  assuranceState_.setWithLevel(data::AssuranceLevel::Unavailable);
  lastKnownGoodPosition_ = data::GeodeticPosition3d();
}

  void IntegrityMonitor::setLastKnownGoodPosition(const data::PositionVelocity& posVel)
  {
      lastKnownGoodPosition_ = posVel.position;

    AssuranceChecks::const_iterator checkIt;
      // set the last good position in all of the checks
      for (checkIt = checks_.begin(); checkIt != checks_.end(); ++checkIt)
      {
        checkIt->second->setLastGoodPosition(posVel.header.timestampValid.sec,
                                             lastKnownGoodPosition_);
      }
  }

  void IntegrityMonitor::clearLastKnownGoodPosition()
  {
      lastKnownGoodPosition_ = data::GeodeticPosition3d();
      
      AssuranceChecks::const_iterator checkIt;
      // set the last good position in all of the checks
      for (checkIt = checks_.begin(); checkIt != checks_.end(); ++checkIt)
      {
        checkIt->second->clearLastGoodPosition();
      }

  }


}  // namespace pnt_integrity
