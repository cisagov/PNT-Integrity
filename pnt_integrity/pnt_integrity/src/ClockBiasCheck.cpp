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
// Assurance Check class defined for the clock bias check
// \author Josh Clanton <josh.clanton@is4s.com>
// \author John David Sprunger <jss0027@tigermail.auburn.edu>
// \date   December 17, 2019
//============================================================================//
#include "pnt_integrity/ClockBiasCheck.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <utility>

using namespace pnt_integrity;

//==============================================================================
//-------------------------------- handleClockOffset----------------------------
//==============================================================================
bool ClockBiasCheck::handleClockOffset(const data::ClockOffset& clockOffset)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  offsetVec_.push_back(clockOffset);

  this->pruneVec();

  return runCheck();
}

//==============================================================================
//--------------------------------- runCheck -----------------------------------
//==============================================================================
bool ClockBiasCheck::runCheck()
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  TimeEntry newestEntry;
  IntegrityDataRepository::getInstance().getNewestEntry(newestEntry);
  double checkTime = newestEntry.timeOfWeek_;

  if (this->offsetVec_.size() < this->minNumSamples_)
  {
    logMsg_(
      "Clock Bias Check: Unavailable, "
      "offsetVec_.size() < minNumSamples_",
      logutils::LogLevel::Debug);
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Unavailable);
    return false;
  }

  if (!this->enoughSampleTime())
  {
    logMsg_(
      "Clock Bias Check: Unavailable, "
      "not enough sample time duration.",
      logutils::LogLevel::Debug);
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Unavailable);
    return false;
  }

  return clockBiasCheck(checkTime);
}

//==============================================================================
//----------------------------- clockBiasCheck ---------------------------------
//==============================================================================
bool ClockBiasCheck::clockBiasCheck(const double& checkTime)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  // Get the expectation and variance of the drift rate
  // from all but the last sample
  double       driftExp = 0.0;
  double       driftVar = 0.0;
  unsigned int i        = 0;
  for (auto it = this->offsetVec_.begin(); it != (this->offsetVec_.end() - 1);
       ++it)
  {
    driftExp += it->drift;
    driftVar += pow(it->drift, 2);
    ++i;
  }

  driftExp = driftExp / i;
  driftVar = driftVar / i;
  driftVar = driftVar - pow(driftExp, 2);
  /// \note driftVar - pow(driftExp,2) can sometimes be slightly negative
  /// due to quantization, set all negative values to 0
  if (driftVar < 0)
  {
    driftVar = 0.0;
  }

  char logBuf[1024];
  sprintf(logBuf, "\ndriftExp: %0.9lf\ndriftVar: %0.9lf", driftExp, driftVar);
  std::string logStr(logBuf);

  logMsg_(logStr, logutils::LogLevel::Debug);

  data::Timestamp dt_timestamp =
    timeDiff((this->offsetVec_.rbegin() + 1)->header.timestampValid,
             (this->offsetVec_.rbegin())->header.timestampValid);
  double dt = dt_timestamp.sec + (dt_timestamp.nanoseconds / nSecPerSec);

  // propagate the next to last sample forward using that average drift rate
  double offset_propd = (offsetVec_.rbegin() + 1)->offset + (driftExp * dt);

  sprintf(logBuf,
          "offset_propd vs offsetVec_.rbegin()->offset:\n  %0.9lf\n  %0.9lf",
          offset_propd,
          offsetVec_.rbegin()->offset);
  logStr.assign(logBuf);
  logMsg_(logStr, logutils::LogLevel::Debug);

  sprintf(logBuf,
          "fabs(offset_propd - offsetVec_.rbegin()->offset) vs driftRateBound_ "
          "* dt:\n  %0.9lf\n  %0.9lf",
          fabs(offset_propd - offsetVec_.rbegin()->offset),
          this->driftRateBound_ * dt);
  logStr.assign(logBuf);
  logMsg_(logStr, logutils::LogLevel::Debug);

  // if last sample is outside the propagated sample +- driftRateBound*dt
  double offsetError = fabs(offset_propd - offsetVec_.rbegin()->offset);
  if (offsetError > (this->driftRateBound_ * dt))
  {
    // Unassured
    logMsg_("Clock Bias Check: Unassured", logutils::LogLevel::Debug2);
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Unassured);
  }
  // else if the variance is outside the driftRateVarBound
  else if (driftVar > this->driftRateVarBound_)
  {
    // Inconsistent
    logMsg_("Clock Bias Check: Inconsistent", logutils::LogLevel::Debug2);
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Inconsistent);
  }
  else
  {
    // Assured
    logMsg_("Clock Bias Check: Assured", logutils::LogLevel::Debug2);
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Assured);
  }

  ClockBiasCheckDiagnostics diagnostics;
  diagnostics.expectedDrift     = driftExp;
  diagnostics.expectedDriftVar  = driftVar;
  diagnostics.propagatedOffset  = offset_propd;
  diagnostics.actualOffset      = offsetVec_.rbegin()->offset;
  diagnostics.offsetError       = offsetError;
  diagnostics.driftRateBound    = driftRateBound_ * dt;
  diagnostics.driftRateVarBound = driftRateVarBound_;

  if (publishDiagnostics_)
  {
    publishDiagnostics_(checkTime, diagnostics);
  }
  return true;
}

//==============================================================================
//----------------------------- pruneVec ---------------------------------
//==============================================================================
void ClockBiasCheck::pruneVec()
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  if (this->offsetVec_.size() > this->maxNumSamples_)
  {
    unsigned int offset = this->offsetVec_.size() - this->maxNumSamples_;
    this->offsetVec_.erase(this->offsetVec_.begin(),
                           this->offsetVec_.begin() + offset);
  }
}

//==============================================================================
//----------------------------- enoughSampleTime ---------------------------------
//==============================================================================
bool ClockBiasCheck::enoughSampleTime()
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  data::Timestamp td =
    timeDiff(this->offsetVec_.begin()->header.timestampValid,
             this->offsetVec_.rbegin()->header.timestampValid);
  return td.sec >= this->minSampleTimeSec_;
}

//==============================================================================
//----------------------------- timeDiff ---------------------------------
//==============================================================================
data::Timestamp ClockBiasCheck::timeDiff(data::Timestamp a, data::Timestamp b)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  data::Timestamp retVal;
  retVal.sec         = b.sec - a.sec;
  retVal.nanoseconds = b.nanoseconds - a.nanoseconds;

  if (retVal.nanoseconds <= -1000000000)
  {
    retVal.sec -= 1;
    retVal.nanoseconds += 1000000000;
  }
  else if (retVal.nanoseconds >= 1000000000)
  {
    retVal.sec += 1;
    retVal.nanoseconds -= 1000000000;
  }

  return retVal;
}
