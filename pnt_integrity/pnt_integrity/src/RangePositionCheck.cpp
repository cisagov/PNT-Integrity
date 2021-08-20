//============================================================================//
//---------------- pnt_integrity/RangePositionCheck.hpp --------*- C++ -*-----//
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
// AssurancCheck class defined for the range / position check
// Josh Clanton <josh.clanton@is4s.com>
// June 11, 2019
//============================================================================//
#include "pnt_integrity/RangePositionCheck.hpp"

#include <chrono>
#include <sstream>

using namespace std::chrono;

namespace pnt_integrity
{
//==============================================================================
//---------------------------- handleMeasuredRange -----------------------------
//==============================================================================
bool RangePositionCheck::handleMeasuredRange(
  const data::MeasuredRange& /*range*/)
{
  // data already added to repo
  // We will run the check when the local position is handled
  return runCheck();
}

//==============================================================================
//---------------------------- handlePositionVelocity --------------------------
//==============================================================================
bool RangePositionCheck::handlePositionVelocity(
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
bool RangePositionCheck::runCheck()
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  // pull the newest entry from the repo
  TimeEntry newestEntry;
  if (IntegrityDataRepository::getInstance().getNewestEntry(newestEntry))
  {
    rangePositionCheck(
      newestEntry.timeOfWeek_, newestEntry.localData_, newestEntry.remoteData_);
    return true;
  }
  else
  {
    return false;
  }
}

//==============================================================================
//---------------------------- RangePositionCheck ------------------------------
//==============================================================================
void RangePositionCheck::rangePositionCheck(
  const double&            checkTime,
  const RepositoryEntry&   localEntry,
  const RemoteRepoEntries& remoteEntries)
{
  static RngPosCheckDiagnostics diagnostics;

  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  diagnostics.clear();
  rangeCheckLevels_.clear();

  data::PositionVelocity localPosVel;
  localEntry.getData(localPosVel);
  if (localPosVel.isPositionValid() && localPosVel.isPositionCovarianceValid())
  {
    // go through each remote and compare the differenced position to the
    // measured range (if available)
    for (auto remoteIt = remoteEntries.begin(); remoteIt != remoteEntries.end();
         ++remoteIt)
    {
      rangeCheckLevels_[remoteIt->first] = data::AssuranceLevel::Unavailable;

      // pull both the range measurement and the remote position and
      // make sure that both are valid before performing check
      data::MeasuredRange range;
      remoteIt->second.getData(range);

      data::PositionVelocity remotePosVel;
      remoteIt->second.getData(remotePosVel);

      // if the range is valid and the remote position is valid
      if (range.rangeValid && remotePosVel.isPositionValid() &&
          remotePosVel.isPositionCovarianceValid())
      {
        RngPosCheckNodeDiagnostic nodeDiagnosticData;
        // compare the measured range to the computed range
        // if function returns true, then range positions and range measurement
        // appear to be valid
        if (compareRanges(localPosVel, remotePosVel, range, nodeDiagnosticData))
        {
          // range measurement checks out
          rangeCheckLevels_[remoteIt->first] = data::AssuranceLevel::Assured;
        }
        else
        {
          // range measurement does not check out
          rangeCheckLevels_[remoteIt->first] = data::AssuranceLevel::Unassured;
        }
        diagnostics[remoteIt->first] = nodeDiagnosticData;
      }
      else
      {
        logMsg_(
          "RangePositionCheck::rangePositionCheck() : Range to remote "
          " or remote position is not valid. Check not valid.",
          logutils::LogLevel::Debug);
      }
    }
  }

  calculateAssuranceLevel(checkTime);
  if (publishDiagnostics_)
  {
    publishDiagnostics_(checkTime, diagnostics);
  }
  else
  {
    logMsg_(
      "RangePositionCheck::rangePositionCheck() : Local position data"
      " is not valid. Check not valid.",
      logutils::LogLevel::Debug);
  }
}

//==============================================================================
//------------------------------- compareRanges --------------------------------
//==============================================================================
bool RangePositionCheck::compareRanges(const data::PositionVelocity& posVel1,
                                       const data::PositionVelocity& posVel2,
                                       const data::MeasuredRange&    measRange,
                                       RngPosCheckNodeDiagnostic&    diagnostic)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  // calculate the range based on the 2 positions
  double calculatedRange =
    calculateDistance(posVel1.position, posVel2.position);

  if (std::isnan(calculatedRange))
  {
    std::stringstream msg;
    msg << "RangePositionCheck:compareRanges() : Cannot compute range with "
           " differenced positions";
    logMsg_(msg.str(), logutils::LogLevel::Error);
    return false;
  }
  // compare the calculated and measured range, taking variances into account

  // first, add the variance to the calculated range
  // for now, take the norm of the 3 components as range variance
  double var1 =
    sqrt(pow(posVel1.covariance[0][0], 2) + pow(posVel1.covariance[1][1], 2) +
         pow(posVel1.covariance[2][2], 2));

  double var2 =
    sqrt(pow(posVel2.covariance[0][0], 2) + pow(posVel2.covariance[1][1], 2) +
         pow(posVel2.covariance[2][2], 2));

  // determine a min/max calculated range threshold by taking the position
  // variances into account
  double maxCalcRange           = calculatedRange + sqrt(var1) + sqrt(var2);
  double minCalcRange           = calculatedRange - sqrt(var1) - sqrt(var2);
  diagnostic.maxCalculatedRange = maxCalcRange;
  diagnostic.minCalculatedRange = minCalcRange;

  // determine a min/max measurement value taking variances into account
  double maxMeasRange     = measRange.range + sqrt(measRange.variance);
  double minMeasRange     = measRange.range - sqrt(measRange.variance);
  diagnostic.maxMeasRange = maxMeasRange;
  diagnostic.minMeasRange = minMeasRange;

  // check to see if range measurement and calculated measurement are
  // reasonable. Ihe max measured value is larger than the min calc val and
  // the min value is smaller than the max calc value, then everything checks
  // out
  if ((maxMeasRange >= minCalcRange) && (minMeasRange <= maxCalcRange))
  {
    return true;
  }
  else
  {
    return false;
  }
}

//==============================================================================
//---------------------------- setAssuranceLevel -------------------------------
//==============================================================================
void RangePositionCheck::calculateAssuranceLevel(const double& checkTime)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  // For this check Unknown and Unusable are considered the same
  // go through each remote range check value and determine how many have
  // have been flagged to set the overall level
  int assuredCount     = 0;
  int unavailableCount = 0;
  int unusableCount    = 0;

  auto rangeIt = rangeCheckLevels_.begin();
  for (; rangeIt != rangeCheckLevels_.end(); ++rangeIt)
  {
    switch (rangeIt->second)
    {
      case data::AssuranceLevel::Assured:
        assuredCount++;
        break;
      case data::AssuranceLevel::Unavailable:
        unavailableCount++;
        break;
      case data::AssuranceLevel::Unassured:
      case data::AssuranceLevel::Inconsistent:
        unusableCount++;
        break;
    }
  }

  // set overall level based on counts
  if (unusableCount > 0)
  {
    // if the check has failed with any remote node
    // mark the total level as unassured
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Unassured);
  }
  else if (assuredCount > 0)
  {
    // if check passes with any remote node
    // mark the level as Assured
    // Other nodes may be unavailable, but data may not
    // be available so allow a single "assured" with a remote
    //  to supercede any number of "unavailables" with remotes
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Assured);
  }
  else if ((checkTime - lastAssuranceUpdate_ > 2.0))
  {
    // set the level as unavailable, as there are no remotes
    // with unusable or assured
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Unavailable);
  }
}
}  // namespace pnt_integrity