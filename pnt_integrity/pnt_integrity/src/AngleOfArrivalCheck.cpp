//============================================================================//
//---------------- pnt_integrity/AngleOfArrivalCheck.cpp -------*- C++ -*-----//
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
// AssurancCheck class defined for the angle of arrival check
// Josh Clanton <josh.clanton@is4s.com>
// June 3, 2019
//============================================================================//
#include "pnt_integrity/AngleOfArrivalCheck.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace logutils;

namespace pnt_integrity
{
//==============================================================================
//------------------------------ handleGnssObservables -------------------------
//==============================================================================
bool AngleOfArrivalCheck::handleGnssObservables(
  const data::GNSSObservables& /*gnssObs*/)
{
  // data has already been added to the repo by the integrity monitor
  return runCheck();
}

//==============================================================================
//--------------------------------- runCheck -----------------------------------
//==============================================================================
bool AngleOfArrivalCheck::runCheck()
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
  // pull the newest entry from the repo
  TimeEntry newestEntry;
  if (IntegrityDataRepository::getInstance().getNewestEntry(newestEntry))
  {
    checkAngleOfArrival(
      newestEntry.timeOfWeek_, newestEntry.localData_, newestEntry.remoteData_);
    return true;
  }
  else
  {
    return false;
  }
}

//==============================================================================
//-------------------------------- check AOA -----------------------------------
//==============================================================================
void AngleOfArrivalCheck::checkAngleOfArrival(
  const double&            checkTime,
  const RepositoryEntry&   localEntry,
  const RemoteRepoEntries& remoteEntries)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  std::stringstream log_str;
  log_str << std::endl << std::endl;
  log_str << __FUNCTION__ << "() : checkTime = " << (int)checkTime;
  logMsg_(log_str.str(), LogLevel::Debug);
  log_str.str(std::string());

  // determine how long it has been since the assurance levels were updated
  // with a remote node, if it was more than a threshold, then clear the
  // multi-prn map (which, in turn, will reset the master level as well)
  // auto now =
  //     std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  // auto lastUpdate =
  // std::chrono::system_clock::to_time_t(lastAssuranceUpdate_);

  // auto timeSinceLastUpdate = now - lastUpdate;

  auto timeSinceLastUpdate = checkTime - lastAssuranceUpdate_;

  if (timeSinceLastUpdate > assuranceLevelPeriod_)
  {
    log_str << __FUNCTION__ << "():  prnAssuranceLevels_.clear()";
    logMsg_(log_str.str(), LogLevel::Debug);
    log_str.str(std::string());
    prnAssuranceLevels_.clear();
  }

  // pull the local observable map
  data::GNSSObservableMap localObsMap;
  localEntry.getData(localObsMap);
  // if there are no local observables set the level to unavailable and exit
  if (localObsMap.size() < prnCountThresh_)
  {
    log_str << __FUNCTION__
            << "():  localObsMap.size() < prnCountThresh_ -> "
               "AssuranceLevel::Unavailable";
    logMsg_(log_str.str(), LogLevel::Debug);
    log_str.str(std::string());
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Unavailable);
    return;
  }

  // perform the check with each remote node
  RemoteRepoEntries::const_iterator remoteIt = remoteEntries.begin();

  // create the map that holds the calculated assurance for each prn that
  // results from AOA calc from each node
  PrnAssuranceEachNode prnAssuranceEachNode;

  // for each remote node
  for (; remoteIt != remoteEntries.end(); ++remoteIt)
  {
    log_str << "Remote entries NodeID = " << remoteIt->first;
    logMsg_(log_str.str(), LogLevel::Debug);
    log_str.str(std::string());

    // pull the remote observable map
    data::GNSSObservableMap remoteObsMap;
    remoteIt->second.getData(remoteObsMap);

    if (remoteObsMap.size() < 1)
    {
      log_str << __FUNCTION__ << "():  remoteObsMap.size() < 1";
      logMsg_(log_str.str(), LogLevel::Debug);
      log_str.str(std::string());
      // changeAssuranceLevel(checkTime, data::AssuranceLevel::Unavailable);
      continue;
    }

    // check remote range
    data::MeasuredRange range;
    remoteIt->second.getData(range);

    // a map to store the single differences
    SingleDiffMap singleDiffMap;

    // then look for a match for each local PRN to compute
    data::GNSSObservableMap::const_iterator localMapIt = localObsMap.begin();
    for (; localMapIt != localObsMap.end(); ++localMapIt)
    {
      log_str << "Local Obs for loop.";
      logMsg_(log_str.str(), LogLevel::Debug);
      log_str.str(std::string());
      // initialize the local assurance level to the received value
      prnAssuranceEachNode[localMapIt->first].push_back(
        localMapIt->second.assurance);

      // If the range measurement is not valid, proceed with the check.
      // If it is valid, then make sure it is above the threshold before
      // proceeding.
      // This is checked here instead of outside the loop is
      // so that the assurance level for the PRN will be set to the received
      // observable's assurance level
      if ((!range.rangeValid) ||
          ((range.range >= rangeThreshold_) && (range.rangeValid)))
      {
        // find the remote prn that matches this prn
        auto remoteMatchIt = remoteObsMap.find(localMapIt->first);

        // determine what data field will be used for the difference
        // and calculate accordingly
        switch (aoaCheckData_)
        {
          case AoaCheckData::UsePseudorange:
          {
            // if a match was found and both pseudoranges are valid
            if (remoteMatchIt != remoteObsMap.end() &&
                localMapIt->second.pseudorangeValid &&
                remoteMatchIt->second.pseudorangeValid)
            {
              // there is a match for this PRN, so compute the single diff
              singleDiffMap[localMapIt->first] =
                localMapIt->second.pseudorange -
                remoteMatchIt->second.pseudorange;

              // log_str << "Single Diff = " <<
              // (double)singleDiffMap[localMapIt->first];
              // logMsg_(log_str.str(), LogLevel::Info);
              // log_str.str(std::string());
            }
            break;
          }  // end UsePseudorange case
          case AoaCheckData::UseCarrierPhase:
          {
            logMsg_(
              "AngleOfArrivalCheck:checkAngleOfArrival: 'UseCarrierPhase'"
              " data method not yet implemented",
              LogLevel::Error);
            break;
          }  // end UseCarrierPhase case
          case AoaCheckData::UseBoth:
          {
            logMsg_(
              "AngleOfArrivalCheck:checkAngleOfArrival: 'UseBoth' "
              "data method not yet implemented",
              LogLevel::Error);
            break;
          }  // end UseBoth case

        }  // end switch
      }
    }                             // end remote  node for loop match search
    if ((publishSingleDiffData_))  //&& (checkTime != lastDiffPublishTime_)
    {
      log_str << __FUNCTION__
              << "():  publishSingleDiffData , checkTime = " << (int)checkTime;
      logMsg_(log_str.str(), LogLevel::Debug);
      log_str.str(std::string());
      publishSingleDiffData_(checkTime, remoteIt->first, singleDiffMap);
      lastDiffPublishTime_ = checkTime;
    }
    nestedForLoopComparison(singleDiffMap, prnAssuranceEachNode);

  }  // end remote node for loop
  setPrnAssuranceLevels(prnAssuranceEachNode);
  calculateAssuranceLevel(checkTime);
}  // end checkAngleOfArrival

//==============================================================================
//-------------------------- nestedForLoopComparison----------------------------
//==============================================================================
void AngleOfArrivalCheck::nestedForLoopComparison(
  const SingleDiffMap&  diffMap,
  PrnAssuranceEachNode& prnAssuranceEachNode)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  // for each PRN determine how many difference values are within a
  // threshold

  // iterate through the single difference map to perform the check for each prn
  SingleDiffMap::const_iterator sdIt = diffMap.begin();

  for (; sdIt != diffMap.end(); ++sdIt)
  {
    // initialize the count that determines the assurance level
    size_t count = 0;

    // iterate through all other difference values, starting at the end
    // (could start at the beginning as well)
    SingleDiffMap::const_reverse_iterator compareIt = diffMap.rbegin();
    for (; compareIt != diffMap.rend(); ++compareIt)
    {
      // only compare different PRNs
      if (sdIt->first != compareIt->first)
      {
        // if the difference between single differences is within a threshold
        // flag this comparison PRN
        double singleDiffDiff = std::fabs(sdIt->second - compareIt->second);
        if (singleDiffDiff < singleDiffCompareThresh_)
        {
          // count this as a single difference that appears suspect when
          // compared to the PRN we are examining
          count++;
        }
      }  // end if (compare different PRNs)
    }    // end compareIt for loop

    if (count >= (prnCountThresh_ - 1))
    {
      // if there are at least a certain number of PRNs that appear suspect
      // then flag the base prn as not assured
      prnAssuranceEachNode[sdIt->first].push_back(
        data::AssuranceLevel::Unassured);
    }
    else
    {
      prnAssuranceEachNode[sdIt->first].push_back(
        data::AssuranceLevel::Assured);
    }
  }  // end outer for loop
}

//==============================================================================
//-------------------------- setPrnAssuranceLevels------------------------------
//==============================================================================
void AngleOfArrivalCheck::setPrnAssuranceLevels(
  const PrnAssuranceEachNode& prnAssuranceEachNode)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  // go through each PRN and take the maximum value across all nodes and
  // assign it as the level for that PRN
  for (auto it = prnAssuranceEachNode.begin(); it != prnAssuranceEachNode.end();
       ++it)
  {
    auto maxIterator = std::max_element(it->second.begin(), it->second.end());
    prnAssuranceLevels_[it->first] = *maxIterator;
  }
}

//==============================================================================
//---------------------------- setAssuranceLevel -------------------------------
//==============================================================================
void AngleOfArrivalCheck::calculateAssuranceLevel(const double& checkTime)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  // go throuh the prn assurance map and determine how many have been flagged
  // to set the overall level
  int assuredCount     = 0;
  int unavailableCount = 0;
  int suspectCount     = 0;
  for (auto it = prnAssuranceLevels_.begin(); it != prnAssuranceLevels_.end();
       ++it)
  {
    if (it->second == data::AssuranceLevel::Assured)
    {
      assuredCount++;
    }
    else if (it->second == data::AssuranceLevel::Unavailable)
    {
      unavailableCount++;
    }
    else
    {
      suspectCount++;
    }

  }  // end for loop

  // if the number of "bad" PRNS are above the unusable thresh, set the
  // check level to unusable
  if (suspectCount >= assuranceUnassuredThresh_)
  {
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Unassured);
  }
  // if the number of uspect PRNs are above the inconsistent thresh, set the
  // check level to inconsistent
  else if (suspectCount >= assuranceInconsistentThresh_)
  {
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Inconsistent);
  }
  // else, if the number of assured PRNs makeup at least 1/2 of the total PRNS
  // set the level to assured
  else if (assuredCount > (double)(0.5 * prnAssuranceLevels_.size()))
  {
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Assured);
  }
  else
  {
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Unavailable);
  }

  std::stringstream msg;
  msg << "AngleOfArrivalCheck::calculateAssuranceLevel() : Calculated level : "
      << (int)getAssuranceLevel() << " , checked time: " << (int)checkTime;
  logMsg_(msg.str(), logutils::LogLevel::Debug);

  if ((publishDiagnostics_))  //&& (checkTime != lastDiagPublishTime_)
  {
    AoaCheckDiagnostics diagnostics;

    diagnostics.singleDiffThresh   = singleDiffCompareThresh_;
    diagnostics.suspectPrnCount    = suspectCount;
    diagnostics.inconsistentThresh = assuranceInconsistentThresh_;
    diagnostics.unassuredThresh    = assuranceUnassuredThresh_;

    publishDiagnostics_(checkTime, diagnostics);

    lastDiagPublishTime_ = checkTime;
  }
}

}  // namespace pnt_integrity
