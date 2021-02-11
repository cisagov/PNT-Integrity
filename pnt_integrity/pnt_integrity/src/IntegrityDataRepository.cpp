//============================================================================//
//--------------- pnt_integrity/IntegrityDataRepository.cpp -----*- C++ -*----//
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
// Defines the IntegrityDataRepository class in pnt_integrity
// Josh Clanton <josh.clanton@is4s.com>
// May 28, 2019
//============================================================================//
#include "pnt_integrity/IntegrityDataRepository.hpp"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace pnt_integrity
{
//==============================================================================
//------------------- Local GNSS Observable accessor functions -----------------
//==============================================================================

void IntegrityDataRepository::addEntry(const double&               timeOfWeek,
                                       const uint32_t&             satelliteID,
                                       const data::GNSSObservable& gnssObs)
{  
  std::lock_guard<std::recursive_mutex> lock(repoMutex_);

  // Make (or copy the existing) a entry for this time
  TimeEntry entry = makeEntry(timeOfWeek);
  // add the data to the local observables
  {
    std::lock_guard<std::recursive_mutex> repoLock(repoMutex_);
    entry.localData_.addEntry(satelliteID, gnssObs);
    // add the entry back to the history
    repository_[timeOfWeek] = entry;
  }
  manageHistory();
}

//------------------------------------------------------------------------------

bool IntegrityDataRepository::getData(const double&         timeOfWeek,
                                      const uint32_t&       satelliteID,
                                      data::GNSSObservable& gnssObs)
{
  std::lock_guard<std::recursive_mutex> lock(repoMutex_);

  TimeEntry timeEntry;
  if (findEntry(timeOfWeek, timeEntry))
  {
    // Time entry exists
    return timeEntry.localData_.getData(satelliteID, gnssObs);
    ;
  }
  else
  {
    // time entry does not exist
    std::stringstream errMsg;
    errMsg << "IntegrityDataRepository::getData() : No entry at time "
           << timeOfWeek;
    logMsg_(errMsg.str(), logutils::LogLevel::Error);
    return false;
  }
}

//==============================================================================
//----------------- Remote GNSS Observable accessor functions ------------------
//==============================================================================

void IntegrityDataRepository::addEntry(const double&               timeOfWeek,
                                       const std::string&          nodeID,
                                       const uint32_t&             satelliteID,
                                       const data::GNSSObservable& gnssObs)
{
  std::lock_guard<std::recursive_mutex> lock(repoMutex_);

  // look for a corresponding time entry
  TimeEntry entry = makeEntry(timeOfWeek);
  
  // make sure remote observable exists for this time, if not create it
  auto remoteIt = entry.remoteData_.find(nodeID);
  if (remoteIt != entry.remoteData_.end())
  {
    // a remote observable exists for this node, add (will overwrite value
    // if it already exists)
    entry.remoteData_[nodeID].addEntry(satelliteID, gnssObs);
  }
  else
  {
    // a remote observable does not exist for this node ID, create it and add
    // it to the remote observables list
    RepositoryEntry remoteEntry(DataLocaleType::Remote, nodeID);

    remoteEntry.setLogMessageHandler(logMsg_);
    // std::bind(&IntegrityDataRepository::logMsg_,
    //           this,
    //           std::placeholders::_1,
    //           std::placeholders::_2));

    remoteEntry.addEntry(satelliteID, gnssObs);
    entry.remoteData_[nodeID] = remoteEntry;
  }
  // add the entry back to the history
  repository_[timeOfWeek] = entry;
  
  manageHistory();
}

//------------------------------------------------------------------------------

bool IntegrityDataRepository::getData(const double&         timeOfWeek,
                                      const std::string&    nodeID,
                                      const uint32_t&       satelliteID,
                                      data::GNSSObservable& gnssObs)
{
  std::lock_guard<std::recursive_mutex> lock(repoMutex_);

  TimeEntry timeEntry;
  if (findEntry(timeOfWeek, timeEntry))
  {
    // Time entry exists, now check for nodeID
    auto remoteIt = timeEntry.remoteData_.find(nodeID);
    if (remoteIt != timeEntry.remoteData_.end())
    {
      // Entry object will determine if satellite id exists or not
      return remoteIt->second.getData(satelliteID, gnssObs);
    }
    else
    {
      // remote entry does not exist at provided time
      std::stringstream errMsg;
      errMsg << "IntegrityDataRepository::getData() : No data for Remote ID '"
             << nodeID << "' at time (" << timeOfWeek << ")";
      logMsg_(errMsg.str(), logutils::LogLevel::Error);
      return false;
    }
  }
  else
  {
    // time entry does not exist
    // time entry does not exist
    std::stringstream errMsg;
    errMsg << "IntegrityDataRepository::getData() : No entry at time "
           << timeOfWeek;
    logMsg_(errMsg.str(), logutils::LogLevel::Error);
    return false;
  }
}

//==============================================================================
//------------------------ Entry accessor functions ----------------------------
//==============================================================================
bool IntegrityDataRepository::findEntry(const double& timeOfWeek,
                                        TimeEntry&    timeEntry)
{
  // attempt to find an exact time match in the history
  std::lock_guard<std::recursive_mutex> repoLock(repoMutex_);

  auto entry = repository_.find(timeOfWeek);

  if (entry != repository_.end())
  {
    // an exact match was found, so return it
    timeEntry = entry->second;
    return true;
  }
  else  // not found
  {
    return false;
  }
}

//------------------------------------------------------------------------------
bool IntegrityDataRepository::getNewestEntry(TimeEntry& timeEntry)
{
  std::lock_guard<std::recursive_mutex> repoLock(repoMutex_);
  
  if (repository_.size() > 0)
  {
    TimeEntryHistory::reverse_iterator rit = repository_.rbegin();
    timeEntry                              = rit->second;
    return true;
  }
  else
  {
    logMsg_("IntegrityDataRepository::getNewestEntry(): Repo is empty",
            logutils::LogLevel::Error);
    return false;
  }
}

//------------------------------------------------------------------------------
bool IntegrityDataRepository::getNewestEntries(
  std::vector<TimeEntry>& timeEntryVec,
  double                  startTime)
{
  std::lock_guard<std::recursive_mutex> repoLock(repoMutex_);

  if (repository_.size() > 0)
  {
    for (TimeEntryHistory::iterator it = repository_.begin();
         it != repository_.end();
         ++it)
    {
      if (it->second.timeOfWeek_ >= startTime)
      {
        timeEntryVec.push_back(it->second);
      }
    }

    // sort entries from oldest to newest
    std::sort(timeEntryVec.begin(),
              timeEntryVec.end(),
              IntegrityDataRepository::sortTimeEntry);

    return true;
  }
  else
  {
    logMsg_("IntegrityDataRepository::getNewestEntries(): Repo is empty",
            logutils::LogLevel::Error);
    return false;
  }
}

//------------------------------------------------------------------------------
TimeEntry& IntegrityDataRepository::makeEntry(const double& timeOfWeek)
{
  std::lock_guard<std::recursive_mutex> lock(repoMutex_);

  // attempt to find an exact time match in the history
  auto entry = repository_.find(timeOfWeek);
  if (entry != repository_.end())
  {
    // an exact match was found, so return it
    return entry->second;
  }
  else  // create the entry and return it
  {
    // create the time entry
    TimeEntry newEntry(timeOfWeek);

    // set the log callback for the local observables (remotes are set as
    // they are created)
    newEntry.localData_.setLogMessageHandler(logMsg_);
    // std::bind(&IntegrityDataRepository::logMsg_,
    //           this,
    //           std::placeholders::_1,
    //           std::placeholders::_2));

    newEntry.timeOfWeek_ = timeOfWeek;

    // enter the entry into the history
    repository_[timeOfWeek] = newEntry;

    // return a copy for processing
    return repository_[timeOfWeek];
  }
}

//------------------------------------------------------------------------------
void IntegrityDataRepository::manageHistory()
{
  std::lock_guard<std::recursive_mutex> lock(repoMutex_);
  TimeEntryHistory::iterator repoIt;

  // the history map is ordered on the key time, so the newest entries will
  // go at the end of the map. The logic here is to get the newest (last) entry
  // in the map, subtract the desired history length (historyPeriod_) from the
  // time key, and delete everying the map that is before that, which is all of
  // the old material.

  // The oldest time allowed will be the newest time minus the desired
  // history period
  repoIt = repository_.end();  // newest time is at the end.
  repoIt--;  // decrement the iterator so it points to the last element
  double oldestHistoryTime = repoIt->first - historyPeriod_;

  bool historyTrimmed = false;
  while (!historyTrimmed)
  {
    // find the first entry with an old time, delete it, and check again
    for (repoIt = repository_.begin(); repoIt != repository_.end(); repoIt++)
    {
      // std::cout << std::setprecision(15) << "entry: " << repoIt->first << std::endl;
      // if a map entry is old, erase it
      if (repoIt->first < oldestHistoryTime)
      {
        // std::stringstream eraseMsg;
        // eraseMsg << "IntegrityDataRepository: Removing time entry at: "
        //          << std::setprecision(20) << repoIt->first;
        // logMsg_(eraseMsg.str(), logutils::LogLevel::Info);

        try 
        {
          // std::cout  << "erasing: " << repoIt->first << ", " << repoIt->second.timeOfWeek_<< std::endl;
          repository_.erase(repoIt);
        }
        catch (std::exception& e)
        {
          std::stringstream log_str;
          log_str << __FUNCTION__ << "() error: " << e.what() << std::endl;
          logMsg_(log_str.str(), logutils::LogLevel::Error);
        }
        // kill the for loop so it will restart with updated iterators after
        // the deletion
        break;
      }
    }
    // std::cout << "-------------------" << std::endl;
    // if program execution makes it here, it means that no old time keys
    // were found, therefore history is trimmed
    historyTrimmed = true;
  }
}
}  // namespace pnt_integrity
