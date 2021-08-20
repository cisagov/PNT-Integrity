//============================================================================//
//--------------- pnt_integrity/IntegrityDataRepository.hpp -----*- C++ -*----//
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
/// \brief    Defines the IntegrityDataRepository class in pnt_integrity
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     May 28, 2019
//============================================================================//
#ifndef PNT_INTEGRITY__INTEGRITY_DATA_REPOSITORY_HPP
#define PNT_INTEGRITY__INTEGRITY_DATA_REPOSITORY_HPP

#include <atomic>
#include <deque>
#include <iostream>
#include <mutex>
#include <sstream>
#include <vector>
#include "logutils/logutils.hpp"
#include "pnt_integrity/RepositoryEntry.hpp"

namespace pnt_integrity
{
/// A type to map remote entries to their node name / device id
using RemoteRepoEntries = std::map<std::string, RepositoryEntry>;

/// \brief Structure for a time entry into the repository
///
/// The structure contains the time corresponding to the observables, the
/// local observables, and a set of remote observables contained in a map
/// that is keyed off of remote node id
struct TimeEntry
{
  /// The time of week the data were measured or correspond to
  double timeOfWeek_;
  /// The local observables
  RepositoryEntry localData_;
  /// A map of remote observables
  RemoteRepoEntries remoteData_;

  /// \brief Default constructor
  ///
  /// Declaring the default constructor implicitly allows for copy construction
  /// which is used when a new time entry is created
  TimeEntry(){};

  /// \brief Constructor for creation of entry with time field already known
  ///
  /// \param timeOfWeek The time of week that all observables in the entry
  ///                   correspond to
  TimeEntry(const double& timeOfWeek) : timeOfWeek_(timeOfWeek){};
};

/// Defining a type for a history of time entries, which is realized by an
/// ordered map keyed on time.
using TimeEntryHistory = std::map<double, TimeEntry>;

//==============================================================================
//---------------------- IntegrityDataRepository Class -------------------------
//==============================================================================
/// \brief Class definition for the history of data at a single PNT node
///
/// The IntegrityDataRepository object is a singleton, so therefore only 1
/// observable history lives in the application
class IntegrityDataRepository
{
public:
  //============================================================================
  //------------------------ Object accessor functions -------------------------
  //============================================================================

  /// \brief Function to gain a singlton instance of the history
  ///
  /// \returns The unique instance of the history object
  static IntegrityDataRepository& getInstance()
  {
    static IntegrityDataRepository instance;
    return instance;
  }

  /// \brief Delete the copy constructor
  ///
  /// Deleting the copy constructor to help ensure singleton
  IntegrityDataRepository(IntegrityDataRepository const&) = delete;

  //============================================================================
  //-------------------- Generic data accessor functions -----------------------
  //============================================================================
  /// \brief Delete the assignment operator
  ///
  /// Deleting the assignment operator to help insure singleton
  void operator=(IntegrityDataRepository constIntegrityDataRepository) = delete;

  /// \brief Adds a local data entry to the repo
  ///
  /// Adds a local measurement to the entry
  ///
  /// \param timeOfWeek The time associated with the observable
  /// \param data The local data structure
  template <class T>
  void addEntry(const double& timeOfWeek, const T& data);

  /// \brief Adds a remote data entry to the repo
  ///
  /// Adds a local remote to the entry
  ///
  /// \param timeOfWeek The time associated with the observable
  /// \param nodeId The name or node ID of the remote
  /// \param data The remote data structure
  template <class T>
  void addEntry(const double&      timeOfWeek,
                const std::string& nodeId,
                const T&           data);

  /// \brief Returns the local data entry at the specified time
  ///
  /// \param timeOfWeek The time of the desired data
  /// \param data The requested local data entry
  template <class T>
  bool getData(const double& timeOfWeek, T& data);

  /// \brief Returns the newest available local data entry of type T
  ///
  /// \param data The requested local data entry
  /// \param time The time of the found data
  template <class T>
  bool getNewestData(T& data, double& time);

  /// \brief Returns the remote data entry at the specified time
  ///
  /// \param timeOfWeek The time of the desired data
  /// \param nodeId The identifier string for the desired node
  /// \param data The requested remote data entry
  template <class T>
  bool getData(const double& timeOfWeek, const std::string& nodeId, T& data);

  /// \brief Returns the newest available remote data entry of type T
  ///
  /// \param nodeId The identifier string for the desired node
  /// \param data The requested remote data entry
  /// \param time The time of the found data
  template <class T>
  bool getNewestData(const std::string& nodeId, T& data, double& time);

  //============================================================================
  //----------------- Local GNSS Observable accessor functions -----------------
  //============================================================================

  /// \brief Adds a local GNSSObservable value entry
  ///
  /// \param timeOfWeek The time associated with the observable
  /// \param satelliteID The ID number for the GNSS observable's origin
  /// \param gnssObs The GNSS observable to add to the repository
  void addEntry(const double&               timeOfWeek,
                const uint32_t&             satelliteID,
                const data::GNSSObservable& gnssObs);

  /// \brief Retrieves a local GNSS observable from the time history
  ///
  /// \param timeOfWeek The time associated with the observable
  /// \param satelliteID The ID number for the GNSS observable's origin
  /// \param gnssObs The GNSS observable.
  /// \returns True if the observable exists
  bool getData(const double&         timeOfWeek,
               const uint32_t&       satelliteID,
               data::GNSSObservable& gnssObs);

  //============================================================================
  //----------------- Remote GNSS Observable accessor functions ----------------
  //============================================================================

  /// \brief Adds a remote GNSSObservable entry
  ///
  /// \param timeOfWeek The time associated with the observable
  /// \param nodeID The identifier of the remote node
  /// \param satelliteID The ID number for the GNSS observable's origin
  /// \param gnssObs The GNSS observable.
  void addEntry(const double&               timeOfWeek,
                const std::string&          nodeID,
                const uint32_t&             satelliteID,
                const data::GNSSObservable& gnssObs);

  /// \brief Retrieves a local GNSS observable from the time history
  ///
  /// \param timeOfWeek The time associated with the observable
  /// \param nodeID The identifier of the remote node
  /// \param satelliteID The ID number for the GNSS observable's origin
  /// \param gnssObs The GNSS observable.
  /// \returns True if the remote node and observable exist
  bool getData(const double&         timeOfWeek,
               const std::string&    nodeID,
               const uint32_t&       satelliteID,
               data::GNSSObservable& gnssObs);

  //============================================================================
  //-------------------------- Management Functions ----------------------------
  //============================================================================

  /// \brief Sets the history period
  ///
  /// Defines the time history length that resides in the data history.
  /// Defaults to 10 if this function is not called
  ///
  /// \param period The time (in seconds) that will be kept in the history
  void setHistoryPeriod(const double& period)
  {
    std::lock_guard<std::recursive_mutex> lock(repoMutex_);
    historyPeriod_ = period;
  };

  /// \brief Sets the log message handler to provided callback
  ///
  /// \param logMsgHandler The provided call back function
  void setLogMessageHandler(const logutils::LogCallback& logMsgHandler)
  {
    std::lock_guard<std::recursive_mutex> lock(repoMutex_);
    logMsg_ = logMsgHandler;
  };

  //============================================================================
  //---------------------------- Repo management--------------------------------
  //============================================================================
  /// \brief Trims the stored data in the repository
  ///
  /// This function will trim the repository to the length determined by
  /// setHistoryPeriod. The default history is 10 if not set
  void manageHistory();

  /// \brief Returns the size of the repo (number of time entries)
  ///
  /// Returns the number of time entries into the repsoitory
  ///
  /// \returns The number of time entries
  size_t getRepoSize()
  {
    std::lock_guard<std::recursive_mutex> lock(repoMutex_);
    return repository_.size();
  };

  /// \brief Returns the newest time entry
  ///
  /// This function will return the newest time entry in the repo
  ///
  /// \param timeEntry The newest time entry returned by reference
  /// \returns True if the repository is not empty
  bool getNewestEntry(TimeEntry& timeEntry);

  /// \brief Returns the time entry for the specified time
  ///
  /// Just a public wrapper for findEntry
  ///
  /// \param timeOfWeek The time to get the entry for
  /// \param timeEntry  The time entry returned by reference
  /// \returns True if the repository is not empty
  bool getEntry(const double& timeOfWeek, TimeEntry& timeEntry);

  /// \brief Returns the newest time entries that start appear after
  /// a given time
  ///
  /// This function will return the time entries that are within
  /// the time range of now and the given start time. It will
  /// return as many as it can before running out of entries.
  ///
  /// \param timeEntryVec The vector of the newest time entries
  /// \param startTime The earliest time entry to return
  /// \returns True if the repository is not empty
  bool getNewestEntries(std::vector<TimeEntry>& timeEntryVec, double startTime);

  /// \brief Comparator function for sorting TimeEntry objects by
  /// their time of week.
  ///
  /// Can be used with std::sort on vectors of TimeEntry objects.
  ///
  /// \param t0 The first TimeEntry to compare
  /// \param t1 The second TimeEntry to compare
  /// \returns true if t0.timeOfWeek_ < t1.timeOfWeek_, false otherwise
  static bool sortTimeEntry(TimeEntry& t0, TimeEntry& t1)
  {
    return (t0.timeOfWeek_ < t1.timeOfWeek_);
  }

  /// \brief Clear the repository contents.
  void clearEntries()
  {
    std::lock_guard<std::recursive_mutex> lock(repoMutex_);
    repository_.clear();
  }

private:
  //============================================================================
  //------------------------ Constructor / Destructor---------------------------
  //============================================================================
  // Private destructor for singleton object
  IntegrityDataRepository()
    : logMsg_(logutils::printLogToStdOut), historyPeriod_(10.0){};

  //============================================================================
  //------------------------------ Log functions -------------------------------
  //============================================================================
  logutils::LogCallback logMsg_;

  //============================================================================
  //------------------------ Entry accessor functions --------------------------
  //============================================================================
  // Find the correct time entry in the history and return it to the caller to
  // add or read data. Flag to indicate create if not found
  bool findEntry(const double& timeOfWeek, TimeEntry& timeEntry);

  TimeEntry& makeEntry(const double& timeOfWeek);

  //============================================================================
  //---------------------------- Member Variables ------------------------------
  //============================================================================
  // A map keyed on time to hold history
  TimeEntryHistory     repository_;
  std::recursive_mutex repoMutex_;
  std::atomic<double>  historyPeriod_;
};

//==============================================================================
//---------------------- Generic data accessor functions -----------------------
//==============================================================================
template <class T>
void IntegrityDataRepository::addEntry(const double& timeOfWeek, const T& data)
{
  std::lock_guard<std::recursive_mutex> lock(repoMutex_);

  // Make (or copy the existing) a entry for this time
  TimeEntry entry = makeEntry(timeOfWeek);

  // add the data to the local observables
  entry.localData_.addEntry(data);
  // add the entry back to the history
  repository_[timeOfWeek] = entry;
  manageHistory();
}

//------------------------------------------------------------------------------
template <class T>
void IntegrityDataRepository::addEntry(const double&      timeOfWeek,
                                       const std::string& nodeID,
                                       const T&           data)
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
    entry.remoteData_[nodeID].addEntry(data);
  }
  else
  {
    // a remote observable does not exist for this node ID, create it and add
    // it to the remote observables list
    RepositoryEntry remoteEntry(DataLocaleType::Remote, nodeID, logMsg_);

    remoteEntry.addEntry(data);
    entry.remoteData_[nodeID] = remoteEntry;

    data::GNSSObservableMap obsMap;
    remoteEntry.getData(obsMap);
  }
  // add the entry back to the history
  repository_[timeOfWeek] = entry;
  manageHistory();
}

//------------------------------------------------------------------------------
// Template function for local data retrieval
template <class T>
bool IntegrityDataRepository::getData(const double& timeOfWeek, T& data)
{
  std::lock_guard<std::recursive_mutex> lock(repoMutex_);

  TimeEntry timeEntry;
  if (findEntry(timeOfWeek, timeEntry))
  {
    // Time entry exists
    timeEntry.localData_.getData(data);
    return true;
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

//------------------------------------------------------------------------------
// Template function for newest available local data retrieval
template <class T>
bool IntegrityDataRepository::getNewestData(T& data, double& time)
{
  if (repository_.size() > 0)
  {
    // TimeEntryHistory::reverse_iterator rit = repository_.rbegin();

    // Search backwards through time history
    // while (rit != repository_.rbegin())
    for (TimeEntryHistory::reverse_iterator rit = repository_.rbegin();
         rit != repository_.rend();
         ++rit)
    {
      // If data is available, return it
      if (rit->second.localData_.getData(data))
      {
        time = rit->first;
        return true;
      }
    }
  }
  else
  {
    // No data found in time history
    std::stringstream errMsg;
    errMsg << "IntegrityDataRepository::getNewestData() : No entry found ";
    logMsg_(errMsg.str(), logutils::LogLevel::Warn);
  }
  return false;
}

//------------------------------------------------------------------------------
template <class T>
bool IntegrityDataRepository::getData(const double&      timeOfWeek,
                                      const std::string& nodeID,
                                      T&                 data)
{
  std::lock_guard<std::recursive_mutex> lock(repoMutex_);

  TimeEntry timeEntry;
  if (findEntry(timeOfWeek, timeEntry))
  {
    // Time entry exists
    // make sure remote observable exists for this time
    auto remoteIt = timeEntry.remoteData_.find(nodeID);

    if (remoteIt != timeEntry.remoteData_.end())
    {
      // The remote entry exists at the provided time, return it
      remoteIt->second.getData(data);
      return true;
    }
    else
    {
      // The remote does not exist at the provided time
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
    std::stringstream errMsg;
    errMsg << "IntegrityDataRepository::getData() : No entry at time "
           << timeOfWeek;
    logMsg_(errMsg.str(), logutils::LogLevel::Error);
    return false;
  }
}

//------------------------------------------------------------------------------
template <class T>
bool IntegrityDataRepository::getNewestData(const std::string& nodeID,
                                            T&                 data,
                                            double&            time)
{
  if (repository_.size() > 0)
  {
    // TimeEntryHistory::reverse_iterator rit = repository_.rbegin();

    // Search backwards through time history
    // while (rit != repository_.rbegin())
    for (TimeEntryHistory::reverse_iterator rit = repository_.rbegin();
         rit != repository_.rend();
         ++rit)
    {
      auto remoteIt = rit->second.remoteData_.find(nodeID);

      if (remoteIt != rit->second.remoteData_.end())
      {
        // The remote entry exists at the provided time

        if (remoteIt->second.getData(data))
        {
          time = rit->first;
          return true;  // The data exists
        }
      }
      else
      {
        // The remote does not exist at the provided time
        std::stringstream errMsg;
        errMsg << "IntegrityDataRepository::getNewestData() : No data for "
                  "Remote ID '"
               << nodeID << "' at time (" << rit->second.timeOfWeek_ << ")";
        logMsg_(errMsg.str(), logutils::LogLevel::Debug);
      }
      // rit++;
    }
  }
  else
  {
    // No data found in time history
    std::stringstream errMsg;
    errMsg << "IntegrityDataRepository::getNewestData() : No entries found ";
    logMsg_(errMsg.str(), logutils::LogLevel::Warn);
  }
  return false;
}
}  // namespace pnt_integrity
#endif
