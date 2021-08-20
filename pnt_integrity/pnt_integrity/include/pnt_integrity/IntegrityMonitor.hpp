//============================================================================//
//---------------------pnt_integrity/IntegrityMonitor.hpp ------*- C++ -*-----//
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
/// \brief    Defines the IntegrityMonitor class in pnt_integrity
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     May 28, 2019
//============================================================================//
#ifndef PNT_INTEGRRITY__INTEGRITY_MONITOR_HPP
#define PNT_INTEGRRITY__INTEGRITY_MONITOR_HPP

#include "logutils/logutils.hpp"
#include "pnt_integrity/AssuranceCheck.hpp"

#include <iomanip>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <sstream>
#include <vector>

/// Namespace for all pnt_integrity applications
namespace pnt_integrity
{
/// A vector type for a collection of AssuranceChecks
using AssuranceChecks = std::map<std::string, AssuranceCheck*>;

/// \brief Class implementation of integrity monitoring using AssuranceChecks
/// and IntegrityData
class IntegrityMonitor
{
public:
  /// \brief Default constructor
  ///
  /// The constructor set's the repository's log handling to use the logging
  /// function provided by the integrity monitor
  ///
  /// \param log A log callback function for log messages
  IntegrityMonitor(
    const logutils::LogCallback& log = logutils::printLogToStdOut);

  /// \brief Returns an instance to the repository
  ///
  /// \returns A singleton instance of the repository
  IntegrityDataRepository& getRepo()
  {
    return IntegrityDataRepository::getInstance();
  };

  /// \brief Function to register user-defined check
  ///
  /// Register's an assurance check with the monitor. The process simply adds
  /// a provided pointer to the check to an internally held vector of check
  /// pointers
  ///
  /// \param checkName The name of the check object
  /// \param checkPtr A pointer to an AssuranceCheck
  /// \returns True if successful
  bool registerCheck(const std::string& checkName, AssuranceCheck* checkPtr);

  /// \brief Return function for the multi-prn assurance data
  // TODO: This method really needs to be smarter, maybe make it a time vector
  void setMultiPrnAssuranceData(MultiPrnAssuranceMap al)
  {
    prnAssuranceLevels_ = al;
  }

  /// \brief Return function for the multi-prn assurance data
  MultiPrnAssuranceMap getMultiPrnAssuranceData()
  {
    std::lock_guard<std::mutex> lock(monitorMutex_);
    return prnAssuranceLevels_;
  };

  /// \brief Returns overall assurance level
  data::AssuranceLevel getAssuranceLevel()
  {
    std::lock_guard<std::mutex> lock(monitorMutex_);
    return assuranceState_.getAssuranceLevel();
  };

  /// \brief Returns overall assurance value
  double getAssuranceValue()
  {
    std::lock_guard<std::mutex> lock(monitorMutex_);
    return assuranceState_.getAssuranceValue();
  }

  /// \brief Returns assurance reports from all registered checks
  data::AssuranceReports getAssuranceReports();

  /// \brief Calculates overall assurance levels accross all registered checks
  void determineAssuranceLevels();

  /// \brief Handler function for GNSSObservables
  ///
  /// Call this function on receipt of a GNSSObservables message. The function
  /// will call the handleGnssObservables in all registered checks
  /// \param localFlag A flag to indicate if the source of the observable data
  ///                  is from a local or remote source (defaults to "True")
  ///
  /// \param gnssObs The provided data message
  void handleGnssObservables(const data::GNSSObservables& gnssObs,
                             const bool&                  localFlag = true);

  /// \brief Handler function for GNSSSubframe
  ///
  /// Call this function on receipt of a GNSSSubframe message. The function
  /// will call the handleGnssSubframe in all registered checks
  /// \param localFlag A flag to indicate if the source of the observable data
  ///                  is from a local or remote source (defaults to "True")
  ///
  /// \param gnssObs The provided data message
  void handleGnssSubframe(const data::GNSSSubframe& gnssObs,
                          const bool&               localFlag = true);

  /// \brief Handler function for PositionVelocity messages
  ///
  /// Call this function on receipt of a PositionVelocity message. The function
  /// will call the handlePositionVelocity in all registered checks
  /// \param localFlag A flag to indicate if the source of the data
  ///                  is from a local or remote source (defaults to "True")
  ///
  /// \param posVel The provided data message
  void handlePositionVelocity(const data::PositionVelocity& posVel,
                              const bool&                   localFlag = true);

  /// \brief Handler function for Estimated PositionVelocity messages
  ///
  /// Call this function on receipt of a PositionVelocity message that contains
  /// an external estimate of the current position and velocity. The function
  /// will call the handleEstimatedPositionVelocity in all registered checks
  /// \param localFlag A flag to indicate if the source of the data
  ///                  is from a local or remote source (defaults to "True")
  ///
  /// \param posVel The provided data message
  void handleEstimatedPositionVelocity(const data::PositionVelocity& posVel,
                                       const bool& localFlag = true);

  /// \brief Handler function for AccumulatedDistranceTraveled messages
  ///
  /// \param dist The provided distance traveled message
  void handleDistanceTraveled(const data::AccumulatedDistranceTraveled& dist);

  /// \brief Handler function for MeasuredRange messages
  ///
  /// Call this function on receipt of a MeasuredRange message. The function
  /// will call the handleMeasuredRange in all registered checks
  /// \param localFlag A flag to indicate if the source of the data
  ///                  is from a local or remote source (defaults to "True")
  ///
  /// \param range The provided measured range message
  void handleMeasuredRange(const data::MeasuredRange& range,
                           const bool&                localFlag = true);

  /// \brief Handler function for IFSampleData messages
  ///
  /// Call this function on receipt of an IFSampleData message. The function
  /// will call the handleIfSampleData on all registered checks
  /// \param time The timestamp of the IF Data
  /// \param ifData The incoming IF sample data
  template <typename samp_type>
  void handleIfSampleData(const double&                                 time,
                          const if_data_utils::IFSampleData<samp_type>& ifData);

  /// \brief Handler function for ClockOffset messages
  ///
  /// Call this function on receipt of a ClockOffset message. The function
  /// will call the handleClockOffset on all registered checks
  /// \param clockOffset The clock bias and drift with Header for timestamp
  /// \param localFlag Flag to indicate local or remote data
  void handleClockOffset(const data::ClockOffset& clockOffset,
                         const bool&              localFlag);

  /// \brief Handler function AGC setting
  ///
  /// Call this function on receipt of an AGC setting
  /// \param agcValue The current AGC setting from a a receiver
  void handleAGC(const data::AgcValue& agcValue);

  /// \brief Template function that determines the correct timestamp
  ///
  /// \param time The timestamp used for time entries into the repo
  /// \param data The received data message /structure to be entered
  /// \param local A flag to indicate if the data is local or remote data
  /// \param deviceId A string to identify remote data entries
  template <class T>
  double getCorrectedEntryTime(const double&      time,
                               const T&           data,
                               const bool&        local    = true,
                               const std::string& deviceId = std::string());

  /// \brief Template function that adds received data to the repository
  ///
  /// \param time The timestamp used for time entries into the repo
  /// \param data The received data message /structure to be entered
  /// \param local A flag to indicate if the data is local or remote data
  /// \param deviceId A string to identify remote data entries
  template <class T>
  void addDataToRepo(const double&      time,
                     const T&           data,
                     const bool&        local    = true,
                     const std::string& deviceId = std::string());

  /// \brief Sets the log message handler to provided callback
  ///
  /// \param logMsgHandler The provided call back function
  void setLogMessageHandler(const logutils::LogCallback& logMsgHandler)
  {
    std::lock_guard<std::mutex> lock(monitorMutex_);
    logMsg_ = logMsgHandler;
    // set the repo's logger to use the integrity monitor's logging
    IntegrityDataRepository::getInstance().setLogMessageHandler(logMsgHandler);
  };

  /// \brief Returns the number of assurance checks currently used in the
  /// monitor
  ///
  /// \returns The number of assurance checks
  size_t getNumUsedChecks()
  {
    std::lock_guard<std::mutex> lock(monitorMutex_);
    return checksUsed_.size();
  };

  /// \brief Returns a flag to indicate if check was used in current
  /// level calculation
  bool isCheckUsed(const std::string& checkName)
  {
    std::lock_guard<std::mutex> lock(monitorMutex_);
    for (auto name : checksUsed_)
    {
      if (name == checkName)
      {
        return true;
      }
    }
    return false;
  };

  /// \brief Reset the integrity monitor
  void reset();

  void setLastKnownGoodPosition(const data::PositionVelocity& posVel);
  
  void clearLastKnownGoodPosition();

private:
  std::shared_timed_mutex checkMutex_;
  AssuranceChecks         checks_;

  // class level mutex for thread safety
  std::mutex monitorMutex_;

  logutils::LogCallback logMsg_;

  MultiPrnAssuranceMap prnAssuranceLevels_;

  data::AssuranceState assuranceState_;

  data::GeodeticPosition3d lastKnownGoodPosition_;

  std::vector<std::string> checksUsed_;

  bool getRoundedValidTime(const data::Header& header, double& timestampValid)
  {
    // throw out measurements with large differences in arrival and validity
    // timestamps
    bool valid =
      std::abs(header.timestampArrival.sec - header.timestampValid.sec) < 5;

    if (valid)
    {
      double fullSeconds = (double)header.timestampValid.sec;
      double fracSeconds = ((double)(header.timestampValid.nanoseconds)) / 1e9;

      timestampValid = std::round(fullSeconds + fracSeconds);
    }
    else
    {
      std::cout << "Ignoring invalid timestamp. Arrival: "
                << header.timestampArrival.sec
                << " Validity: " << header.timestampValid.sec << std::endl;
      timestampValid = 0;
    }

    return valid;
  }
};

//==================================================================
//------------------------getCorrectedEntryTime-----------------------
//==================================================================
template <class T>
double IntegrityMonitor::getCorrectedEntryTime(const double&      time,
                                               const T&           data,
                                               const bool&        localFlag,
                                               const std::string& deviceId)
{
  std::stringstream msg;
  msg << std::setprecision(10);
  msg << "IntegrityMonitor::getCorrectEntryTime: input time: " << time;

  // Retrieve most recent available type T data from repository
  T      lastData;
  double lastDataTime;
  double newTime       = time;
  bool   foundLastData = false;

  if (localFlag)
  {
    foundLastData = IntegrityDataRepository::getInstance().getNewestData(
      lastData, lastDataTime);
  }
  else
  {
    foundLastData = IntegrityDataRepository::getInstance().getNewestData(
      deviceId, lastData, lastDataTime);
  }

  if (!foundLastData)
  {
    // logMsg_("!foundLastData", logutils::LogLevel::Debug);
    // Do nothing
  }
  else if ((time - lastDataTime) == 1)
  {  // If TimeEnty time indices are consitent, do nothing
     // logMsg_("(time - lastDataTime) == 1", logutils::LogLevel::Debug);
  }
  else
  {  // Else TimeEnty time indices are inconsitent, check conditions and repair
    logMsg_("Inconsistent times!!", logutils::LogLevel::Debug);
    if ((data.header.seq_num - lastData.header.seq_num) == 1)
    {  // If seq_num's are consecutive
      logMsg_("(data.header.seq_num - lastData.header.seq_num) == 1",
              logutils::LogLevel::Debug);
      if (std::abs(data.header.timestampArrival.nanoseconds - 0.5e9) < 0.25e9)
      {  // If I'm in the window around 0.5 where rounding direction switches
         // occur
        logMsg_("std::abs(data.header.timestampArrival.nanoseconds - 5e9) < 25",
                logutils::LogLevel::Debug);
        double lastTimestampArrival =
          ((double)lastData.header.timestampArrival.sec +
           ((double)(lastData.header.timestampArrival.nanoseconds)) / 1e9);
        double timestampArrival =
          (double)data.header.timestampArrival.sec +
          ((double)(data.header.timestampArrival.nanoseconds)) / 1e9;
        double actualDt = timestampArrival - lastTimestampArrival;

        if ((actualDt < 1.5) & (actualDt > 0.5))
        {  // If actual Dt is approximately 1

          if ((time - lastDataTime) == 0)  // Adjust time up 1
          {
            newTime++;
            msg << " , time+1 = " << newTime;
            logMsg_("Increase time 1", logutils::LogLevel::Debug);
          }
          else if ((time - lastDataTime) == 2)  // Adjust time down 1
          {
            newTime--;
            msg << " , time-1 = " << newTime;
            logMsg_("Decrease time 1", logutils::LogLevel::Debug);
          }
        }
      }
    }
  }

  // add the provided observable to the repos as either a local or remote
  // determined by the provided flag
  if (localFlag)
  {
    msg << " (local)";
  }
  else
  {
    msg << " (remote)";
  }
  // logMsg_(msg.str(), logutils::LogLevel::Debug);
  return newTime;
}

//==================================================================
//---------------------------addDataToRepo--------------------------
//==================================================================
template <class T>
void IntegrityMonitor::addDataToRepo(const double&      time,
                                     const T&           data,
                                     const bool&        localFlag,
                                     const std::string& deviceId)
{
  // add the provided observable to the repos as either a local or remote
  // determined by the provided flag
  if (localFlag)
  {
    IntegrityDataRepository::getInstance().addEntry(time, data);
  }
  else
  {
    IntegrityDataRepository::getInstance().addEntry(time, deviceId, data);
  }
}

//==================================================================
//----------------------handleIfSampleData--------------------------
//==================================================================
template <typename samp_type>
void IntegrityMonitor::handleIfSampleData(
  const double&                                 checkTime,
  const if_data_utils::IFSampleData<samp_type>& ifData)
{
  if_data_utils::IFSampleType sampType = ifData.getHeader().sampleType_;
  if ((sampType == if_data_utils::IFSampleType::SC8) or
      (sampType == if_data_utils::IFSampleType::SC16))
  {
    // grant this thread shared read access to checks_
    std::shared_lock<std::shared_timed_mutex> sharedLock(checkMutex_);
    // loop through all checks and call the handler for this data type
    AssuranceChecks::const_iterator checkIt;
    for (checkIt = checks_.begin(); checkIt != checks_.end(); ++checkIt)
    {
      checkIt->second->handleIFSampleData(checkTime, ifData);
    }
  }
  else
  {
    std::lock_guard<std::mutex> lock(monitorMutex_);
    logMsg_(
      "IntegrityMonitor::handleIfSampleData(): sample type not supported. "
      "Currently only SC8 and SC16 are supported data types",
      logutils::LogLevel::Debug);
    // error message
  }
}

}  // namespace pnt_integrity
#endif
