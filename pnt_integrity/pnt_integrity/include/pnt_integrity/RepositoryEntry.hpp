//============================================================================//
//--------------------- pnt_integrity/RepositoryEntry.hpp ------*- C++ -*-----//
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
/// \brief    Defines the RepositoryEntry class in pnt_integrity
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     May 28, 2019
//============================================================================//
#ifndef REPOSITORY_ENTRY_HPP
#define REPOSITORY_ENTRY_HPP

#include <cmath>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "logutils/logutils.hpp"
#include "pnt_integrity/IntegrityData.hpp"
namespace pnt_integrity
{
//==============================================================================
//----------------------------- Data Structures --------------------------------
//==============================================================================
/// Defines the possible observable types
enum class DataLocaleType
{
  Local  = 0,
  Remote = 1
};

//==============================================================================
//---------------------- IntegrityDataRepository Class -------------------------
//==============================================================================

/// \brief Class definition for an entry into the repository
///
/// A RepositoryEntry represents a collection of integrity data measurements
/// from a single node at a unique time. Currently the object containes an
/// RfRange and a GNSSObservableMap. More data structures will be added as
/// new integrity checks are added to the framework.
class RepositoryEntry
{
public:
  /// \brief Constructor for an entry into the repository
  ///
  /// The constructor takes in a string that indicates what node the
  /// data / measurement / observable  belongs to. Defaults to "local" to
  /// indicate that the observable was taken at this node's location.
  RepositoryEntry(const DataLocaleType&        type   = DataLocaleType::Local,
                  const std::string&           nodeID = "local",
                  const logutils::LogCallback& log = logutils::printLogToStdOut)
    : nodeId_(nodeID), dataLocaleType_(type), logMsg_(log){};

  //============================================================================
  //---------------------- GNSS Observable accessor functions ------------------
  //============================================================================

  /// \brief Adds a provided GNSS observable into the data entry
  ///
  /// This function will place the provided GNSS observable structure in the map
  /// with the corresponding satellite ID key.
  ///
  /// \note If data for the provided satelite ID already exists, it will be
  ///       overwritten.
  ///
  /// \param satelliteID The satellite id number (or PRN)
  /// \param gnssObs The GNSS observable data structure
  void addEntry(const uint32_t&             satelliteID,
                const data::GNSSObservable& gnssObs)
  {
    gnssObsMap_[satelliteID] = gnssObs;
  };

  /// \brief Returns a GNSS Observable
  ///
  /// \param satelliteID The satellite id number (or PRN)
  /// \param gnssObs The returned GNSS observable data structure
  /// \returns True if the observable exists
  bool getData(const uint32_t&       satelliteID,
               data::GNSSObservable& gnssObs) const;

  //============================================================================
  //-----------------  GNSS Observable map accessor functions ------------------
  //============================================================================

  /// \brief Adds a provided GNSS observable map into the data entry
  ///
  /// This function will place the provided GNSS observable map into the entry.
  /// It overwrites the existing map entry with the provided one. Use the
  /// addEntry function for a single GNSSObservable to add to the existing map
  ///
  /// \param gnssObsMap The provided GNSS observableMap
  void addEntry(const data::GNSSObservableMap& gnssObsMap)
  {
    gnssObsMap_ = gnssObsMap;
  };

  /// \brief Returns a GNSS Observable
  ///
  /// \param gnssObsMap The returned GNSS observable map
  /// \returns True if the observable map exists
  void getData(data::GNSSObservableMap& gnssObsMap) const
  {
    gnssObsMap = gnssObsMap_;
  };

  //============================================================================
  //----------------------- RF Range accessor functions ------------------------
  //============================================================================

  /// \brief Adds an a measured (RF) range to another location or node
  ///
  /// \note Any existing value will be overwritten
  ///
  /// \param range The measured range
  void addEntry(const data::MeasuredRange& range);

  /// \brief Returns the RF range observable
  ///
  /// \param range The returned value
  void getData(data::MeasuredRange& range) const { range = range_; };

  //============================================================================
  //------------------ Position / velocity accessor functions ------------------
  //============================================================================
  /// \brief Adds position velocity measurement data to the entry
  ///
  /// \param posVel The provided position / velocity structure
  void addEntry(const data::PositionVelocity& posVel)
  {
    positionVelocity_ = posVel;
  };

  /// \brief Returns the position velocity data from the repo entry
  ///
  //// \param posVel The returned data structure
  void getData(data::PositionVelocity& posVel) const
  {
    posVel = positionVelocity_;
  };

  //============================================================================
  //----------------------- Clock Offset accessor functions --------------------
  //============================================================================
  /// \brief Adds clock offset data to the entry
  ///
  /// \param clockOffset The provided clock offset structure
  void addEntry(const data::ClockOffset& clockOffset)
  {
    clockOffset_ = clockOffset;
  };

  /// \brief Returns the clock offset data from the repo entry
  ///
  //// \param clockOffset The returned data structure
  void getData(data::ClockOffset& clockOffset) const
  {
    clockOffset = clockOffset_;
  };

  //============================================================================
  //-------------------------- Management Functions ----------------------------
  //============================================================================
  /// \brief Sets the log message handler to provided callback
  ///
  /// \param logMsgHandler The provided call back function
  void setLogMessageHandler(const logutils::LogCallback& logMsgHandler)
  {
    logMsg_ = logMsgHandler;
  };

private:
  std::string    nodeId_;
  DataLocaleType dataLocaleType_;

  data::GNSSObservableMap gnssObsMap_;
  data::MeasuredRange     range_;
  data::PositionVelocity  positionVelocity_;
  data::ClockOffset       clockOffset_;

  logutils::LogCallback logMsg_;
};

}  // namespace pnt_integrity

#endif
