//============================================================================//
//---------------- pnt_integrity/StaticPositionCheck.hpp -------*- C++ -*-----//
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
/// \brief    AssurancCheck class defined for the position velocity consistency
///           check
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \author   John David Sprunger <jss0027@tigermail.auburn.edu>
/// \date     September 18, 2019
//============================================================================//
#ifndef PNT_INTEGRITY__POSITON_VELOCITY_CHECK_HPP
#define PNT_INTEGRITY__POSITON_VELOCITY_CHECK_HPP

#include <cstring>

#include "pnt_integrity/AssuranceCheck.hpp"

namespace pnt_integrity
{
/// String ID for the position-velocity consistent check diagnostics data
const std::string INTEGRITY_PVC_DIAGNOSTICS = "INTEGRITY_PVC_DIAGNOSTICS";
/// String ID for PVC diagnostic key for the "percent bad" variable
const std::string INTEGRITY_PVC_DIAG_PB = "INTEGRITY_PVC_DIAG_PB";
/// String ID for the PVC diagnostic key for the inconsistent threshold
const std::string INTEGRITY_PVC_DIAG_ITHRESH = "INTEGRITY_PVC_DIAG_ITHRESH";
/// String ID for the PVC diagnostic key for the unassured threshold
const std::string INTEGRITY_PVC_DIAG_UTHRESH = "INTEGRITY_PVC_DIAG_UTHRESH";
/// String ID for the PVC diagnostic key for error values
const std::string INTEGRITY_PVC_DIAG_ERR_VAL = "INTEGRITY_PVC_DIAG_ERR_VAL";
/// String ID for the PVC diagnostic key for error thresh values
const std::string INTEGRITY_PVC_DIAG_ERR_THRESH =
  "INTEGRITY_PVC_DIAG_ERR_THRESH";

/// \brief Structure used to publish diagnostic data
struct PosVelConsCheckDiagnostics
{
  /// The error values over all examined pairs
  std::vector<double> errorVals;
  /// The threshold for each error based on velocity variance
  std::vector<double> errorThresh;
  /// Percentage of pairs that are above the threshold
  double percentBad;
  /// The inconsistent threshold used
  double inconsistentThresh;
  /// The unassured threshold used
  double unassuredThresh;
};

/// \brief Class implementation for the position velocity check
class PositionVelocityConsistencyCheck : public AssuranceCheck
{
public:
  /// \brief Default constructor for the check class.
  ///
  /// Constructor explicitly disables multi-prn support.
  ///
  /// \param name The name of the check object
  /// \param sampleWindow The duration of time (in seconds) over which to get
  /// position and velocity data for checking integrity
  /// \param errorThreshSF The scale factor to apply to the velocity variance
  /// that is used as an error threshold
  /// \param log A provided log callback function to use
  PositionVelocityConsistencyCheck(
    const std::string&           name          = "Position Velocity Check",
    const double&                sampleWindow  = 5.0,
    const double&                errorThreshSF = 2.0,
    const logutils::LogCallback& log           = logutils::printLogToStdOut)
    : AssuranceCheck::AssuranceCheck(false, name, log)
    , sampleWindow_(sampleWindow)
    , errorThreshScaleFactor_(errorThreshSF)
  {
    std::stringstream initMsg;
    initMsg << "Initializing Position Velocity Consistency Check (" << name
            << ") with parameters: " << std::endl;
    initMsg << "Sample window: " << sampleWindow << std::endl;
    initMsg << "Error Thresh SF: " << errorThreshSF;
    logMsg_(initMsg.str(), logutils::LogLevel::Info);
  };

  /// \brief Handler function for Position / Velocity message
  ///
  /// Function to handle provided posivion / velocity messages
  ///
  /// \param posVel The provided position velocity message / structure
  /// \param localFlag Indicates if this is a local or remote message
  /// \returns True if successful
  bool handlePositionVelocity(const data::PositionVelocity& posVel,
                              const bool&                   localFlag);

  /// \brief Function to explicitly set the assurance level of the check
  ///
  /// Uses whatever data is available to calculate the assurance level
  void calculateAssuranceLevel(const double& /*time*/) { runCheck(); };

  /// \brief Triggers a manual check calculation
  ///
  /// Use this function to run a manual check calculation that is not triggered
  /// off the receipt of a message (pure virtual)
  ///
  /// \returns True if successful
  bool runCheck();

  /// \brief Connects the internal publishing function to external interface
  ///
  /// This function connects the internal "publishDiagnostics" function
  /// to an external, custom function of choice
  ///
  /// \param handler Provided handler function
  void setPublishDiagnostics(
    std::function<void(const double&                     timestamp,
                       const PosVelConsCheckDiagnostics& checkData)> handler)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    publishDiagnostics_ = handler;
  };

private:
  /// \brief Checks if, in a certain period of time up until now,
  /// a stationary position drifts off of the original position
  ///
  /// \param timeEntryVec the list of time entries to exam1ine
  /// \returns true if there's enough samples to make an informed
  /// statement about the position
  bool posVelCheck(const double&                 time,
                   const std::vector<TimeEntry>& timeEntryVec);

  /// \brief Propagates the given position foward based on the
  /// given velocity and delta time
  ///
  /// \param p Position to propagate
  /// \param v Velocity, must be an array of 3 doubles
  /// \param dt Change in time
  /// \returns Position after moving at the given velocity
  /// for the given amount of time
  data::GeodeticPosition3d propagatePosition(data::GeodeticPosition3d p,
                                             double*                  v,
                                             double                   dt);

  double sampleWindow_;

  double errorThreshScaleFactor_;

  std::function<void(const double&                     timestamp,
                     const PosVelConsCheckDiagnostics& checkData)>
    publishDiagnostics_;
};

}  // namespace pnt_integrity
#endif
