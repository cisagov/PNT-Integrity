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
/// \file
/// \brief    AssurancCheck class defined for the range / position check
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     June 11, 2019
//============================================================================//
#ifndef PNT_INTEGRITY__RANGE_POSITION_CHECK_HPP
#define PNT_INTEGRITY__RANGE_POSITION_CHECK_HPP

#include "pnt_integrity/AssuranceCheck.hpp"

namespace pnt_integrity
{
/// String ID for the range-position check diagnostic data
const std::string INTEGRITY_RNG_POS_DIAGNOSTICS =
  "INTEGRITY_RNG_POS_DIAGNOSTICS";
/// String ID for the range-position check max calculated range
const std::string INTEGRITY_RNG_POS_DIAG_MAX_CALC =
  "INTEGRITY_RNG_POS_DIAG_MAX_CALC";
/// String ID for the range-position check min calculated range
const std::string INTEGRITY_RNG_POS_DIAG_MIN_CALC =
  "INTEGRITY_RNG_POS_DIAG_MIN_CALC";
/// String ID for the range-position check max measured range
const std::string INTEGRITY_RNG_POS_DIAG_MAX_MEAS =
  "INTEGRITY_RNG_POS_DIAG_MAX_MEAS";
/// String ID for the range-position check min measured range
const std::string INTEGRITY_RNG_POS_DIAG_MIN_MEAS =
  "INTEGRITY_RNG_POS_DIAG_MIN_MEAS";

/// Definition of 'pi' to use in this check
const double pi = 3.1415926535898;
/// Convenience constant for 1/2 pi
const double deg2rad = pi / 180.0;
/// Convenience converter for radians to degrees
const double rad2deg = 1.0 / deg2rad;

/// \brief Structure for check diagnostics
struct RngPosCheckNodeDiagnostic
{
  /// The minimum calculated distance based on both positions and variances
  double minCalculatedRange;
  /// The  maximum calculated distance based on both positions and variances
  double maxCalculatedRange;
  /// The minimum possible distance based on measured range and variance
  double minMeasRange;
  /// The maximum possible distance based on measured range and variance
  double maxMeasRange;
};
/// \brief Defined type for check diagnostics
using RngPosCheckDiagnostics = std::map<std::string, RngPosCheckNodeDiagnostic>;

/// \brief Class implementation for the range / position check
class RangePositionCheck : public AssuranceCheck
{
public:
  /// \brief Default constructor for the check class.
  ///
  /// Constructor explicitly disables multi-prn support.
  ///
  /// \param name The name of the check object
  /// \param log A provided log callback function to use
  RangePositionCheck(
    const std::string&           name = "Range-Position check",
    const logutils::LogCallback& log  = logutils::printLogToStdOut)
    : AssuranceCheck::AssuranceCheck(false, name, log)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    std::stringstream                     initMsg;
    initMsg << "Initializing Range Position Check (" << name
            << ")with parameters: ";

    logMsg_(initMsg.str(), logutils::LogLevel::Info);
  };

  /// \brief Triggers a manual check calculation
  ///
  /// Use this function to run a manual check calculation that is not triggered
  /// off the receipt of a message (pure virtual)
  ///
  /// \returns True if successful
  bool runCheck();

  /// \brief Handler function for Position / Velocity message
  ///
  /// Function to handle provided posivion / velocity messages
  ///
  /// \param posVel The provided position velocity message / structure
  /// \param local Indicates if this is a local or remote message
  /// \returns True if successful
  virtual bool handlePositionVelocity(const data::PositionVelocity& posVel,
                                      const bool&                   local);

  /// \brief Handler function for measrured range
  ///
  /// Function to handle provided range measurements (pure virtual)
  ///
  /// \param range The provided range measurement message / structure
  /// \returns True if successful
  virtual bool handleMeasuredRange(const data::MeasuredRange& range);

  /// \brief Function to explicitly set the assurance level of the check
  ///
  /// For this check, this function cycles through all of the individual PRN
  /// assurance values, analyzes them, and then sets the master assurance level
  /// associted with the check.
  void calculateAssuranceLevel(const double& time);

  /// \brief Connects the internal publishing function to external interface
  ///
  /// This function connects the internal "publishDiagnostics" function
  /// to an external, custom function of choice
  ///
  /// \param handler Provided handler function
  void setPublishDiagnostics(
    std::function<void(const double& /*timestamp*/,
                       const RngPosCheckDiagnostics& /*data*/)> handler)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    publishDiagnostics_ = handler;
  };

private:
  void rangePositionCheck(const double&            time,
                          const RepositoryEntry&   localEntry,
                          const RemoteRepoEntries& remoteEntries);

  bool compareRanges(const data::PositionVelocity& posVel1,
                            const data::PositionVelocity& posVel2,
                            const data::MeasuredRange&    measRange,
                            RngPosCheckNodeDiagnostic&    diagnostic);

  std::map<std::string, data::AssuranceLevel> rangeCheckLevels_;

  std::function<void(const double&                 timestamp,
                     const RngPosCheckDiagnostics& diagnostics)>
    publishDiagnostics_;
};

}  // namespace pnt_integrity
#endif
