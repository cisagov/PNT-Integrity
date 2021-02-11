//============================================================================//
//---------------- pnt_integrity/PositionJumpCheck.hpp ---------*- C++ -*-----//
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
/// \brief    AssuranceCheck class defined for the position jump check
/// \author   Will Travis <will.travis@is4s.com>
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     November 27, 2019
//============================================================================//
#ifndef PNT_INTEGRITY__POSITON_JUMP_CHECK_HPP
#define PNT_INTEGRITY__POSITON_JUMP_CHECK_HPP

#include "pnt_integrity/AssuranceCheck.hpp"
#include "pnt_integrity/GeodeticConverter.hpp"

namespace pnt_integrity
{
/// String ID for the position-jump check diagnostic data
const std::string INTEGRITY_POS_JUMP_DIAGNOSTICS =
  "INTEGRITY_POS_JUMP_DIAGNOSTICS";
/// String ID for the position-jump check bound
const std::string INTEGRITY_POS_JUMP_DIAG_BOUND =
  "INTEGRITY_POS_JUMP_DIAG_BOUND";
/// String ID for the position-jump check distance
const std::string INTEGRITY_POS_JUMP_DIAG_DIST = "INTEGRITY_POS_JUMP_DIAG_DIST";

/// \brief Structure for check diagnostics
struct PosJumpCheckDiagnostics
{
  /// Depending on the mode, this is either the distance to the last known
  /// good position, or the distance to the current estimated position
  double distance;
  /// The bound on the distance, determined by maximum velocity, distance
  /// traveled, or the covariance on the estimated position
  double bound;
};
/// \brief Class implementation for the position-jump check
class PositionJumpCheck : public AssuranceCheck
{
public:
  /// \brief Constructor for the check class
  ///
  /// \param name The string name of the check
  ///
  /// \param minimumBound The minimum amount of position jump that will trigger
  /// the check (meters).
  ///
  /// \param useEstimatedPv Flag to tell check to use the incoming estimated
  /// position rather than distance traveled or max velocity propagation
  ///
  /// \param useDistTraveled Flag to indicate whether or not the check should
  /// use a provided distance traveled to compute the jump bound
  ///
  /// \param maximumVelocity The maximum velocity of the platform that will be
  /// used to calculate the bound if a distance traveled is not used (m/s)
  ///
  /// \param posStdDevMultiplier Scale factor on input position standard
  /// deviation
  ///
  /// \param log A provided log callback function to use for log mesages
  PositionJumpCheck(
    const std::string&           name                = "position_jump_check",
    const double&                minimumBound        = 15.0,
    const bool&                  useEstimatedPv      = false,
    const bool&                  useDistTraveled     = false,
    const double&                maximumVelocity     = 5.0,
    const double&                posStdDevMultiplier = 6.0,
    const logutils::LogCallback& log = logutils::printLogToStdOut)
    : AssuranceCheck(false, name, log)
    , minimumBound_(minimumBound)
    , useEstimatedPv_(useEstimatedPv)
    , useDistTraveled_(useDistTraveled)
    , maximumVelocity_(maximumVelocity)
    , posStdDevMultiplier_(posStdDevMultiplier)
    , distanceTraveled_(0.0)
    , distanceTraveledReceived_(false)
    , positionJumpBound_(minimumBound_)
    , currentEstPvSet_(false)
  {
    if (useEstimatedPv && useDistTraveled)
    {
      std::stringstream errMsg;
      errMsg << "PositionJumpCheck(): both the 'useEstimatedPv' and ";
      errMsg << "the 'useDistTraveled' flags have been set to true. Only one ";
      errMsg << "of these may be used. The 'useDistTraveled' flag is being";
      errMsg << " set to false.";
      logMsg_(errMsg.str(), logutils::LogLevel::Error);
    }

    std::stringstream initMsg;
    initMsg << "Initializing Position-Jump Check (" << name
            << ") with parameters: " << std::endl;
    initMsg << "minimum bound: " << minimumBound << " m" << std::endl;
    initMsg << "using estimated pv: " << useEstimatedPv << std::endl;
    initMsg << "using distance traveled : " << useDistTraveled << std::endl;
    initMsg << "maximum velocity: " << maximumVelocity << " m";
    logMsg_(initMsg.str(), logutils::LogLevel::Info);
  }

  /// \brief Handler function for Position / Velocity message
  ///
  /// Function to handle provided posivion / velocity messages
  ///
  /// \param posVel The provided position velocity message / structure
  /// \param localFlag Indicates if this is a local or remote message
  /// \returns True if successful
  virtual bool handlePositionVelocity(const data::PositionVelocity& posVel,
                                      const bool&                   localFlag);

  /// \brief Handler function for an estimated Position / Velocity message
  ///
  /// Function to handle provided posivion / velocity messages (virtual)
  ///
  /// \param pv The provided estimated position velocity message / structure
  /// \returns True if successful
  virtual bool handleEstimatedPositionVelocity(
    const data::PositionVelocity& pv);

  /// \brief Handler function for AccumulatedDistranceTraveled messages
  ///
  /// \param dist The provided distance traveled message
  /// \returns True if successful
  virtual bool handleDistanceTraveled(
    const data::AccumulatedDistranceTraveled& dist)
  {
    distanceTraveledReceived_ = true;
    if (useDistTraveled_)
    {
      distanceTraveled_ += dist.distance;
      updateBound();
    }
    return true;
  };

  /// \brief Sets the last known good position
  ///
  /// Provides if the assurance check with knowledge of a last known
  /// good position for use in calculations (if needed by the specific
  /// implementation)
  ///
  /// \param updateTime The time associated with the last good position
  /// \param position The last known good position
  void setLastGoodPosition(const double&                   updateTime,
                           const data::GeodeticPosition3d& position);

  /// \brief Triggers a manual check calculation
  ///
  /// Use this function to run a manual check calculation that is not triggered
  /// off the receipt of a message (pure virtual)
  ///
  /// \returns True if successful
  virtual bool runCheck();

  /// \brief Function to explicitly set the assurance level of the check
  ///
  /// Uses whatever data is available to calculate the current assurance level
  virtual void calculateAssuranceLevel(const double& /*time*/) { runCheck(); };

  /// \brief Returns the calculated distance between the current position to the
  /// last good position
  /// \returns The calculated distance
  double getCalculatedDistance() 
  { 
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    return distanceToLastGoodPos_; 
  };

  /// \brief Returns the currently estimated distance traveled since the
  /// last known good position
  /// \returns The estimated distance traveled
  double getDistanceTraveled() 
  { 
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    return distanceTraveled_; 
  };

  /// \brief Returns the current bound that is used by the check
  /// \returns The current jump bound
  double getBound() 
  { 
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    return positionJumpBound_; 
  };

  /// \brief Connects the internal publishing function to external interface
  ///
  /// This function connects the internal "publishDiagnostics" function
  /// to an external, custom function of choice
  ///
  /// \param handler Provided handler function
  void setPublishDiagnostics(
    std::function<void(const double& /*timestamp*/,
                       const PosJumpCheckDiagnostics& /*data*/)> handler)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    publishDiagnostics_ = handler;
  };

private:
  geodetic_converter::GeodeticConverter geodeticConverter_;

  double minimumBound_;     // Minimum position jump bound (m)
  bool   useEstimatedPv_;   // Flag to indicate use of estimated position and
                            // covariane for jump bounds
  bool   useDistTraveled_;  // Flag to indicate use of dist traveled for bounds
  double maximumVelocity_;  // maximum velocity of the platform
  double posStdDevMultiplier_;  // Multiplier for bound based on std dev.
  double distanceTraveled_;     // distance traveled since last good position

  bool distanceTraveledReceived_;  // flag to indicate if distance messages
  // have begun to show up

  double positionJumpBound_;

  double distanceToLastGoodPos_;  // calculated distance to last good pos

  // update the bound with distance traveled
  void updateBound();

  // update the bound with time (when not using distance traveled)
  void updateBound(const double& updateTime);

  data::GeodeticPosition3d currentEstimatedPv_;
  bool                     currentEstPvSet_;

  std::function<void(const double&                  timestamp,
                     const PosJumpCheckDiagnostics& diagnostics)>
    publishDiagnostics_;
};

}  // namespace pnt_integrity
#endif