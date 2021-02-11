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
/// \brief    AssurancCheck class defined for the static position check
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \author   John David Sprunger <jss0027@tigermail.auburn.edu>
/// \date     September 3, 2019
//============================================================================//
#ifndef PNT_INTEGRITY__STATIC_POSITON_CHECK_HPP
#define PNT_INTEGRITY__STATIC_POSITON_CHECK_HPP

#include <cstring>

#include "pnt_integrity/AssuranceCheck.hpp"

namespace pnt_integrity
{
/// String ID for the static position check diagnostic data
const std::string INTEGRITY_STATIC_POS_DIAGNOSTICS =
  "INTEGRITY_STATIC_POS_DIAGNOSTICS";
/// String ID for the static position check survey latitude
const std::string INTEGRITY_STAIC_POS_DIAG_POS_LAT =
  "INTEGRITY_STAIC_POS_DIAG_POS_LAT";
/// String ID for the static position check survey longitdue
const std::string INTEGRITY_STAIC_POS_DIAG_POS_LON =
  "INTEGRITY_STAIC_POS_DIAG_POS_LON";
/// String ID for the static position check survey altitude
const std::string INTEGRITY_STAIC_POS_DIAG_POS_ALT =
  "INTEGRITY_STAIC_POS_DIAG_POS_ALT";
/// String ID for the static position check change threshold
const std::string INTEGRITY_STAIC_POS_DIAG_POS_CHNG_THRESH =
  "INTEGRITY_STAIC_POS_DIAG_POS_CHNG_THRESH";
/// String ID for the static position check percentage threshold
const std::string INTEGRITY_STAIC_POS_DIAG_PERCENT_OVER =
  "INTEGRITY_STAIC_POS_DIAG_PERCENT_OVER";
/// String ID for the static position check survey inconsistent thresh
const std::string INTEGRITY_STAIC_POS_DIAG_ITHRESH =
  "INTEGRITY_STAIC_POS_DIAG_ITHRESH";
/// String ID for the static position check survey unassured thresh
const std::string INTEGRITY_STAIC_POS_DIAG_UTHRESH =
  "INTEGRITY_STAIC_POS_DIAG_UTHRESH";

/// \brief Structure used to publish diagnostic data
struct StaticPosCheckDiagnostics
{
  /// The static position used in the check (surveyed or provided)
  data::GeodeticPosition3d staticPosition;
  /// Threshold to check current position against static position
  double posChangeThresh;
  /// The percent of positions in the window that are over the threshold
  double percentOverThresh;
  /// The threshold used for the INCONSISTENT assurance level
  double inconsistentThresh;
  /// The threshold used for the UNASSURED assurance level
  double unassuredThresh;
};

/// \brief Class implementation for the static-position check
class StaticPositionCheck : public AssuranceCheck
{
public:
  /// \brief Default constructor for the check class.
  ///
  /// Constructor explicitly disables multi-prn support.
  ///
  /// \param name The name of the check object
  ///
  /// \param numPositionsForInit The number of static positiosn required for
  /// the initialization survey
  ///
  /// \param checkWindowSize The minimum number of samples (after throwing
  /// out invalid positions) necessary to make an informed statement
  /// about integrity, includes start positions if in averaging mode
  ///
  /// \param posChangeThresh The threshold radius (in meters) for
  /// noisy position changes
  ///
  /// \param log A provided log callback function to use
  ///
  StaticPositionCheck(
    const std::string&           name                = "static_position_check",
    const size_t&                numPositionsForInit = 60,
    const unsigned int&          checkWindowSize     = 10,
    const double&                posChangeThresh     = 5.0,
    const logutils::LogCallback& log = logutils::printLogToStdOut)
    : AssuranceCheck::AssuranceCheck(false, name, log)
    , numPositionsForInit_(numPositionsForInit)
    , checkWindowSize_(checkWindowSize)
    , posChangeThresh_(posChangeThresh)
    , staticPositionInitialized_(false)
    , lastSurveyPointTime_(0.0)
  {
    std::stringstream initMsg;
    initMsg << "Initializing Static-Position Check (" << name
            << ") with parameters: " << std::endl;
    initMsg << "num positions for init : " << numPositionsForInit << std::endl;
    initMsg << "check window size (smaples): " << checkWindowSize << std::endl;
    initMsg << "position change threshold (m): " << posChangeThresh;
    logMsg_(initMsg.str(), logutils::LogLevel::Info);
  };

  /// \brief Handler function for Position / Velocity message
  ///
  /// Function to handle provided posivion / velocity messages
  ///
  /// \param posVel The provided position velocity message / structure
  /// \param local Indicates if this is a local or remote message
  /// \returns True if successful
  bool handlePositionVelocity(const data::PositionVelocity& posVel,
                              const bool&                   local);

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

  /// \brief Sets the expected static position that will be used for the check
  ///
  /// \param staticPos The provided position value
  void setStaticPosition(const data::GeodeticPosition3d& staticPos);

  /// \brief Connects the internal publishing function to external interface
  ///
  /// This function connects the internal "publishDiagnostics" function
  /// to an external, custom function of choice
  ///
  /// \param handler Provided handler function
  void setPublishDiagnostics(
    std::function<void(const double&                    timestamp,
                       const StaticPosCheckDiagnostics& checkData)> handler)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    publishDiagnostics_ = handler;
  };

private:
  /// \brief Compares current reported position to known static location
  ///
  /// Checks if, in a certain period of time up until now,
  /// a stationary position drifts off of the original position
  void posHistoryCheck(TimeEntry& timeEntryVec);

  /// \brief Adds position b to position a, i.e. a+b
  ///
  /// \param a The position to add to
  /// \param b The position to add
  /// \returns The position created from a+b
  data::GeodeticPosition3d addTwoPositions(data::GeodeticPosition3d a,
                                           data::GeodeticPosition3d b);

  /// \brief Calculates if a given GeodeticPosition3d is within a
  /// given threshold
  ///
  /// The threshold is analagous to the radius of a circle,
  /// not its diameter.
  ///
  /// \param p The position to examine
  /// \param mean The given mean
  /// \param thresh The given threshold, when all dimensions are the same
  /// \param size The size of the vectors
  /// \returns true if p is within a thresh distance of mean
  bool isWithinThreshold(data::GeodeticPosition3d p,
                         data::GeodeticPosition3d mean,
                         double                   thresh);

  /// \brief Returns the percentage of positions that are within
  /// a threshold distance of the center
  ///
  /// \param center The center point to check proximity against
  /// \param positions The positions to check for proximity
  /// \param thresh The maximum euclidean distance before returning false
  double percentNearPos(const data::GeodeticPosition3d&              center,
                        const std::vector<data::GeodeticPosition3d>& positions,
                        double                                       thresh);

  data::GeodeticPosition3d computeSurveyedPosition(
    const std::vector<data::GeodeticPosition3d>& positions);

  // THe number of saved points required for survey initialization
  size_t numPositionsForInit_;

  // The minimum number of samples required for a position check
  unsigned int checkWindowSize_;

  // The threshold used to compare incoming position data with the static
  // position
  double posChangeThresh_;

  // vector to hold positions for surveying
  std::vector<data::GeodeticPosition3d> positionsToSurvey_;

  /// The static position used by the check (calculated through survey or
  /// provided)
  data::GeodeticPosition3d staticPosition_;

  // Flag to indicate whether or not position has been initialized
  bool staticPositionInitialized_;

  // The last time a valid position was pulled from the repo
  double lastSurveyPointTime_;

  std::deque<data::GeodeticPosition3d> positionsToCheck_;

  // std::chrono::system_clock::time_point lastAssuranceUpdate_;

  std::function<void(const double&                    timestamp,
                     const StaticPosCheckDiagnostics& checkData)>
    publishDiagnostics_;
};

}  // namespace pnt_integrity
#endif
