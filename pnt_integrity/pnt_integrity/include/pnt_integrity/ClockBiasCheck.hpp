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
/// \brief    AssurancCheck class defined for the clock bias check
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \author   John David Sprunger <jss0027@tigermail.auburn.edu>
/// \date     December 17, 2019
//============================================================================//
#ifndef PNT_INTEGRITY__CLOCK_BIAS_CHECK_HPP
#define PNT_INTEGRITY__CLOCK_BIAS_CHECK_HPP
#include <cstring>
#include "pnt_integrity/AssuranceCheck.hpp"

namespace pnt_integrity
{
/// String ID for the clock-bias check diagnostic data
const std::string INTEGRITY_CLOCK_BIAS_DIAGNOSTICS =
  "INTEGRITY_CLOCK_BIAS_DIAGNOSTICS";
/// String ID for the clock-bias check expected drift
const std::string INTEGRITY_CLOCK_BIAS_DIAG_EXP_DRIFT =
  "INTEGRITY_CLOCK_BIAS_DIAG_EXP_DRIFT";
/// String ID for the clock-bias check drift variance
const std::string INTEGRITY_CLOCK_BIAS_DIAG_EXP_DRIFT_VAR =
  "INTEGRITY_CLOCK_BIAS_DIAG_EXP_DRIFT_VAR";
/// String ID for the clock-bias check propagation offset
const std::string INTEGRITY_CLOCK_BIAS_DIAG_PROP_OFFSET =
  "INTEGRITY_CLOCK_BIAS_DIAG_PROP_OFFSET";
/// String ID for the clock-bias check actual offset
const std::string INTEGRITY_CLOCK_BIAS_DIAG_ACTUAL_OFFSET =
  "INTEGRITY_CLOCK_BIAS_DIAG_ACTUAL_OFFSET";
/// String ID for the clock-bias check offset error
const std::string INTEGRITY_CLOCK_BIAS_DIAG_OFFSET_ERROR =
  "INTEGRITY_CLOCK_BIAS_DIAG_OFFSET_ERROR";
/// String ID for the clock-bias check drift rate bound
const std::string INTEGRITY_CLOCK_BIAS_DIAG_DRIFT_RATE_BOUND =
  "INTEGRITY_CLOCK_BIAS_DIAG_DRIFT_RATE_BOUND";
/// String ID for the clock-bias check drift rate var bound
const std::string INTEGRITY_CLOCK_BIAS_DIAG_DRIFT_RATE_VAR_BOUND =
  "INTEGRITY_CLOCK_BIAS_DIAG_DRIFT_RATE_VAR_BOUND";

/// \brief Structure used to publish diagnostic data
struct ClockBiasCheckDiagnostics
{
  /// The excpected drift based on the recent history
  double expectedDrift;
  /// The expected variance of the drift based on recent history
  double expectedDriftVar;
  /// The clock bias propagated to the current time step
  double propagatedOffset;
  /// The actual clock bias at the current time stamp
  double actualOffset;
  /// The error threshold for comparing the actual and propagated
  double offsetError;
  /// The error bound on the drift rate
  double driftRateBound;
  /// The error baound on the drift rate variance
  double driftRateVarBound;
};
/// \brief Class implementation for the position velocity check
///
/// The Clock Bias Check calculates the expectation and variance
/// of the clock drift for the most recent set of clock samples,
/// minus the most recent sample.
///  The expectation is used to propagate the clock forward to the
/// most recent single sample's arrival time and check if it is
/// within reasonable bounds.
/// The variance is used to check for zero-bias disruption.
/// The expectation and variance are calculated like normal,
/// using the drift value in each sample.
/// The propagated sample's clock offset is calculated by
/// multiplying the drift expectation by the sample time
/// difference between the second to most recent sample and the
/// most recent sample (called dt), and then adding the second
/// to most recent sample's clock offset.
/// (i.e. velocity*dt + last position).
/// The propagated sample's clock offset is subtracted from the
/// most recent sample's clock offset (and then run through the
/// absolute value function) to obtain the offset error.
/// If the offset error is greater than the drift rate bound
/// multiplied by dt, then the clock bias check returns
/// Unassured.
/// Else if the clock drift variance is greater than the
/// predetermined drift variance bound, then it returns
/// Inconsistent.
// Else it returns Assured.
class ClockBiasCheck : public AssuranceCheck
{
public:
  /// \brief Default constructor for the check class.
  ///
  /// Constructor explicitly disables multi-prn support.
  ///
  /// \param name The name of the check object
  /// \param minNumSamples The minimum number of samples required for the check
  /// \param maxNumSamples The maximum number of samples required for the check
  /// \param minSampleTimeSec The duration of time (in seconds) over which to
  /// \param driftRateBound Maximum allowable drift rate
  /// \param driftRateVarBound Maximum allowable drift rate variance
  /// get clock data for checking integrity
  /// \param log A provided log callback function to use
  ClockBiasCheck(const std::string&           name = "Clock Bias Check",
                 const unsigned int&          minNumSamples     = 10,
                 const unsigned int&          maxNumSamples     = 30,
                 const double&                minSampleTimeSec  = 10.0,
                 const double&                driftRateBound    = 5e-7,
                 const double&                driftRateVarBound = 1e-6,
                 const logutils::LogCallback& log = logutils::printLogToStdOut)
    : AssuranceCheck::AssuranceCheck(false, name, log)
    , minNumSamples_(minNumSamples)
    , maxNumSamples_(maxNumSamples)
    , minSampleTimeSec_(minSampleTimeSec)
    , driftRateBound_(driftRateBound)
    , driftRateVarBound_(driftRateVarBound)
  {
    allowPositiveWeighting_ = false;

    std::stringstream initMsg;
    initMsg << "Initializing Clock Bias Check (" << name
            << ") with parameters: " << std::endl;
    initMsg << "Min number of samples : " << minNumSamples << std::endl;
    initMsg << "Max number of samples : " << maxNumSamples << std::endl;
    initMsg << "Min sample time (sec) : " << minSampleTimeSec << std::endl;
    initMsg << "Drift rate bound (sec / sec) : " << driftRateBound << std::endl;
    initMsg << "Drift rate variance bound : " << driftRateVarBound << std::endl;
    logMsg_(initMsg.str(), logutils::LogLevel::Info);
  };

  /// \brief Handler function for clock offset (bias and drift)
  ///
  /// Function to handle provided clock offset.
  ///
  /// \param clockOffset The provided clock offset message
  /// \returns True if successful
  bool handleClockOffset(const data::ClockOffset& clockOffset);

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
    std::function<void(const double&                    timestamp,
                       const ClockBiasCheckDiagnostics& checkData)> handler)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    publishDiagnostics_ = handler;
  };

private:
  /// \brief Checks if, in a certain period of time up until now,
  /// a stationary position drifts off of the original position
  ///
  /// \param startTime the
  /// \param timeEntryVec the list of time entries to examine
  /// \returns true if there's enough samples to make an informed
  /// statement about the position
  bool clockBiasCheck(const double& checkTime);

  /// \brief Prunes the time map of old times
  void pruneVec();

  bool enoughSampleTime();

  /// \brief Performs b - a
  data::Timestamp timeDiff(data::Timestamp a, data::Timestamp b);

  constexpr static double nSecPerSec = 1000000000.0;

  unsigned int minNumSamples_;
  unsigned int maxNumSamples_;
  double       minSampleTimeSec_;
  double       driftRateBound_;
  double       driftRateVarBound_;

  std::vector<data::ClockOffset> offsetVec_;

  std::function<void(const double& /*timestamp*/,
                     const ClockBiasCheckDiagnostics& /*checkData*/)>
    publishDiagnostics_;
};

}  // namespace pnt_integrity
#endif
