//============================================================================//
//------------------------ pnt_integrity/AgcCheck.hpp ----------*- C++ -*-----//
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
/// \brief    Class defined for the AGC check
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     February 18, 2020
//============================================================================//
#ifndef PNT_INTEGRITY__AGC_CHECK_HPP
#define PNT_INTEGRITY__AGC_CHECK_HPP

#include "pnt_integrity/AssuranceCheck.hpp"

namespace pnt_integrity
{
/// String ID for the AGC check diagnostic data
const std::string INTEGRITY_AGC_DIAGNOSTICS = "INTEGRITY_AGC_DIAGNOSTICS";
/// String ID for the AGC check survey inconsistent thresh
const std::string INTEGRITY_AGC_DIAG_ITHRESH = "INTEGRITY_AGC_DIAG_ITHRESH";

/// Diagnostic data for AGC check
struct AgcCheckDiagnostics
{
  /// The AGC values
  data::AgcValue values;
  /// The inconsistent threshold
  double inconsistentThresh;
};

/// \brief Class implementation for the AGC check
class AgcCheck : public AssuranceCheck
{
public:
  /// \brief Constructor for the AgcCheck class
  ///
  /// \param name The name of the check
  /// \param minValue The minimum possible reported value for the AGC
  /// \param maxValue The maximum possible reported value for the AGC
  /// \param log Log handler function
  AgcCheck(const std::string&           name     = "agc_check",
           const double&                minValue = 0.0,
           const double&                maxValue = 10000,
           const logutils::LogCallback& log      = logutils::printLogToStdOut)
    : AssuranceCheck::AssuranceCheck(true, name, log)
    , maxValue_(maxValue)
    , minValue_(minValue)
    , lastDiagPublishTime_(0.0)
  {
    allowPositiveWeighting_ = false;

    std::stringstream initMsg;
    initMsg << "Initializing AGC Check (" << name << ") with parameters: ";
    initMsg << std::endl;
    initMsg << "min value: " << minValue << std::endl;
    initMsg << "max value: " << maxValue;
    logMsg_(initMsg.str(), logutils::LogLevel::Info);
  };

  /// \brief Handler function for AGC value
  ///
  /// Function to handle provided AGC values (virtual)
  ///
  /// \param agcValue The provided AGC message / structure
  /// \returns True if successful
  bool handleAGC(const data::AgcValue& agcValue)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    currentAgcVals_ = agcValue;
    return runCheck();
  }

  /// \brief Function to explicitly set the assurance level of the check
  void calculateAssuranceLevel(const double& /*time*/) { runCheck(); };

  /// \brief Connects the internal publishing function to external interface
  ///
  /// This function connects the internal "publishDiagnostics" function
  /// to an external, custom function of choice
  ///
  /// \param handler Provided handler function
  void setPublishDiagnostics(
    std::function<void(const double&              timestamp,
                       const AgcCheckDiagnostics& checkData)> handler)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    publishDiagnostics = handler;
  };

private:
  double maxValue_;
  double minValue_;

  data::AgcValue currentAgcVals_;

  bool runCheck();

  std::function<void(const double& /*timestamp*/,
                     const AgcCheckDiagnostics& /*checkData*/)>
    publishDiagnostics;

  double lastDiagPublishTime_;

};  // namespace pnt_integrity
}  // namespace pnt_integrity

#endif
