//============================================================================//
//-------------------- pnt_integrity/AquisitionCheck.hpp -------*- C++ -*-----//
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
/// \brief    Class defined for the carrier-to-noise ratio (Cno) checks
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     October 23, 2019
//============================================================================//
#ifndef PNT_INTEGRITY__CNO_CHECK_HPP
#define PNT_INTEGRITY__CNO_CHECK_HPP

#include "pnt_integrity/AssuranceCheck.hpp"

namespace pnt_integrity
{
/// String ID for the CNO check diagnostic data
const std::string INTEGRITY_CN0_DIAGNOSTICS = "INTEGRITY_CN0_DIAGNOSTICS";
/// String ID for the CNO check average count
const std::string INTEGRITY_CN0_DIAG_AVG_COUNT = "INTEGRITY_CN0_DIAG_AVG_COUNT";
/// String ID for the CNO check survey inconsistent thresh
const std::string INTEGRITY_CN0_DIAG_ITHRESH = "INTEGRITY_CN0_DIAG_ITHRESH";
/// String ID for the CNO check survey unassured thresh
const std::string INTEGRITY_CN0_DIAG_UTHRESH = "INTEGRITY_CN0_DIAG_UTHRESH";

/// Diagnostic data for the check
struct CnoCheckDiagnostics
{
  /// the number of PRNs within 1 unit of the mode
  int averageCount;
  /// The threshold for the inconsistent assurance level
  double inconsistentThresh;
  /// The threshold for the unassured assurance level
  double unassuredThresh;
};

/// \brief Class implementation of the carrier-to-noise (CnO) assurance check.
/// The check analyzes the CnO values for abnormalities.
class CnoCheck : public AssuranceCheck
{
  /// \brief Constructor for the CnoCheck object
  ///
  /// \param name The name identifier of the check
  /// \param cnoFilterWindow The time window across which CnO values are
  /// analyzed
  /// \param log A provided log callback function to use
public:
  CnoCheck(const std::string&           name            = "Cno Check",
           const size_t&                cnoFilterWindow = 10,
           const logutils::LogCallback& log = logutils::printLogToStdOut)
    : AssuranceCheck::AssuranceCheck(false, name, log)
    , cnoFilterWindow_(cnoFilterWindow)
    , lastPublishTime_(0.0)
  {
    allowPositiveWeighting_ = false;

    std::stringstream initMsg;
    initMsg << "Initializing Cn0 Check (" << name
            << ") with parameters: " << std::endl;
    initMsg << "filter window (samples): " << cnoFilterWindow;
    logMsg_(initMsg.str(), logutils::LogLevel::Info);
  };

  /// \brief Handler function for GNSS Observables
  ///
  /// Function to handle provided GNSS Observables. This function simply calls
  /// runCheck(), as the provided data has already been added to the repository
  ///
  /// \param gnssObs The provided GNSS observable data
  /// \returns True if successful
  bool handleGnssObservables(const data::GNSSObservables& gnssObs,
                             const double&                time = 0);

  /// \brief Function to explicitly set the assurance level of the check
  ///
  /// For this check, this function cycles through all of the individual PRN
  /// assurance values, analyzes them, and then sets the master assurance level
  /// associted with the check.
  void calculateAssuranceLevel(const double& /*time*/) { runCheck(); };

  /// \brief Triggers a manual check calculation
  ///
  /// Use this function to run a manual check calculation that is not triggered
  /// off the receipt of a message (pure virtual)
  ///
  /// \returns True if successful
  virtual bool runCheck();

  /// \brief Sets the time filter window for CnO analysis
  ///
  /// \param windowSize The time of the analysis window to use
  void setFilterWindow(const size_t& windowSize)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    cnoFilterWindow_ = windowSize;
  }

  /// \brief Connects the internal publishing function to external interface
  ///
  /// This function connects the internal "publishDiagnostics" function
  /// to an external, custom function of choice
  ///
  /// \param handler Provided handler function
  void setPublishDiagnostics(
    std::function<void(const double&              timestamp,
                       const CnoCheckDiagnostics& checkData)> handler)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    publishDiagnostics_ = handler;
  };

private:
  double computeCnoMode(const std::vector<double>& cnoVals);

  std::deque<size_t> cnoCheckCountHist_;
  size_t             cnoFilterWindow_;

  std::function<void(const double& /*timestamp*/,
                     const CnoCheckDiagnostics& /*checkData*/)>
    publishDiagnostics_;

  double lastPublishTime_;
};
}  // namespace pnt_integrity
#endif
