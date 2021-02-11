//============================================================================//
//---------------- pnt_integrity/AngleOfArrivalCheck.hpp -------*- C++ -*-----//
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
/// \brief    AssurancCheck class defined for the angle of arrival check
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     June 3, 2019
//============================================================================//
#ifndef PNT_INTEGRITY__ANGLE_OF_ARRIVAL_CHECK_HPP
#define PNT_INTEGRITY__ANGLE_OF_ARRIVAL_CHECK_HPP

#include <chrono>

#include "pnt_integrity/AssuranceCheck.hpp"
#include "pnt_integrity/IntegrityData.hpp"

namespace pnt_integrity
{
/// String ID for the AOA check difference diagnostic data
const std::string INTEGRITY_AOA_DIFF_DIAGNOSTICS =
  "INTEGRITY_AOA_DIFF_DIAGNOSTICS";
/// String ID for the AOA check diagnostic node id
const std::string INTEGRITY_AOA_DIFF_NODE_ID = "INTEGRITY_AOA_DIFF_NODE_ID";
/// String ID for the AOA check diagnostic data
const std::string INTEGRITY_AOA_DIAGNOSTICS = "INTEGRITY_AOA_DIAGNOSTICS";
/// String ID for the AOA check diagnostic difference threshold
const std::string INTEGRITY_AOA_DIAG_DIFF_THRESH =
  "INTEGRITY_AOA_DIAG_DIFF_THRESH";
/// String ID for the AOA check diagnostic suspect prn count
const std::string INTEGRITY_AOA_DIAG_SUSPECT_PRN_COUNT =
  "INTEGRITY_AOA_DIAG_SUSPECT_PRN_COUNT";
/// String ID for the AOA check survey inconsistent thresh
const std::string INTEGRITY_AOA_DIAG_ITHRESH = "INTEGRITY_AOA_DIAG_ITHRESH";
/// String ID for the AOA check survey unassured thresh
const std::string INTEGRITY_AOA_DIAG_UTHRESH = "INTEGRITY_AOA_DIAG_UTHRESH";

/// \brief Structure used to publish diagnostic data
struct AoaCheckDiagnostics
{
  /// The threshold that is used when comparing single differences
  double singleDiffThresh;
  /// The number of PRNS that appear suspect (UNASSURED or INCONSISTENT)
  int suspectPrnCount;
  /// The threshold used to check against the number of suspect PRNS
  double inconsistentThresh;
  /// The threshold used to check against the number of suspect PRNS
  double unassuredThresh;
};

/// Enumeration to indicate what data field to use for the AOA check
enum class AoaCheckData
{
  UsePseudorange = 0,
  UseCarrierPhase,
  UseBoth
};

/// Defines a type that maps PRN to a calculated difference
using SingleDiffMap = std::map<int, double>;

/// Defines a map that holds an assurance level for each prn for each node
using PrnAssuranceEachNode = std::map<int, std::vector<data::AssuranceLevel> >;

/// \brief Class implementation for the angle of arrival check
///
/// Class implementation of the angle of arrival check. The class is a child
/// class of AssuranceCheck
class AngleOfArrivalCheck : public AssuranceCheck
{
public:
  /// \brief Constructor
  ///
  /// Constructor for the angle of arrival check class. The constructor
  /// defaults the multi-prn support to true for this check, so
  /// enableMultiPrnSupport need not be called.
  ///
  /// \param name The name associated with the check
  /// \param aoaCheckData Sets which type of data will be used
  /// \param singleDiffCompareThresh Sets the threshold for comparing single
  ///                                difference values
  /// \param prnCountThresh A threshold used in the AOA check to determine
  ///                       if a PRN has a common AOA with other PRNs.
  /// \param rangeThreshold A distance threshold that is used when to
  ///                       determine if a remote node is to close to the
  ///                       local node to perform the AOA check
  /// \param log A provided log callback function to use
  AngleOfArrivalCheck(
    const std::string&           name         = "AOA check",
    const AoaCheckData&          aoaCheckData = AoaCheckData::UsePseudorange,
    const double&                singleDiffCompareThresh = 5.0,
    const int&                   prnCountThresh          = 5,
    const double&                rangeThreshold          = 5.0,
    const logutils::LogCallback& log = logutils::printLogToStdOut)
    : AssuranceCheck::AssuranceCheck(true, name, log)
    , aoaCheckData_(aoaCheckData)
    , singleDiffCompareThresh_(singleDiffCompareThresh)
    , prnCountThresh_(prnCountThresh)
    , rangeThreshold_(rangeThreshold)
    , lastDiagPublishTime_(0.0)
    , lastDiffPublishTime_(0.0)
  {
    std::stringstream initMsg;
    initMsg << "Initializing AOA Check (" << name
            << ") with parameters: " << std::endl;
    initMsg << "data mode (0-psr, 1-cp, 2-both): " << (int)aoaCheckData
            << std::endl;
    initMsg << "single diff thresh (m / deg): " << singleDiffCompareThresh
            << std::endl;
    initMsg << "prn count thresh:" << prnCountThresh << std::endl;
    initMsg << "range threshold: " << rangeThreshold;
    logMsg_(initMsg.str(), logutils::LogLevel::Info);
  };

  /// \brief Handler function for GNSS Observables
  ///
  /// Function to handle provided GNSS Observables. This function simply calls
  /// runCheck(), as the provided data has already been added to the repository
  ///
  /// \param gnssObs The provided GNSS observable data
  /// \returns True if successful
  bool handleGnssObservables(const data::GNSSObservables& gnssObs);

  /// \brief Triggers a manual check calculation
  ///
  /// Use this function to run a manual check calculation that is not triggered
  /// off the receipt of a message (pure virtual)
  ///
  /// \returns True if successful
  bool runCheck();

  /// \brief Function to explicitly set the assurance level of the check
  ///
  /// For this check, this function cycles through all of the individual PRN
  /// assurance values, analyzes them, and then sets the master assurance level
  /// associted with the check.
  void calculateAssuranceLevel(const double& time);

  /// \brief Sets the difference comparison threshold
  ///
  /// This threshold is used to determine when 2 separate single differences
  /// (between a local and remote node) should be flagged has having a common
  /// angle of arrival. The units on this threshold depend on the type of data
  /// that is being used for the check (AoaCheckData). For example if,
  /// AoaCheckData::UsePseudorange is being used, then the units are in meters.
  ///
  /// \param thresh The threshold value to use
  void setDifferenceComparisonThreshold(const double& thresh)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    singleDiffCompareThresh_ = thresh;
  };

  /// \brief Sets the prn count threshold
  ///
  /// This threshold is used to determine when to raise the assurance level
  /// of a particular prn. If a PRN is found to have a common AOA with at least
  /// [threshold] other PRNS, then the AssuranceLevel is raised
  ///
  /// \param thresh The threshold value to use
  void setPrnCountThreshold(const int& thresh) 
  { 
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    prnCountThresh_ = thresh; 
  };

  /// \brief Sets the range threshold
  ///
  /// When calculating differences between local and remote observables, if
  /// a measured range is available between the two, it is compared to this
  /// threshold. If the measured range is less than the threshold, then the
  /// difference is not calculated
  ///
  /// \param thresh The threshold to use
  void setRangeThreshold(const double& thresh) 
  { 
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    rangeThreshold_ = thresh; 
  };

  /// \brief Connects the internal publishing function to external interface
  ///
  /// This function connects the internal "publishSingleDiffData" function
  /// to an external, custom function of choice
  ///
  /// \param handler Provided handler function
  void setPublishDiffData(std::function<void(const double&      time,
                                             const std::string& remoteNodeId,
                                             const SingleDiffMap&)> handler)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    publishSingleDiffData_ = handler;
  };

  /// \brief Connects the internal publishing function to external interface
  ///
  /// This function connects the internal "publishDiagnostics" function
  /// to an external, custom function of choice
  ///
  /// \param handler Provided handler function
  void setPublishDiagnostics(
    std::function<void(const double&              timestamp,
                       const AoaCheckDiagnostics& checkData)> handler)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    publishDiagnostics_ = handler;
  };

private:
  void checkAngleOfArrival(const double&            time,
                           const RepositoryEntry&   localEntry,
                           const RemoteRepoEntries& remoteEntries);

  void nestedForLoopComparison(const SingleDiffMap&  singDiffMap,
                               PrnAssuranceEachNode& prnAssuranceEachNode);

  void setPrnAssuranceLevels(const PrnAssuranceEachNode& prnAssuranceEachNode);

  AoaCheckData aoaCheckData_;

  double singleDiffCompareThresh_;
  size_t prnCountThresh_;

  // When calculating differences between local and remote observables, if
  // a measured range is available between the two, it is compared to this
  // threshold. If the measured range is less than the threshold, then the
  // difference is not calculated
  double rangeThreshold_;

  std::function<void(const double&      time,
                     const std::string& remoteNodeId,
                     const SingleDiffMap&)>
    publishSingleDiffData_;

  std::function<void(const double& /*timestamp*/,
                     const AoaCheckDiagnostics& /*checkData*/)>
    publishDiagnostics_;

  double lastDiagPublishTime_;
  double lastDiffPublishTime_;

};  // end AngleOfArrival class

}  // namespace pnt_integrity
#endif
