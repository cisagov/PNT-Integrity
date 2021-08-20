//============================================================================//
//-------------- pnt_integrity/NavigationDataCheck.hpp ---------*- C++ -*-----//
//============================================================================//
// BSD 3-Clause License
//
// Copyright (C) 2021 Integrated Solutions for Systems, Inc
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
/// \brief    AssuranceCheck class for checking broadcast navigation data
/// \author   David Hodo <david.hodo@is4s.com>
/// \date     May 14, 2021
//============================================================================//
#ifndef PNT_INTEGRITY__NAVIGATION_DATA_CHECK_HPP
#define PNT_INTEGRITY__NAVIGATION_DATA_CHECK_HPP

#include "pnt_integrity/AssuranceCheck.hpp"
#include "pnt_integrity/GPSAlmanac.hpp"
#include "pnt_integrity/GPSEphemeris.hpp"

#include <future>
#include <thread>
namespace pnt_integrity
{
/// String ID for the nav data check diagnostic data
const std::string INTEGRITY_NAV_DATA_DIAGNOSTICS =
  "INTEGRITY_NAV_DATA_DIAGNOSTICS";
/// String ID for the nav data check data valid flag
const std::string INTEGRITY_NAV_DATA_VALID = "INTEGRITY_NAV_DATA_VALID";
/// String ID for the nav data check data valid msg
const std::string INTEGRITY_NAV_DATA_VALID_MSG = "INTEGRITY_NAV_DATA_VALID_MSG";
/// String ID for the nav data check tow valid flag
const std::string INTEGRITY_NAV_DATA_TOW_VALID = "INTEGRITY_NAV_DATA_TOW_VALID";
/// String ID for the nav data check tow valid flag msg
const std::string INTEGRITY_NAV_DATA_TOW_VALID_MSG =
  "INTEGRITY_NAV_DATA_TOW_VALID_MSG";
/// String ID for the nav data check week number valid flag
const std::string INTEGRITY_NAV_DATA_WN_VALID = "INTEGRITY_NAV_DATA_WN_VALID";
/// String ID for the nav data check week number valid flag msg
const std::string INTEGRITY_NAV_DATA_WN_VALID_MSG =
  "INTEGRITY_NAV_DATA_WN_VALID_MSG";

/// \brief Structure for check diagnostics
struct NavDataCheckDiagnostics
{
  bool        dataValid;
  std::string dataValidMsg;
  bool        towValid;
  std::string towValidMsg;
  bool        wnValid;
  std::string wnValidMsg;
  NavDataCheckDiagnostics()
    : dataValid(true)
    , dataValidMsg("")
    , towValid(true)
    , towValidMsg("")
    , wnValid(true)
    , wnValidMsg(""){};
};

/// \brief Class implementation for the navigation data check
class NavigationDataCheck : public AssuranceCheck
{
public:
  /// \brief Constructor for the check class
  ///
  /// \param name The name of the check
  ///
  /// \param bounds The minimum amount of position jump that will trigger
  /// the check (meters).
  ///
  /// \param log A provided log callback function to use for log mesages
  NavigationDataCheck(
    const std::string&           name = "navigation_data_check",
    const logutils::LogCallback& log  = logutils::printLogToStdOut)
    : AssuranceCheck(false, name, log)
    , towOffset_(std::numeric_limits<double>::quiet_NaN())
    , lastWeekNumber_(std::numeric_limits<uint16_t>::max())
    , lastTow_(std::numeric_limits<uint32_t>::max())
    , lastTowTimeStamp_(std::numeric_limits<double>::quiet_NaN())
    , checkThread_(&NavigationDataCheck::runCheckThread, this)
  {
    futureObj_ = exitSignal_.get_future();
    checkThread_.detach();
    std::stringstream initMsg;
    initMsg << "Initializing Navigation Data Check (" << name << ")"
            << std::endl;
    logMsg_(initMsg.str(), logutils::LogLevel::Info);
  }

  ~NavigationDataCheck() 
  { 
    stopThreads();
  }

  void stopThreads()
  {
    exitSignal_.set_value();
    checkThread_.join();
  }

  /// \brief Handler function for GNSS Subframes
  ///
  /// Function to handle provided GNSS Broadcast Nav. Data.
  ///
  /// \returns True if successful
  virtual bool handleGnssSubframe(const data::GNSSSubframe& gnssSubframe);

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

  /// \brief Connects the internal publishing function to external interface
  ///
  /// This function connects the internal "publishDiagnostics" function
  /// to an external, custom function of choice
  ///
  /// \param handler Provided handler function
  void setPublishDiagnostics(
    std::function<void(const double& /*timestamp*/,
                       const NavDataCheckDiagnostics& /*data*/)> handler)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    publishDiagnostics_ = handler;
  };

private:
  std::function<void(const double&                  timestamp,
                     const NavDataCheckDiagnostics& diagnostics)>
    publishDiagnostics_;

  double towOffset_;

  uint16_t lastWeekNumber_;    // last valid week number received
  uint32_t lastTow_;           // last valid tow received
  double   lastTowTimeStamp_;  // last PC time that tow was received

  // double lastDiagPubTime_;
  std::vector<NavDataCheckDiagnostics> diagBuffer_;

  std::thread        checkThread_;
  void               runCheckThread();
  std::promise<void> exitSignal_;
  std::future<void>  futureObj_;

  std::string getSubframeErrorStr(int                 prn,
                                  uint16_t            subframeId,
                                  const GpsEphemeris& ephem);

  bool getSubframeErrorStr(int               prn,
                           uint16_t          subframeId,
                           const GpsAlmanac& almanac,
                           std::string&      erroStr);

  std::string getSubframe1ErrorStr(pnt_integrity::Subframe1Fault faults);
  std::string getSubframe2ErrorStr(pnt_integrity::Subframe2Fault faults);
  std::string getSubframe3ErrorStr(pnt_integrity::Subframe3Fault faults);
  std::string getAlmanacErrorStr(pnt_integrity::AlmanacSubframeFaults faults);
};

}  // namespace pnt_integrity
#endif