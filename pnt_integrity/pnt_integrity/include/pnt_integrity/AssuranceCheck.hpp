//============================================================================//
//-------------------- pnt_integrity/AssuranceCheck.hpp --------*- C++ -*-----//
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
/// \brief    Base / parent class for a PNT assurance check
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     May 28, 2019
//============================================================================//
#ifndef PNT_INTEGRITY__ASSURANCE_CHECK_HPP
#define PNT_INTEGRITY__ASSURANCE_CHECK_HPP

#include "if_data_utils/IFSampleData.hpp"
#include "logutils/logutils.hpp"
#include "pnt_integrity/IntegrityData.hpp"
#include "pnt_integrity/IntegrityDataRepository.hpp"

namespace pnt_integrity
{
/// A map for pairing an assurance level to each PRN
using MultiPrnAssuranceMap = std::map<int, data::AssuranceLevel>;

/// \brief Parent class for all integrity checks
///
/// Pure virtual parent class that holds common functionality accross all
/// assurance checks. Any child class that inherits from this must lock
/// assuranceCheckMutex_ before access any protected data in this class.
/// Any child class that inherits from this class can also use 
/// assuranceCheckMutex_ to protect private data in the child class.
class AssuranceCheck
{
public:
  /// \brief Constructor
  ///
  /// Constructor for the parent class. Multiple PRN support is disabled by
  /// default
  ///
  /// \param multiPrnSupport Constructor argument to enable / disable
  ///                        multiple PRN support
  /// \param checkName A string name identifier for the check
  /// \param log The provided log callback function
  AssuranceCheck(const bool&                  multiPrnSupport = false,
                 const std::string&           checkName = "AssuranceCheck",
                 const logutils::LogCallback& log = logutils::printLogToStdOut)
    : logMsg_(log)
    , prnAssuranceLevels_()
    , assuranceInconsistentThresh_(1.0)
    , assuranceUnassuredThresh_(2.0)
    , checkName_(checkName)
    , assuranceLevelPeriod_(2.0)
    , lastAssuranceUpdate_(0.0)
    , lastKnownGoodPosition_()
    , lastKnownGoodPositionTime_(0)
    , lastKnownGoodSet_(false)
    , allowPositiveWeighting_(true)
    , multiPrnSupport_(multiPrnSupport)
    , assuranceState_()
    , weight_(1.0)
  {
  };

  virtual ~AssuranceCheck() = default;

  /// \brief Handler function for GNSS Observables
  ///
  /// Function to handle provided GNSS Observables. (virtual)
  ///
  /// \returns True if successful
  virtual bool handleGnssObservables(const data::GNSSObservables& /*gnssObs*/)
  {
    return false;
  };

  /// \brief Return function for the multi-prn assurance data
  ///
  /// Returns the multiple-prn assurance levels for the check. An assertion
  /// is implemented to gaurantee that the function only returns the map when
  /// multi-prn support is enabled for the check.
  ///
  /// \returns The prn-to-assurance level map
  MultiPrnAssuranceMap getMultiPrnAssuranceData()
  {
    //    assert(multiPrnSupport_);
    return prnAssuranceLevels_;
  };

  /// \brief Handler function for Position / Velocity message
  ///
  /// Function to handle provided posivion / velocity messages (virtual)
  ///
  /// \returns True if successful
  virtual bool handlePositionVelocity(const data::PositionVelocity& /*posVel*/,
                                      const bool& /*local*/)
  {
    return false;
  };

  /// \brief Handler function for an estimated Position / Velocity message
  ///
  /// Function to handle provided posivion / velocity messages (virtual)
  ///
  /// \returns True if successful
  virtual bool handleEstimatedPositionVelocity(const data::PositionVelocity&)
  {
    return false;
  };

  /// \brief Handler function for AccumulatedDistranceTraveled messages
  ///
  /// \returns True if successful
  virtual bool handleDistanceTraveled(
    const data::AccumulatedDistranceTraveled& /*dist*/)
  {
    return false;
  }

  /// \brief Handler function for measured range
  ///
  /// Function to handle provided range measurements (virtual)
  ///
  /// \returns True if successful
  virtual bool handleMeasuredRange(const data::MeasuredRange& /*range*/)
  {
    return false;
  };

  /// \brief Handler function for IF sample data (SC8)
  ///
  /// Function to handle provided IF data (virtual)
  ///
  /// \returns True if successful
  virtual bool handleIFSampleData(
    const double& /*time*/,
    const if_data_utils::IFSampleData<if_data_utils::IFSampleSC8>& /*ifData*/)
  {
    return false;
  };

  /// \brief Handler function for IF sample data (SC16)
  ///
  /// Function to handle provided IF data (virtual)
  ///
  /// \returns True if successful
  virtual bool handleIFSampleData(
    const double& /*time*/,
    const if_data_utils::IFSampleData<if_data_utils::IFSampleSC16>& /*ifData*/)
  {
    return false;
  };

  /// \brief Handler function for Clock Offset sample data
  ///
  /// Function to handle provided Clock Offset data (virtual)
  ///
  /// \returns True if successful
  virtual bool handleClockOffset(const data::ClockOffset& /*clockOffset*/)
  {
    return false;
  };

  /// \brief Handler function for AGC value
  ///
  /// Function to handle provided AGC values (virtual)
  ///
  /// \returns True if successful
  virtual bool handleAGC(const data::AgcValue& /*value*/) { return false; };

  /// \brief Returns the AssuranceLevel enumeration value associated with the
  /// check's AssuranceState
  data::AssuranceLevel getAssuranceLevel()
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    return assuranceState_.getAssuranceLevel();
  };

  /// \brief Returns the interger value associated with the check's
  /// AssuranceState
  double getAssuranceValue() 
  { 
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    return assuranceState_.getAssuranceValue();;
  };

  /// \brief Returns the AssuranceState of the check
  data::AssuranceState getAssuranceState() 
  { 
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    return assuranceState_; 
  };

  /// \brief Function to calculate the assurance level of the check
  ///
  /// Child classes should define this function to calculate the
  /// assurance level of the check by using whatever data / calculation 
  /// necessary
  virtual void calculateAssuranceLevel(const double& time) = 0;

  /// \brief Sets the assurance level transition thresholds
  ///
  /// Sets arbitrary thresholds that can be used in child classes to
  /// indicate or trigger a transition into different assurance levels
  /// associated with the check
  ///
  /// \param unknownThresh Use this threshold to trigger or indicate a
  ///                      transition into AssuranceLevel::Inconsistent
  /// \param unusableThresh Use this value to trigger or indicate a
  ///                      transition into AssuranceLevel::Unassured
  void setAssuranceThresholds(const double& unknownThresh,
                              const double& unusableThresh)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

    std::stringstream threshMsg;
    threshMsg << checkName_ << ": changing thresholds to: " << std::endl;
    threshMsg << "unknown thresh: " << unknownThresh << std::endl;
    threshMsg << "unusable thresh: " << unusableThresh;
    logMsg_(threshMsg.str(), logutils::LogLevel::Info);

    assuranceInconsistentThresh_ = unknownThresh;
    assuranceUnassuredThresh_    = unusableThresh;
  }

  /// \brief Triggers a manual check calculation
  ///
  /// Use this function to run a manual check calculation that is not
  /// triggered off the receipt of a message (pure virtual)
  ///
  /// \returns True if successful
  virtual bool runCheck() = 0;

  /// \brief Sets the log message handler to provided callback
  ///
  /// \param logMsgHandler The provided call back function
  void setLogMessageHandler(const logutils::LogCallback& logMsgHandler)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    logMsg_ = logMsgHandler;
  };

  /// \brief Enables support for multiple prn checks
  void enableMultiPrnSupport() 
  { 
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    multiPrnSupport_ = true; 
  };

  /// \brief Returns the name of the check
  std::string getName() 
  { 
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    return checkName_; 
  };

  /// \brief Changes the check's assurance level to the provided value
  ///
  /// This function will change the assurance level of the check. Usually called
  /// by internal functions after a calculation based on provided assurance
  /// data. The function will raise the assurance level immediately if the
  /// provided level is higher than the current level. If the level is lower,
  /// the function will not change the level unless a certain period of time has
  /// passed since the last assurnace level upgrade (assuranceLevelPeriod_)
  ///
  /// \param updateTime The timestampe associated with the requested level
  /// change \param newLevel The newly provided / requested assurance level
  void changeAssuranceLevel(const double&               updateTime,
                            const data::AssuranceLevel& newLevel);

  /// \brief Sets the assurance level period
  ///
  /// The assurance level period is the amount of time required to hold an
  /// elevated assurance level.
  ///
  /// \param levelPeriod The period (in seconds) the the check is required to
  /// hold an elevated assurance level before lowering
  ///
  void setAssuranceLevelPeriod(const double& levelPeriod)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    std::stringstream levelMsg;
    levelMsg << checkName_
             << ": changing assurance level period to: " << levelPeriod;
    logMsg_(levelMsg.str(), logutils::LogLevel::Info);

    assuranceLevelPeriod_ = levelPeriod;
  }

  /// \brief Sets the last known good position
  ///
  /// Provides if the assurance check with knowledge of a last known
  /// good position for use in calculations (if needed by the specific
  /// implementation)
  ///
  /// \param updateTime The timestamp associated with the provided position
  /// \param position The last known good position
  virtual void setLastGoodPosition(const double&                   updateTime,
                                   const data::GeodeticPosition3d& position)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

    lastKnownGoodPositionTime_ = updateTime;
    lastKnownGoodPosition_     = position;
    lastKnownGoodSet_          = true;
  }

  /// \brief Provides the check with an updated position and assuarance level
  ///
  /// This method provides the check function with an updted position and
  /// associated assurance level for use in the check's calculation. The default
  /// behavior is null, but can be overridden in child classes.
  ///
  virtual void setPositionAssurance(
    const double& /*timestamp*/,
    const data::GeodeticPosition3d& /*position*/,
    const data::AssuranceLevel& /*level*/){};

  /// \brief Sets the weight of the check
  ///
  /// The weight of the check is used when combining the assurance level of this
  /// check with other checks for a cumulative assurance level
  ///
  /// \param weightVal The weight for this check
  void setWeight(const double& weightVal) 
  { 
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    weight_ = weightVal; 
  }

  /// \brief Returns the weight for the check
  ///
  /// \returns The weight for the check
  double getWeight() 
  { 
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    return weight_; 
  };

  /// \brief Sets the positive check weighting allowed boolean
  ///
  /// Positive weighting allows a check to increase the overall assurance level
  ///
  /// \param allowVal The flag indicating whether to allow positive weights
  void setAllowPositiveWeighting(const bool& allowVal)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    allowPositiveWeighting_ = allowVal;
  }

  /// \brief Returns whether or not the check's level should be weighted
  ///
  /// If the assurance level is Assured and postive weighting is not allowed,
  /// then this function will return false. It will also return false if
  /// the level is Unavailable
  ///
  /// \returns The flag to indicate if weighting should be used
  bool isCheckUsed()
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    if ((getAssuranceLevel() == data::AssuranceLevel::Assured) &&
        (!allowPositiveWeighting_))
    {
      return false;
    }
    else if (getAssuranceLevel() == data::AssuranceLevel::Unavailable)
    {
      return false;
    }
    else
    {
      return true;
    }
  }

protected:

  // Mutex to protect all class memmber data - any child class that accesses
  // protected data directly, must first lock this mutex
  std::recursive_mutex assuranceCheckMutex_; 

  // Local log message function
  logutils::LogCallback logMsg_;

  /// The assurance level for each PRN (if applicable to the defined check).
  /// Should only be populated if enableMultiPrnSupport() has been called
  MultiPrnAssuranceMap prnAssuranceLevels_;

  /// The arbritrary threshold for elevating the check's overall assurance
  /// level to AssuranceLevel::Inconsistent. It is up to the AssuranceCheck
  /// implementation on how this threshold is used internally.
  double assuranceInconsistentThresh_;

  /// The arbitrary threshold for elevating the check's overall assurance
  /// level to AssuranceLevel::Unassured. It is up to the AssuranceCheck
  /// implementation on how this threshold is used internally.
  double assuranceUnassuredThresh_;

  /// The name of the check
  std::string checkName_;

  /// The hold time for an elevated assurance level
  double assuranceLevelPeriod_;

  /// The last time the assurance level was updated
  double lastAssuranceUpdate_;

  /// The last known good position set by external application
  data::GeodeticPosition3d lastKnownGoodPosition_;

  /// The time associated with the last known good position
  double lastKnownGoodPositionTime_;

  /// Flag to indicate that last known good has been set
  double lastKnownGoodSet_;

  /// flag to indicate if the check can be used in a positive weighting (i.e. do
  /// you weight the check when its level is assured)
  bool allowPositiveWeighting_;

  /// \brief Computes the distance between two geodetic coordinates
  ///
  /// \param pos1 The first position
  /// \param pos2 The second position
  /// \returns The calculated distance
  static double calculateDistance(const data::GeodeticPosition3d& pos1,
                           const data::GeodeticPosition3d& pos2);

  /// \brief Checks if the distance between two points is greater than the
  /// provided threshold
  /// \param pos1 The first position
  /// \param pos2 The second position
  /// \param distanceThresh The provided threshold do compare against
  /// \param distance The calculated distance
  /// \returns True if distance is greater than provided threshold
  static bool checkDistance(const data::GeodeticPosition3d& pos1,
                     const data::GeodeticPosition3d& pos2,
                     const double&                   distanceThresh,
                     double&                         distance)
  {
    distance = calculateDistance(pos1, pos2);
    return checkDistance(distance, distanceThresh);
  }

  /// \brief Compares the provided distance value with the threshold
  /// \param dist The provided distance
  /// \param thresh The threshold to compare against
  /// \returns True if distance is greater than provided threshold
  static bool checkDistance(const double& dist, const double& thresh)
  {
    return dist > thresh;
  }

private:
  bool multiPrnSupport_;

  // The overall assurance level calculated for the check
  data::AssuranceState assuranceState_;

  // The weight of this check that will be used when combining with other
  // checks
  double weight_;
};

}  // namespace pnt_integrity
#endif
