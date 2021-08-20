//============================================================================//
//--------------------- pnt_integrity/GPSEphemeris.hpp ---------*- C++ -*-----//
//============================================================================//
// BSD 3-Clause License
//
// Copyright (C) 2013-2021 Integrated Solutions for Systems, Inc
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
/// \brief    Stores a set of GPS ephemeris data and computes satellite pos.
/// \author   William Travis <william.travis@is4s.com>
/// \author   David Hodo <david.hodo@is4s.com>
/// \date     August 2013
//============================================================================//

#ifndef PNT_INTEGRITY__GPS_EPHEMERIS_HPP
#define PNT_INTEGRITY__GPS_EPHEMERIS_HPP

#include <stdint.h>
#include <limits>  // std::numeric_limits
#include <limits>
#include <string>
#include <utility>  // std::pair

#include "pnt_integrity/GPSNavDataCommon.hpp"

namespace pnt_integrity
{
/// Anti-spoof flag given in bit 19 of the handover word (HOW)
/// If anti-spoof is on, P(Y) code is transmitted
/// If anti-spoof is off, P code is transmitted
/// Ref: IS-GPS-200 - 20.3.3.2
enum class AntiSpoofFlag
{
  Off = 0,
  On  = 1
};

/// Code on L2.  Indicates if C/A code, P code, or both are on on L2
/// Given in bits 11 and 12 of subframe 1
/// Ref: IS-GPS-200 - 20.3.3.3.1.2
enum class L2CodeType
{
  Reserved     = 0,
  PCodeOn      = 1,
  CACodeOn     = 2,
  CAAndPCodeOn = 3
};

/// Fit interval for the ephemeris data provided in subframe 2.
/// Indicates if the satellite is under normal operations (fit interval = 4)
/// or extended operations with a fit interval of greater than 4 hours.
/// Ref: IS-GPS-200 - 20.3.3.4.3.1
enum class FitInterval
{
  FourHrs            = 0,
  GreaterThanFourHrs = 1
};

/// Enumeration to define the alert flag given in bit 18 of the HOW
/// If the alert flag is raised, it indicates to the user that the URA may
/// be worse than indicated in subframe 1 and that the SV should be used
/// only at the user's risk
/// Rcef: IS-GPS-200 - 20.3.3.2
enum class AlertFlag
{
  ALERT_OFF    = 0,
  ALERT_RAISED = 1
};

/// Enumeration to define the data flag for L2 P-code.  Indicates if the
/// NAV data stream has been turned on the P-code channel on L2
/// Ref: IS-GPS-200, 20.3.3.3.1.6
enum class L2NavDataFlag
{
  On  = 0,
  Off = 1
};

/// Structure to hold the SV health status from subframe 1, word 3, bits 17-22
struct SVHealth
{
  /// Summary of the navigation data health field given in the ephemeris data
  bool someOrAllNavDataBad;
  /// 5-bit satellite signal health given in the ephemeris data
  SVSignalHealth signalHealth;
};

/// Structure to hold the ephemeris parameters as provided in subframes 1 - 3
/// of IS-GPS-200
struct EphemerisParameters
{
  uint16_t      prn;
  AlertFlag     alertFlag;
  AntiSpoofFlag asFlag;
  uint32_t      towSf1;
  uint32_t      towM1;
  uint16_t      weekNumber;
  L2CodeType    codeOnL2;
  uint16_t      uraIndex;
  SVHealth      svHealth;
  uint16_t      iodc;
  L2NavDataFlag l2PDataFlag;
  double        groupDelay;
  double        clockCorrectionTime;
  double        clockAging3;
  double        clockAging2;
  double        clockAging1;
  double        inPhaseInterSignalCorrection;
  double        quadratureInterSignalCorrection;
  uint32_t      towSf2;
  uint32_t      towM2;
  uint16_t      iodeSf2;
  double        sinOrbitRadius;
  double        meanMotionDifference;
  double        meanMotionDifferenceRate;
  double        meanAnomaly;
  double        cosLatitude;
  double        eccentricity;
  double        sinLatitude;
  double        sqrtSemiMajorAxis;
  double        semiMajorAxisDifference;
  double        semiMajorAxisRate;
  double        timeOfEphemeris;
  FitInterval   fitInterval;
  uint16_t      ageOfDataOffset;
  uint32_t      towSf3;
  uint32_t      towM3;
  double        cosInclination;
  double        rightAscension;
  double        ascensionRateDifference;
  double        sinInclination;
  double        inclinationAngle;
  double        cosOrbitRadius;
  double        argumentOfPerigee;
  double        ascensionRate;
  uint16_t      iodeSf3;
  double        inclinationRate;
};

struct Subframe1Fault
{
  bool towSf1;
  bool weekNumber;
  bool codeOnL2;
  bool uraIndex;
  bool svHealth;
  bool iodc;
  bool l2PDataFlag;
  bool groupDelay;
  bool clockCorrectionTime;
  bool clockAging3;
  bool clockAging2;
  bool clockAging1;
};

struct Subframe2Fault
{
  bool towSf2;
  bool iodeSf2;
  bool sinOrbitRadius;
  bool meanMotionDifference;
  bool meanAnomaly;
  bool cosLatitude;
  bool eccentricity;
  bool sinLatitude;
  bool sqrtSemiMajorAxis;
  bool timeOfEphemeris;
  bool fitInterval;
  bool ageOfDataOffset;
};

struct Subframe3Fault
{
  bool towSf3;
  bool cosInclination;
  bool rightAscension;
  bool sinInclination;
  bool inclinationAngle;
  bool cosOrbitRadius;
  bool argumentOfPerigee;
  bool ascensionRate;
  bool iodeSf3;
  bool inclinationRate;
};

class GpsEphemeris
{
public:
  GpsEphemeris();

  // GpsEphemeris: constructor
  //   prn: (-)
  //   alertFlag,
  //   asFlag,
  //   towSf1,
  //   weekNumber,
  //   codeOnL2,
  //   uraIndex,
  //   svHealth,
  //   iodc,
  //   l2PDataFlag,
  //   groupDelay: (s)
  //   clockCorrectionTime: (s into GPS week)
  //   clockAging3: (s/s^2)
  //   clockAging2: (s/s)
  //   clockAging1: (s)
  //   cosOrbitRadius: (m)
  //   sinOrbitRadius: (m)
  //   cosLatitude: (rad)
  //   sinLatitude: (rad)
  //   cosInclination: (rad)
  //   sinInclination: (rad)
  //   meanMotionDifference: (rad/s)
  //   meanAnomaly: (rad)
  //   eccentricity: (-)
  //   sqrtSemiMajorAxis: (m^(1/2))
  //   rightAscension: (rad)
  //   inclinationAngle: (rad)
  //   argumentOfPerigee: (rad)
  //   ascensionRate: (rad/s)
  //   inclinationRate: (rad/s)
  //   timeOfEphemeris: (s into GPS week)
  GpsEphemeris(const uint16_t&      prn,
               const AlertFlag&     alertFlag,
               const AntiSpoofFlag& asFlag,
               const uint32_t&      towSf1,
               const uint16_t&      weekNumber,
               const L2CodeType&    codeOnL2,
               const uint16_t&      uraIndex,
               const SVHealth&      svHealth,
               const uint16_t&      iodc,
               const L2NavDataFlag& l2PDataFlag,
               const double&        groupDelay,
               const double&        clockCorrectionTime,
               const double&        clockAging3,
               const double&        clockAging2,
               const double&        clockAging1,
               const uint32_t&      towSf2,
               const uint16_t&      iodeSf2,
               const double&        sinOrbitRadius,
               const double&        meanMotionDifference,
               const double&        meanAnomaly,
               const double&        cosLatitude,
               const double&        eccentricity,
               const double&        sinLatitude,
               const double&        sqrtSemiMajorAxis,
               const double&        timeOfEphemeris,
               const FitInterval&   fitInterval,
               const uint16_t&      ageOfDataOffset,
               const uint32_t&      towSf3,
               const double&        cosInclination,
               const double&        rightAscension,
               const double&        sinInclination,
               const double&        inclinationAngle,
               const double&        cosOrbitRadius,
               const double&        argumentOfPerigee,
               const double&        ascensionRate,
               const uint16_t&      iodeSf3,
               const double&        inclinationRate,
               bool                 checkForValidity = true);

  GpsEphemeris(uint16_t prn,
               const uint32_t (&subframe1)[10],
               const uint32_t (&subframe2)[10],
               const uint32_t (&subframe3)[10],
               bool checkForValidity = true);

  GpsEphemeris(uint16_t prn,
               const uint8_t (&subframe1)[30],
               const uint8_t (&subframe2)[30],
               const uint8_t (&subframe3)[30],
               bool checkForValidity = true);

  ~GpsEphemeris(){};

  /// \brief Compute satellite ECEF position and velocity
  ///
  /// \param receiveTime measurement time associated with the pseudorange
  ///                    measurement (s into GPS week).
  /// \param positionEcefX ECEF X position (m)
  /// \param positionEcefY ECEF Y position (m)
  /// \param positionEcefY ECEF Z position (m)
  /// \param velocityEcefX ECEF X speed (m/s)
  /// \param velocityEcefY ECEF Y speed (m/s)
  /// \param velocityEcefZ ECEF Z speed (m/s)
  /// \param svClockCorrection: satellite clock correction including polynomial
  ///                           fit, group delay, and relativistic effect (s)
  /// \param pseudorange (optional) user to satellite range (m). Inputting
  ///                 a pseudorange improves SV position and velocity accuracy.
  ///
  bool getSvState(const double& receiveTime,
                  double&       positionEcefX,
                  double&       positionEcefY,
                  double&       positionEcefZ,
                  double&       velocityEcefX,
                  double&       velocityEcefY,
                  double&       velocityEcefZ,
                  double&       svClockCorrection,
                  const double& pseudorange = 0.0) const;

  /// \brief Set the parsed ephemeris values for the ephemeris object
  ///
  /// \param prn PRN number [1-32]
  /// \param groupDelay T_GD - Estimated group delay differential [sec]
  /// \param clockCorrectionTime t_0c -
  /// \param clockAging3 a_f2 - Sv clock drift rate [sec/sec^2]
  /// \param clockAging2 a_f1 - Sv clock drift [sec/sec]
  /// \param clockAging1 a_f0 - Sv clock bias [sec]
  /// \param cosOrbitRadius C_rc - Amplitude of the cosine harmonic correction
  ///                         term to the orbit radius [m]
  /// \param sinOrbitRadius C_rs - Amplitude of the sine harmonic correction
  ///                         term to the orbit radius [m]
  /// \param cosLatitude C_uc - Amplitude of the cosine harmonic correction
  ///                      term to the argument of latitude [rad]
  /// \param sinLatitude C_us - Amplitude of the sin harmonic correction term
  ///                      to the argument of latitude [rad]
  /// \param cosInclination C_ic - Amplitude of the cosine harmonic correction
  ///                         term to the angle of inclination [rad]
  /// \param sinInclination C_is - Amplitude of the sine harmonic correction
  ///                         term to the angle of inclination [rad]
  /// \param meanMotionDifference deltaN - Mean motion difference from the
  ///                               computed value [rad/sec]
  /// \param meanAnomaly M_0 - Mean anomaly at reference time [rad]
  /// \param eccentricity e - Eccentricity [dimensionless]
  /// \param sqrtSemiMajorAxis sqrtA - Square root of the semi-major
  ///                            axis [m^1/2]
  /// \param rightAscension Omega_0 - Longitude of ascending node of orbit
  ///                         plane at weekly epoch [rad]
  /// \param inclinationAngle i_0 - Inclination angle at reference
  ///                           time [semicircle]
  /// \param argumentOfPerigee omega - Argument of perigee [rad]
  /// \param ascensionRate Omega_dot - Rate of right ascension [rad/s]
  /// \param inclinationRate I_dot - Rate of inclination angle [rad/s]
  /// \param timeOfEphemeris t_0e - Ephmeris reference time [sec]
  ///
  /// \returns true if the ephemeris data contains no faults or faults were not
  /// checked
  bool setEphemeris(const uint16_t&      prn,
                    const AlertFlag&     alertFlag,
                    const AntiSpoofFlag& asFlag,
                    const uint32_t&      towSf1,
                    const uint16_t&      weekNumber,
                    const L2CodeType&    codeOnL2,
                    const uint16_t&      uraIndex,
                    const SVHealth&      svHealth,
                    const uint16_t&      iodc,
                    const L2NavDataFlag& l2PDataFlag,
                    const double&        groupDelay,
                    const double&        clockCorrectionTime,
                    const double&        clockAging3,
                    const double&        clockAging2,
                    const double&        clockAging1,
                    const uint32_t&      towSf2,
                    const uint16_t&      iodeSf2,
                    const double&        sinOrbitRadius,
                    const double&        meanMotionDifference,
                    const double&        meanAnomaly,
                    const double&        cosLatitude,
                    const double&        eccentricity,
                    const double&        sinLatitude,
                    const double&        sqrtSemiMajorAxis,
                    const double&        timeOfEphemeris,
                    const FitInterval&   fitInterval,
                    const uint16_t&      ageOfDataOffset,
                    const uint32_t&      towSf3,
                    const double&        cosInclination,
                    const double&        rightAscension,
                    const double&        sinInclination,
                    const double&        inclinationAngle,
                    const double&        cosOrbitRadius,
                    const double&        argumentOfPerigee,
                    const double&        ascensionRate,
                    const uint16_t&      iodeSf3,
                    const double&        inclinationRate,
                    bool                 checkForValidity = true);

  /// \brief Set the parsed ephemeris values for the ephemeris object
  ///
  /// Internally this method calls void setEphemeris(...).
  ///
  /// \param EphemerisParameters structure containing parsed data
  bool setEphemeris(const EphemerisParameters& params,
                    bool                       checkForValidity = true);

  /// \brief Get stored ephemeris parameters
  ///
  /// Input arguments are populated with the appropriate parameters.
  ///
  /// \return True if all subframes are valid, false if one or more are not
  bool getEphemeris(uint16_t&      prn,
                    AlertFlag&     alertFlag,
                    AntiSpoofFlag& asFlag,
                    uint32_t&      towSf1,
                    uint16_t&      weekNumber,
                    L2CodeType&    codeOnL2,
                    uint16_t&      uraIndex,
                    SVHealth&      svHealth,
                    uint16_t&      iodc,
                    L2NavDataFlag& l2PDataFlag,
                    double&        groupDelay,
                    double&        clockCorrectionTime,
                    double&        clockAging3,
                    double&        clockAging2,
                    double&        clockAging1,
                    uint32_t&      towSf2,
                    uint16_t&      iodeSf2,
                    double&        sinOrbitRadius,
                    double&        meanMotionDifference,
                    double&        meanAnomaly,
                    double&        cosLatitude,
                    double&        eccentricity,
                    double&        sinLatitude,
                    double&        sqrtSemiMajorAxis,
                    double&        timeOfEphemeris,
                    FitInterval&   fitInterval,
                    uint16_t&      ageOfDataOffset,
                    uint32_t&      towSf3,
                    double&        cosInclination,
                    double&        rightAscension,
                    double&        sinInclination,
                    double&        inclinationAngle,
                    double&        cosOrbitRadius,
                    double&        argumentOfPerigee,
                    double&        ascensionRate,
                    uint16_t&      iodeSf3,
                    double&        inclinationRate) const;

  /// \brief Get strored ephemeris parameters
  ///
  /// \return Structure of stored ephemeris parameters
  EphemerisParameters getEphemeris() const;

  /// \brief Get the satellite PRN ID
  uint16_t getPrn() const { return (prn_); }

  /// \brief Get the time of ephemeris
  /// \returns the time of ephemeris in seconds into the week
  double getTimeOfEphemeris() const { return (timeOfEphemeris_); }

  uint32_t getTowSf1() const { return towSf1_; }
  uint32_t getTowSf2() const { return towSf2_; }
  uint32_t getTowSf3() const { return towSf3_; }

  /// \brief Get the transmission time week number
  /// \returns the GPS week number modulo 1024
  uint16_t getWeekNumber() const { return weekNumber_; }

  /// Indicates if the satellite signal and navigation data are healthy
  bool isSvHealthy() const
  {
    return (!svHealth_.someOrAllNavDataBad) &&
           (svHealth_.signalHealth == SVSignalHealth::AllSignalsOk);
  }

  /// \brief Get fault status of subframe data
  ///
  /// Faults are detected by thresholding parsed subframe data. If data
  /// exceeds a minimum or maximum threshold, a fault is indicated.
  ///
  /// \returns Fault status structure, where 1 indicates a detected fault
  Subframe1Fault getSubframe1Faults() const { return (subframe1Fault_); }
  Subframe2Fault getSubframe2Faults() const { return (subframe2Fault_); }
  Subframe3Fault getSubframe3Faults() const { return (subframe3Fault_); }

  /// \brief Adds an individual subframe to the ephemeris object
  ///
  /// Add and parse a single subframe. The subframe ID is extracted and the
  /// appropriate member variables are set.
  ///
  /// \param subframe Subframe data formated into 30 8 bit words with no parity
  ///
  /// \returns True if the subframe is 1,2, or 3 and successfully parsed.
  bool setSubframe(const uint16_t& prn,
                   const uint8_t (&subframe)[30],
                   bool checkForValidity = true);

  /// \brief Adds all subframes to the ephemeris object
  ///
  /// Add and parse subframes. The subframe ID is extracted and the
  /// appropriate member variables are set.
  ///
  /// \param subframe1 Subframe 1 formated into 30 8 bit words with no parity
  /// \param subframe2 Subframe 2 formated into 30 8 bit words with no parity
  /// \param subframe3 Subframe 3 formated into 30 8 bit words with no parity
  ///
  /// \returns True if all subframes are successfully parsed and valid.
  bool setSubframes(const uint16_t prn,
                    const uint8_t (&subframe1)[30],
                    const uint8_t (&subframe2)[30],
                    const uint8_t (&subframe3)[30],
                    bool checkForValidity = true);

  /// Check if the issue date on the given subframes matches the
  /// issue date on any other subframes that have been already set.
  ///   0 - subframe matches issue date of current ephemeris subframes
  ///   1 - subframe is older than current ephemeris subframes
  ///   2 - subframe is newer than current ephemeris subframes
  int checkSubframeIssueDate(const uint8_t (&subframe)[30]);

  /// \brief Set bounds used to define validity of parsed subframe data
  ///
  /// \param bounds Pair of EphemerisParameters structs defining
  ///  minimum (first) and maximum (second) thresholds for each field.
  static void setBounds(
    const std::pair<EphemerisParameters, EphemerisParameters>& bounds);

  /// \brief Return stored threshold values
  ///
  /// Note static member function cannot be const
  static std::pair<EphemerisParameters, EphemerisParameters> getBounds();

  /// \brief Get a copy of subframe 1 data
  /// \params subframe array to copy subframe 1 data into
  void getSubframe1(uint8_t (&subframe)[30]);

  /// \brief Get a copy of subframe 2 data
  /// \params subframe array to copy subframe 2 data into
  void getSubframe2(uint8_t (&subframe)[30]);

  /// \brief Get a copy of subframe 3 data
  /// \params subframe array to copy subframe 3 data into
  void getSubframe3(uint8_t (&subframe)[30]);

  /// \brief Get a pointer to the compressed subframe 1 data
  /// \returns a constant pointer to a 30 byte array containing the subframe
  /// data
  const uint8_t* getSubframe1() const { return (uint8_t*)&subframe1_; };

  /// \brief Get a pointer to the compressed subframe 2 data
  /// \returns a constant pointer to a 30 byte array containing the subframe
  /// data
  const uint8_t* getSubframe2() const { return (uint8_t*)&subframe2_; };

  /// \brief Get a pointer to the compressed subframe 3 data
  /// \returns a constant pointer to a 30 byte array containing the subframe
  /// data
  const uint8_t* getSubframe3() const { return (uint8_t*)&subframe3_; };

  /// \brief Indicates if the subframe 1 data is valid
  /// \returns True if subframe 1 has been set and passes validity checks
  bool isSubframe1Valid() { return subframe1Valid_; };

  /// \brief Indicates if the subframe 2 data is valid
  /// \returns True if subframe 2 has been set and passes validity checks
  bool isSubframe2Valid() { return subframe2Valid_; };

  /// \brief Indicates if the subframe 3 data is valid
  /// \returns True if subframe 3 has been set and passes validity checks
  bool isSubframe3Valid() { return subframe3Valid_; };

  bool areAllSubramesValid()
  {
    return subframe1Valid_ && subframe2Valid_ && subframe3Valid_;
  }

  /// \brief Indicates if the ephemeris data is valid
  /// This checks that all subframes have been set and pass validity checks and
  /// that the issue date matches on each subframe
  /// \returns True if
  bool isEphemerisValid()
  {
    return subframe1Valid_ && subframe2Valid_ && subframe3Valid_ &&
           checkIssueNumber();
  };

  void clear()
  {
    prn_                  = 0;
    alertFlag_            = AlertFlag::ALERT_OFF;
    asFlag_               = AntiSpoofFlag::On;
    towSf1_               = std::numeric_limits<uint32_t>::max();
    weekNumber_           = 0;
    codeOnL2_             = L2CodeType::CACodeOn;
    uraIndex_             = 0;
    iodc_                 = std::numeric_limits<uint16_t>::max();
    l2pDataFlag_          = L2NavDataFlag::On;
    groupDelay_           = 0.0;
    clockCorrectionTime_  = 0.0;
    clockAging3_          = 0.0;
    clockAging2_          = 0.0;
    clockAging1_          = 0.0;
    towSf2_               = std::numeric_limits<uint32_t>::max();
    iodeSf2_              = std::numeric_limits<uint16_t>::max();
    sinOrbitRadius_       = 0.0;
    meanAnomaly_          = 0.0;
    meanMotionDifference_ = 0.0;
    cosLatitude_          = 0.0;
    eccentricity_         = 0.0;
    sinLatitude_          = 0.0;
    sqrtSemiMajorAxis_    = 0.0;
    timeOfEphemeris_      = 0.0;
    fitInterval_          = FitInterval::FourHrs;
    ageOfDataOffset_      = 0;
    towSf3_               = std::numeric_limits<uint32_t>::max();
    cosInclination_       = 0.0;
    rightAscension_       = 0.0;
    sinInclination_       = 0.0;
    inclinationAngle_     = 0.0;
    cosOrbitRadius_       = 0.0;
    argumentOfPerigee_    = 0.0;
    ascensionRate_        = 0.0;
    iodeSf3_              = std::numeric_limits<uint16_t>::max();
    inclinationRate_      = 0.0;
    subframe1Valid_       = false;
    subframe2Valid_       = false;
    subframe3Valid_       = false;
  }

  /// \brief Check that ephemeris parameters are witihin defined thresholds
  ///
  /// Checks subframe parameters to determine if their values exceed minimum
  /// and maximum thresholds defined in the following member variable:
  ///   std::pair<EphemerisParameters, EphemerisParameters> thresholds_;
  /// Note fields alertFlag, asFlag, codeOnL2, and fitInterval are not checked.
  ///
  /// \returns True if data is valid, false if one or more parameters exceed
  /// threshold
  bool checkSubframesForFaults();

  /// \brief Converts the Ephemeris fault data into a readable string
  std::string sf1FaultsToString() const;
  std::string sf2FaultsToString() const;
  std::string sf3FaultsToString() const;

  /// \brief Converts the Ephemeris data into a readable string
  std::string toString() const;

  /// \brief Converts the raw ephemeris subframes into a readable hex string
  std::string toHexString() const;

  std::string sf1ToHexString() const;
  std::string sf2ToHexString() const;
  std::string sf3ToHexString() const;

private:
  // Pack the ephemeris parameter values into subframes 1 -3
  void generateSubframes();

  // Pack the ephemeris parameter values into subframe 1
  void generateSubframe1();

  // Pack the ephemeris parameter values into subframe 2
  void generateSubframe2();

  // Pack the ephemeris parameter values into subframe 3
  void generateSubframe3();

  // parseSubframe1 parses a 30 byte, 240 bit array containing subframe data
  // with no zero padding.
  //
  // Returns true on success, returns false if input subframe is not subframe 1.
  //
  // Output parameters are as follows:
  //   tow: Time of week (seconds)
  //   weekNumber: Modulo 1024 week number
  //   codeOnL2: Indicates which code is commanded ON for the L2 channel
  //   uraIndex: Index indicating SV accuracy
  //   svHealth: Summary of nav and signal health
  //   iodc: Issue of data, Clock
  //   L2PDataFlag: Indicates if nav data stream was commanded off on the P-Code
  //                of the L2 channel
  //   tgd: Estimated group delay differential (seconds)
  //   toc: Clock data reference time (seconds)
  //   af2: Clock correction coefficient (seconds/second^2)
  //   af1: Clock correction coefficient (seconds/second)
  //   af0: Clock correction coefficient (seconds)
  bool parseSubframe1(const uint8_t (&subframe1)[30],
                      bool checkForValidity = true);

  // parseSubframe2 parses a 30 byte, 240 bit array containing subframe data
  // with no zero padding.
  //
  // Returns true on success, returns false if input subframe is not subframe 1.
  //
  // Output parameters are as follows:
  //   tow
  //   iode: Issue of data (ephemeris)
  //   crs: Amplitude of the sine harmonic correction term to the orbit radius
  //   (m)
  //   deltaN: Mean motion difference from computed value (rad/s)
  //   m0: Mean anomaly at reference time (rad)
  //   cuc: Amplitude of the cosine harmonic correction term to the argument
  //        of latitude (rad)
  //   eccentricity: Eccentricity of orbit (dimensionless)
  //   cus: Amplitude of the sine harmonic correction term to the argument
  //        of latitude (rad)
  //   sqrtA: Square root of the semi major axis (sqrt(m))
  //   toe: Reference time ephemeris (s)
  //   fitInterval
  //   ageOfDataOffset
  bool parseSubframe2(const uint8_t (&subframe2)[30],
                      bool checkForValidity = true);

  // parseSubframe3 parses a 30 byte, 240 bit array containing subframe data
  // with no zero padding.
  //
  // Returns true on success, returns false if input subframe is not subframe 1.
  //
  // Output parameters are as follows:
  //   tow:
  //   cic: Amplitude of the cosine harmonic correction term to the angle of
  //        inclination (rad)
  //   omega0: Longitude of ascending node of orbit plane at weekly epoch (rad)
  //   cis: Amplitude of the sine harmonic correction term to the angle of
  //        inclination (rad)
  //   i0: Inclination angle at reference time (rad)
  //   crc: Amplitude of the cosine harmonic correction term to the orbit
  //        radius (m)
  //   w: Argument of perigee (rad)
  //   omegaDot: Rate of right ascension (rad/s)
  //   iode: Issue of data (ephemeris)
  //   iDot: Rate of inclination angle (rad/s)
  bool parseSubframe3(const uint8_t (&subframe3)[30],
                      bool checkForValidity = true);

  uint8_t parseEphemerisIode(const uint8_t (&subframe)[30]);

  // getSVHealth decodes the health bits in
  SVHealth decodeEphemSVHealthBits(const uint8_t& svHealthBits);
  void decodeEphemSVHealthBits(const uint8_t& svHealthBits, SVHealth& svHealth);

  // Correct time values for a week crossover
  double weekCrossoverCheck(const double& time) const;

  //////////////////////////////////////////////////////////////////////////////
  // Channels Parameters

  // PRN Number of the SV the ephemeris was received from
  // This value is not contained in the ephemeris data itself, but must
  // be determined from the PRN being tracked on the channel that downloaded
  // the data
  unsigned int prn_;

  //////////////////////////////////////////////////////////////////////////////
  // Handover Word (HOW)

  // Indicates that the user range error is greater than the value
  // given by uraIndex_ and the SV should be used with caution.
  // Ref: IS-GPS-200, 20.3.3.2
  AlertFlag alertFlag_;

  //! Antispoofing flag - indicates A-S mode on the SV
  // Ref: IS-GPS-200, 20.3.3.2
  AntiSpoofFlag asFlag_;

  //////////////////////////////////////////////////////////////////////////////
  // Subframe 1

  // Word 2 - HOW
  //! Time of week stamp in Subframe 1 - Ref: IS-GPS-200, 20.3.3.2
  uint32_t towSf1_;
  uint32_t towM1_;

  // Word 3
  // WN - 10 LSBs of the Transmission week number
  // mod 1024 of the current gps week (10 bits)
  // Ref: IS-GPS-200, 20.3.3.3.1.1
  uint16_t weekNumber_;

  // Indicate if C/A and/or P code is being transmitted on L2
  // Ref: IS-GPS-200, 20.3.3.3.1.2
  L2CodeType codeOnL2_;

  // User range error index - specifies the SV accuracy
  // Bits 13 through 16 of word 3
  // Ref: IS-GPS-200, 20.3.3.3.1.3
  uint16_t uraIndex_;  // TOOD: change to enum

  // Indicates if the navigation data transmitted by the SV is OK.
  // Bits 17 to 22 of word 3
  // Ref: IS-GPS-200, 20.3.3.3.1.4
  SVHealth svHealth_;

  // issue of data clock - issue number of the data set for detecting changes
  // Bits 23 and 24 of word 3
  // Ref: IS-GPS-200, 20.3.3.3.1.5
  uint16_t iodc_;

  // Word 4 and Word 8
  // Indicates if the nav data stream is turned off on L2 P-code
  // Bit 1 of word 4
  // Ref: IS-GPS-200, 20.3.3.3.1.6
  L2NavDataFlag l2pDataFlag_;

  // Word 7
  //! TGD - Estimated group delay differential [seconds]
  // Scale factor 2^-31
  // Bits 17-24 of word 7
  // Ref: IS-GPS-200, 20.3.3.3.1.7
  double groupDelay_;

  // Word 8
  // TOC - Clock correction data reference time [seconds]
  // Scale factor 2^4
  // Bits 9-24 of word 8
  // Ref: IS-GPS-200, 20.3.3.3.1.8
  double clockCorrectionTime_;

  // Word 9
  // AF2 - clock aging term 3
  // Scale factor 2^-55
  // Ref: IS-GPS-200, 20.3.3.3.1.8
  double clockAging3_;

  //! AF1 - clock aging term 2
  // Scale factor 2^-43
  // Ref: IS-GPS-200, 20.3.3.3.1.8
  double clockAging2_;

  // Word 10
  // AF0 - clock aging term 1
  // Scale factor 2^-31
  // Ref: IS-GPS-200, 20.3.3.3.1.8
  double clockAging1_;

  //////////////////////////////////////////////////////////////////////////////
  // Subframe 2

  // Word 2
  //! Time of week stamp in HOW of Subframe 2
  uint32_t towSf2_;
  uint32_t towM2_;

  // Word 3
  // Issue of data ephemeris for subframe 2 - 8 LSBs of IODC from subframe 1
  // Used to indicate changes in the ephemeris data
  // Ref: IS-GPS-200, 20.3.3.4.1
  uint16_t iodeSf2_;

  //! Crs - amplitude of the sine harmonic correction term to the orbit radius
  double sinOrbitRadius_;

  // Word 4
  //! Delta N - mean motion difference from computed value
  double meanMotionDifference_;

  double meanMotionDifferenceRate_;

  // Word 4 & 5
  //! M_0 - mean anomaly at reference time
  double meanAnomaly_;

  // Word 6
  //! Cuc - amplitude of the cosine harm. term to the argument of latitude
  double cosLatitude_;

  // Word 6 & 7
  //! e - Eccentricity
  double eccentricity_;

  // Word 8
  //! Cus - amplitude of the sine harm. term to the argument of latitude
  double sinLatitude_;

  // Word 8 & 9
  //! Sqrt(A) - square root of the semi major axis
  double sqrtSemiMajorAxis_;

  // Word 10
  //! Toe - time of ephemeris
  double timeOfEphemeris_;

  // Curve fit interval used to determine the ephemeris parameters
  // Ref: IS-GPS-200, 20.3.3.4.3.1
  FitInterval fitInterval_;

  // AODO - age of data offset - Used with the navigation message correction
  // table given in subframe 4.
  // Ref: IS-GPS_200, 20.3.3.4.1
  uint16_t ageOfDataOffset_;

  //////////////////////////////////////////////////////////////////////////////
  // Subframe 3

  // Word 2
  //! Time of week stamp in Subframe 3
  uint32_t towSf3_;
  uint32_t towM3_;

  // Word 3
  //! Cic - Ampl. of the cos. harm. correction term to the angle of inclination
  double cosInclination_;

  // Word 3 & 4
  //! Omega_0 - Longitude of the ascending node of orbit plane at weekly epoch
  double rightAscension_;

  // Word 5
  //! Cis - Ampl of the sine harmonic term to the angle of inclination
  double sinInclination_;

  // Word 5 & 6
  //! i_0 - inclination angle at reference time
  double inclinationAngle_;

  // Word 7
  //! Crc - amplitude of the cosine harmonic correction term to the orbit radius
  double cosOrbitRadius_;

  // Word 7 & 8
  //! omega - argument of perigee
  double argumentOfPerigee_;

  // Word 9
  //! Omega_dot - rate of right ascension
  double ascensionRate_;

  double ascensionRateDifference_;

  // Word 10
  // Issue of data ephemeris for subframe 3 - 8 LSBs of IODC from subframe 1
  // Used to indicate changes in the ephemeris data
  // Ref: IS-GPS-200, 20.3.3.4.1
  uint16_t iodeSf3_;

  //! IDOT - rate of inclination angle
  double inclinationRate_;

  //  double semiMajorAxisRate_;
  double meanMotionRate_;

  //////////////////////////////////////////////////////////////////////////////
  // Raw subframe values
  bool subframe1Valid_;  // indicates if a valid subframe 1 has been loaded
  bool subframe2Valid_;  // indicates if a valid subframe 2 has been loaded
  bool subframe3Valid_;  // indicates if a valid subframe 3 has been loaded

  uint8_t subframe1_[30];
  uint8_t subframe2_[30];
  uint8_t subframe3_[30];

  //////////////////////////////////////////////////////////////////////////////
  // Ephemeris validation variables

  static std::pair<EphemerisParameters, EphemerisParameters> bounds_;

  // Not initialized in ctor list
  Subframe1Fault subframe1Fault_;
  Subframe2Fault subframe2Fault_;
  Subframe3Fault subframe3Fault_;

  void clearSubframeFaults();
  void initializeBounds();

  /// \brief Check issue number of stored subframes
  ///
  /// Check the consistency of IODC from Subframe 1 and IODE in Subframes 2 and
  /// 3 to determine if subframes are from the same issue number. See
  /// IS-GPS-200 20.3.3.3.1.5 and 20.3.4.4. If a descrepancy exists,
  /// subframeValid_ flags are set to false, and the IODC and IODE faults are
  /// set to 1.
  ///
  bool checkIssueNumber();

  /// \brief Check that ephemeris parameters are witihin defined thresholds
  ///
  /// Checks subframe parameters to determine if their values exceed minimum
  /// and maximum thresholds defined in the following member variable:
  ///   std::pair<EphemerisParameters, EphemerisParameters> thresholds_;
  /// Note fields alertFlag, asFlag, codeOnL2, and fitInterval are not checked.
  ///
  /// \returns True if data is valid, false if one or more parameters exceed
  /// threshold
  bool checkSubframe1();
  bool checkSubframe2();
  bool checkSubframe3();
};

}  // namespace pnt_integrity

#endif
