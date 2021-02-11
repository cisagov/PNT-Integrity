//============================================================================//
//--------------------- pnt_integrity/IntegrityData.hpp --------*- C++ -*-----//
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
/// \brief    Defines all data types and structure definitions
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     May 28, 2019
//============================================================================//
#ifndef PNT_INTEGRITY__INTEGRITY_DATA_HPP
#define PNT_INTEGRITY__INTEGRITY_DATA_HPP

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace pnt_integrity
{
/// Namespace for all integrity data definitions
namespace data
{
//==============================================================================
/// \brief A timestamp used in all headers
struct Timestamp
{
  /// The whole seconds portion of the timestamp
  int64_t sec;

  /// Fractional portion of the timestamp represented in ns, giving the
  /// timestamp 1 ns resolution
  int32_t nanoseconds;

  /// Indicator for timebase, 0 if synced to TAI, non-zero if device using a
  /// specific timebase
  int8_t timecode;

  /// \brief Default constructor for timestamp
  ///
  /// \param secIn The whole seconds portion of the timestamp
  /// \param nsIn Fractional portion of the timestamp represented in ns
  /// \param timecodeIn Indicator for timebase, 0 for TAI, non-zero for other
  Timestamp(const int64_t& secIn      = 0,
            const int32_t& nsIn       = 0,
            const int8_t&  timecodeIn = 0)
    : sec(secIn), nanoseconds(nsIn), timecode(timecodeIn){};
};

//==============================================================================
/// Enumeration for all available satellite-based time system sources
enum class TimeSystem
{
  GLO = 0,
  GPS,
  GAL,
  BDT
};

//==============================================================================
/// \brief A GNSS time
struct GNSSTime
{
  /// The number of elapsed week since a pre-defined epoch (non-rollover)
  int weekNumber;
  /// Seconds into the week
  double secondsOfWeek;
  /// The reference time system
  TimeSystem timeSystem;

  /// \brief Default constructor for GNSSTime
  ///
  /// \param week The number of elapsed week since a pre-defined epoch
  /// (non-rollover) \param seconds Seconds into the week \param system The base
  /// timesystem used for the GNSS time
  GNSSTime(const int&        week    = 0,
           const double&     seconds = 0.0,
           const TimeSystem& system  = TimeSystem::GPS)
    : weekNumber(week), secondsOfWeek(seconds), timeSystem(system){};
};

//==============================================================================
/// \brief The header used for all associated data types
struct Header
{
  /// The sequence number of the header
  long seq_num;
  /// The arrival time of the header at the data transport layer
  Timestamp timestampArrival;
  /// The valid time of the header / measurement data
  Timestamp timestampValid;
  /// Unique identifier for the measurement system / sensor / source
  std::string deviceId;

  /// \brief Default constructor for a header
  ///
  /// \param seq The sequence number of the header
  /// \param ts_arrival The arrival time of the header at the data
  ///                          transport layer
  /// \param ts_valid The valid time of the header / data
  /// \param dev_id Unique identifier for the measurement source
  Header(const long&        seq        = 0,
         const Timestamp&   ts_arrival = Timestamp(),
         const Timestamp&   ts_valid   = Timestamp(),
         const std::string& dev_id     = "")
    : seq_num(seq)
    , timestampArrival(ts_arrival)
    , timestampValid(ts_valid)
    , deviceId(dev_id){};
};

//==============================================================================
/// \brief A structure for measuring the offset between two clocks
struct ClockOffset
{
  /// A header for the message
  Header header;

  /// Indicator for clock 1 timebase, 0 if synced to TAI, non-zero if device
  /// using a specific timebase
  int8_t timecode1;

  /// Indicator for clock 2 timebase, 0 if synced to TAI, non-zero if device
  /// using a specific timebase
  int8_t timecode2;

  /// Time offset between the two clocks (sec)
  double offset;

  /// The drift between the two clocks (sec / sec)
  double drift;

  /// The measurement covariance of the offset parameters (2x2 matrix)
  double covariance[2][2];

  /// \brief Constructor for the ClockOffset structure
  ///
  /// The default constructor for the clock offset message can be
  /// optionally provided with a pre-built header. The offset, drift, and
  /// time error covariance is initialized to NaN and must be set
  /// to desired values after object construction. Timecodes are initialized
  /// to -1 and must also be set after construction to desired values
  ///
  /// \param headerIn A provided header object
  ClockOffset(const Header& headerIn = Header())
    : header(headerIn)
    , timecode1(-1)
    , timecode2(-1)
    , offset(std::numeric_limits<double>::quiet_NaN())
    , drift(std::numeric_limits<double>::quiet_NaN())
  {
    // initialize covariance to Nan
    for (auto ii = 0; ii < 2; ++ii)
    {
      for (auto jj = 0; jj < 2; ++jj)
      {
        covariance[ii][jj] = std::numeric_limits<double>::quiet_NaN();
      }
    }
  };
};

//=============================================================================
/// Enumeration for satellite system identification
enum class SatelliteSystem : uint8_t
{
  GPS = 0,
  Glonass,
  Galileo,
  QZSS,
  BeiDou,
  IRNSS,
  SBAS,
  Mixed,
  Other
};

//==============================================================================
/// Defines all possible frequency types
enum class FrequencyBand : uint8_t
{
  Band1 = 0,  // L1 (GPS, QZSS, SBAS), G1 (GLO), E1 (GAL).
  Band2,      // L2 (GPS, QZSS), G2 (GLO), B1 (BDS).
  Band5,      // L5 (GPS, QZSS, SBAS), E5a (GAL), L5 (IRNSS).
  Band6,      // E6 (GAL), LEX (QZSS), B3 (BDS).
  Band7,      // E5b (GAL), B2(BDS).
  Band8,      // E5a+b (GAL).
  Band9,      // S (IRNSS).
  Band0,      // Type X (all).
  Band10      // not specified
};

//==============================================================================
/// Defines all possible code types
enum class CodeType : uint8_t
{
  SigP = 0,  // P code-based (GPS, GLO).
  SigC,      // C code-based (GPS, SBAS, GLO, QZSS).
             // C channel (GAL, IRNSS).
  SigD,      // Semi-codeless (GPS).
  SigY,      // Y code-based (GPS).
  SigM,      // M code-based (GPS).
  SigN,      // codeless (GPS).
  SigA,      // A channel (GAL, IRNSS).
  SigB,      // B channel (GAL, IRNSS).
  SigI,      // I channel (GPS, GAL, QZSS, BDS).
  SigQ,      // Q channel (GPS, GAL, QZSS, BDS).
  SigS,      // M channel (L2C GPS, QZSS),
             // D channel (GPS, QZSS).
  SigL,      // L channel (L2C GPS, QZSS),
             // P channel (GPS, QZSS).
  SigX,      // B + C channels (GAL, IRNSS).
             // I + Q channels (GPS, GAL, QZSS, BDS).
             // M + L channels (GPS, QZSS).
             // D + P channels (GPS, QZSS).
  SigW,      // based on Z tracking (see Table A2 Rinex V3.03) (GPS).
  SigZ,      // Z channel (GPS).
  SigBLANK   // not-specified
};

//==============================================================================
/// Defines all available assurance level values
enum class AssuranceLevel : int8_t
{
  Unavailable = 0,  // will not be used
  Unassured,
  Inconsistent,
  Assured
};

/// \brief A structure to hold an AssuranceLevel and value
///
/// A structure for holding the idea of assurance both as an enumeration and
/// separate numeric value.
class AssuranceState
{
public:
  AssuranceState()
    : level_(AssuranceLevel::Unavailable)
    , value_(-2.0)
    , name_("")
    , weight_(0.0){};

  /// \brief Sets the state with a provided value
  ///
  /// This method allows the state to be set by an arbritrary value which is
  /// usually produced by a weighting function. The provided assurance value
  /// will be rounded to an integer and then thresholded to the appropriate
  /// value and the level enumeration is set appropriately.
  ///
  /// \param valueIn The provided value.
  bool setWithValue(const double& valueIn)
  {
    value_           = valueIn;
    int roundedValue = std::round(valueIn);

    if (roundedValue == -1)
    {
      level_ = AssuranceLevel::Unassured;
      return true;
    }
    else if (roundedValue == 0)
    {
      level_ = AssuranceLevel::Inconsistent;
      return true;
    }
    else if (roundedValue >= 1)
    {
      level_ = AssuranceLevel::Assured;
      return true;
    }
    else  // roundedValue < =1
    {
      level_ = AssuranceLevel::Unavailable;
      return true;
    }
  };

  /// \brief Sets the state with a provided enumeration
  ///
  /// This function will set the AssuranceState's level enumeration to the
  /// provided level and the value is set accordingly. The function returns
  /// false if the level is "Unavailable" to indicate that it should not be
  /// used in any cumulative calculations.
  ///
  /// \brief levelIn The provided level enumeration
  bool setWithLevel(const AssuranceLevel& levelIn)
  {
    switch (levelIn)
    {
      case AssuranceLevel::Unavailable:
        setWithValue(-2.0);
        break;
      case AssuranceLevel::Unassured:
        setWithValue(-1.0);
        break;
      case AssuranceLevel::Inconsistent:
        setWithValue(0.0);
        break;
      case AssuranceLevel::Assured:
        setWithValue(1.0);
        break;
    };

    return true;
  };
  /// \brief Retrieves the internal assurance value
  /// \returns The assurance value
  double getAssuranceValue() { return value_; };

  /// \brief Retrieves the internal assurance level
  /// \returns The assurance level
  data::AssuranceLevel getAssuranceLevel() { return level_; };

  /// \brief Retrieves the internal assurance value as an integer
  /// \returns An integer representation of the assurance value
  int getIntegerAssuranceValue() const { return static_cast<int>(value_) + 2; };

  /// \brief Sets the weight associated with the state
  void setWeight(const double& weight) { weight_ = weight; };

  /// \brief Retrieves the weight associated with the state
  /// \returns The weight value
  double getWeight() { return weight_; };

  /// \brief Sets the string name of the state
  void setName(const std::string& name) { name_ = name; };

  /// \brief Retrieves the name of the check
  /// \returns The string name of the check
  std::string getName() { return name_; };

private:
  /// The enumeration that represents the assurance level
  AssuranceLevel level_;

  // /// An integer represetnation of the assurance level
  // int value;
  double value_;

  /// A string name for the state. Typically represents the source of the
  /// generated state (i.e. a particular assurance check algorithm)
  std::string name_;

  /// An arbitrary weighting factor used to calculate the state.
  double weight_;
};

/// \brief A structure to hold a single assurance report
struct AssuranceReport
{
  /// The header for the structure message
  Header header;

  /// The assurance state
  AssuranceState state;
};

/// \brief A structure to hold assurance data for all registered checks
struct AssuranceReports
{
  /// The header for the structure message
  Header header;

  /// Number of assurance states reported
  long numStates;

  /// A vector of AssuranceState, length numStates
  std::vector<AssuranceState> states;

  /// \brief Default constructor for the struct. Initializes numStates to 0
  AssuranceReports() : numStates(0){};

  /// \brief Adds a reported state to the vector, increments count
  void addReport(const AssuranceState& state)
  {
    states.push_back(state);
    numStates++;
  };
};

//==============================================================================
/// \brief A structure for pseudorange observables
struct GNSSObservable
{
  /// Satellite ID or PRN
  uint16_t prn;

  /// The satellite system that the observable originates
  SatelliteSystem satelliteType;

  /// The code type of the received signal
  CodeType codeType;

  /// The frequency carrier of the received signal
  FrequencyBand frequencyType;

  /// Assurance level for this observable
  AssuranceLevel assurance;

  /// The carrier to noise ratio (C_no) of the received signal
  double carrierToNoise;

  /// Flag to indiate the validity of the observable pseudorange
  bool pseudorangeValid;

  /// The pseudorange measurement
  double pseudorange;

  /// The pseudorange measurement's variance
  double pseudorangeVariance;

  /// Flag to indicate the validity of the observable doppler
  bool dopplerValid;

  /// The doppler measurement
  double doppler;

  /// The variance of the doppler measurement
  double dopplerVariance;

  /// Flag to indicate the validity of the observable carrier phase
  bool carrierPhaseValid;

  /// The carrier-phase measurement
  double carrierPhase;

  /// The variance of the carrier-phase measurement
  double carrierPhaseVariance;

  /// Flag to indicate loss of carrier lock
  bool lossOfLock;

  /// \brief Default constructor to initialize values
  ///
  /// The constructor initializes all member variables to null states (i.e.
  /// NAN for double values, false for booleans, and unknown types for signal
  /// parameters
  ///
  /// \param prnIn Satellite ID or PRN
  /// \param satTypeIn The satellite system that the observable originates
  /// \param codeTypeIn The code type of the observable
  /// \param freqTypeIn The frequency carrier of the received signal
  /// \param assuranceLevelIn The assurance level for this observable
  /// \param cnoIn The carrier to noise ratio (C_no) of the received signal
  /// \param psrValIn Flag to indiate the validity of the pseudorange
  /// \param psrIn The pseudorange measurement
  /// \param psrVarIn The pseudorange measurement's variance
  /// \param doppValIn Flag to indicate the validity of the doppler
  /// \param doppIn The doppler measurement
  /// \param doppVarIn The variance of the doppler measurement
  /// \param cpValIn Flag to indicate the validity of the carrier phase
  /// \param cpIn The carrier-phase measurement
  /// \param cpVarIn The variance of the carrier-phase measurement
  /// \param lossOfLockIn Flag to indicate loss of carrier lock
  GNSSObservable(
    const uint16_t&        prnIn            = 0,
    const SatelliteSystem& satTypeIn        = SatelliteSystem::Other,
    const CodeType&        codeTypeIn       = CodeType::SigBLANK,
    const FrequencyBand&   freqTypeIn       = FrequencyBand::Band0,
    const AssuranceLevel&  assuranceLevelIn = AssuranceLevel::Unavailable,
    const double&          cnoIn     = std::numeric_limits<double>::quiet_NaN(),
    const bool&            psrValIn  = false,
    const double&          psrIn     = std::numeric_limits<double>::quiet_NaN(),
    const double&          psrVarIn  = std::numeric_limits<double>::quiet_NaN(),
    const bool&            doppValIn = false,
    const double&          doppIn    = std::numeric_limits<double>::quiet_NaN(),
    const double&          doppVarIn = std::numeric_limits<double>::quiet_NaN(),
    const bool&            cpValIn   = false,
    const double&          cpIn      = std::numeric_limits<double>::quiet_NaN(),
    const double&          cpVarIn   = std::numeric_limits<double>::quiet_NaN(),
    const bool&            lossOfLockIn = false)
    : prn(prnIn)
    , satelliteType(satTypeIn)
    , codeType(codeTypeIn)
    , frequencyType(freqTypeIn)
    , assurance(assuranceLevelIn)
    , carrierToNoise(cnoIn)
    , pseudorangeValid(psrValIn)
    , pseudorange(psrIn)
    , pseudorangeVariance(psrVarIn)
    , dopplerValid(doppValIn)
    , doppler(doppIn)
    , dopplerVariance(doppVarIn)
    , carrierPhaseValid(cpValIn)
    , carrierPhase(cpIn)
    , carrierPhaseVariance(cpVarIn)
    , lossOfLock(lossOfLockIn){};

  /// \brief Returns a unique identifier for the observable
  ///
  /// Returns a unique identifier by multiplying the prn, satellite type,
  /// code type, and frequency type enumeration values (adjusted for
  /// zero-based entries). This function assumes that there are no
  /// enumeration values that have a value of -1.
  ///
  /// \returns A long integer representing the unique identifier
  uint64_t getUniqueID()
  {
    uint64_t uniqueID = (((uint64_t)prn) << 24) |
                        (((uint64_t)satelliteType) << 16) |
                        (((uint64_t)codeType) << 8) | ((uint64_t)frequencyType);

    return uniqueID;
  }
};

//==============================================================================
/// A map to relate a GNSSObservable to a PRN
using GNSSObservableMap = std::map<uint64_t, GNSSObservable>;

/// \brief The GNSSObservables message
///
/// This data structure represents the message format for a GNSS observable
struct GNSSObservables
{
  /// The message header
  Header header;
  /// The GNSSTime associated with the observable data
  GNSSTime gnssTime;
  /// A map of observables, keyed off of satellite id (or prn)
  GNSSObservableMap observables;

  /// \brief Default constructor for the structure
  GNSSObservables()
    : header(Header())
    , gnssTime(GNSSTime())
    , observables(GNSSObservableMap()){};

  /// \brief Constructor with provided data
  ///
  /// \param header The provided header structure
  /// \param gnssTime A provided GNSSTime object
  /// \param obsMap The map of observables, keyed off of prn
  GNSSObservables(const Header&           header,
                  const GNSSTime&         gnssTime,
                  const GNSSObservableMap obsMap)
    : header(header), gnssTime(gnssTime), observables(obsMap){};
};

//============================================================================
/// \brief A structure to represent 3D geodetic position
///
/// This structure represents that latitude, longitude, and altitude of a
/// geodetic position
struct GeodeticPosition3d
{
  /// The latitude in radians
  double latitude;  // radians
  /// The longitude in radians
  double longitude;  // radians
  /// The altitude in meters above the WGS-84 ellipsoid
  double altitude;  // meters above the WGS-84 ellipsoid

  /// \brief Constructor for the 3D geodetic position
  ///
  /// \param latIn The latitude of the 3d position (radians)
  /// \param lonIn The longitude of the 3d position (radians)
  /// \param altIn The altitude of the 3d position (meters above WGS-84)
  GeodeticPosition3d(
    const double& latIn = std::numeric_limits<double>::quiet_NaN(),
    const double& lonIn = std::numeric_limits<double>::quiet_NaN(),
    const double& altIn = std::numeric_limits<double>::quiet_NaN())
    : latitude(latIn), longitude(lonIn), altitude(altIn){};

  /// \brief Returns the coordinates from lla to ecef
  /// using WGS84
  ///
  /// \param ecef The output location for the generated
  /// coordinates, must be at least 3*sizeof(double) large
  /// \returns false if any values are NaN, true otherwise
  bool getECEF(double* ecef) const
  {
    if (latitude == std::numeric_limits<double>::quiet_NaN() ||
        longitude == std::numeric_limits<double>::quiet_NaN() ||
        altitude == std::numeric_limits<double>::quiet_NaN())
    {
      return false;
    }

    double a = 6378137.0;
    double e = 0.081819190842622;
    // double asq = pow(a, 2);
    double esq = pow(e, 2);
    double N   = a / sqrt(1 - esq * pow(sin(latitude), 2));
    double x   = (N + altitude) * cos(latitude) * cos(longitude);
    double y   = (N + altitude) * cos(latitude) * sin(longitude);
    double z   = ((1 - esq) * N + altitude) * sin(latitude);

    ecef[0] = x;
    ecef[1] = y;
    ecef[2] = z;

    return true;
  };
};

//==============================================================================
/// \brief A structure to represent a Position / Velocity message
///
/// This structure represents a data message that contains a geodetic 3d
/// position, an NED velocity, and a 6x6 cross-covariance of position /
/// velocity
struct PositionVelocity
{
  /// The message header
  Header header;
  /// The 3D geodetic position
  GeodeticPosition3d position;
  /// The velocity in north-east-down (NED)
  double velocity[3];
  /// The cross-covariance for position / velocity in NED (6x6)
  double covariance[6][6];

  /// \brief Constructor for the PositionVelocity structure
  ///
  /// The default constructor for the position / velocity message can be
  /// optionally provided with a pre-built header and position structure. The
  /// velocity and covariance arrays are initialized to NaN and must be set
  /// to desired values after object construction
  ///
  /// \param headerIn A provided header object
  /// \param positionIn A provided 3D position (geodetic)
  PositionVelocity(const Header&             headerIn   = Header(),
                   const GeodeticPosition3d& positionIn = GeodeticPosition3d())
    : header(headerIn), position(positionIn)
  {
    // initialize velocity to NaN
    for (auto ii = 0; ii < 3; ++ii)
    {
      velocity[ii] = std::numeric_limits<double>::quiet_NaN();
    }
    // initialize covariance to Nan
    for (auto ii = 0; ii < 6; ++ii)
    {
      for (auto jj = 0; jj < 6; ++jj)
      {
        covariance[ii][jj] = std::numeric_limits<double>::quiet_NaN();
      }
    }
  };

  /// \brief Checks the validity of the position
  bool isPositionValid()
  {
    return (!std::isnan(position.latitude) && !std::isnan(position.longitude) &&
            !std::isnan(position.altitude));
  }

  /// \brief Checks the validity of the covariance
  bool isPositionCovarianceValid()
  {
    bool validFlag = true;

    for (auto ii = 0; ii < 3; ++ii)
    {
      for (auto jj = 0; jj < 3; ++jj)
      {
        validFlag &= !std::isnan(covariance[ii][jj]);
      }
    }

    return validFlag;
  }

  /// \brief Checks the validity of the veocity
  bool isVelocityValid()
  {
    bool validFlag = true;

    for (auto ii = 0; ii < 3; ++ii)
    {
      validFlag &= !std::isnan(velocity[ii]);
    }

    return validFlag;
  }

  /// \brief Checks the validity of the covariance
  bool isVelocityCovarianceValid()
  {
    bool validFlag = true;

    for (auto ii = 3; ii < 6; ++ii)
    {
      for (auto jj = 3; jj < 6; ++jj)
      {
        validFlag &= !std::isnan(covariance[ii][jj]);
      }
    }

    return validFlag;
  }

  /// \brief Checks the structure to make sure all data fields are valid
  bool checkValidity()
  {
    bool returnFlag = isPositionValid() && isVelocityValid() &&
                      isPositionCovarianceValid() &&
                      isVelocityCovarianceValid();

    return returnFlag;
  }
};

//==============================================================================
/// \brief A structure that represents a distance traveled over a time period
struct AccumulatedDistranceTraveled
{
  /// The message header
  Header header;

  /// Time span of accumulated distance (s)
  double dt;

  /// Accumulated distance traveled over time period (m)
  double distance;

  /// Accumulated distance traveled variance (m^2)
  double variance;
};

//==============================================================================
/// \brief A structure that represents IMU measurement data
struct IMU
{
  /// The message header
  Header header;

  /// Acceleration integrated over period delta_t, providing an "average
  /// change in velocity" measurement. units: m/s
  double delta_v[3];

  /// Angular rate integrated over period delta_t, providing an "average
  /// change in angle" measurement. units: rad
  double delta_theta[3];
};

//==============================================================================
/// \brief A structure that represents a distance measurement to a known point
///
/// This structure holds all relative data that represents a measured distance
/// to a feature with a known location
struct MeasuredRange
{
  /// The message header
  Header header;

  /// Flag to indicate validity of range measurement
  bool rangeValid;
  /// The range measurement to the feature
  double range;
  /// The variance associated with the range measurement
  double variance;
  /// The feature location
  GeodeticPosition3d featurePosition;
  /// The covariance of the geodetic position
  double feature_position_covariance_[3][3];

  /// \brief Default constructor
  ///
  /// \param valid Flag to indicate measurement validity
  MeasuredRange(const bool& valid = false) : rangeValid(valid){};
};

//==============================================================================
/// \brief A structure to represent an AGC measurement
struct AgcValue
{
  /// The message header
  Header header;

  /// A vector for AGC values (multiple bands possible)
  std::map<FrequencyBand, double> agcValues;
};

//==============================================================================
/// \brief An enumeration for diagnostic level
enum class LevelEnum
{
  DIAG_OK = 0,
  DIAG_WARN,
  DIAG_ERROR,
  DIAG_STALE
};

/// \brief A structure for key value pairs
struct KeyValue
{
  /// Label for the value
  std::string key;

  /// Value
  std::string value;
};

//==============================================================================

/// \brief A structure for general diagnostic messages
struct Diagnostics
{
  /// The message header
  data::Header header;

  /// Enumeration field to indicate operating level of source application or
  /// component
  data::LevelEnum level;

  /// The name of the diagnostic
  std::string name;

  /// Detailed description of message
  std::string message;

  /// An identifier for the source of the diagnostic (i.e. hardware serial
  /// number or name of the generating application)
  std::string hardwareId;

  /// The number of key/value pairs in the diagnostic message
  long numValues;

  /// A vector of id / value pairs
  std::vector<data::KeyValue> values;
};
//==============================================================================
/// \brief An enumeration for event log type
enum class EventLogType
{
  NotSet = 0,
  Debug,
  Info,
  Warning,
  Error,
  Critical
};

/// \brief Structure for event log messages
struct EventLog
{
  /// The message header
  data::Header header;

  /// The type of event log
  data::EventLogType eventLogType;

  /// The log message
  std::string eventLog;
};
//==============================================================================
/// \brief The command / response type enumeration
enum class CommandResponseType
{
  COMMAND = 0,
  RESPONSE
};

/// \brief A structure for command / response messages
struct CommandResponse
{
  /// The header associated with the command / response structure
  data::Header header;

  /// The device id of the command target (only used for responses)
  long deviceId;

  /// The identifier for the command
  long commandId;

  /// Tye type of message
  data::CommandResponseType type;

  /// The command or response message
  std::string message;
};

}  // namespace data

}  // namespace pnt_integrity
#endif
