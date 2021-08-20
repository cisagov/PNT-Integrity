//============================================================================//
//--------------------- pnt_integrity/GPSAlmanac.hpp -----------*- C++ -*-----//
//============================================================================//
// BSD 3-Clause License
//
// Copyright (C) 2015-2021 Integrated Solutions for Systems, Inc
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
/// \brief    Stores a set of GPS almanac data and computes satellite pos.
/// \author   David Hodo <david.hodo@is4s.com>
/// \date     January 2015
//============================================================================//
#ifndef PNT_INTEGRITY__GPS_ALMANAC_HPP
#define PNT_INTEGRITY__GPS_ALMANAC_HPP

#include <cstdint>
#include <string>

#include "pnt_integrity/GPSNavDataCommon.hpp"

namespace pnt_integrity
{
/// Enumeration to define the top 3 bits of the 8-bith satellite
/// health field included in Almanac subframes 4 and 5
/// Defined in paragraph 20.3.3.5.1.3 of IS-GPS-200
enum class SVNavHealth : uint8_t
{
  AllDataOK             = 0,
  ParityFailure         = 1,
  TlmHowFormatProblem   = 2,
  ZCountInHowBad        = 3,
  Subframe_1_2_or_3_Bad = 4,
  Subframe_4_or_5_Bad   = 5,
  AllUploadedDataBad    = 6,
  AllDataBad            = 7
};

/// Structure to hold the satellite health
/// field given in bits 18 to 22 of subframe 1 and in the bottom 5 bits
/// of the 8 bit satellite health field in Almanac subframes 4 and 5
/// Defined in paragraph 20.3.3.5.1.3 of IS-GPS-200
struct SVAlmHealth
{
  SVNavHealth    navDataHealth;
  SVSignalHealth signalHealth;
};

struct AlmanacParameters
{
  // Almanac values
  uint16_t    prn;           // Satellite PRN ID
  double      tow;           // Time of week from subframe HOW (sec)
  SVAlmHealth svHealth;      //
  double      eccentricity;  // Eccentricity [dimensionless]
  double      toa;           // Time of almanac [seconds]
  double      deltaI;        // delta inclination angle [rad]
  double      omegaDot;      // Rate of right ascension [rad/sec]
  double      sqrtA;   // Square root of the semi-major axis [sqrt(meters)]
  double      omega0;  // Right ascension angle [rad]
  double      omega;   // Argument of perigee [rad]
  double      m0;      // Mean anomaly at reference tim [rad]
  double      af0;     // Clock aging term 1 [seconds]
  double      af1;     // Clock aging term 2 [seconds/second]
  // Full week number
  uint16_t referenceWeek;  // Almanac reference week (Full week number)
};

union AlmanacSubframeFaults
{
  struct FaultType
  {
    uint16_t prn : 1, tow : 1, svHealth : 1, eccentricity : 1, toa : 1,
      deltaI : 1, omegaDot : 1, sqrtA : 1, omega0 : 1, omega : 1, m0 : 1,
      af0 : 1, af1 : 1, referenceWeek : 1;
  };
  FaultType faultType;
  uint16_t  bitfield;
};

/// \brief Class to parse and store almanac data for a GPS Satellite
///
/// The GpsAlmanac class stores almanac data from a GPS navigation data subframe
/// in both parsed and compressed forms.  The class is capable of both parsing
/// compressed almanac subframes into its engineering unit components and
/// generating compressed subframes from the individual almanac values.
class GpsAlmanac
{
public:
  GpsAlmanac();

  /// \brief Constructs a GpsAlmanac object from parsed values
  /// \param prn Satellite PRN ID
  /// \param tow Time of week from subframe HOW
  /// \param eccentricity Eccentricity [dimensionless]
  /// \param toa Time of almanac [seconds]
  /// \param deltaI delta inclination angle [rad]
  /// \param omegaDot Rate of right ascension [rad/sec]
  /// \param sqrtA Square root of the semi-major axis [sqrt(meters)]
  /// \param omega0 Right ascension angle [rad]
  /// \param omega Argument of perigee [rad]
  /// \param m0 Mean anomaly at reference tim [rad]
  /// \param af0 Clock aging term 1 [seconds]
  /// \param af1 Clock aging term 2 [seconds/second]
  /// \param wna Almanac reference week (Full week number)
  GpsAlmanac(unsigned int& prn,
             double&       tow,
             SVAlmHealth&  svHealth,
             double&       eccentricity,
             double&       toa,
             double&       deltaI,
             double&       omegaDot,
             double&       sqrtA,
             double&       omega0,
             double&       omega,
             double&       m0,
             double&       af0,
             double&       af1,
             uint16_t&     wna);

  /// \brief Constructs a GpsAlmanac object from a compressed subframe
  /// Constructs a GpsAlmanac object by parsing the contents of the subframe
  /// into its engineering unit values.  The subframe contains the 10, 30-bit
  /// words of a GPS subframe minus the 6 bits of parity resulting in 10, 24-bit
  /// words for a total of 30 bytes.
  ///
  /// \param prn PRN number of the Almanac subframe
  /// \param subframe Byte array containing the 240 bit subframe with no parity
  GpsAlmanac(const unsigned int prn, const uint8_t (&subframe)[30]);

  /// \brief Destructor for the GpsAlmanac object
  ~GpsAlmanac(){};

  /// \brief Set the Almanac fields using engineering parameter values
  /// \param prn Satellite PRN ID
  /// \param tow Time of week from subframe HOW
  /// \param eccentricity Eccentricity [dimensionless]
  /// \param toa Time of almanac [seconds]
  /// \param deltaI Rate of inclination angle??? [semi-circles]
  /// \param omegaDot Rate of right ascension [semi-circles/sec]
  /// \param sqrtA Square root of the semi-major axis [sqrt(meters)]
  /// \param omega0 Right ascension angle [semi-circles]
  /// \param omega Argument of perigee [semi-circles]
  /// \param m0 Mean anomaly at reference tim [semi-circles]
  /// \param af0 Clock aging term 1 [seconds]
  /// \param af1 Clock aging term 2 [seconds/second]
  void setAlmanac(const unsigned int& prn,
                  const double&       tow,
                  const SVAlmHealth&  svHealth,
                  const double&       eccentricity,
                  const double&       toa,
                  const double&       deltaI,
                  const double&       omegaDot,
                  const double&       sqrtA,
                  const double&       omega0,
                  const double&       omega,
                  const double&       m0,
                  const double&       af0,
                  const double&       af1,
                  bool                checkForValidity = true);

  /// \brief Set the parsed almanac values for the almanac object
  ///
  /// Internally this method calls void setAlmanac(...).
  ///
  /// \param EphemerisParameters structure containing parsed data
  void setAlmanac(const AlmanacParameters& param, bool checkForValidity = true);

  /// \brief Get the Almanac fields in engineering units
  /// \param prn, Satellite PRN ID
  /// \param tow, Time of week from subframe HOW [sec]
  /// \param eccentricity, Eccentricity [dimensionless]
  /// \param toa, Time of almanac [seconds]
  /// \param deltaI, delta inclination angle [rad]
  /// \param omegaDot, Rate of right ascension [rad/sec]
  /// \param sqrtA, Square root of the semi-major axis [sqrt(meters)]
  /// \param omega0, Right ascension angle [rad]
  /// \param omega, Argument of perigee [rad]
  /// \param m0, Mean anomaly at reference tim [rad]
  /// \param af0, Clock aging term 1 [seconds]
  /// \param af1, Clock aging term 2 [seconds/second]
  bool getAlmanac(unsigned int& prn,
                  double&       tow,
                  SVAlmHealth&  svHealth,
                  double&       eccentricity,
                  double&       toa,
                  double&       deltaI,
                  double&       omegaDot,
                  double&       sqrtA,
                  double&       omega0,
                  double&       omega,
                  double&       m0,
                  double&       af0,
                  double&       af1) const;

  /// \brief Get almanac parameters
  ///
  /// \return Structure of stored almanac parameters
  AlmanacParameters getAlmanac() const;

  void getSvState(const double& receiveTime,
                  double&       positionEcefX,
                  double&       positionEcefY,
                  double&       positionEcefZ,
                  double&       velocityEcefX,
                  double&       velocityEcefY,
                  double&       velocityEcefZ,
                  double&       svClockCorrection,
                  const double& pseudorange = 0.0) const;

  /// \brief Get the satellite PRN ID
  unsigned int getPrn() const { return (prn_); }

  /// \brief Set the full reference GPS week
  void setReferenceWeek(const uint16_t week, bool checkForValidity = true);

  /// \brief Get the full GPS reference week
  uint16_t getReferenceWeek() const { return (referenceWeek_); };

  AlmanacSubframeFaults getSubframeFaults() const { return subframeFaults_; };

  /// \brief Parse the given subframe into the Almanac object
  /// \param subframe Byte array containing the 240 bit subframe with no parity
  bool setSubframe(const unsigned int prn,
                   const uint8_t (&subframe)[30],
                   bool checkForValidity = true);

  /// \brief Get a pointer to the compressed subframe data
  /// \returns a constant pointer to a 30 byte array containing the almanac data
  const uint8_t* getSubframe() const { return (uint8_t*)&subframe_; };

  /// \brief Get a copy of the subframe data
  /// \params subframe array to copy subframe data into
  void getSubframe(uint8_t (&subframe)[30]);

  double getTow() { return tow_; };

  /// \brief Returns a hex string containing the raw subframe value
  std::string subframeToString() const;

  /// \brief Indicates if the Alamanac subframe is valid
  /// \returns true if the almanac data has ben set and is valid
  bool isSubframeValid() const { return subframeValid_; };

  /// \brief Checks to see if almanac data for this SV is listed as healthy
  bool isSvHealthy() const;

  /// \brief Set thresholds used to define validity of parsed subframe data
  ///
  /// \param thresholds Pair of AlmanacParameters structs defining
  ///  minimum (first) and maximum (second) thresholds for each field.
  static void setThresholds(
    const std::pair<AlmanacParameters, AlmanacParameters>& thresholds);

  /// \brief Return stored threshold values
  ///
  /// Note static member function cannot be const
  static std::pair<AlmanacParameters, AlmanacParameters> getThresholds();

  /// \brief Checks to see if the supplied subframe is newer than the existing
  /// subframe by comparing TOA (Almanac Reference Time)
  NavDataTimeOfArrival checkSubframeTOA(const uint8_t (&subframe)[30]);

private:
  // Parse the subframe into engineering unit values
  bool parseSubframe(const uint8_t (&subframe)[30]);

  // getSVHealth decodes the health bits in
  SVAlmHealth decodeAlmSVHealthBits(const uint8_t& svHealthBits);
  void        decodeAlmSVHealthBits(const uint8_t& svHealthBits,
                                    SVAlmHealth&   svHealth);

  uint16_t parseAlmanacSVID(const uint8_t (&subframe)[30]);
  void     parseAlmanacSVID(const uint8_t (&subframe)[30], uint16_t& svid);

  /// \brief  Parse eccentricity [dimensionless]
  double parseAlmanacEccentricity(const uint8_t (&subframe)[30]);
  void   parseAlmanacEccentricity(const uint8_t (&subframe)[30],
                                  double& eccentricity);

  /// \brief  Parse Time of Almanac [sec]
  static void   parseTimeOfAlmanac(const uint8_t (&subframe)[30], double& toa);
  static double parseTimeOfAlmanac(const uint8_t (&subframe)[30]);

  /// \brief  Parse Delta Inclination Angle [rad]
  double parseAlmanacDeltaI(const uint8_t (&subframe)[30]);
  void   parseAlmanacDeltaI(const uint8_t (&subframe)[30], double& deltaI);

  /// \brief  Parse Rate of Right Ascension (omegaDot) [rad/sec]
  double parseAlmanacRateOfRightAscension(const uint8_t (&subframe)[30]);
  void   parseAlmanacRateOfRightAscension(const uint8_t (&subframe)[30],
                                          double& omegaDot);

  /// \brief  Parse SV Health Bits
  uint8_t parseAlmanacSVHealth(const uint8_t (&subframe)[30]);
  void    parseAlmanacSVHealth(const uint8_t (&subframe)[30],
                               uint8_t& svHealthBits);

  /// \brief  Parse Sqrt Semi-Major Axis [m^1/2]
  double parseAlmanacSqrtSemiMajorAxis(const uint8_t (&subframe)[30]);
  void   parseAlmanacSqrtSemiMajorAxis(const uint8_t (&subframe)[30],
                                       double& sqrtA);

  /// \brief  Parse Longitude of Ascending Node (omega0) [rad]
  double parseAlmanacLongitudeOfAscendingNode(const uint8_t (&subframe)[30]);
  void   parseAlmanacLongitudeOfAscendingNode(const uint8_t (&subframe)[30],
                                              double& omega0);

  /// \brief  Parse Argument of Perigee (w) [rad]
  double parseAlmanacArgumentOfPerigee(const uint8_t (&subframe)[30]);
  void parseAlmanacArgumentOfPerigee(const uint8_t (&subframe)[30], double& w);

  /// \brief  Parse Mean Anomoly [rad]
  double parseAlmanacMeanAnomaly(const uint8_t (&subframe)[30]);
  void   parseAlmanacMeanAnomaly(const uint8_t (&subframe)[30], double& m0);

  /// \brief  Parse Clock Aging Term 1 (af0) [sec]
  double parseAlmanacClockCoefficient0(const uint8_t (&subframe)[30]);
  void   parseAlmanacClockCoefficient0(const uint8_t (&subframe)[30],
                                       double& af0);

  /// \brief  Parse Clock Aging Term 2 (af1) [sec/sec]
  double parseAlmanacClockCoefficient1(const uint8_t (&subframe)[30]);
  void   parseAlmanacClockCoefficient1(const uint8_t (&subframe)[30],
                                       double& af1);

  // Pack the engineering unit values into a subframe
  void generateSubframe();

  // Correct time values for a week crossover
  double weekCrossoverCheck(const double& time) const;

  //////////////////////////////////////////////////////////////////////////////
  /// Almanac parameters

  // Almanac PRN Number
  uint16_t prn_;

  /// Time of week from subframe HOW [sec]
  double tow_;

  /// Almanac Health Bits
  SVAlmHealth svHealth_;

  /// Eccentricity [dimensionless]
  double eccentricity_;

  /// Time of almanac [seconds]
  double toa_;

  /// delta inclination angle [rad]
  double deltaI_;

  /// Rate of right ascension [rad/sec]
  double omegaDot_;

  /// Square root of the semi-major axis [sqrt(meters)]
  double sqrtA_;

  /// Right ascension angle [rad]
  double omega0_;

  /// Argument of perigee [rad]
  double omega_;

  /// Mean anomaly at reference tim [rad]
  double m0_;

  /// Clock aging term 1 [seconds]
  double af0_;

  /// Clock aging term 2 [seconds/second]
  double af1_;

  // Full GPS week number, which toa is referenced to
  // Modulo 256 week number is found in  subframe 5, page 25, word 3, bits 17-24
  // of data message
  uint16_t referenceWeek_;

  // 30 byte array containing almanac subframe minus parity bits
  uint8_t subframe_[30];

  //////////////////////////////////////////////////////////////////////////////
  // Almanac validation variables

  // subframe validity
  bool subframeValid_;

  // Subframe parameter faults
  AlmanacSubframeFaults subframeFaults_;
  void                  initializeSubframeFaults(uint16_t var);

  /// Lower and Upper thresholds on almanac parameters
  static std::pair<AlmanacParameters, AlmanacParameters> thresholds_;

  /// \brief Check that almanac parameters are witihin defined thresholds
  ///
  /// Checks subframe parameters to determine if their values exceed minimum
  /// and maximum thresholds defined in the following member variable:
  ///   std::pair<AlmanacParameters, AlmanacParameters> thresholds_;
  ///
  /// \returns  True if data is valid, false if one or more parameters exceed
  ///           threshold
  bool checkSubframe();

  /// \brief Check the reference week is witihin defined thresholds
  ///
  /// This check is seperate from the subframe parameters because it is often
  /// received in a different message
  ///
  /// \returns  True if data is valid, false otherwise
  bool checkReferenceWeek();
};

}  // namespace pnt_integrity

#endif
