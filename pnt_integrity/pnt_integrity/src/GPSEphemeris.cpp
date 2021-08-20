//============================================================================//
//--------------------------- GPSEphemeris.cpp -------------------------------//
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
//
// Stores a set of GPS ephemeris data and computes satellite position.
// William Travis <william.travis@is4s.com>
// David Hodo <david.hodo@is4s.com>
// August 2013
//============================================================================//
#include "pnt_integrity/GPSEphemeris.hpp"

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <sstream>

namespace
{
pnt_integrity::EphemerisParameters lowerBoundDefaults = {
  1,                                       // prn
  pnt_integrity::AlertFlag::ALERT_RAISED,  // alert flag
  pnt_integrity::AntiSpoofFlag::On,        // as flag
  std::numeric_limits<uint16_t>::min(),    // towSf1
  std::numeric_limits<uint16_t>::min(),    // towM1
  std::numeric_limits<uint16_t>::min(),    // week number
  pnt_integrity::L2CodeType::CACodeOn,     // code on L2
  0,                                       // uraIndex
  pnt_integrity::SVHealth(),               // sv health
  std::numeric_limits<uint16_t>::min(),    // iodc
  pnt_integrity::L2NavDataFlag::On,        // l2p data flag
  -std::numeric_limits<double>::max(),     // group delay
  0.0,                                     // clock correction time
  -std::numeric_limits<double>::max(),     // clock aging 3
  -std::numeric_limits<double>::max(),     // clock aging 2
  -std::numeric_limits<double>::max(),     // clock again 1
  -std::numeric_limits<double>::max(),     // inPhase Inter Signal Correction;
  -std::numeric_limits<double>::max(),   // quadrature Inter Signal Correction;
  std::numeric_limits<uint32_t>::min(),  // towSf2
  std::numeric_limits<uint32_t>::min(),  // towM2
  std::numeric_limits<uint16_t>::min(),  // iodeSf2
  -std::numeric_limits<double>::max(),   // C_rs sin of orbit radius
  -std::numeric_limits<double>::max(),   // Delta N, mean motion difference
  -std::numeric_limits<double>::max(),   // mean motion difference rate
  -std::numeric_limits<double>::max(),   // M_0, mean anomaly
  -std::numeric_limits<double>::max(),   // C_uc, cosine of latitude
  0.0,                                   // e, eccentricity
  -std::numeric_limits<double>::max(),   // C_us, sine of latitude
  2530,                                  // sqrtA, sqrt of semi major axis
  -std::numeric_limits<double>::max(),   // DeltaA, semi major axis difference
  -std::numeric_limits<double>::max(),   // Adot, semi major axis rate
  0,                                     // toe, time of ephemeris
  pnt_integrity::FitInterval::FourHrs,   // fit interval
  std::numeric_limits<uint16_t>::min(),  // AODO, age of data offset
  std::numeric_limits<uint32_t>::min(),  // towSf3
  std::numeric_limits<uint32_t>::min(),  // towM3
  -std::numeric_limits<double>::max(),   // C_ic, cosine of inclination
  -std::numeric_limits<double>::max(),   // Omega_0, right ascension
  -std::numeric_limits<double>::max(),   // right ascension rate difference
  -std::numeric_limits<double>::max(),   // C_is, sine of inclination
  -std::numeric_limits<double>::max(),   // i_0, inclination angle
  -std::numeric_limits<double>::max(),   // C_rc, cosine of orbit radius
  -std::numeric_limits<double>::max(),   // omega, argument of perigree
  -6.33e-7,                              // Omega_dot, ascension rate
  std::numeric_limits<uint16_t>::min(),  // iodeSf3
  -std::numeric_limits<double>::max()    // I_dot, inclination rate
};

pnt_integrity::EphemerisParameters upperBoundDefaults = {
  32,                                      // prn
  pnt_integrity::AlertFlag::ALERT_RAISED,  // alert flag
  pnt_integrity::AntiSpoofFlag::On,        // as flag
  std::numeric_limits<uint32_t>::max(),    // towSf1
  std::numeric_limits<uint32_t>::max(),    // towM1
  std::numeric_limits<uint16_t>::max(),    // week number
  pnt_integrity::L2CodeType::CACodeOn,     // code on L2
  15,                                      // uraIndex
  pnt_integrity::SVHealth(),               // sv health
  std::numeric_limits<uint16_t>::max(),    // iodc
  pnt_integrity::L2NavDataFlag::Off,       // l2p data flag
  std::numeric_limits<double>::max(),      // group delay
  604784,                                  // clock correction time
  std::numeric_limits<double>::max(),      // clock aging 3
  std::numeric_limits<double>::max(),      // clock aging 2
  std::numeric_limits<double>::max(),      // clock aging 1
  std::numeric_limits<double>::max(),      // inPhase Inter Signal Correction;
  std::numeric_limits<double>::max(),    // quadrature Inter Signal Correction;
  std::numeric_limits<uint32_t>::max(),  // towSf2
  std::numeric_limits<uint32_t>::max(),  // towM2
  std::numeric_limits<uint16_t>::max(),  // iodeSf2
  std::numeric_limits<double>::max(),    // C_rs, sin of orbit radius
  std::numeric_limits<double>::max(),    // Delta N, mean motion difference
  std::numeric_limits<double>::max(),    // mean motion difference rate
  std::numeric_limits<double>::max(),    // M_0, mean anomaly
  std::numeric_limits<double>::max(),    // C_uc, cosine of latitude
  3.0e-2,                                // e, eccentricity
  std::numeric_limits<double>::max(),    // C_us, sine of latitude
  8192,                                  // sqrtA, sqrt of semi major
  std::numeric_limits<double>::max(),    // DeltaA, semi major axis difference
  std::numeric_limits<double>::max(),    // Adot, semi major axis rate
  604784,                                // toe, time of ephemeris
  pnt_integrity::FitInterval::GreaterThanFourHrs,  // fit interval
  std::numeric_limits<uint16_t>::max(),            // AODO, age of data offset
  std::numeric_limits<uint32_t>::max(),            // towSf3
  std::numeric_limits<uint32_t>::max(),            // towM3
  std::numeric_limits<double>::max(),    // C_ic, cosine of inclination
  std::numeric_limits<double>::max(),    // Omega_0, right ascension
  std::numeric_limits<double>::max(),    // right ascension rate difference
  std::numeric_limits<double>::max(),    // C_is, sine of inclination
  std::numeric_limits<double>::max(),    // i_0, inclination angle
  std::numeric_limits<double>::max(),    // C_rc, cosine of orbit rad
  std::numeric_limits<double>::max(),    // omega, argument of perigre
  0,                                     // Omega_dot, ascension rate
  std::numeric_limits<uint16_t>::max(),  // iodeSf3
  std::numeric_limits<double>::max()     // I_dot, inclination rate
};

std::pair<pnt_integrity::EphemerisParameters,
          pnt_integrity::EphemerisParameters>
  createDefaultBounds(lowerBoundDefaults, upperBoundDefaults);
}  // namespace

namespace pnt_integrity
{
// Initialize static member variable
std::pair<EphemerisParameters, EphemerisParameters> GpsEphemeris::bounds_ =
  createDefaultBounds;

//-----------------------------------------------------------------------------
GpsEphemeris::GpsEphemeris()
  : prn_(0)
  , alertFlag_(AlertFlag::ALERT_OFF)
  , asFlag_(AntiSpoofFlag::On)
  , towSf1_(std::numeric_limits<uint32_t>::max())
  , weekNumber_(0)
  , codeOnL2_(L2CodeType::CACodeOn)
  , uraIndex_(0)
  , iodc_(std::numeric_limits<uint16_t>::max())
  , l2pDataFlag_(L2NavDataFlag::On)
  , groupDelay_(0.0)
  , clockCorrectionTime_(0.0)
  , clockAging3_(0.0)
  , clockAging2_(0.0)
  , clockAging1_(0.0)
  , towSf2_(std::numeric_limits<uint32_t>::max())
  , iodeSf2_(std::numeric_limits<uint16_t>::max())
  , sinOrbitRadius_(0.0)
  , meanMotionDifference_(0.0)
  , meanAnomaly_(0.0)
  , cosLatitude_(0.0)
  , eccentricity_(0.0)
  , sinLatitude_(0.0)
  , sqrtSemiMajorAxis_(0.0)
  , timeOfEphemeris_(0.0)
  , fitInterval_(FitInterval::FourHrs)
  , ageOfDataOffset_(0)
  , towSf3_(std::numeric_limits<uint32_t>::max())
  , cosInclination_(0.0)
  , rightAscension_(0.0)
  , sinInclination_(0.0)
  , inclinationAngle_(0.0)
  , cosOrbitRadius_(0.0)
  , argumentOfPerigee_(0.0)
  , ascensionRate_(0.0)
  , iodeSf3_(std::numeric_limits<uint16_t>::max())
  , inclinationRate_(0.0)
  , subframe1Valid_(false)
  , subframe2Valid_(false)
  , subframe3Valid_(false)
  , subframe1_()
  , subframe2_()
  , subframe3_()
{
  clearSubframeFaults();
}

//-----------------------------------------------------------------------------
GpsEphemeris::GpsEphemeris(const uint16_t&      prn,
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
                           bool                 checkForValidity)
  : prn_(prn)
  , alertFlag_(alertFlag)
  , asFlag_(asFlag)
  , towSf1_(towSf1)
  , weekNumber_(weekNumber)
  , codeOnL2_(codeOnL2)
  , uraIndex_(uraIndex)
  , svHealth_(svHealth)
  , iodc_(iodc)
  , l2pDataFlag_(l2PDataFlag)
  , groupDelay_(groupDelay)
  , clockCorrectionTime_(clockCorrectionTime)
  , clockAging3_(clockAging3)
  , clockAging2_(clockAging2)
  , clockAging1_(clockAging1)
  , towSf2_(towSf2)
  , iodeSf2_(iodeSf2)
  , sinOrbitRadius_(sinOrbitRadius)
  , meanMotionDifference_(meanMotionDifference)
  , meanAnomaly_(meanAnomaly)
  , cosLatitude_(cosLatitude)
  , eccentricity_(eccentricity)
  , sinLatitude_(sinLatitude)
  , sqrtSemiMajorAxis_(sqrtSemiMajorAxis)
  , timeOfEphemeris_(timeOfEphemeris)
  , fitInterval_(fitInterval)
  , ageOfDataOffset_(ageOfDataOffset)
  , towSf3_(towSf3)
  , cosInclination_(cosInclination)
  , rightAscension_(rightAscension)
  , sinInclination_(sinInclination)
  , inclinationAngle_(inclinationAngle)
  , cosOrbitRadius_(cosOrbitRadius)
  , argumentOfPerigee_(argumentOfPerigee)
  , ascensionRate_(ascensionRate)
  , iodeSf3_(iodeSf3)
  , inclinationRate_(inclinationRate)
  , subframe1Valid_(false)
  , subframe2Valid_(false)
  , subframe3Valid_(false)
  , subframe1_()
  , subframe2_()
  , subframe3_()
{
  checkIssueNumber();

  generateSubframe1();
  generateSubframe2();
  generateSubframe3();

  if (checkForValidity)
  {
    checkSubframesForFaults();
  }

}  // End GpsEphemeris::GpsEphemeris()

//-----------------------------------------------------------------------------
GpsEphemeris::GpsEphemeris(uint16_t prn,
                           const uint8_t (&subframe1)[30],
                           const uint8_t (&subframe2)[30],
                           const uint8_t (&subframe3)[30],
                           bool checkForValidity)
{
  std::memcpy(&subframe1_, &subframe1, sizeof(subframe1));
  std::memcpy(&subframe2_, &subframe2, sizeof(subframe2));
  std::memcpy(&subframe3_, &subframe3, sizeof(subframe3));

  prn_ = prn;
  parseSubframe1(subframe1_, checkForValidity);
  parseSubframe2(subframe2_, checkForValidity);
  parseSubframe3(subframe3_, checkForValidity);
}

//-----------------------------------------------------------------------------
GpsEphemeris::GpsEphemeris(uint16_t prn,
                           const uint32_t (&subframe1)[10],
                           const uint32_t (&subframe2)[10],
                           const uint32_t (&subframe3)[10],
                           bool checkForValidity)
{
  prn_ = prn;

  convertSubframeFrom10To30Word(subframe1, subframe1_);
  convertSubframeFrom10To30Word(subframe2, subframe2_);
  convertSubframeFrom10To30Word(subframe3, subframe3_);

  parseSubframe1(subframe1_, checkForValidity);
  parseSubframe2(subframe2_, checkForValidity);
  parseSubframe3(subframe3_, checkForValidity);
}

bool GpsEphemeris::getSvState(const double& receiveTime,
                              double&       positionEcefX,
                              double&       positionEcefY,
                              double&       positionEcefZ,
                              double&       velocityEcefX,
                              double&       velocityEcefY,
                              double&       velocityEcefZ,
                              double&       svClockCorrection,
                              const double& pseudorange) const
{
  double semiMajorAxis = sqrtSemiMajorAxis_ * sqrtSemiMajorAxis_;

  double transitTime = pseudorange / speedOfLight;

  double transmitTime = receiveTime - transitTime;

  double dt = weekCrossoverCheck(transmitTime - clockCorrectionTime_);

  // Compute relativistic correction
  // TOOD: why is this 0?
  double dtr = 0.0;  // eFsqrtA_ * sin(E);

  double dtClkModel =
    (clockAging3_ * dt + clockAging2_) * dt + clockAging1_ - groupDelay_;

  svClockCorrection = dtClkModel + dtr;

  double time = transmitTime - svClockCorrection;

  // Time from reference epoch
  double tk = weekCrossoverCheck(time - timeOfEphemeris_);

  if (semiMajorAxis == 0.0)
  {
    positionEcefX     = 0;
    positionEcefY     = 0;
    positionEcefZ     = 0;
    velocityEcefX     = 0;
    velocityEcefY     = 0;
    velocityEcefZ     = 0;
    svClockCorrection = 0;
    return false;
  }

  // Mean anomaly
  double correctedMeanMotion =
    sqrt(gpsGM / pow(semiMajorAxis, 3.0)) + meanMotionDifference_;

  double M = meanAnomaly_ + correctedMeanMotion * tk;

  M = fmod(M + twoGpsPi, twoGpsPi);

  // Initial guess of the eccentric anomaly
  double E    = M;
  double sinE = 0.0;
  double oldE = E;

  unsigned int kk = 0;
  do
  {
    ++kk;
    oldE = E;
    E    = M + eccentricity_ * sin(E);
  } while ((fabs(E - oldE) > 1e-12) && kk < 10);

  E = fmod(E + twoGpsPi, twoGpsPi);

  double cosE       = cos(E);
  sinE              = sin(E);
  double eFsqrtA    = eccentricity_ * gpsF * sqrtSemiMajorAxis_;
  dtr               = eFsqrtA * sinE;
  svClockCorrection = dtClkModel + dtr;

  // True anomaly
  double trueAnomaly = atan2(sqrt(1.0 - eccentricity_ * eccentricity_) * sinE,
                             cosE - eccentricity_);

  // Arguement of latitude
  double argOfLatitude =
    fmod(trueAnomaly + argumentOfPerigee_ + twoGpsPi, twoGpsPi);

  double twoArgOfLatitude = 2.0 * argOfLatitude;

  double cos2p = cos(twoArgOfLatitude);
  double sin2p = sin(twoArgOfLatitude);

  // Corrected argument of latitude
  double u = argOfLatitude + cosLatitude_ * cos2p + sinLatitude_ * sin2p;

  // Corrected radius
  double r = semiMajorAxis * (1 - eccentricity_ * cosE) +
             cosOrbitRadius_ * cos2p + sinOrbitRadius_ * sin2p;

  // Corrected inclination
  double i = inclinationAngle_ + inclinationRate_ * tk +
             cosInclination_ * cos2p + sinInclination_ * sin2p;

  double lonAscNodeDot = ascensionRate_ - gpsEarthRotationRate;

  // Angle between ascending node and Greenwich meridian
  double longitudeOfAscendingNode =
    rightAscension_ + lonAscNodeDot * tk -
    gpsEarthRotationRate * (timeOfEphemeris_ + transitTime);

  // Corrected longitude of ascending node
  longitudeOfAscendingNode =
    fmod(longitudeOfAscendingNode + twoGpsPi, twoGpsPi);

  double xOrbitalPlane = r * cos(u);
  double yOrbitalPlane = r * sin(u);
  double cosi          = cos(i);
  double sini          = sin(i);
  double coso          = cos(longitudeOfAscendingNode);
  double sino          = sin(longitudeOfAscendingNode);

  // ECEF SV position
  positionEcefX = xOrbitalPlane * coso - yOrbitalPlane * cosi * sino;
  positionEcefY = xOrbitalPlane * sino + yOrbitalPlane * cosi * coso;
  positionEcefZ = yOrbitalPlane * sini;

  // TODO: check this
  // satClkCorr = (a_f2 * dt + a_f1) * dt + a_f0 - T_GD + dtr;

  double cosu = cos(u);
  double sinu = sin(u);

  // TODO: check this
  // double cos2u = cos(2.0 * u);
  // double sin2u = sin(2.0 * u);

  if ((1.0 - eccentricity_ * cosE) == 0.0)
  {
    positionEcefX     = 0;
    positionEcefY     = 0;
    positionEcefZ     = 0;
    velocityEcefX     = 0;
    velocityEcefY     = 0;
    velocityEcefZ     = 0;
    svClockCorrection = 0;
    return false;
  }
  double EDot = correctedMeanMotion / (1.0 - eccentricity_ * cosE);

  double argLatDot =
    EDot * sqrt(1 - eccentricity_ * eccentricity_) / (1 - eccentricity_ * cosE);

  double twoArgLatDot = 2.0 * argLatDot;

  double uDot =
    twoArgLatDot * (sinLatitude_ * cos2p - cosLatitude_ * sin2p) + argLatDot;

  double rDot =
    twoArgLatDot * (sinOrbitRadius_ * cos2p - cosOrbitRadius_ * sin2p) +
    (semiMajorAxis * eccentricity_ * sinE * EDot);

  double iDot =
    twoArgLatDot * (sinInclination_ * cos2p - cosInclination_ * sin2p) +
    inclinationRate_;

  double vxOrbitalPlane = rDot * cosu - r * sinu * uDot;
  double vyOrbitalPlane = rDot * sinu + r * cosu * uDot;

  velocityEcefX = vxOrbitalPlane * coso - vyOrbitalPlane * cosi * sino +
                  yOrbitalPlane * sini * sino * iDot -
                  positionEcefY * lonAscNodeDot;

  velocityEcefY = vxOrbitalPlane * sino + vyOrbitalPlane * cosi * coso -
                  yOrbitalPlane * sini * coso * iDot +
                  positionEcefX * lonAscNodeDot;

  velocityEcefZ = vyOrbitalPlane * sini + yOrbitalPlane * cosi * iDot;

  return true;

}  // End GpsEphemeris::getSvState()

//-----------------------------------------------------------------------------
bool GpsEphemeris::setEphemeris(const uint16_t&      prn,
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
                                bool                 checkForValidity)
{
  prn_                  = prn;
  towSf1_               = towSf1;
  alertFlag_            = alertFlag;
  asFlag_               = asFlag;
  weekNumber_           = weekNumber;
  codeOnL2_             = codeOnL2;
  uraIndex_             = uraIndex;
  svHealth_             = svHealth;
  iodc_                 = iodc;
  l2pDataFlag_          = l2PDataFlag;
  groupDelay_           = groupDelay;
  clockCorrectionTime_  = clockCorrectionTime;
  clockAging3_          = clockAging3;
  clockAging2_          = clockAging2;
  clockAging1_          = clockAging1;
  towSf2_               = towSf2;
  iodeSf2_              = iodeSf2;
  sinOrbitRadius_       = sinOrbitRadius;
  meanMotionDifference_ = meanMotionDifference;
  meanAnomaly_          = meanAnomaly;
  cosLatitude_          = cosLatitude;
  eccentricity_         = eccentricity;
  sinLatitude_          = sinLatitude;
  sqrtSemiMajorAxis_    = sqrtSemiMajorAxis;
  timeOfEphemeris_      = timeOfEphemeris;
  fitInterval_          = fitInterval;
  ageOfDataOffset_      = ageOfDataOffset;
  towSf3_               = towSf3;
  cosInclination_       = cosInclination;
  rightAscension_       = rightAscension;
  sinInclination_       = sinInclination;
  inclinationAngle_     = inclinationAngle;
  cosOrbitRadius_       = cosOrbitRadius;
  argumentOfPerigee_    = argumentOfPerigee;
  ascensionRate_        = ascensionRate;
  iodeSf3_              = iodeSf3;
  inclinationRate_      = inclinationRate;

  generateSubframe1();
  generateSubframe2();
  generateSubframe3();

  if (checkForValidity)
  {
    return checkSubframesForFaults();
  }
  else
  {
    subframe1Valid_ = true;
    subframe2Valid_ = true;
    subframe3Valid_ = true;
    return true;
  }
}

bool GpsEphemeris::setEphemeris(const EphemerisParameters& param,
                                bool                       checkForValidity)
{
  return setEphemeris(param.prn,
                      param.alertFlag,
                      param.asFlag,
                      param.towSf1,
                      param.weekNumber,
                      param.codeOnL2,
                      param.uraIndex,
                      param.svHealth,
                      param.iodc,
                      param.l2PDataFlag,
                      param.groupDelay,
                      param.clockCorrectionTime,
                      param.clockAging3,
                      param.clockAging2,
                      param.clockAging1,
                      param.towSf2,
                      param.iodeSf2,
                      param.sinOrbitRadius,
                      param.meanMotionDifference,
                      param.meanAnomaly,
                      param.cosLatitude,
                      param.eccentricity,
                      param.sinLatitude,
                      param.sqrtSemiMajorAxis,
                      param.timeOfEphemeris,
                      param.fitInterval,
                      param.ageOfDataOffset,
                      param.towSf3,
                      param.cosInclination,
                      param.rightAscension,
                      param.sinInclination,
                      param.inclinationAngle,
                      param.cosOrbitRadius,
                      param.argumentOfPerigee,
                      param.ascensionRate,
                      param.iodeSf3,
                      param.inclinationRate,
                      checkForValidity);
}

//-----------------------------------------------------------------------------
bool GpsEphemeris::getEphemeris(uint16_t&      prn,
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
                                double&        inclinationRate) const
{
  prn         = prn_;
  alertFlag   = alertFlag_;
  asFlag      = asFlag_;
  towSf1      = towSf1_;
  weekNumber  = weekNumber_;
  codeOnL2    = codeOnL2_;
  uraIndex    = uraIndex_;
  svHealth    = svHealth_;
  iodc        = iodc_;
  l2PDataFlag = l2pDataFlag_, groupDelay = groupDelay_;
  clockCorrectionTime  = clockCorrectionTime_;
  clockAging3          = clockAging3_;
  clockAging2          = clockAging2_;
  clockAging1          = clockAging1_;
  towSf2               = towSf2_;
  iodeSf2              = iodeSf2_;
  sinOrbitRadius       = sinOrbitRadius_;
  meanMotionDifference = meanMotionDifference_;
  meanAnomaly          = meanAnomaly_;
  cosLatitude          = cosLatitude_;
  eccentricity         = eccentricity_;
  sinLatitude          = sinLatitude_;
  sqrtSemiMajorAxis    = sqrtSemiMajorAxis_;
  timeOfEphemeris      = timeOfEphemeris_;
  fitInterval          = fitInterval_;
  ageOfDataOffset      = ageOfDataOffset_;
  towSf3               = towSf3_;
  cosInclination       = cosInclination_;
  rightAscension       = rightAscension_;
  sinInclination       = sinInclination_;
  inclinationAngle     = inclinationAngle_;
  cosOrbitRadius       = cosOrbitRadius_;
  argumentOfPerigee    = argumentOfPerigee_;
  ascensionRate        = ascensionRate_;
  iodeSf3              = iodeSf3_;
  inclinationRate      = inclinationRate_;

  return (subframe1Valid_ && subframe2Valid_ && subframe3Valid_);
}

//-----------------------------------------------------------------------------
EphemerisParameters GpsEphemeris::getEphemeris() const
{
  EphemerisParameters param;

  param.prn                  = prn_;
  param.towSf1               = towSf1_;
  param.alertFlag            = alertFlag_;
  param.asFlag               = asFlag_;
  param.weekNumber           = weekNumber_;
  param.codeOnL2             = codeOnL2_;
  param.uraIndex             = uraIndex_;
  param.iodc                 = iodc_;
  param.groupDelay           = groupDelay_;
  param.clockCorrectionTime  = clockCorrectionTime_;
  param.clockAging3          = clockAging3_;
  param.clockAging2          = clockAging2_;
  param.clockAging1          = clockAging1_;
  param.towSf2               = towSf2_;
  param.iodeSf2              = iodeSf2_;
  param.sinOrbitRadius       = sinOrbitRadius_;
  param.meanMotionDifference = meanMotionDifference_;
  param.meanAnomaly          = meanAnomaly_;
  param.cosLatitude          = cosLatitude_;
  param.eccentricity         = eccentricity_;
  param.sinLatitude          = sinLatitude_;
  param.sqrtSemiMajorAxis    = sqrtSemiMajorAxis_;
  param.timeOfEphemeris      = timeOfEphemeris_;
  param.fitInterval          = fitInterval_;
  param.ageOfDataOffset      = ageOfDataOffset_;
  param.towSf3               = towSf3_;
  param.cosInclination       = cosInclination_;
  param.rightAscension       = rightAscension_;
  param.sinInclination       = sinInclination_;
  param.inclinationAngle     = inclinationAngle_;
  param.cosOrbitRadius       = cosOrbitRadius_;
  param.argumentOfPerigee    = argumentOfPerigee_;
  param.ascensionRate        = ascensionRate_;
  param.iodeSf3              = iodeSf3_;
  param.inclinationRate      = inclinationRate_;

  return (param);
}

//-----------------------------------------------------------------------------
bool GpsEphemeris::setSubframe(const uint16_t& prn,
                               const uint8_t (&subframe)[30],
                               bool checkForValidity)
{
  if (parseSubframeID(subframe) == 1)
  {
    prn_ = prn;
    memcpy(subframe1_, subframe, 30);
    return parseSubframe1(subframe, checkForValidity);
  }
  else if (parseSubframeID(subframe) == 2)
  {
    prn_ = prn;
    memcpy(subframe2_, subframe, 30);
    return parseSubframe2(subframe, checkForValidity);
  }
  else if (parseSubframeID(subframe) == 3)
  {
    prn_ = prn;
    memcpy(subframe3_, subframe, 30);
    return parseSubframe3(subframe, checkForValidity);
  }
  else
  {
    return false;
  }
}

//-----------------------------------------------------------------------------
bool GpsEphemeris::setSubframes(const uint16_t prn,
                                const uint8_t (&subframe1)[30],
                                const uint8_t (&subframe2)[30],
                                const uint8_t (&subframe3)[30],
                                bool checkForValidity)
{
  return (setSubframe(prn, subframe1, checkForValidity) &&
          setSubframe(prn, subframe2, checkForValidity) &&
          setSubframe(prn, subframe3, checkForValidity));
}

int GpsEphemeris::checkSubframeIssueDate(const uint8_t (&subframe)[30])
{
  // get the subframe id
  uint16_t subframeId = parseSubframeID(subframe);

  // parse iode value from the new subframe
  uint8_t newIode = parseEphemerisIode(subframe);

  // determine iode value to compare against
  uint8_t curIode = 0;

  if (subframeId == 1)
  {
    if (subframe3Valid_)
    {
      curIode = iodeSf3_;
    }
    else if (subframe2Valid_)
    {
      curIode = iodeSf2_;
    }
  }
  else if (subframeId == 2)
  {
    if (subframe1Valid_)
    {
      curIode = iodc_ & 0xFF;
    }
    else if (subframe3Valid_)
    {
      curIode = iodeSf3_;
    }
  }
  else if (subframeId == 3)
  {
    if (subframe2Valid_)
    {
      curIode = iodeSf2_;
    }
    else if (subframe1Valid_)
    {
      curIode = iodc_ & 0xFF;
    }
  }

  int16_t iodeDiff = (int16_t)newIode - (int16_t)curIode;

  if ((iodeDiff > 0) || (iodeDiff < -128))  // handle wrap
    return 2;                               // subframe is newer
  else if (iodeDiff < 0)
    return 1;  // subframe is older
  else
    return 0;  // subframe is same
}

void GpsEphemeris::setBounds(
  const std::pair<EphemerisParameters, EphemerisParameters>& bounds)
{
  GpsEphemeris::bounds_ = bounds;
}

std::pair<EphemerisParameters, EphemerisParameters> GpsEphemeris::getBounds()
{
  return (GpsEphemeris::bounds_);
}

void GpsEphemeris::getSubframe1(uint8_t (&subframe)[30])
{
  memcpy(subframe, &subframe1_, 30);
}

void GpsEphemeris::getSubframe2(uint8_t (&subframe)[30])
{
  memcpy(subframe, &subframe2_, 30);
}

void GpsEphemeris::getSubframe3(uint8_t (&subframe)[30])
{
  memcpy(subframe, &subframe3_, 30);
}

//-----------------------------------------------------------------------------
std::string GpsEphemeris::sf1FaultsToString() const
{
  std::stringstream out;
  out << "PRN: " << prn_ << std::endl;
  out << "Subframe 1. Fault: " << !subframe1Valid_ << std::endl;
  out << "  TOW: " << subframe1Fault_.towSf1 << std::endl;
  out << "  Week #: " << subframe1Fault_.weekNumber << std::endl;
  // out << "  AS Flag: " << static_cast<int>(asFlag_) << std::endl;
  out << "  Code on L2: " << subframe1Fault_.codeOnL2 << std::endl;
  out << "  URA Index: " << subframe1Fault_.uraIndex << std::endl;
  out << "  SV Health: " << subframe1Fault_.svHealth << std::endl;
  out << "  IODC: " << subframe1Fault_.iodc << std::endl;
  out << "  L2P Data Flag: " << subframe1Fault_.l2PDataFlag << std::endl;
  out << "  Group Delay: " << subframe1Fault_.groupDelay << std::endl;
  out << "  Toc: " << subframe1Fault_.clockCorrectionTime << std::endl;
  out << "  AF0: " << subframe1Fault_.clockAging1 << std::endl;
  out << "  AF1: " << subframe1Fault_.clockAging2 << std::endl;
  out << "  AF2: " << subframe1Fault_.clockAging3 << std::endl;

  return out.str();
}

std::string GpsEphemeris::sf2FaultsToString() const
{
  std::stringstream out;

  out << "Subframe 2. Fault: " << !subframe2Valid_ << std::endl;
  out << "  TOW: " << subframe2Fault_.towSf2 << std::endl;
  out << "  IODE SF2: " << subframe2Fault_.iodeSf2 << std::endl;
  out << "  Crs: " << subframe2Fault_.sinOrbitRadius << std::endl;
  out << "  Delta N: " << subframe2Fault_.meanMotionDifference << std::endl;
  out << "  M0: " << subframe2Fault_.meanAnomaly << std::endl;
  out << "  Cuc: " << subframe2Fault_.cosLatitude << std::endl;
  out << "  Cus: " << subframe2Fault_.sinLatitude << std::endl;
  out << "  sqrtA: " << subframe2Fault_.sqrtSemiMajorAxis << std::endl;
  out << "  TOE: " << subframe2Fault_.timeOfEphemeris << std::endl;
  out << "  Fit Interval: " << subframe2Fault_.fitInterval << std::endl;
  out << "  Age of data offset: " << subframe2Fault_.ageOfDataOffset
      << std::endl;

  return out.str();
}

std::string GpsEphemeris::sf3FaultsToString() const
{
  std::stringstream out;

  out << "Subframe 3. Fault: " << !subframe3Valid_ << std::endl;
  out << "  TOW: " << subframe3Fault_.towSf3 << std::endl;
  out << "  IODE SF3: " << subframe3Fault_.iodeSf3 << std::endl;
  out << "  Cic: " << subframe3Fault_.cosInclination << std::endl;
  out << "  Omega0: " << subframe3Fault_.rightAscension << std::endl;
  out << "  Cis: " << subframe3Fault_.sinInclination << std::endl;
  out << "  i0: " << subframe3Fault_.inclinationAngle << std::endl;
  out << "  Crc: " << subframe3Fault_.cosOrbitRadius << std::endl;
  out << "  w: " << subframe3Fault_.argumentOfPerigee << std::endl;
  out << "  Omega Dot: " << subframe3Fault_.ascensionRate << std::endl;
  out << "  i Dot: " << subframe3Fault_.inclinationRate << std::endl;

  return out.str();
}

//-----------------------------------------------------------------------------
std::string GpsEphemeris::toString() const
{
  std::stringstream out;
  out << "PRN: " << prn_ << std::endl;
  out << "Subframe 1:" << std::endl;
  out << "  TOW: " << towSf1_ << std::endl;
  out << "  Week #: " << weekNumber_ << std::endl;
  out << "  AS Flag: " << static_cast<int>(asFlag_) << std::endl;
  out << "  Code on L2: " << static_cast<int>(codeOnL2_) << std::endl;
  out << "  URA Index: " << uraIndex_ << std::endl;
  out << "  SV Health: " << static_cast<int>(svHealth_.signalHealth)
      << std::endl;
  out << "  IODC: " << iodc_ << std::endl;
  out << "  L2P Data Flag: " << static_cast<int>(l2pDataFlag_) << std::endl;
  out << "  Group Delay: " << groupDelay_ << std::endl;
  out << "  Toc: " << clockCorrectionTime_ << std::endl;
  out << "  AF0: " << clockAging1_ << std::endl;
  out << "  AF1: " << clockAging2_ << std::endl;
  out << "  AF2: " << clockAging3_ << std::endl;

  out << "Subframe 2:" << std::endl;
  out << "  TOW: " << towSf2_ << std::endl;
  out << "  IODE SF2: " << iodeSf2_ << std::endl;
  out << "  Crs: " << sinOrbitRadius_ << std::endl;
  out << "  Delta N: " << meanMotionDifference_ << std::endl;
  out << "  M0: " << meanAnomaly_ << std::endl;
  out << "  Cuc: " << cosLatitude_ << std::endl;
  out << "  Cus: " << sinLatitude_ << std::endl;
  out << "  sqrtA: " << sqrtSemiMajorAxis_ << std::endl;
  out << "  TOE: " << timeOfEphemeris_ << std::endl;
  out << "  Fit Interval: " << static_cast<int>(fitInterval_) << std::endl;
  out << "  Age of data offset: " << ageOfDataOffset_ << std::endl;

  out << "Subframe 3:" << std::endl;
  out << "  TOW: " << towSf3_ << std::endl;
  out << "  IODE SF3: " << iodeSf3_ << std::endl;
  out << "  Cic: " << cosInclination_ << std::endl;
  out << "  Omega0: " << rightAscension_ << std::endl;
  out << "  Cis: " << sinInclination_ << std::endl;
  out << "  i0: " << inclinationAngle_ << std::endl;
  out << "  Crc: " << cosOrbitRadius_ << std::endl;
  out << "  w: " << argumentOfPerigee_ << std::endl;
  out << "  Omega Dot: " << ascensionRate_ << std::endl;
  out << "  i Dot: " << inclinationRate_ << std::endl;

  return out.str();
}

std::string GpsEphemeris::toHexString() const
{
  std::stringstream out;
  out << "PRN = " << prn_ << std::endl
      << sf1ToHexString() << std::endl
      << sf2ToHexString() << std::endl
      << sf3ToHexString() << std::endl;

  return out.str();
}

std::string GpsEphemeris::sf1ToHexString() const
{
  std::string subframe1;
  toHex((unsigned char*)&subframe1_, 30, subframe1);

  return "Subframe 1: 0x" + subframe1;
}

std::string GpsEphemeris::sf2ToHexString() const
{
  std::string subframe2;
  toHex((unsigned char*)&subframe2_, 30, subframe2);

  return "Subframe 2: 0x" + subframe2;
}

std::string GpsEphemeris::sf3ToHexString() const
{
  std::string subframe3;
  toHex((unsigned char*)&subframe3_, 30, subframe3);

  return "Subframe 3: 0x" + subframe3;
}

//-----------------------------------------------------------------------------
double GpsEphemeris::weekCrossoverCheck(const double& time) const
{
  // checkTime accounting for beginning or end of week crossover.
  double corrTime = time;
  if (time > secondsInHalfWeek)
  {
    corrTime = time - secondsInWeek;
  }
  else if (time < -secondsInHalfWeek)
  {
    corrTime = time + secondsInWeek;
  }
  return (corrTime);

}  // end check_t

void GpsEphemeris::generateSubframes()
{
  generateSubframe1();
  generateSubframe2();
  generateSubframe3();
}

void GpsEphemeris::generateSubframe1()
{
  // Subframe id
  int sfID1 = 1;
  // TOOD: this doesn't look correct - overwritten by temptow??
  subframe1_[5] = sfID1 << 2;

  // TOW into new subframe
  uint32_t temptow = towSf1_ * 2 / 3;
  subframe1_[3]    = temptow >> 11;
  subframe1_[4]    = temptow >> 3;
  subframe1_[5] |= temptow << 5;

  // AS flag into new subframe
  subframe1_[5] |= static_cast<uint8_t>(asFlag_) << 5;

  // L2P Data Flag into new subframe
  subframe1_[9] = static_cast<uint8_t>(l2pDataFlag_) << 7;

  // Week Number into new subframe
  subframe1_[6] = weekNumber_ >> 2;
  subframe1_[7] = weekNumber_ << 6;

  // Code on L2 into new subframe
  subframe1_[7] |= static_cast<uint8_t>(codeOnL2_) << 4;

  // URA Index into new subframe
  subframe1_[7] |= uraIndex_;

  // SV Health into new subframe
  // subframe1_[8] = ((int)svHealth_) << 2;

  // IODC into new subframe
  subframe1_[21] = iodc_;
  subframe1_[8] |= iodc_ >> 8;

  // Tgd into new subframe
  subframe1_[20] = groupDelay_ * exp2(31);

  // Toc into new subframe
  uint16_t temptoc = clockCorrectionTime_ / 16;
  // uint8_t temptocMSB = temptoc >> 8;
  // uint8_t temptocLSB = temptoc;
  subframe1_[22] = temptoc >> 8;
  subframe1_[23] = temptoc;

  // af2 into new subframe
  subframe1_[24] = clockAging3_ * exp2(55.0);

  // af1 into new subframe
  uint16_t tempaf1 = clockAging2_ * exp2(43);
  // uint8_t tempaf1MSB = tempaf1 >> 8;
  // uint8_t tempaf1LSB = tempaf1;
  subframe1_[25] = tempaf1 >> 8;
  subframe1_[26] = tempaf1;

  // af0 into new subframe
  uint32_t tempaf0 = clockAging1_ * exp2(31);
  subframe1_[27]   = tempaf0 >> 14;
  subframe1_[28]   = tempaf0 >> 6;
  subframe1_[29]   = tempaf0 << 2;
}

void GpsEphemeris::generateSubframe2()
{
  // TOW
  uint32_t temptow2 = towSf2_ * 2 / 3;
  subframe2_[3]     = temptow2 >> 11;
  subframe2_[4]     = temptow2 >> 3;
  subframe2_[5]     = temptow2 << 5;

  // Subframe id
  int sfID2 = 2;
  subframe2_[5] |= sfID2 << 2;

  // AS Flag
  subframe2_[5] |= static_cast<uint8_t>(asFlag_) << 5;

  // IODE
  subframe2_[6] = iodeSf2_;

  // CRS
  uint16_t crsint = sinOrbitRadius_ * exp2(5);
  subframe2_[7]   = crsint >> 8;
  subframe2_[8]   = crsint;

  // Delta n
  uint16_t deltaNint = meanMotionDifference_ * exp2(43) / gpsPi;
  subframe2_[9]      = deltaNint >> 8;
  subframe2_[10]     = deltaNint;

  // M0
  uint32_t m0int = meanAnomaly_ * exp2(31) / gpsPi;
  subframe2_[11] = m0int >> 24;
  subframe2_[12] = m0int >> 16;
  subframe2_[13] = m0int >> 8;
  subframe2_[14] = m0int;
  // Cuc
  uint16_t cucint = cosLatitude_ * exp2(29);
  subframe2_[15]  = cucint >> 8;
  subframe2_[16]  = cucint;

  // Eccentricity
  uint32_t eccint = eccentricity_ * exp2(33);
  subframe2_[17]  = eccint >> 24;
  subframe2_[18]  = eccint >> 16;
  subframe2_[19]  = eccint >> 8;
  subframe2_[20]  = eccint;

  // Cus
  uint16_t cusint = sinLatitude_ * exp2(29);
  subframe2_[21]  = cusint >> 8;
  subframe2_[22]  = cusint;

  // SqrtA
  uint32_t sqrtAint = sqrtSemiMajorAxis_ * exp2(19);
  subframe2_[23]    = sqrtAint >> 24;
  subframe2_[24]    = sqrtAint >> 16;
  subframe2_[25]    = sqrtAint >> 8;
  subframe2_[26]    = sqrtAint;

  // toe
  uint16_t toeint = timeOfEphemeris_ * exp2(-4);
  subframe2_[27]  = toeint >> 8;
  subframe2_[28]  = toeint;

  // Fit Interval
  subframe2_[29] = static_cast<uint8_t>(fitInterval_) << 7;

  // Age of data offset
  subframe2_[29] |= ageOfDataOffset_ / 900 << 2;
}

void GpsEphemeris::generateSubframe3()
{
  // TOW
  uint32_t temptow3 = towSf3_ * 2 / 3;
  subframe3_[3]     = temptow3 >> 11;
  subframe3_[4]     = temptow3 >> 3;
  subframe3_[5]     = temptow3 << 5;

  // Subframe id
  int sfID3 = 3;
  subframe3_[5] |= sfID3 << 2;

  // AS Flag
  subframe3_[5] |= static_cast<uint8_t>(asFlag_) << 5;

  // Cic
  uint16_t cicint = cosInclination_ * exp2(29);
  subframe3_[6]   = cicint >> 8;
  subframe3_[7]   = cicint;

  // Omega0
  uint32_t omega0int = rightAscension_ * exp2(31) / gpsPi;
  subframe3_[8]      = omega0int >> 24;
  subframe3_[9]      = omega0int >> 16;
  subframe3_[10]     = omega0int >> 8;
  subframe3_[11]     = omega0int;

  // Cis
  uint16_t cisint = sinInclination_ * exp2(29);
  subframe3_[12]  = cisint >> 8;
  subframe3_[13]  = cisint;

  // i0
  uint32_t i0int = inclinationAngle_ * exp2(31) / gpsPi;
  subframe3_[14] = i0int >> 24;
  subframe3_[15] = i0int >> 16;
  subframe3_[16] = i0int >> 8;
  subframe3_[17] = i0int;

  // Crc
  uint16_t crcint = cosOrbitRadius_ * exp2(5);
  subframe3_[18]  = crcint >> 8;
  subframe3_[19]  = crcint;

  // w
  uint32_t wInt  = argumentOfPerigee_ * exp2(31) / gpsPi;
  subframe3_[20] = wInt >> 24;
  subframe3_[21] = wInt >> 16;
  subframe3_[22] = wInt >> 8;
  subframe3_[23] = wInt;

  // Omega dot
  uint32_t omegaDotint = ascensionRate_ * exp2(43) / gpsPi;
  subframe3_[24]       = omegaDotint >> 16;
  subframe3_[25]       = omegaDotint >> 8;
  subframe3_[26]       = omegaDotint;

  // IDOE 2
  subframe3_[27] = iodeSf3_;

  // IDOT
  uint16_t iDotInt = inclinationRate_ * exp2(43) / gpsPi;
  subframe3_[28]   = iDotInt >> 6;
  subframe3_[29]   = iDotInt << 2;
}

//-----------------------------------------------------------------------------
bool GpsEphemeris::parseSubframe1(const uint8_t (&subframe1)[30],
                                  bool checkForValidity)
{
  if (parseSubframeID(subframe1) != 1)
  {
    return (false);
  }

  // Temporary storage of bit groupings
  uint32_t tmp   = 0;
  uint16_t tmp16 = 0;

  // Time of week
  towSf1_ = parseTimeOfWeek(subframe1);

  // AS Flag
  asFlag_ = (AntiSpoofFlag)((subframe1[5] & 0x20) >> 5);

  // Week number
  weekNumber_ = (uint16_t)(subframe1[7] >> 6);
  weekNumber_ |= ((uint16_t)subframe1[6]) << 2;

  codeOnL2_ = (L2CodeType)((subframe1[7] & 0x30) >> 4);

  // URA Index
  uraIndex_ = subframe1[7] & 0x0F;

  // SVHealth
  svHealth_ = decodeEphemSVHealthBits((uint8_t)((subframe1[8] & 0xFC) >> 2));

  // IODC
  iodc_ = (uint16_t)((subframe1[8] & 0x03)) << 8;
  iodc_ |= (uint16_t)subframe1[21];

  // L2P flag
  l2pDataFlag_ = (L2NavDataFlag)(subframe1[8] >> 7);

  // TGD - estimated group delay differential
  groupDelay_ = ((int8_t)subframe1[20]) / exp2(31.0);

  // TOC
  tmp16 = ((uint16_t)subframe1[22]) << 8;
  tmp16 |= (uint16_t)subframe1[23];
  clockCorrectionTime_ = tmp16 * 16;

  // af2
  clockAging3_ = ((int8_t)subframe1[24]) / exp2(55.0);

  // af1
  tmp16 = ((uint16_t)subframe1[25]) << 8;
  tmp16 |= (uint16_t)subframe1[26];

  clockAging2_ = ((int16_t)tmp16) / exp2(43.0);

  // af0
  tmp = (uint32_t)(subframe1[29] & 0xFC) << 8;
  tmp |= ((uint32_t)subframe1[28]) << 16;
  tmp |= ((uint32_t)subframe1[27]) << 24;
  clockAging1_ = (((int32_t)tmp) >> 10) / exp2(31.0);

  if (checkForValidity)
  {
    subframe1Valid_ = checkSubframe1();
  }
  else
  {
    subframe1Valid_ = true;
    memset(&subframe1Fault_, 0, sizeof(subframe1Fault_));
  }

  return (subframe1Valid_);
}

//------------------------------------------------------------------------------
bool GpsEphemeris::parseSubframe2(const uint8_t (&subframe2)[30],
                                  bool checkForValidity)
{
  if (parseSubframeID(subframe2) != 2)
  {
    return (false);
  }

  // Temporary storage of bit groupings
  uint32_t tmp32 = 0;
  uint16_t tmp16 = 0;

  // Time of week
  towSf2_ = parseTimeOfWeek(subframe2);
  // iode
  iodeSf2_ = (uint16_t)subframe2[6];

  // Crs
  tmp16 = (uint16_t)subframe2[8];
  tmp16 |= ((uint16_t)subframe2[7]) << 8;

  sinOrbitRadius_ = ((int16_t)tmp16) / 32.0;

  // delta n
  tmp16 = (uint16_t)subframe2[10];
  tmp16 |= ((uint16_t)subframe2[9]) << 8;

  meanMotionDifference_ = ((int16_t)tmp16) * gpsPi / exp2(43.0);

  // M0
  tmp32 = (uint32_t)subframe2[14];
  tmp32 |= ((uint32_t)subframe2[13]) << 8;
  tmp32 |= ((uint32_t)subframe2[12]) << 16;
  tmp32 |= ((uint32_t)subframe2[11]) << 24;

  meanAnomaly_ = ((int32_t)tmp32) * gpsPi / exp2(31.0);

  // Cuc
  tmp16 = (uint16_t)subframe2[16];
  tmp16 |= ((uint16_t)subframe2[15]) << 8;

  cosLatitude_ = ((int16_t)tmp16) / exp2(29.0);

  // eccentricity
  tmp32 = (uint32_t)subframe2[20];
  tmp32 |= ((uint32_t)subframe2[19]) << 8;
  tmp32 |= ((uint32_t)subframe2[18]) << 16;
  tmp32 |= ((uint32_t)subframe2[17]) << 24;

  eccentricity_ = ((int32_t)tmp32) / exp2(33.0);

  // Cus
  tmp16 = (uint16_t)subframe2[22];
  tmp16 |= ((uint16_t)subframe2[21]) << 8;

  sinLatitude_ = ((int16_t)tmp16) / exp2(29.0);

  // sqrtA
  tmp32 = (uint32_t)subframe2[26];
  tmp32 |= ((uint32_t)subframe2[25]) << 8;
  tmp32 |= ((uint32_t)subframe2[24]) << 16;
  tmp32 |= ((uint32_t)subframe2[23]) << 24;

  sqrtSemiMajorAxis_ = (tmp32) / exp2(19.0);

  // toe
  tmp32 = (((uint32_t)subframe2[28]) | (((uint32_t)subframe2[27]) << 8));
  timeOfEphemeris_ = tmp32 * 16.0;

  // fit interval
  fitInterval_ = (FitInterval)(subframe2[29] >> 7);

  // AODO
  ageOfDataOffset_ = ((uint16_t)((subframe2[29] & 0x7C) >> 2)) * 900;

  if (checkForValidity)
  {
    subframe2Valid_ = checkSubframe2();
  }
  else
  {
    subframe2Valid_ = true;
    memset(&subframe2Fault_, 0, sizeof(subframe2Fault_));
  }

  return (subframe2Valid_);
}

//------------------------------------------------------------------------------
bool GpsEphemeris::parseSubframe3(const uint8_t (&subframe3)[30],
                                  bool checkForValidity)
{
  if (parseSubframeID(subframe3) != 3)
  {
    return (false);
  }

  // Temporary storage of bit groupings
  uint32_t tmp32 = 0;
  uint16_t tmp16 = 0;

  // Time of week
  towSf3_ = parseTimeOfWeek(subframe3);

  // Cic
  tmp16 = (uint16_t)subframe3[7];
  tmp16 |= ((uint16_t)subframe3[6] << 8);

  cosInclination_ = ((int16_t)tmp16) / exp2(29.0);

  // Omega0
  tmp32 = (uint32_t)subframe3[11];
  tmp32 |= ((uint32_t)subframe3[10]) << 8;
  tmp32 |= ((uint32_t)subframe3[9]) << 16;
  tmp32 |= ((uint32_t)subframe3[8]) << 24;

  rightAscension_ = ((int32_t)tmp32) * gpsPi / exp2(31.0);

  // Cis
  tmp16 = (uint16_t)subframe3[13];
  tmp16 |= ((uint16_t)subframe3[12] << 8);

  sinInclination_ = ((int16_t)tmp16) / exp2(29.0);

  // i0
  tmp32 = (uint32_t)subframe3[17];
  tmp32 |= ((uint32_t)subframe3[16]) << 8;
  tmp32 |= ((uint32_t)subframe3[15]) << 16;
  tmp32 |= ((uint32_t)subframe3[14]) << 24;

  inclinationAngle_ = ((int32_t)tmp32) * gpsPi / exp2(31.0);

  // Crc
  tmp16 = (uint16_t)subframe3[19];
  tmp16 |= ((uint16_t)subframe3[18] << 8);

  cosOrbitRadius_ = ((int16_t)tmp16) / 32.0;

  // w
  tmp32 = (uint32_t)subframe3[23];
  tmp32 |= ((uint32_t)subframe3[22]) << 8;
  tmp32 |= ((uint32_t)subframe3[21]) << 16;
  tmp32 |= ((uint32_t)subframe3[20]) << 24;

  argumentOfPerigee_ = ((int32_t)tmp32) * gpsPi / exp2(31.0);

  // Omega dot
  tmp32 = subframe3[26] << 8;
  tmp32 |= subframe3[25] << 16;
  tmp32 |= subframe3[24] << 24;
  ascensionRate_ = (((int32_t)tmp32) >> 8) * gpsPi / exp2(43.0);

  // iode
  iodeSf3_ = (uint16_t)subframe3[27];

  // iDot
  tmp16 = ((uint16_t)subframe3[29]) & 0xFC;
  tmp16 |= ((uint16_t)subframe3[28]) << 8;

  inclinationRate_ = (((int16_t)tmp16) >> 2) * gpsPi / exp2(43.0);

  if (checkForValidity)
  {
    subframe3Valid_ = checkSubframe3();
  }
  else
  {
    subframe3Valid_ = true;
    memset(&subframe3Fault_, 0, sizeof(subframe3Fault_));
  }
  return (subframe3Valid_);
}

uint8_t GpsEphemeris::parseEphemerisIode(const uint8_t (&subframe)[30])
{
  uint16_t subframeId = parseSubframeID(subframe);

  if (subframeId == 1)
  {
    return subframe[21];
  }
  else if (subframeId == 2)
  {
    return subframe[6];
  }
  else if (subframeId == 3)
  {
    return subframe[27];
  }
  else
  {
    return 0;
  }
}

//------------------------------------------------------------------------------
SVHealth GpsEphemeris::decodeEphemSVHealthBits(const uint8_t& svHealthBits)
{
  SVHealth svHealth;

  decodeEphemSVHealthBits(svHealthBits, svHealth);

  return (svHealth);

}  // getSVHealth

//------------------------------------------------------------------------------
void GpsEphemeris::decodeEphemSVHealthBits(const uint8_t& svHealthBits,
                                           SVHealth&      svHealth)
{
  // 0x20 = 0b00100000
  // 0x1F = 0b00011111
  uint8_t svNavDatMask = (svHealthBits & 0x20);
  uint8_t svSignalBits = (svHealthBits & 0x1F);

  if ((svHealthBits & svNavDatMask) != 0)
  {
    svHealth.someOrAllNavDataBad = true;
  }
  else
  {
    svHealth.someOrAllNavDataBad = false;
  }

  svHealth.signalHealth = (SVSignalHealth)svSignalBits;
}

//------------------------------------------------------------------------------
void decodeEphemSVHealthBits(const uint8_t& svHealthBits, SVHealth& svHealth)
{
  // 0x20 = 0b00100000
  // 0x1F = 0b00011111
  uint8_t svNavDatMask = (svHealthBits & 0x20);
  uint8_t svSignalBits = (svHealthBits & 0x1F);

  if ((svHealthBits & svNavDatMask) != 0)
  {
    svHealth.someOrAllNavDataBad = true;
  }
  else
  {
    svHealth.someOrAllNavDataBad = false;
  }

  svHealth.signalHealth = (SVSignalHealth)svSignalBits;
}

//-----------------------------------------------------------------------------
void GpsEphemeris::clearSubframeFaults()
{
  memset(&subframe1Fault_, 0, sizeof(subframe1Fault_));
  memset(&subframe2Fault_, 0, sizeof(subframe2Fault_));
  memset(&subframe3Fault_, 0, sizeof(subframe3Fault_));
}

//-----------------------------------------------------------------------------
bool GpsEphemeris::checkIssueNumber()
{
  bool issueNumberMatches = true;

  // Grab 8 LSBs of IODC
  uint8_t iodc8 = iodc_ & 255;

  if (subframe1Valid_ && subframe2Valid_)
  {
    if (iodc8 != iodeSf2_)
    {
      subframe1Valid_         = false;
      subframe2Valid_         = false;
      subframe3Valid_         = false;
      subframe1Fault_.iodc    = 1;
      subframe2Fault_.iodeSf2 = 1;
      issueNumberMatches      = false;
    }
  }

  if (subframe1Valid_ && subframe3Valid_)
  {
    if (iodc8 != iodeSf3_)
    {
      subframe1Valid_         = false;
      subframe2Valid_         = false;
      subframe3Valid_         = false;
      subframe1Fault_.iodc    = 1;
      subframe3Fault_.iodeSf3 = 1;
      issueNumberMatches      = false;
    }
  }

  if (subframe2Valid_ && subframe3Valid_)
  {
    if (iodeSf2_ != iodeSf3_)
    {
      subframe1Valid_         = false;
      subframe2Valid_         = false;
      subframe3Valid_         = false;
      subframe2Fault_.iodeSf2 = 1;
      subframe3Fault_.iodeSf3 = 1;
      issueNumberMatches      = false;
    }
  }

  return (issueNumberMatches);
}

//-----------------------------------------------------------------------------
bool GpsEphemeris::checkSubframesForFaults()
{
  subframe1Valid_ = checkSubframe1();
  subframe2Valid_ = checkSubframe2();
  subframe3Valid_ = checkSubframe3();

  return subframe1Valid_ && subframe2Valid_ && subframe3Valid_;
}

bool GpsEphemeris::checkSubframe1()
{
  bool faultNotFound = true;
  memset(&subframe1Fault_, 0, sizeof(subframe1Fault_));

  if ((towSf1_ < GpsEphemeris::bounds_.first.towSf1) ||
      (towSf1_ > GpsEphemeris::bounds_.second.towSf1))
  {
    subframe1Fault_.towSf1 = 1;
    faultNotFound          = false;
  }

  if ((weekNumber_ < GpsEphemeris::bounds_.first.weekNumber) ||
      (weekNumber_ > GpsEphemeris::bounds_.second.weekNumber))
  {
    subframe1Fault_.weekNumber = 1;
    faultNotFound              = false;
  }

  if ((uraIndex_ < GpsEphemeris::bounds_.first.uraIndex) ||
      (uraIndex_ > GpsEphemeris::bounds_.second.uraIndex))
  {
    subframe1Fault_.uraIndex = 1;
    faultNotFound            = false;
  }

  if ((iodc_ < GpsEphemeris::bounds_.first.iodc) ||
      (iodc_ > GpsEphemeris::bounds_.second.iodc))
  {
    subframe1Fault_.iodc = 1;
    faultNotFound        = false;
  }

  if ((groupDelay_ < GpsEphemeris::bounds_.first.groupDelay) ||
      (groupDelay_ > GpsEphemeris::bounds_.second.groupDelay))
  {
    subframe1Fault_.groupDelay = 1;
    faultNotFound              = false;
  }

  if ((clockCorrectionTime_ <
       GpsEphemeris::bounds_.first.clockCorrectionTime) ||
      (clockCorrectionTime_ > GpsEphemeris::bounds_.second.clockCorrectionTime))
  {
    subframe1Fault_.clockCorrectionTime = 1;
    faultNotFound                       = false;
  }

  if ((clockAging3_ < GpsEphemeris::bounds_.first.clockAging3) ||
      (clockAging3_ > GpsEphemeris::bounds_.second.clockAging3))
  {
    subframe1Fault_.clockAging3 = 1;
    faultNotFound               = false;
  }

  if ((clockAging2_ < GpsEphemeris::bounds_.first.clockAging2) ||
      (clockAging2_ > GpsEphemeris::bounds_.second.clockAging2))
  {
    subframe1Fault_.clockAging2 = 1;
    faultNotFound               = false;
  }

  if ((clockAging1_ < GpsEphemeris::bounds_.first.clockAging1) ||
      (clockAging1_ > GpsEphemeris::bounds_.second.clockAging1))
  {
    subframe1Fault_.clockAging1 = 1;
    faultNotFound               = false;
  }

  return (faultNotFound);
}

//-----------------------------------------------------------------------------
bool GpsEphemeris::checkSubframe2()
{
  bool faultNotFound = true;
  memset(&subframe2Fault_, 0, sizeof(subframe2Fault_));

  if ((towSf2_ < GpsEphemeris::bounds_.first.towSf2) ||
      (towSf2_ > GpsEphemeris::bounds_.second.towSf2))
  {
    subframe2Fault_.towSf2 = 1;
    faultNotFound          = false;
  }

  if ((iodeSf2_ < GpsEphemeris::bounds_.first.iodeSf2) ||
      (iodeSf2_ > GpsEphemeris::bounds_.second.iodeSf2))
  {
    subframe2Fault_.iodeSf2 = 1;
    faultNotFound           = false;
  }

  if ((sinOrbitRadius_ < GpsEphemeris::bounds_.first.sinOrbitRadius) ||
      (sinOrbitRadius_ > GpsEphemeris::bounds_.second.sinOrbitRadius))
  {
    subframe2Fault_.sinOrbitRadius = 1;
    faultNotFound                  = false;
  }

  if ((meanMotionDifference_ <
       GpsEphemeris::bounds_.first.meanMotionDifference) ||
      (meanMotionDifference_ >
       GpsEphemeris::bounds_.second.meanMotionDifference))
  {
    subframe2Fault_.meanMotionDifference = 1;
    faultNotFound                        = false;
  }

  if ((meanAnomaly_ < GpsEphemeris::bounds_.first.meanAnomaly) ||
      (meanAnomaly_ > GpsEphemeris::bounds_.second.meanAnomaly))
  {
    subframe2Fault_.meanAnomaly = 1;
    faultNotFound               = false;
  }

  if ((cosLatitude_ < GpsEphemeris::bounds_.first.cosLatitude) ||
      (cosLatitude_ > GpsEphemeris::bounds_.second.cosLatitude))
  {
    subframe2Fault_.cosLatitude = 1;
    faultNotFound               = false;
  }

  if ((eccentricity_ < GpsEphemeris::bounds_.first.eccentricity) ||
      (eccentricity_ > GpsEphemeris::bounds_.second.eccentricity))
  {
    subframe2Fault_.eccentricity = 1;
    faultNotFound                = false;
  }

  if ((sinLatitude_ < GpsEphemeris::bounds_.first.sinLatitude) ||
      (sinLatitude_ > GpsEphemeris::bounds_.second.sinLatitude))
  {
    subframe2Fault_.sinLatitude = 1;
    faultNotFound               = false;
  }

  if ((sqrtSemiMajorAxis_ < GpsEphemeris::bounds_.first.sqrtSemiMajorAxis) ||
      (sqrtSemiMajorAxis_ > GpsEphemeris::bounds_.second.sqrtSemiMajorAxis))
  {
    subframe2Fault_.sqrtSemiMajorAxis = 1;
    faultNotFound                     = false;
  }

  if ((timeOfEphemeris_ < GpsEphemeris::bounds_.first.timeOfEphemeris) ||
      (timeOfEphemeris_ > GpsEphemeris::bounds_.second.timeOfEphemeris))
  {
    subframe2Fault_.timeOfEphemeris = 1;
    faultNotFound                   = false;
  }

  if ((ageOfDataOffset_ < GpsEphemeris::bounds_.first.ageOfDataOffset) ||
      (ageOfDataOffset_ > GpsEphemeris::bounds_.second.ageOfDataOffset))
  {
    subframe2Fault_.ageOfDataOffset = 1;
    faultNotFound                   = false;
  }

  return (faultNotFound);
}

//-----------------------------------------------------------------------------
bool GpsEphemeris::checkSubframe3()
{
  bool faultNotFound = true;
  memset(&subframe3Fault_, 0, sizeof(subframe3Fault_));

  if ((towSf3_ < GpsEphemeris::bounds_.first.towSf3) ||
      (towSf3_ > GpsEphemeris::bounds_.second.towSf3))
  {
    subframe3Fault_.towSf3 = 1;
    faultNotFound          = false;
  }

  if ((cosInclination_ < GpsEphemeris::bounds_.first.cosInclination) ||
      (cosInclination_ > GpsEphemeris::bounds_.second.cosInclination))
  {
    subframe3Fault_.cosInclination = 1;
    faultNotFound                  = false;
  }

  if ((rightAscension_ < GpsEphemeris::bounds_.first.rightAscension) ||
      (rightAscension_ > GpsEphemeris::bounds_.second.rightAscension))
  {
    subframe3Fault_.rightAscension = 1;
    faultNotFound                  = false;
  }

  if ((sinInclination_ < GpsEphemeris::bounds_.first.sinInclination) ||
      (sinInclination_ > GpsEphemeris::bounds_.second.sinInclination))
  {
    subframe3Fault_.sinInclination = 1;
    faultNotFound                  = false;
  }

  if ((inclinationAngle_ < GpsEphemeris::bounds_.first.inclinationAngle) ||
      (inclinationAngle_ > GpsEphemeris::bounds_.second.inclinationAngle))
  {
    subframe3Fault_.inclinationAngle = 1;
    faultNotFound                    = false;
  }

  if ((cosOrbitRadius_ < GpsEphemeris::bounds_.first.cosOrbitRadius) ||
      (cosOrbitRadius_ > GpsEphemeris::bounds_.second.cosOrbitRadius))
  {
    subframe3Fault_.cosOrbitRadius = 1;
    faultNotFound                  = false;
  }

  if ((argumentOfPerigee_ < GpsEphemeris::bounds_.first.argumentOfPerigee) ||
      (argumentOfPerigee_ > GpsEphemeris::bounds_.second.argumentOfPerigee))
  {
    subframe3Fault_.argumentOfPerigee = 1;
    faultNotFound                     = false;
  }

  if ((ascensionRate_ < GpsEphemeris::bounds_.first.ascensionRate) ||
      (ascensionRate_ > GpsEphemeris::bounds_.second.ascensionRate))
  {
    subframe3Fault_.ascensionRate = 1;
    faultNotFound                 = false;
  }

  if ((iodeSf3_ < GpsEphemeris::bounds_.first.iodeSf3) ||
      (iodeSf3_ > GpsEphemeris::bounds_.second.iodeSf3))
  {
    subframe3Fault_.iodeSf3 = 1;
    faultNotFound           = false;
  }

  if ((inclinationRate_ < GpsEphemeris::bounds_.first.inclinationRate) ||
      (inclinationRate_ > GpsEphemeris::bounds_.second.inclinationRate))
  {
    subframe3Fault_.inclinationRate = 1;
    faultNotFound                   = false;
  }

  return (faultNotFound);
}

}  // namespace pnt_integrity
