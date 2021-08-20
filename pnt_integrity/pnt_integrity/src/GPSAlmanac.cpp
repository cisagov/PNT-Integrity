//============================================================================//
//---------------------------- GPSAlmanac.cpp --------------------------------//
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
#include <math.h>
#include <cstring>
#include <limits>

// #include "pnt_integrity/GNSSConstants.hpp"
#include "pnt_integrity/GPSAlmanac.hpp"
// #include "pnt_integrity/GPSSubframeUtils.hpp"
// #include "pnt_integrity/coordinateConversions.hpp"
// #include "pnt_integrity/coordinateConversionsEigen.hpp"
// #include "pnt_integrity/geodeticConstants.hpp"

namespace
{
pnt_integrity::AlmanacParameters lowerThresholdInitialization = {
  1,                             // prn
  0,                             // tow
  pnt_integrity::SVAlmHealth(),  //
  1.2574e-5,                     // eccentricity
  0,                             // toa
  (double)(std::numeric_limits<int16_t>::min() / exp2(19) *
           pnt_integrity::gpsPi),  // deltaI (deg)
  -9.26250e-9,                     // omegaDot, Rate of right ascension
  5141.4895,                       // sqrtA
  -3.1416,                         // omega0, right ascension
  -3.1416,                         // omega, argument of perigee
  -3.1416,                         // m0, Mean anomaly
  -7.9945e-4,                      // af0, Sv clock bias [sec]
  -5.5684e-10,                     // af1, Sv clock drift [sec/sec]
  0                                // referenceWeek
};

pnt_integrity::AlmanacParameters upperThresholdInitialization = {
  32,                            // prn
  604800,                        // tow
  pnt_integrity::SVAlmHealth(),  //
  2.3411e-2,                     // eccentricity
  602112,                        // toa
  (double)(std::numeric_limits<int16_t>::max() / exp2(19) *
           pnt_integrity::gpsPi),       // deltaI (deg)
  -7.1428e-9,                           // omegaDot, Rate of right ascension
  5181.2430,                            // sqrtA
  3.1415,                               // omega0, right ascension
  3.1415,                               // omega, argument of perigee
  3.1415,                               // m0, Mean anomaly
  9.2248e-4,                            // af0, Sv clock bias [sec]
  1.5746e-10,                           // af1, Sv clock drift [sec/sec]
  std::numeric_limits<uint16_t>::max()  // Full reference GPS Week
};

std::pair<pnt_integrity::AlmanacParameters, pnt_integrity::AlmanacParameters>
  thresholdsInitilization(lowerThresholdInitialization,
                          upperThresholdInitialization);
}  // namespace

namespace pnt_integrity
{
// Must initialize static member variable
std::pair<AlmanacParameters, AlmanacParameters> GpsAlmanac::thresholds_ =
  thresholdsInitilization;

GpsAlmanac::GpsAlmanac()
  : prn_(0)
  , tow_(NAN)
  , svHealth_(SVAlmHealth())
  , eccentricity_(NAN)
  , toa_(NAN)
  , deltaI_(NAN)
  , omegaDot_(NAN)
  , sqrtA_(NAN)
  , omega0_(NAN)
  , omega_(NAN)
  , m0_(NAN)
  , af0_(NAN)
  , af1_(NAN)
  , referenceWeek_(0)
  , subframeValid_(false)
{
  initializeSubframeFaults(1);
}

GpsAlmanac::GpsAlmanac(unsigned int& prn,
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
                       uint16_t&     wna)
{
  setAlmanac(prn,
             tow,
             svHealth,
             eccentricity,
             toa,
             deltaI,
             omegaDot,
             sqrtA,
             omega0,
             omega,
             m0,
             af0,
             af1);
  setReferenceWeek(wna);
  generateSubframe();
}

GpsAlmanac::GpsAlmanac(const unsigned int prn, const uint8_t (&subframe)[30])
{
  setSubframe(prn, subframe);
}

void GpsAlmanac::setAlmanac(const unsigned int& prn,
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
                            bool                checkForValidity)
{
  prn_          = prn;
  tow_          = tow;
  svHealth_     = svHealth;
  eccentricity_ = eccentricity;
  toa_          = toa;
  deltaI_       = deltaI;
  omegaDot_     = omegaDot;
  sqrtA_        = sqrtA;
  omega0_       = omega0;
  omega_        = omega;
  m0_           = m0;
  af0_          = af0;
  af1_          = af1;

  if (checkForValidity)
  {
    subframeValid_ = checkSubframe();
  }
  else
  {
    subframeValid_ = true;
  }
  generateSubframe();
}

void GpsAlmanac::setAlmanac(const AlmanacParameters& param,
                            bool                     checkForValidity)
{
  setReferenceWeek(param.referenceWeek);

  setAlmanac(param.prn,
             param.tow,
             param.svHealth,
             param.eccentricity,
             param.toa,
             param.deltaI,
             param.omegaDot,
             param.sqrtA,
             param.omega0,
             param.omega,
             param.m0,
             param.af0,
             param.af1);

  if (checkForValidity)
  {
    subframeValid_ = checkSubframe();
  }
  else
  {
    subframeValid_ = true;
  }
}

bool GpsAlmanac::getAlmanac(unsigned int& prn,
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
                            double&       af1) const
{
  prn          = prn_;
  tow          = tow_;
  svHealth     = svHealth_;
  eccentricity = eccentricity_;
  toa          = toa_;
  deltaI       = deltaI_;
  omegaDot     = omegaDot_;
  sqrtA        = sqrtA_;
  omega0       = omega0_;
  omega        = omega_;
  m0           = m0_;
  af0          = af0_;
  af1          = af1_;

  return isSubframeValid();
}

AlmanacParameters GpsAlmanac::getAlmanac() const
{
  AlmanacParameters param;

  param.prn           = prn_;
  param.tow           = tow_;
  param.svHealth      = svHealth_;
  param.eccentricity  = eccentricity_;
  param.toa           = toa_;
  param.deltaI        = deltaI_;
  param.omegaDot      = omegaDot_;
  param.sqrtA         = sqrtA_;
  param.omega0        = omega0_;
  param.omega         = omega_;
  param.m0            = m0_;
  param.af0           = af0_;
  param.af1           = af1_;
  param.referenceWeek = referenceWeek_;

  return (param);
}

void GpsAlmanac::getSvState(const double& receiveTime,
                            double&       positionEcefX,
                            double&       positionEcefY,
                            double&       positionEcefZ,
                            double&       velocityEcefX,
                            double&       velocityEcefY,
                            double&       velocityEcefZ,
                            double&       svClockCorrection,
                            const double& pseudorange) const
{
  double semiMajorAxis = sqrtA_ * sqrtA_;

  double transitTime = pseudorange / speedOfLight;

  // double transmitTime = weekCrossoverCheck(receiveTime - transitTime);
  double transmitTime = receiveTime - transitTime;

  double dt = weekCrossoverCheck(transmitTime - toa_);

  // Compute relativistic correction
  double dtr = 0.0;  // eFsqrtA_ * sin(E);

  double dtClkModel = (af1_ * dt) + af0_;

  svClockCorrection = dtClkModel + dtr;

  double time = transmitTime - svClockCorrection;

  // Time from reference epoch
  double tk = weekCrossoverCheck(time - toa_);

  // Mean anomaly
  double correctedMeanMotion = sqrt(gpsGM / pow(semiMajorAxis, 3.0));

  double M = m0_ + correctedMeanMotion * tk;

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
  double eFsqrtA    = eccentricity_ * gpsF * sqrtA_;
  dtr               = eFsqrtA * sinE;
  svClockCorrection = dtClkModel + dtr;

  // True anomaly
  double trueAnomaly = atan2(sqrt(1.0 - eccentricity_ * eccentricity_) * sinE,
                             cosE - eccentricity_);

  // Arguement of latitude
  double argOfLatitude = fmod(trueAnomaly + omega_ + twoGpsPi, twoGpsPi);

  double twoArgOfLatitude = 2.0 * argOfLatitude;

  double cos2p = cos(twoArgOfLatitude);
  double sin2p = sin(twoArgOfLatitude);

  // Corrected argument of latitude
  double u = argOfLatitude + 0 * cos2p + 0 * sin2p;

  // Corrected radius
  double r = semiMajorAxis * (1 - eccentricity_ * cosE) + 0 * cos2p + 0 * sin2p;

  // Corrected inclination
  double i = deltaI_ + 0 * tk + 0 * cos2p + 0 * sin2p + 0.3 * gpsPi;

  double lonAscNodeDot = omegaDot_ - gpsEarthRotationRate;

  // Angle between ascending node and Greenwich meridian
  double longitudeOfAscendingNode =
    omega0_ + lonAscNodeDot * tk - gpsEarthRotationRate * (toa_ + transitTime);

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

  // satClkCorr = (a_f2 * dt + a_f1) * dt + a_f0 - T_GD + dtr;

  double cosu = cos(u);
  double sinu = sin(u);

  // double cos2u = cos(2.0 * u);
  // double sin2u = sin(2.0 * u);

  double EDot = correctedMeanMotion / (1.0 - eccentricity_ * cosE);

  double argLatDot =
    EDot * sqrt(1 - eccentricity_ * eccentricity_) / (1 - eccentricity_ * cosE);

  double twoArgLatDot = 2.0 * argLatDot;

  double uDot = twoArgLatDot * (0 * cos2p - 0 * sin2p) + argLatDot;

  double rDot = twoArgLatDot * (0 * cos2p - 0 * sin2p) +
                (semiMajorAxis * eccentricity_ * sinE * EDot);

  double iDot = twoArgLatDot * (0 * cos2p - 0 * sin2p);

  double vxOrbitalPlane = rDot * cosu - r * sinu * uDot;
  double vyOrbitalPlane = rDot * sinu + r * cosu * uDot;

  velocityEcefX = vxOrbitalPlane * coso - vyOrbitalPlane * cosi * sino +
                  yOrbitalPlane * sini * sino * iDot -
                  positionEcefY * lonAscNodeDot;

  velocityEcefY = vxOrbitalPlane * sino + vyOrbitalPlane * cosi * coso -
                  yOrbitalPlane * sini * coso * iDot +
                  positionEcefX * lonAscNodeDot;

  velocityEcefZ = vyOrbitalPlane * sini + yOrbitalPlane * cosi * iDot;

}  // End GpsAlmanac::getState()

void GpsAlmanac::setReferenceWeek(const uint16_t week, bool checkForValidity)
{
  referenceWeek_ = week;
  if (checkForValidity)
  {
    checkReferenceWeek();
  }
  else
  {
    subframeValid_ = true;
  }
}

bool GpsAlmanac::setSubframe(const unsigned int prn,
                             const uint8_t (&subframe)[30],
                             bool checkForValidity)
{
  bool valid;

  prn_ = prn;
  // copy the subframe data into the class
  memcpy(subframe_, subframe, 30);

  // parse the data into engineering units
  valid = parseSubframe(subframe_);

  if (checkForValidity)
  {
    valid &= checkSubframe();
  }
  subframeValid_ = valid;

  return valid;
}

void GpsAlmanac::getSubframe(uint8_t (&subframe)[30])
{
  memcpy(subframe, &subframe_, 30);
}

std::string GpsAlmanac::subframeToString() const
{
  std::string retVal;
  toHex((unsigned char*)&subframe_, 30, retVal);

  return "0x" + retVal;
}

bool GpsAlmanac::parseSubframe(const uint8_t (&subframe)[30])
{
  uint16_t sfID = parseSubframeID(subframe);
  uint16_t svId = parseAlmanacSVID(subframe);

  //  std::stringstream out;
  //  out << "Parse Almanac sfid = " << sfID << " , SVID = " << svId;
  //  std::cout << out.str();

  // If subframeID is not 4 or 5 and SV ID is not 1-32
  if (!((sfID == 4) || (sfID == 5)) || (svId > 32) || (svId < 1))
  {
    return (false);
  }

  parseTimeOfWeek(subframe, tow_);

  parseAlmanacEccentricity(subframe, eccentricity_);

  parseTimeOfAlmanac(subframe, toa_);

  parseAlmanacDeltaI(subframe, deltaI_);

  parseAlmanacRateOfRightAscension(subframe, omegaDot_);

  uint8_t svHealthBits;
  parseAlmanacSVHealth(subframe, svHealthBits);

  decodeAlmSVHealthBits(svHealthBits, svHealth_);

  parseAlmanacSqrtSemiMajorAxis(subframe, sqrtA_);

  parseAlmanacLongitudeOfAscendingNode(subframe, omega0_);

  parseAlmanacArgumentOfPerigee(subframe, omega_);

  parseAlmanacMeanAnomaly(subframe, m0_);

  parseAlmanacClockCoefficient0(subframe, af0_);

  parseAlmanacClockCoefficient1(subframe, af1_);

  return true;
}

//------------------------------------------------------------------------------
SVAlmHealth GpsAlmanac::decodeAlmSVHealthBits(const uint8_t& svHealthBits)
{
  SVAlmHealth svHealth;

  decodeAlmSVHealthBits(svHealthBits, svHealth);

  return (svHealth);

}  // getSVHealth
   //------------------------------------------------------------------------------
void GpsAlmanac::decodeAlmSVHealthBits(const uint8_t& svHealthBits,
                                       SVAlmHealth&   svHealth)
{
  // 0xE0 = 0b11100000
  // 0x1F = 0b00011111
  uint8_t svNavBits    = (svHealthBits & 0xE0) >> 5;
  uint8_t svSignalBits = (svHealthBits & 0x1F);

  svHealth.navDataHealth = (SVNavHealth)svNavBits;
  svHealth.signalHealth  = (SVSignalHealth)svSignalBits;
}

//------------------------------------------------------------------------------
uint16_t GpsAlmanac::parseAlmanacSVID(const uint8_t (&subframe)[30])
{
  uint16_t svid;

  parseAlmanacSVID(subframe, svid);

  return (svid);
}

//------------------------------------------------------------------------------
void GpsAlmanac::parseAlmanacSVID(const uint8_t (&subframe)[30], uint16_t& svid)
{
  svid = (uint16_t)(subframe[6] & 0x3F);
}

//------------------------------------------------------------------------------
double GpsAlmanac::parseAlmanacEccentricity(const uint8_t (&subframe)[30])
{
  double ecc;

  parseAlmanacEccentricity(subframe, ecc);

  return (ecc);
}

//------------------------------------------------------------------------------
void GpsAlmanac::parseAlmanacEccentricity(const uint8_t (&subframe)[30],
                                          double& ecc)
{
  uint16_t tmp = 0;
  tmp |= subframe[8];
  tmp |= subframe[7] << 8;

  ecc = (double)tmp * exp2(-21.0);
}

//------------------------------------------------------------------------------
double GpsAlmanac::parseTimeOfAlmanac(const uint8_t (&subframe)[30])
{
  double toa;

  parseTimeOfAlmanac(subframe, toa);

  return (toa);
}

//------------------------------------------------------------------------------
void GpsAlmanac::parseTimeOfAlmanac(const uint8_t (&subframe)[30], double& toa)
{
  toa = subframe[9] * exp2(12.0);
}

//------------------------------------------------------------------------------
double GpsAlmanac::parseAlmanacDeltaI(const uint8_t (&subframe)[30])
{
  double deltaI;

  parseAlmanacDeltaI(subframe, deltaI);

  return (deltaI);
}

//------------------------------------------------------------------------------
void GpsAlmanac::parseAlmanacDeltaI(const uint8_t (&subframe)[30],
                                    double& deltaI)
{
  uint16_t tmp16 = 0;
  tmp16 |= subframe[11];
  tmp16 |= (subframe[10] << 8);

  deltaI = ((int16_t)tmp16) * gpsPi / exp2(19.0);
}

//------------------------------------------------------------------------------
double GpsAlmanac::parseAlmanacRateOfRightAscension(
  const uint8_t (&subframe)[30])
{
  double omegaDot;

  parseAlmanacRateOfRightAscension(subframe, omegaDot);

  return (omegaDot);
}

//------------------------------------------------------------------------------
void GpsAlmanac::parseAlmanacRateOfRightAscension(const uint8_t (&subframe)[30],
                                                  double& omegaDot)
{
  uint16_t tmp16 = 0;
  tmp16 |= (uint16_t)subframe[13];
  tmp16 |= ((uint16_t)subframe[12] << 8);

  omegaDot = ((int16_t)tmp16) * gpsPi / exp2(38.0);
}
//------------------------------------------------------------------------------
uint8_t GpsAlmanac::parseAlmanacSVHealth(const uint8_t (&subframe)[30])
{
  uint8_t svHealthBits;

  parseAlmanacSVHealth(subframe, svHealthBits);

  return (svHealthBits);
}

//------------------------------------------------------------------------------
void GpsAlmanac::parseAlmanacSVHealth(const uint8_t (&subframe)[30],
                                      uint8_t& svHealthBits)
{
  svHealthBits = subframe[14];
}

//------------------------------------------------------------------------------
double GpsAlmanac::parseAlmanacSqrtSemiMajorAxis(const uint8_t (&subframe)[30])
{
  double sqrtA;

  parseAlmanacSqrtSemiMajorAxis(subframe, sqrtA);

  return (sqrtA);
}

//------------------------------------------------------------------------------
void GpsAlmanac::parseAlmanacSqrtSemiMajorAxis(const uint8_t (&subframe)[30],
                                               double& sqrtA)
{
  uint32_t tmp32 = 0;
  tmp32 |= subframe[15] << 24;
  tmp32 |= subframe[16] << 16;
  tmp32 |= subframe[17] << 8;

  sqrtA = ((tmp32) >> 8) / exp2(11.0);
}

//------------------------------------------------------------------------------
double GpsAlmanac::parseAlmanacLongitudeOfAscendingNode(
  const uint8_t (&subframe)[30])
{
  double omega0;

  parseAlmanacLongitudeOfAscendingNode(subframe, omega0);

  return (omega0);
}

//------------------------------------------------------------------------------
void GpsAlmanac::parseAlmanacLongitudeOfAscendingNode(
  const uint8_t (&subframe)[30],
  double& omega0)
{
  uint32_t tmp32 = 0;
  tmp32 |= subframe[18] << 24;
  tmp32 |= subframe[19] << 16;
  tmp32 |= subframe[20] << 8;

  omega0 = (((int32_t)tmp32) >> 8) * gpsPi / exp2(23.0);
}

//------------------------------------------------------------------------------
double GpsAlmanac::parseAlmanacArgumentOfPerigee(const uint8_t (&subframe)[30])
{
  double w;

  parseAlmanacArgumentOfPerigee(subframe, w);

  return (w);
}

//------------------------------------------------------------------------------
void GpsAlmanac::parseAlmanacArgumentOfPerigee(const uint8_t (&subframe)[30],
                                               double& w)
{
  uint32_t tmp32 = 0;
  tmp32 |= subframe[21] << 24;
  tmp32 |= subframe[22] << 16;
  tmp32 |= subframe[23] << 8;

  w = (((int32_t)tmp32) >> 8) * gpsPi / exp2(23.0);
}

//------------------------------------------------------------------------------
double GpsAlmanac::parseAlmanacMeanAnomaly(const uint8_t (&subframe)[30])
{
  double m0;

  parseAlmanacMeanAnomaly(subframe, m0);

  return (m0);
}

//------------------------------------------------------------------------------
void GpsAlmanac::parseAlmanacMeanAnomaly(const uint8_t (&subframe)[30],
                                         double& m0)
{
  uint32_t tmp32 = 0;
  tmp32 |= subframe[24] << 24;
  tmp32 |= subframe[25] << 16;
  tmp32 |= subframe[26] << 8;

  m0 = (((int32_t)tmp32) >> 8) * gpsPi / exp2(23.0);
}

//------------------------------------------------------------------------------
double GpsAlmanac::parseAlmanacClockCoefficient0(const uint8_t (&subframe)[30])
{
  double af0;

  parseAlmanacClockCoefficient0(subframe, af0);

  return (af0);
}

//------------------------------------------------------------------------------
void GpsAlmanac::parseAlmanacClockCoefficient0(const uint8_t (&subframe)[30],
                                               double& af0)
{
  uint16_t tmp16 = 0;
  tmp16 |= subframe[27] << 8;
  tmp16 |= (subframe[29] & 0x1C) << 3;

  af0 = (((int16_t)tmp16) >> 5) / exp2(20.0);
}

//------------------------------------------------------------------------------
double GpsAlmanac::parseAlmanacClockCoefficient1(const uint8_t (&subframe)[30])
{
  double af1;

  parseAlmanacClockCoefficient1(subframe, af1);

  return (af1);
}

//------------------------------------------------------------------------------
void GpsAlmanac::parseAlmanacClockCoefficient1(const uint8_t (&subframe)[30],
                                               double& af1)
{
  uint16_t tmp16 = 0;
  tmp16 |= subframe[28] << 8;
  tmp16 |= (subframe[29] & 0xE0);

  af1 = (((int16_t)tmp16) >> 5) / exp2(38.0);
}
//------------------------------------------------------------------------------

void GpsAlmanac::GpsAlmanac::generateSubframe()
{
  uint16_t tmp16 = 0;
  uint32_t tmp32 = 0;

  uint8_t subframe[30];
  memset(subframe, 0, 30);

  //    subframe[0] = ;
  //    subframe[1] = ;
  //    subframe[2] = ;

  // generateSfID()
  if (prn_ <= 24)
  {  // SF5 PRNs 1-24
    subframe[5] = 5 << 2;
  }
  else
  {  // SF4 PRNs 25-32
    subframe[5] = 4 << 2;
  }

  // tow [3 4 5]
  /// Time of week from subframe HOW [sec]
  //    double      tow_;
  uint32_t temptow = tow_ * 2 / 3;
  subframe[3]      = temptow >> 11;
  subframe[4]      = temptow >> 3;
  subframe[5] |= temptow << 5;

  // AS flag into new subframe
  subframe[5] |= 1 << 5;  // Set A-S ON

  // Data ID
  subframe[6] = 1 << 6;

  // Almanac PRN Number
  //    uint16_t    prn_;
  subframe[6] |= (0x3F & prn_);

  /// Eccentricity [dimensionless]
  //    double      eccentricity_;
  tmp16       = (uint16_t)(eccentricity_ * exp2(21.0));
  subframe[7] = tmp16 >> 8;
  subframe[8] = tmp16 & 0xFF;

  /// Time of almanac [seconds]
  //    double      toa_;
  subframe[9] = (uint8_t)(toa_ / exp2(12.0));

  /// delta inclination angle [rad]
  tmp16        = (uint16_t)(deltaI_ * exp2(19.0) / gpsPi);
  subframe[10] = tmp16 >> 8;
  subframe[11] = tmp16 & 0xFF;

  /// Rate of right ascension [rad/sec]
  tmp16        = (uint16_t)(omegaDot_ * exp2(38.0) / gpsPi);
  subframe[12] = tmp16 >> 8;
  subframe[13] = tmp16 & 0xFF;

  /// Almanac Health Bits
  //    SVAlmHealth svHealth_;
  subframe[14] =
    ((uint8_t)svHealth_.navDataHealth << 5) | ((uint8_t)svHealth_.signalHealth);

  /// Square root of the semi-major axis [sqrt(meters)]
  //    double      sqrtA_;
  tmp32        = (uint32_t)(sqrtA_ * exp2(11.0));
  subframe[15] = tmp32 >> 16;
  subframe[16] = tmp32 >> 8;
  subframe[17] = tmp32;

  /// Right ascension angle [rad]
  //    double      omega0_;
  tmp32        = (int32_t)(omega0_ / gpsPi * exp2(23.0));
  subframe[18] = tmp32 >> 16;
  subframe[19] = tmp32 >> 8;
  subframe[20] = tmp32;

  /// Argument of perigee [rad]
  //    double      omega_;
  tmp32        = (int32_t)(omega_ / gpsPi * exp2(23.0));
  subframe[21] = tmp32 >> 16;
  subframe[22] = tmp32 >> 8;
  subframe[23] = tmp32;

  /// Mean anomaly at reference time [rad]
  //    double      m0_;
  tmp32        = (int32_t)(m0_ / gpsPi * exp2(23.0));
  subframe[24] = tmp32 >> 16;
  subframe[25] = tmp32 >> 8;
  subframe[26] = tmp32;

  /// Clock aging term 1 [seconds] [27 29]
  //    double      af0_;

  tmp16        = (int16_t)(af0_ * exp2(20.0));
  subframe[27] = (tmp16 >> 3);
  subframe[29] = (tmp16 << 2) & 0x1C;

  /// Clock aging term 2 [seconds/second] [28 29]
  //    double      af1_;
  tmp16        = (int16_t)(af1_ * exp2(38.0));
  subframe[28] = tmp16 >> 3;
  subframe[29] |= tmp16 << 8;

  memcpy(subframe_, subframe, 30);
}

double GpsAlmanac::weekCrossoverCheck(const double& time) const
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

bool GpsAlmanac::checkSubframe()
{
  bool faultNotFound       = true;
  subframeFaults_.bitfield = 0;

  if ((prn_ < GpsAlmanac::thresholds_.first.prn) ||
      (prn_ > GpsAlmanac::thresholds_.second.prn))
  {
    subframeFaults_.faultType.tow = 1;
    faultNotFound                 = false;
  }

  if ((tow_ < GpsAlmanac::thresholds_.first.tow) ||
      (tow_ > GpsAlmanac::thresholds_.second.tow))
  {
    subframeFaults_.faultType.tow = 1;
    faultNotFound                 = false;
  }

  //    if ((svHealth_ < GpsAlmanac::thresholds_.first.svHealth) ||
  //        (svHealth_ > GpsAlmanac::thresholds_.second.svHealth))
  //    {
  //      subframeFaults_.svHealth = 1;
  //      faultNotFound = false;
  //    }

  if ((eccentricity_ < GpsAlmanac::thresholds_.first.eccentricity) ||
      (eccentricity_ > GpsAlmanac::thresholds_.second.eccentricity))
  {
    subframeFaults_.faultType.eccentricity = 1;
    faultNotFound                          = false;
  }

  if ((toa_ < GpsAlmanac::thresholds_.first.toa) ||
      (toa_ > GpsAlmanac::thresholds_.second.toa))
  {
    subframeFaults_.faultType.toa = 1;
    faultNotFound                 = false;
  }

  if ((omegaDot_ < GpsAlmanac::thresholds_.first.omegaDot) ||
      (omegaDot_ > GpsAlmanac::thresholds_.second.omegaDot))
  {
    subframeFaults_.faultType.omegaDot = 1;
    faultNotFound                      = false;
  }

  if ((sqrtA_ < GpsAlmanac::thresholds_.first.sqrtA) ||
      (sqrtA_ > GpsAlmanac::thresholds_.second.sqrtA))
  {
    subframeFaults_.faultType.sqrtA = 1;
    faultNotFound                   = false;
  }

  if ((omega0_ < GpsAlmanac::thresholds_.first.omega0) ||
      (omega0_ > GpsAlmanac::thresholds_.second.omega0))
  {
    subframeFaults_.faultType.omega0 = 1;
    faultNotFound                    = false;
  }

  if ((omega_ < GpsAlmanac::thresholds_.first.omega) ||
      (omega_ > GpsAlmanac::thresholds_.second.omega))
  {
    subframeFaults_.faultType.omega = 1;
    faultNotFound                   = false;
  }

  if ((m0_ < GpsAlmanac::thresholds_.first.m0) ||
      (m0_ > GpsAlmanac::thresholds_.second.m0))
  {
    subframeFaults_.faultType.m0 = 1;
    faultNotFound                = false;
  }

  if ((af0_ < GpsAlmanac::thresholds_.first.af0) ||
      (af0_ > GpsAlmanac::thresholds_.second.af0))
  {
    subframeFaults_.faultType.af0 = 1;
    faultNotFound                 = false;
  }

  if ((af1_ < GpsAlmanac::thresholds_.first.af1) ||
      (af1_ > GpsAlmanac::thresholds_.second.af1))
  {
    subframeFaults_.faultType.af1 = 1;
    faultNotFound                 = false;
  }

  //    if ((referenceWeek_ < GpsAlmanac::thresholds_.first.referenceWeek) ||
  //        (referenceWeek_ > GpsAlmanac::thresholds_.second.referenceWeek))
  //    {
  //      subframeFaults_.referenceWeek = 1;
  //      faultNotFound = false;
  //    }

  return (faultNotFound);
}

bool GpsAlmanac::checkReferenceWeek()
{
  if ((referenceWeek_ < GpsAlmanac::thresholds_.first.referenceWeek) ||
      (referenceWeek_ > GpsAlmanac::thresholds_.second.referenceWeek))
  {
    subframeFaults_.faultType.referenceWeek = 1;
    return false;
  }
  subframeFaults_.faultType.referenceWeek = 0;
  return true;
}

void GpsAlmanac::setThresholds(
  const std::pair<AlmanacParameters, AlmanacParameters>& thresholds)
{
  GpsAlmanac::thresholds_ = thresholds;
}

std::pair<AlmanacParameters, AlmanacParameters> GpsAlmanac::getThresholds()
{
  return (GpsAlmanac::thresholds_);
}

void GpsAlmanac::initializeSubframeFaults(uint16_t var)
{
  subframeFaults_.faultType.prn          = var;
  subframeFaults_.faultType.tow          = var;
  subframeFaults_.faultType.svHealth     = var;
  subframeFaults_.faultType.eccentricity = var;
  subframeFaults_.faultType.toa          = var;
  subframeFaults_.faultType.deltaI       = var;
  subframeFaults_.faultType.omegaDot     = var;
  subframeFaults_.faultType.sqrtA        = var;
  subframeFaults_.faultType.omega0       = var;
  subframeFaults_.faultType.omega        = var;
  subframeFaults_.faultType.m0           = var;
  subframeFaults_.faultType.af0          = var;
  subframeFaults_.faultType.af1          = var;
}

NavDataTimeOfArrival GpsAlmanac::checkSubframeTOA(const uint8_t (&subframe)[30])
{
  double newToa = parseTimeOfAlmanac(subframe);

  if (isnan(toa_))
    toa_ = 0;

  double toaDiff = newToa - toa_;

  if (toaDiff > 0)
    return NavDataTimeOfArrival::Newer;  // newer subframe
  else if (toaDiff < 0)
    return NavDataTimeOfArrival::Older;  // older subframe
  else
    return NavDataTimeOfArrival::Same;  // same subframe
}

bool GpsAlmanac::isSvHealthy() const
{
  return ((svHealth_.navDataHealth == SVNavHealth::AllDataOK) &&
          (svHealth_.signalHealth == SVSignalHealth::AllSignalsOk));
}
}  // namespace pnt_integrity
