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
//
//  Class defined for the acquisition level checks
//  Josh Clanton <josh.clanton@is4s.com>
//  September 30, 2019
//============================================================================//
#include "pnt_integrity/AcquisitionCheck.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>

// #include "gnsscommon/GNSSConstants.hpp"

using namespace if_data_utils;

namespace pnt_integrity
{

/// PI as defined in IS-GPS-200 (30.3.3.1.3)
const double gpsPi = 3.1415926535898;
/// 2 * PI as defined in IS-GPS-200 (convenience constant)
const double twoGpsPi = 2.0 * gpsPi;

//==============================================================================
//---------------------------- acquisitionSetup() ------------------------------
//==============================================================================
void AcquisitionCheck::acquisitionSetup()
{
  samplesPerIntPeriod_ = samplingFrequency_ * integrationPeriod_;
  samplesPerCode_ =
    std::round(samplingFrequency_ / (codeFrequencyBasis_ / codeLength_));

  generateCaCodeMap();
  generateFreqBins();

  std::stringstream setupMsg;
  setupMsg << "AcquisitionCheck::acquisitionSetup: "
           << "samps per int period = " << samplesPerIntPeriod_
           << ", num freq bins = " << freqBins_.size();
  logMsg_(setupMsg.str(), logutils::LogLevel::Warn);

  logMsg_("AcquisitionCheck::acquisitionSetup(): Code replicas initialized",
          logutils::LogLevel::Info);

  replicasInitialized_ = true;
}

//==============================================================================
//---------------------------- generateFreqBins() ------------------------------
//==============================================================================
void AcquisitionCheck::generateFreqBins()
{
  // numFreqBins_ = std::round(acquisitionSearchBand_ * 2) + 1;
  for (float curFreq = (intermediateFrequency_ - acquisitionSearchBand_);
       curFreq <= (intermediateFrequency_ + acquisitionSearchBand_);
       curFreq += searchStepSize_)
  {
    freqBins_.push_back(curFreq);
  }
}

//==============================================================================
//---------------------------- generateCaCodeMap()------------------------------
//==============================================================================
void AcquisitionCheck::generateCaCodeMap()
{
  prnList_.clear();
  for (int ii = 1; ii <= 32; ++ii)
  {
    prnList_.push_back(ii);

    caCodeMap_.insert(CodeMapEntry(ii,
                                   upsampleCaCode(generateCaCode(ii, 0),
                                                  samplingFrequency_,
                                                  codeFrequencyBasis_,
                                                  samplesPerIntPeriod_)));

    std::vector<std::complex<float> > caCodeFD;
    fftEngine_.fwd(caCodeFD, caCodeMap_[ii]);

    // convert to Eigen VectorXcf and take conjugate
    Eigen::Map<Eigen::VectorXcf> caFD_map(&caCodeFD[0], caCodeFD.size());
    caCodeMapFD_.insert(CodeFreqMapEntry(ii, caFD_map.conjugate()));
  }
}

//==============================================================================
//------------------------------ generateCacode() ------------------------------
//==============================================================================
std::vector<float> AcquisitionCheck::generateCaCode(signed int   _prn,
                                                    unsigned int _chip_shift)
{
  // initialize vector to hold result
  std::vector<float> ca_code;

  unsigned int G1[1023];
  unsigned int G2[1023];
  unsigned int G1_register[10], G2_register[10];
  unsigned int feedback1, feedback2;
  unsigned int lcv, lcv2;
  unsigned int delay;
  signed int   prn = _prn - 1;  // Move the PRN code to fit an array indices

  // G2 Delays as defined in IS-GPS-200E
  signed int delays[32] = {5,   6,   7,   8,   17,  18,  139, 140,
                           141, 251, 252, 254, 255, 256, 257, 258,
                           469, 470, 471, 472, 473, 474, 509, 512,
                           513, 514, 515, 516, 859, 860, 861, 862};
  // PRN sequences 33 through 37 are reserved for other uses (e.g. ground
  // transmitters)

  // A simple error check
  if ((prn < 0) || (prn > 32))
    return ca_code;

  for (lcv = 0; lcv < 10; lcv++)
  {
    G1_register[lcv] = 1;
    G2_register[lcv] = 1;
  }

  // Generate G1 & G2 Register
  for (lcv = 0; lcv < 1023; lcv++)
  {
    G1[lcv] = G1_register[0];
    G2[lcv] = G2_register[0];

    feedback1 = G1_register[7] ^ G1_register[0];
    feedback2 = (G2_register[8] + G2_register[7] + G2_register[4] +
                 G2_register[2] + G2_register[1] + G2_register[0]) &
                0x1;

    for (lcv2 = 0; lcv2 < 9; lcv2++)
    {
      G1_register[lcv2] = G1_register[lcv2 + 1];
      G2_register[lcv2] = G2_register[lcv2 + 1];
    }

    G1_register[9] = feedback1;
    G2_register[9] = feedback2;
  }

  // Set the delay
  delay = 1023 - delays[prn];
  delay += _chip_shift;
  delay %= 1023;
  // Generate PRN from G1 and G2 Registers
  for (lcv = 0; lcv < 1023; lcv++)
  {
    ca_code.push_back(G1[(lcv + _chip_shift) % 1023] ^ G2[delay]);
    if (ca_code[lcv] == 0.0)  // javi
    {
      ca_code[lcv] = -1.0;
    }
    delay++;
    delay %= 1023;
  }

  return ca_code;
}

//==============================================================================
//------------------------------ upsampleCaCode() ------------------------------
//==============================================================================
std::vector<float> AcquisitionCheck::upsampleCaCode(std::vector<float> codes,
                                                    double sampleFrequency,
                                                    double codeFrequency,
                                                    size_t samples,
                                                    double chip_shift)
{
  double codePhaseStep    = codeFrequency / sampleFrequency;
  int    samples_per_code = sampleFrequency / 1000;  // code is 1ms

  // if samples argument is set to 0, generate 1 ms of samples
  if (samples == 0)
    samples = samples_per_code;

  std::vector<float> digitized;

  // generate array that counts from 0 to 1023 and
  // has the same # of values as 1 period of the
  // sampled CA code

  int ca_idx = 0;
  for (size_t idx = 0; idx < samples; idx++)
  {
    // calculate C/A code index
    // add 1 to index to make it exactly match matlab code
    ca_idx = ceil((codePhaseStep * (idx + 1)) + chip_shift) - 1;
    // ca_idx = ((codePhaseStep * (idx)) + chip_shift) ;

    // wrap index into range of codes vector
    ca_idx = (ca_idx + 1023) % 1023;

    digitized.push_back(codes[ca_idx]);
  }

  return digitized;
}

//==============================================================================
//------------------------------ generateCarrier() -----------------------------
//==============================================================================
double AcquisitionCheck::generateCarrier(double           initPhase,
                                         double           phaseStep,
                                         size_t           length,
                                         Eigen::VectorXf& sine,
                                         Eigen::VectorXf& cosine)
{
  Eigen::VectorXf CarrArg(length);
  // TOOD: remove this if statement and test
  if (length == 1)
  {
    CarrArg(0) = initPhase + phaseStep;
  }
  else
  {
    CarrArg = Eigen::VectorXf::LinSpaced(
      length, initPhase + phaseStep, initPhase + phaseStep * (length));
  }

  sine   = CarrArg.array().sin();
  cosine = CarrArg.array().cos();

  return CarrArg(length - 1);
}

//==============================================================================
//--------------------------------- sinFast() ----------------------------------
//==============================================================================
float AcquisitionCheck::sinFast(float x)
{
  x = fmod(x + M_PI, M_PI * 2) - M_PI;  // restrict x so that -pi<x<pi

  if (x < 0)
  {
    return 1.27323954f * x + 0.405284735f * x * x;
  }
  else
  {
    return 1.27323954f * x - 0.405284735f * x * x;
  }
}

//==============================================================================
//------------------------- generateAcquisitionPlane() -------------------------
//==============================================================================
bool AcquisitionCheck::generateAcquisitionPlane(
  const Eigen::ArrayXcf& signalSamples)
{
  auto start = std::chrono::high_resolution_clock::now();

  // make sure CA tables have been initialized and settings set
  // before attempting acquisition
  if (!replicasInitialized_)
  {
    logMsg_("CA code replicas must be initialized before acquiring.",
            logutils::LogLevel::Error);
    return false;
  }

  // TODO: check that samples size and ca replica size matches
  size_t numSamples = signalSamples.size();

  // generate time vector for carrier replica
  Eigen::VectorXcf phasePoints(numSamples);
  for (size_t ii = 0; ii < numSamples; ii++)
  {
    phasePoints[ii] = twoGpsPi * (double)ii / samplingFrequency_;
  }

  // define PRN to search for
  std::vector<std::thread> fftThreads;

  Eigen::ArrayXXf results(freqBins_.size(), numSamples);

  for (PrnList::iterator prnIt = prnList_.begin(); prnIt != prnList_.end();
       ++prnIt)
  {
    // make sure PRN is between 1 and 32
    if ((*prnIt < 1) || (*prnIt > 32))
    {
      logMsg_("PRN must be between 1 and 32.", logutils::LogLevel::Error);
      return false;
    }
    // add the prn entry to the results map
    correlationResultsMap_.insert(
      std::pair<int, Eigen::ArrayXXf>(*prnIt, results));

    fftThreads.push_back(
      std::thread(std::bind(&AcquisitionCheck::acquisitionCorrelation,
                            this,
                            *prnIt,
                            signalSamples,
                            phasePoints)));
  }

  // close the FFT worker threads
  // size_t threadCount = 1;
  for (auto threadIt = fftThreads.begin(); threadIt != fftThreads.end();
       ++threadIt)
  {
    (*threadIt).join();
  }

  // publish the correlation data

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::stringstream             timeMsg;
  timeMsg << "Elapsed time: " << elapsed.count() << " s";
  logMsg_(timeMsg.str(), logutils::LogLevel::Debug);

  if (publishAquisitionData_)
  {
    publishAquisitionData_(correlationResultsMap_);
  }
  if (publishPeakData_)
  {
    publishPeakData_(lastProcessTime_, peakResultsMap_);
  }

  return true;
}

//==============================================================================
//-------------------------- acquisitionCorrelation ----------------------------
//==============================================================================
void AcquisitionCheck::acquisitionCorrelation(
  const int&              prn,
  const Eigen::ArrayXcf&  signalSamples,
  const Eigen::VectorXcf& phasePoints)
{
  // initialize fft engine
  Eigen::FFT<float> fftEngine;

  size_t numSamples = signalSamples.size();

  std::vector<std::complex<float> > inputSignalDemod(numSamples);
  Eigen::Map<Eigen::ArrayXcf>       inputSignalDemodMap(&inputSignalDemod[0],
                                                  inputSignalDemod.size());

  std::vector<std::complex<float> > signalFft(numSamples);
  Eigen::Map<Eigen::ArrayXcf> signalFftMap(&signalFft[0], signalFft.size());

  std::vector<std::complex<float> > corrFreqDom(numSamples);
  Eigen::Map<Eigen::ArrayXcf>       corrFreqDomMap(&corrFreqDom[0],
                                             corrFreqDom.size());

  std::vector<std::complex<float> > corrTimeDom(numSamples);
  Eigen::Map<Eigen::ArrayXcf>       corrTimeDomMap(&corrTimeDom[0],
                                             corrTimeDom.size());

  // initialize arrays for peak
  auto samplesPerCodeChip =
    std::round(samplingFrequency_ / codeFrequencyBasis_);

  // set variables to save peak and it's location
  float                  peakValue      = 0.0;
  size_t                 peakFreqBinIdx = 0;
  Eigen::VectorXf::Index peakCodeIdx    = 0;

  // initialize array to hold results of acqusition tests
  size_t curBin = 0;
  for (auto freqIt = freqBins_.begin(); freqIt != freqBins_.end(); ++freqIt)
  {
    inputSignalDemodMap = (intermediateFrequency_ + *freqIt) * phasePoints *
                          std::complex<float>(0, 1);

    inputSignalDemodMap = inputSignalDemodMap.exp() * signalSamples;
    fftEngine.fwd(signalFft, inputSignalDemod);

    // multiply the complex conjugate of the CA replica with the demodulated
    // signal to get correlation in the frequency domain
    Eigen::Map<Eigen::ArrayXcf> caFftConj(&caCodeMapFD_[prn][0],
                                          caCodeMapFD_[prn].size());

    corrFreqDomMap = caFftConj * signalFftMap;

    fftEngine.inv(corrTimeDom, corrFreqDom);

    auto binResult                          = corrTimeDomMap.abs2();
    correlationResultsMap_[prn].row(curBin) = binResult;
    // resultsPtr->row(curBin)                 = binResult;

    // find the peak in this bin and the corresponding code idx
    Eigen::VectorXf::Index peakInBinCodeIdx;
    auto                   peakInBin = binResult.maxCoeff(&peakInBinCodeIdx);
    // if the bin peak is bigger than the previous, save it
    if (peakInBin > peakValue)
    {
      peakValue      = peakInBin;
      peakFreqBinIdx = curBin;
      peakCodeIdx    = peakInBinCodeIdx;
    }

    curBin++;
  }
  // define the exclusion zone around the peak
  auto excludeRangeLowIdx  = peakCodeIdx - samplesPerCodeChip;
  auto excludeRangeHighIdx = peakCodeIdx + samplesPerCodeChip;

  // pull out the frequency bin that has the max peak
  auto freqBinWithPeak = correlationResultsMap_[prn].row(peakFreqBinIdx);

  float secondPeakValue = 0.0;
  // auto secondPeakCodeIdx = 0;
  // find the second peak in this freqBin w.r.t. the exculsion zone
  for (auto codeIdx = 0; codeIdx < freqBinWithPeak.size(); codeIdx++)
  {
    // test for max everywehre but exclusion zone
    if ((codeIdx <= excludeRangeLowIdx) or (codeIdx >= excludeRangeHighIdx))
    {
      if (freqBinWithPeak(codeIdx) > secondPeakValue)
      {
        // secondPeakCodeIdx = codeIdx;
        secondPeakValue = freqBinWithPeak(codeIdx);
      }
    }
  }

  peakResultsMap_[prn] = std::pair<double, double>(peakValue, secondPeakValue);
}

//==============================================================================
//-------------------------------- runCheck ------------------------------------
//==============================================================================
bool AcquisitionCheck::runCheck()
{
  setPrnAssuranceLevels();
  calculateAssuranceLevel(lastProcessTime_);
  return true;
}

//==============================================================================
//-------------------------- setPrnAssuranceLevels ----------------------------
//==============================================================================
void AcquisitionCheck::setPrnAssuranceLevels()
{
  std::map<int, double> ratioMap;
  // look at the peak results map and make determinations bas
  for (auto prnIt = peakResultsMap_.begin(); prnIt != peakResultsMap_.end();
       ++prnIt)
  {
    double peakRatio       = (prnIt->second.first / prnIt->second.second);
    ratioMap[prnIt->first] = peakRatio;

    if (prnIt->second.first > highPowerThreshold_)
    {
      // power level is suspect, check ratio of 1st and 2nd peak

      if (peakRatio > peakRatioThreshold_)
      {
        prnAssuranceLevels_[prnIt->first] = data::AssuranceLevel::Unassured;
      }
      else
      {
        prnAssuranceLevels_[prnIt->first] = data::AssuranceLevel::Inconsistent;
      }
    }
    // check the first peak against the threshold
    else if (prnIt->second.first > acquisitionThreshold_)
    {
      prnAssuranceLevels_[prnIt->first] = data::AssuranceLevel::Assured;
    }
    // PRN is not visible
    else
    {
      prnAssuranceLevels_[prnIt->first] = data::AssuranceLevel::Unavailable;
    }
  }

  if (publishDiagnostics_)
  {
    diagnostics_.ratioMap = ratioMap;
  }
}

//==============================================================================
//---------------------------- setAssuranceLevel -------------------------------
//==============================================================================
void AcquisitionCheck::calculateAssuranceLevel(const double& checkTime)
{
  // go throuh the prn assurance map and determine how many have been flagged
  // to set the overall level
  int assuredCount      = 0;
  int unavailableCount  = 0;
  int unassuredCount    = 0;
  int inconsistentCount = 0;
  for (auto it = prnAssuranceLevels_.begin(); it != prnAssuranceLevels_.end();
       ++it)
  {
    if (it->second == data::AssuranceLevel::Unassured)
    {
      unassuredCount++;
    }
    else if (it->second == data::AssuranceLevel::Inconsistent)
    {
      inconsistentCount++;
    }
    else if (it->second == data::AssuranceLevel::Assured)
    {
      assuredCount++;
    }
    else if (it->second == data::AssuranceLevel::Unavailable)
    {
      unavailableCount++;
    }
  }  // end for loop

  if (unassuredCount >= assuranceUnassuredThresh_)
  {
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Unassured);
  }
  else if (inconsistentCount >= assuranceInconsistentThresh_)
  {
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Inconsistent);
  }
  else if (assuredCount >= 4)
  {
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Assured);
  }
  else
  {
    changeAssuranceLevel(checkTime, data::AssuranceLevel::Unavailable);
  }

  if (publishDiagnostics_)
  {
    diagnostics_.highPowerThresh    = highPowerThreshold_;
    diagnostics_.peakRatioThresh    = peakRatioThreshold_;
    diagnostics_.acquisitionThresh  = acquisitionThreshold_;
    diagnostics_.inconsistentThresh = assuranceInconsistentThresh_;
    diagnostics_.unassuredThresh    = assuranceUnassuredThresh_;
    diagnostics_.unassuredCount     = unassuredCount;
    diagnostics_.inconsistentCount  = inconsistentCount;

    publishDiagnostics_(lastProcessTime_, diagnostics_);
  }
  // std::stringstream levelMsg;
  // levelMsg << std::endl;
  // levelMsg << "Unusable : " << unassuredCount << " prns " << std::endl;
  // levelMsg << "Unknown : " << inconsistentCount << " prns " << std::endl;
  // levelMsg << "Assured : " << assuredCount << " prns " << std::endl;
  // levelMsg << "Unavailable : " << unavailableCount << " prns ";
  // logMsg_(levelMsg.str(), "info");

  // std::stringstream msg;
  // msg << "AcquisitionCheck::calculateAssuranceLevel() : Calculated level : "
  //     << (int)assuranceLevel_;
  // logMsg_(msg.str(), "info");
}

}  // namespace pnt_integrity
