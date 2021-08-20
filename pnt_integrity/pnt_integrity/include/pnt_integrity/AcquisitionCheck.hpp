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
/// \brief    Class defined for the acquisition level checks
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     September 30, 2019
//============================================================================//
#ifndef PNT_INTEGRITY__ACQUISITION_CHECK_HPP
#define PNT_INTEGRITY__ACQUISITION_CHECK_HPP

#include "if_data_utils/IFSampleData.hpp"
#include "pnt_integrity/AssuranceCheck.hpp"

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <list>
#include <map>
#include <unsupported/Eigen/FFT>
#include <vector>

namespace pnt_integrity
{
/// String ID for the ACQ check peak vals
const std::string INTEGRITY_ACQ_PEAK_VALS = "INTEGRITY_ACQ_PEAK_VALS";
/// String ID for the ACQ check peak 1 key
const std::string INTEGRITY_ACQ_PEAK1_KEY = "INT_ACQ_PEAK1_";
/// String ID for the ACQ check peak 2 key
const std::string INTEGRITY_ACQ_PEAK2_KEY = "INT_ACQ_PEAK2_";
/// String ID for the ACQ check diagnostic data
const std::string INTEGRITY_ACQ_DIAGNOSTICS = "INTEGRITY_ACQ_DIAGNOSTICS";
/// String ID for the ACQ check high power threshold
const std::string INT_ACQ_DIAG_HI_PWR_THRESH = "INT_ACQ_DIAG_HI_PWR_THRESH";
/// String ID for the ACQ check peak ratio threshold
const std::string INT_ACQ_DIAG_PEAK_RATIO_THRESH =
  "INT_ACQ_DIAG_PEAK_RATIO_THRESH";
/// String ID for the ACQ check acquisition threshold
const std::string INT_ACQ_DIAG_ACQ_THRESH = "INT_ACQ_DIAG_ACQ_THRESH";
/// String ID for the ACQ check survey inconsistent thresh
const std::string INT_ACQ_DIAG_ITHRESH = "INT_ACQ_DIAG_ITHRESH";
/// String ID for the ACQ check survey unassured thresh
const std::string INT_ACQ_DIAG_UTHRESH = "INT_ACQ_DIAG_UTHRESH";
/// String ID for the ACQ check survey inconsistent count
const std::string INT_ACQ_DIAG_ICOUNT = "INT_ACQ_DIAG_ICOUNT";
/// String ID for the ACQ check survey unassured count
const std::string INT_ACQ_DIAG_UCOUNT = "INT_ACQ_DIAG_UCOUNT";
/// String ID for the ACQ check survey peak ratio key
const std::string INT_ACQ_DIAG_PEAK_RATIO_KEY = "INT_ACQ_DIAG_PEAK_RATIO_KEY_";

/// A map for holding PRN codes, indexed on prn
using CodeMap = std::map<int, std::vector<float>>;
/// A pair for holding a PRN and it's code
using CodeMapEntry = std::pair<int, std::vector<float>>;
/// A map for holding frequency bin values
using CodeFreqMap = std::map<int, Eigen::ArrayXcf>;
/// A pair for holding a frequency bin number its values
using CodeFreqMapEntry = std::pair<int, Eigen::ArrayXcf>;
/// A map that stores the correlation results for a prn
using CorrelationResultsMap = std::map<int, Eigen::ArrayXXf>;
/// A map that holds the first and second peak values in each
/// acquisition plan
using PeakResultsMap = std::map<int, std::pair<double, double>>;
/// A vector type for a list of prns
using PrnList = std::vector<int>;

/// \brief Structure for publishing Acquisition Check diagnostics
struct AcqCheckDiagnostics
{
  /// The threshold to indicate high-power in a prn acquisition
  double highPowerThresh;
  /// The threshold on peak 1 to peak 2 ratio to determine a suspect prn
  double peakRatioThresh;
  /// The threshold on the acuqisition plane to indicate a good prn
  double acquisitionThresh;
  /// The threshold used for determining an overall inconsistent assurance level
  double inconsistentThresh;
  /// The threshold used for determining an overall unassured assurance level
  double unassuredThresh;
  /// The number of prns flagged as unassured
  double unassuredCount;
  /// The number of prns flagged as inconsistent
  double inconsistentCount;
  /// A map that pairs PRN id to the peak ratio
  std::map<int, double> ratioMap;
};
/// \brief Class implementation for the acquisition check
///
/// Class implementation of the acquisition check. The class is a child
/// class of AssuranceCheck
class AcquisitionCheck : public AssuranceCheck
{
public:
  /// \brief Constructor for the check class
  ///
  /// Constructor for the acuqisiton check, the default constructor
  /// configures the check for L1-CA at 5 MSps. Acquisition parameters will
  /// be recalculated if a different sampling frequency is detected
  ///
  /// \param name A string name for the check instance
  /// \param highPowerThreshold A threshold that indicates abnormally high power
  /// levels
  /// \param peakRatioThreshold A threshold for the ratio of the first and
  /// second peaks in the acquisition plane, indicating a possible unauthentic
  /// signal
  /// \param acqusitionThreshold A threshold for classifiing a signal as
  /// acquired or unacquired
  /// \param expectedSamplingFreq The expected sampling frequency
  /// \param intermediateFreq The intermediate frequency of incoming sample
  /// stream
  /// \param searchBand The acquisition search band
  /// \param searchStepSize The acquisition search step size (defines bins)
  /// \param integrationPeriod Integration period to use for acquisition
  /// \param codeFrequencyBasis Freqeuncy basis for the code of interest
  /// \param codeLength Length of the code (in chips)
  /// \param log The provided log handler function
  AcquisitionCheck(
    const std::string&           name                 = "Acquisition check",
    const double&                highPowerThreshold   = 2.5e7,
    const double&                peakRatioThreshold   = 7.0,
    const double&                acqusitionThreshold  = 3e6,
    const double&                expectedSamplingFreq = 5e6,
    const double&                intermediateFreq     = 0.0,
    const double&                searchBand           = 10e3,
    const double&                searchStepSize       = 0.5e3,
    const double&                integrationPeriod    = 1e-3,
    const double&                codeFrequencyBasis   = 1.023e6,
    const int&                   codeLength           = 1023,
    const logutils::LogCallback& log = logutils::printLogToStdOut)
    : AssuranceCheck::AssuranceCheck(true, name, log)
    , acquisitionSearchBand_(searchBand)
    , integrationPeriod_(integrationPeriod)
    , codeFrequencyBasis_(codeFrequencyBasis)
    , samplingFrequency_(expectedSamplingFreq)
    , intermediateFrequency_(intermediateFreq)
    , searchStepSize_(searchStepSize)
    , highPowerThreshold_(highPowerThreshold)
    , peakRatioThreshold_(peakRatioThreshold)
    , acquisitionThreshold_(acqusitionThreshold)
    , lastProcessTime_(0.0)
    , samplesPerIntPeriod_(0)
    , samplesPerCode_(0)
    , codeLength_(codeLength)
    , replicasInitialized_(false)
  {
    std::stringstream initMsg;
    initMsg << "Initializing Acquisition Check (" << name
            << ") with parameters: " << std::endl;
    initMsg << "IF sampling rate (Hz): " << expectedSamplingFreq << std::endl;
    initMsg << "Intermediate frequency (Hz):" << intermediateFreq << std::endl;
    initMsg << "search band (Hz): " << searchBand << std::endl;
    initMsg << "search step size (Hz): " << searchStepSize << std::endl;
    initMsg << "integration period (s): " << integrationPeriod << std::endl;
    initMsg << "code frequency basis: " << codeFrequencyBasis << std::endl;
    initMsg << "code length (chips): " << codeLength << std::endl;
    initMsg << "high power threshold: " << highPowerThreshold << std::endl;
    initMsg << "peak ratio threshold: " << peakRatioThreshold << std::endl;
    initMsg << "acquisition threshold: " << acqusitionThreshold;
    logMsg_(initMsg.str(), logutils::LogLevel::Info);
  };

  /// \brief Handler function for IF sample data (SC8)
  ///
  /// Function to handle provided IF data. (Overriding inherited function from
  /// parent class). Calls the common templated function processIfSampleData for
  /// convenience
  ///
  /// \param checkTime The timestamp associated with the data
  /// \param ifData The provided IF data sample set
  /// \returns True if successful
  bool handleIFSampleData(
    const double&                                                  checkTime,
    const if_data_utils::IFSampleData<if_data_utils::IFSampleSC8>& ifData)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

    // return processIFSampleData(ifData);
    if (processIFSampleData(ifData))
    {
      lastProcessTime_ = checkTime;
      return runCheck();
    }
    else
    {
      return false;
    }
  }

  /// \brief Handler function for IF sample data (SC16)
  ///
  /// Function to handle provided IF data (Overriding inherited function from
  /// parent class). Calls the common templated function processIfSampleData for
  /// convenience
  ///
  /// \param checkTime The timestamp associated with the data
  /// \param ifData The provided IF data sample set
  /// \returns True if successful
  bool handleIFSampleData(
    const double&                                                   checkTime,
    const if_data_utils::IFSampleData<if_data_utils::IFSampleSC16>& ifData)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

    // return processIFSampleData(ifData);
    if (processIFSampleData(ifData))
    {
      lastProcessTime_ = checkTime;
      return runCheck();
    }
    else
    {
      return false;
    }
  }

  /// \brief Functon to processing incoming samples
  ///
  /// Template function for processing incoming samples
  ///
  /// \param sampleData Incoming sample data
  template <typename samp_type>
  bool processIFSampleData(
    const if_data_utils::IFSampleData<samp_type>& sampleData);

  /// \brief Function to explicitly set the assurance level of the check
  ///
  /// For this check, this function cycles through all of the individual PRN
  /// assurance values, analyzes them, and then sets the master assurance level
  /// associted with the check.
  void calculateAssuranceLevel(const double& time);

  /// \brief Connects the internal publishing function to external interface
  ///
  /// This function connects the internal "publishAcquisitionData" function
  /// to an external, custom function of choice
  ///
  /// \param handler Provided handler function
  void setPublishAquisition(
    std::function<void(const CorrelationResultsMap&)> handler)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    publishAquisitionData_ = handler;
  };

  /// \brief Connects the internal publishing function to external interface
  ///
  /// This function connects the internal "publishPeakData" function
  /// to an external, custom function of choice
  ///
  /// \param handler Provided handler function
  void setPublishPeakData(
    std::function<void(const double&, const PeakResultsMap&)> handler)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    publishPeakData_ = handler;
  };

  /// \brief Connects the internal publishing function to external interface
  ///
  /// This function connects the internal "publishDiagnostics" function
  /// to an external, custom function of choice
  ///
  /// \param handler Provided handler function
  void setPublishDiagnostics(
    std::function<void(const double&              timestamp,
                       const AcqCheckDiagnostics& checkData)> handler)
  {
    std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
    publishDiagnostics_ = handler;
  };

private:
  double acquisitionSearchBand_;
  double integrationPeriod_;
  double codeFrequencyBasis_;
  double samplingFrequency_;
  double intermediateFrequency_;
  double searchStepSize_;
  double highPowerThreshold_;
  double peakRatioThreshold_;
  double acquisitionThreshold_;

  double lastProcessTime_;

  size_t samplesPerIntPeriod_;
  // size_t numFreqBins_;
  size_t samplesPerCode_;

  int codeLength_;

  bool replicasInitialized_;

  CodeMap caCodeMap_;

  //! map of conj of fft of ca code replicas (index is prn num)
  CodeFreqMap caCodeMapFD_;

  PrnList prnList_;

  std::list<double> freqBins_;

  // initialize fft engine
  Eigen::FFT<float> fftEngine_;

  CorrelationResultsMap correlationResultsMap_;
  PeakResultsMap        peakResultsMap_;

  void acquisitionSetup();

  void generateFreqBins();

  bool runCheck();
  void setPrnAssuranceLevels();

  //! Generate a sequence of C/A code samples for the given prn and chip offset
  std::vector<float> generateCaCode(signed int _prn, unsigned int _chip_shift);

  /// Generate a map for holding all CA codes, indexed on PRN
  void generateCaCodeMap();

  /// Digitize C/A code samples for the given sample rate
  std::vector<float> upsampleCaCode(std::vector<float> codes,
                                    double             sampleFrequency,
                                    double             codeFrequency,
                                    size_t             samples    = 0,
                                    double             chip_shift = 0);

  /// \brief Generates a local carrier replica.
  ///
  /// Generates the requested number of samples of a sine and cosine with
  /// amplitude 1 to be used as a local carrier replica for acquisition and
  /// tracking.  Note that the first value in the returned vectors will
  /// be sin(initPhase + phaseStep) and cos(initPhase + phaseStep).
  ///
  /// \param initPhase - Initial phase [radians]
  /// \param phaseStep - Phase to increment by for each new sample [radians]
  /// \param length - Number of samples to generate
  /// \param sine - Reference to a VectorXf that will contain the sine values
  /// \param cosine - Reference to a VectorXf that will contain the cos values
  ///
  /// \returns the final phase value (used to generate the last entry
  ///          in the sine and cosine vectors) [radians]
  double generateCarrier(double           initPhase,
                         double           phaseStep,
                         size_t           length,
                         Eigen::VectorXf& sine,
                         Eigen::VectorXf& cosine);

  float sinFast(float x);
  float cosFast(float x) { return sinFast(x + M_PI_2); }

  bool checkForDifferentSettings(const if_data_utils::IFSampleHeader& header)
  {
    return ((header.fs_ != samplingFrequency_) or
            (header.if_ != intermediateFrequency_));
  }

  bool generateAcquisitionPlane(const Eigen::ArrayXcf& signalSamples);

  void acquisitionCorrelation(const int&              prn,
                              const Eigen::ArrayXcf&  signalSamples,
                              const Eigen::VectorXcf& phasePoints);

  template <typename samp_type>
  void buildSampleVector(const samp_type*                  bufferPtr,
                         const size_t&                     numSamples,
                         std::vector<std::complex<float>>& sampleVec);

  std::function<void(const CorrelationResultsMap&)> publishAquisitionData_;
  std::function<void(const double&, const PeakResultsMap&)> publishPeakData_;

  std::function<void(const double& /*timestamp*/,
                     const AcqCheckDiagnostics& /*checkData*/)>
    publishDiagnostics_;

  AcqCheckDiagnostics diagnostics_;
};

//==============================================================================
//--------------------------- processIFSampleData()-----------------------------
//==============================================================================
template <typename samp_type>
bool AcquisitionCheck::processIFSampleData(
  const if_data_utils::IFSampleData<samp_type>& sampleData)
{
  // if the sampling rate has changed, recalculate necessary parameters
  if (checkForDifferentSettings(sampleData.getHeader()) or
      (!replicasInitialized_))
  {
    std::stringstream recalcMsg;
    recalcMsg
      << "AcquisitionCheck::processIFSampleData(): calculating acquisition"
      << " paramters with sampling frequency " << sampleData.getSampleRate();
    logMsg_(recalcMsg.str(), logutils::LogLevel::Warn);

    samplingFrequency_ = sampleData.getSampleRate();
    acquisitionSetup();
  }

  size_t numSampsToProcess = 2 * samplesPerIntPeriod_;
  if (sampleData.getNumberOfSamples() >= numSampsToProcess)
  {
    // convert samples to a vector of floats
    std::vector<std::complex<float>> sampleVecFloat;
    buildSampleVector(
      sampleData.getBufferPtr(), numSampsToProcess, sampleVecFloat);

    // Extract 1 integration period for processing
    // Eigen::ArrayXXf resultsP1;
    Eigen::Map<Eigen::ArrayXcf> sampleVecP1(&sampleVecFloat[0],
                                            samplesPerIntPeriod_);

    generateAcquisitionPlane(sampleVecP1);
    // std::cout << "results[0][0]" << resultsP1(0,0) << std::endl;
    // samplesP1.size());

    return true;
  }
  else
  {
    logMsg_(
      "AcquisitionCheck::processIFSampleData(): did not receive "
      " enough samples to process",
      logutils::LogLevel::Warn);
    return false;
  }
}

template <typename samp_type>
void AcquisitionCheck::buildSampleVector(
  const samp_type*                  bufferPtr,
  const size_t&                     numSamples,
  std::vector<std::complex<float>>& sampleVec)
{
  sampleVec.clear();
  for (size_t ii = 0; ii < numSamples; ++ii)
  {
    samp_type sampleValue = *(bufferPtr + ii);
    // std::cout << "sample value = " << (int16_t)(sampleValue.real()) << ", "
    //           << (int16_t)(sampleValue.imag()) << std::endl;

    // std::complex<float> tmpVal = static_cast<std::complex<float>>(*(bufferPtr
    // + ii)); std::cout << "converted value = " << tmpVal << std::endl;
    sampleVec.push_back(
      std::complex<float>(sampleValue.real(), sampleValue.imag()));
    ;
  }
}

}  // namespace pnt_integrity
#endif
