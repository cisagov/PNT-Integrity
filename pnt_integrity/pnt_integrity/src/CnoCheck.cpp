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
// Class defined for the carrier-to-noise ratio (Cno) checks
// Josh Clanton <josh.clanton@is4s.com>
// October 23, 2019
//============================================================================//
#include "pnt_integrity/CnoCheck.hpp"

#include <iomanip>
#include <numeric>
namespace pnt_integrity
{
//==============================================================================
//------------------------ handleGnssObservables -------------------------------
//==============================================================================
bool CnoCheck::handleGnssObservables(const data::GNSSObservables& /*gnssObs*/,
                                     const double& /*time*/)
{
  // data has already been added to the repo by the integrity monitor
  return runCheck();
}

//==============================================================================
//-------------------------------- runCheck ------------------------------------
//==============================================================================
bool CnoCheck::runCheck()
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  std::vector<TimeEntry> timeEntries;
  if (IntegrityDataRepository::getInstance().getNewestEntries(timeEntries, 0.0))
  {
    auto entryIt = timeEntries.begin();

    double updateTime = entryIt->timeOfWeek_;
    for (; entryIt != timeEntries.end(); ++entryIt)
    {
      updateTime           = entryIt->timeOfWeek_;
      size_t cnoCheckCount = 0;
      // pull the local observable map
      data::GNSSObservableMap localObsMap;
      entryIt->localData_.getData(localObsMap);

      // collect all of the cno's for this time entry
      std::vector<double> cnoVals;
      for (auto mapIt = localObsMap.begin(); mapIt != localObsMap.end();
           ++mapIt)
      {
        if (mapIt->second.carrierToNoise > 0)
        {
          cnoVals.push_back(mapIt->second.carrierToNoise);
        }
      }

      // proceed only if we have cno values to process
      if (cnoVals.size() > 0)
      {
        // compute the mode of the cno values
        double mode = computeCnoMode(cnoVals);

        // determine how many values are within 1 unit of the mode
        for (auto mapIt = localObsMap.begin(); mapIt != localObsMap.end();
             ++mapIt)
        {
          if ((std::abs(mapIt->second.carrierToNoise - mode)) < 1.0)
          {
            cnoCheckCount++;
          }
        }
        // store the count for assurance level calculations
        cnoCheckCountHist_.push_back(cnoCheckCount);
      }
    }

    if (cnoCheckCountHist_.size() >= cnoFilterWindow_)
    {
      // calculate the mean and then remove 1 vlue
      uint32_t sum = std::accumulate(
        cnoCheckCountHist_.begin(), cnoCheckCountHist_.end(), 0);

      double avgCount = ((double)sum) / ((double)cnoCheckCountHist_.size());
      if (avgCount >= assuranceUnassuredThresh_)
      {
        changeAssuranceLevel(updateTime, data::AssuranceLevel::Unassured);
      }
      else if (avgCount >= assuranceInconsistentThresh_)
      {
        changeAssuranceLevel(updateTime, data::AssuranceLevel::Inconsistent);
      }
      else
      {
        changeAssuranceLevel(updateTime, data::AssuranceLevel::Assured);
      }

      // only allow publishing when new data has arrived
      if ((publishDiagnostics_) && (updateTime != lastPublishTime_))
      {
        CnoCheckDiagnostics diagnostics;
        diagnostics.averageCount       = avgCount;
        diagnostics.inconsistentThresh = assuranceInconsistentThresh_;
        diagnostics.unassuredThresh    = assuranceUnassuredThresh_;

        publishDiagnostics_(updateTime, diagnostics);

        lastPublishTime_ = updateTime;
      }

      // remove the last value
      cnoCheckCountHist_.pop_front();
    }
    //}

    return true;
  }
  else
  {
    return false;
  }
}

//==============================================================================
//----------------------------- computeCnoMode ---------------------------------
//==============================================================================
double CnoCheck::computeCnoMode(const std::vector<double>& cnoVals)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);

  if (cnoVals.size() > 0)
  {
    //   // Allocate an int array of the same size to hold the
    //   // repetition count
    //   int* ipRepetition = new int[iSize];
    std::vector<int> ipRepetition(cnoVals.size());

    for (size_t ii = 0; ii < cnoVals.size(); ++ii)
    {
      ipRepetition[ii] = 0;
      size_t jj        = 0;
      // bool bFound      = false;

      while ((jj < ii) && (cnoVals[ii] != cnoVals[jj]))
      {
        if (cnoVals[ii] != cnoVals[jj])
        {
          ++jj;
        }
      }
      ++(ipRepetition[jj]);
    }

    int iMaxRepeat = 0;
    for (size_t ii = 1; ii < cnoVals.size(); ++ii)
    {
      if (ipRepetition[ii] > ipRepetition[iMaxRepeat])
      {
        iMaxRepeat = ii;
      }
    }
    return cnoVals[iMaxRepeat];
  }
  else
  {
    return -1.0;
  }
}

}  // namespace pnt_integrity
