//============================================================================//
//------------------------ pnt_integrity/AgcCheck.hpp ----------*- C++ -*-----//
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
// Class defined for the AGC check
// Josh Clanton <josh.clanton@is4s.com>
// February 18, 2020
//============================================================================//
#include "pnt_integrity/AgcCheck.hpp"

namespace pnt_integrity
{
//==============================================================================
//--------------------------------- runCheck -----------------------------------
//==============================================================================
bool AgcCheck::runCheck()
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
  
  double checkTime = currentAgcVals_.header.timestampValid.sec;

  AgcCheckDiagnostics diagnostics;
  diagnostics.inconsistentThresh = assuranceInconsistentThresh_;

  // go through each provided band of AGC values
  // if any of them are below the threshold, lower the assurance level
  size_t inconsistentCount = 0;
  for (auto agcIt = currentAgcVals_.agcValues.begin();
       agcIt != currentAgcVals_.agcValues.end();
       ++agcIt)
  {
    // nomralize the agc count with the max and min values
    // then compare to threshold
    double normalizedValue =
      (agcIt->second - minValue_) / (maxValue_ - minValue_);
    if (normalizedValue < assuranceInconsistentThresh_)
    {
      inconsistentCount++;
    }
    diagnostics.values.agcValues[agcIt->first] = normalizedValue;
  }
  if (inconsistentCount)
  {
    changeAssuranceLevel(currentAgcVals_.header.timestampValid.sec,
                         data::AssuranceLevel::Inconsistent);
  }
  else if (currentAgcVals_.agcValues.size() > 0)
  {
    changeAssuranceLevel(currentAgcVals_.header.timestampValid.sec,
                         data::AssuranceLevel::Assured);
  }

  if (publishDiagnostics && (lastDiagPublishTime_ != checkTime))
  {
    publishDiagnostics(checkTime, diagnostics);
    lastDiagPublishTime_ = checkTime;
  }
  return true;
}

}  // namespace pnt_integrity
