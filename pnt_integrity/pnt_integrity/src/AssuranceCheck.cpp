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
//
// Base / parent class for a PNT assurance check
// Josh Clanton <josh.clanton@is4s.com>
// May 28, 2019
//============================================================================//
#include "pnt_integrity/AssuranceCheck.hpp"

#include <iomanip>

namespace pnt_integrity
{
//==============================================================================
//--------------------------- changeAssuranceLevel -----------------------------
//==============================================================================
void AssuranceCheck::changeAssuranceLevel(const double& updateTime,
                                          const data::AssuranceLevel& newLevel)
{
  std::lock_guard<std::recursive_mutex> lock(assuranceCheckMutex_);
  if (newLevel <= assuranceState_.getAssuranceLevel())
  {
    // update immediately if the level is going up
    assuranceState_.setWithLevel(newLevel);
    lastAssuranceUpdate_ = updateTime;

    std::stringstream changeMsg;
    changeMsg << "AssuranceCheck::changeAssuranceLevel() : '" << checkName_
              << "' changing level to: "
              << (int)assuranceState_.getAssuranceLevel()
              << " at time : " << std::setprecision(20) << updateTime;

    logMsg_(changeMsg.str(), logutils::LogLevel::Debug);
  }
  else if (newLevel > assuranceState_.getAssuranceLevel())
  {
    // has it been x seconds since the level was changed?
    // if yes, allow the level to be lowered
    auto timeSinceLastUpdate = updateTime - lastAssuranceUpdate_;

    // if current level is Unavailable, allow immedate update
    if ((assuranceState_.getAssuranceLevel() ==
         data::AssuranceLevel::Unavailable) ||
        (timeSinceLastUpdate > assuranceLevelPeriod_))
    {
      assuranceState_.setWithLevel(newLevel);
      lastAssuranceUpdate_ = updateTime;

      std::stringstream changeMsg;
      changeMsg << "AssuranceCheck::changeAssuranceLevel() : " << checkName_
                << " changing level to: "
                << (int)assuranceState_.getAssuranceLevel()
                << " at time : " << updateTime;
      logMsg_(changeMsg.str(), logutils::LogLevel::Debug2);
    }
  }
}

//==============================================================================
//----------------------------- calculateDistance ------------------------------
//==============================================================================
double AssuranceCheck::calculateDistance(const data::GeodeticPosition3d& pos1,
                                         const data::GeodeticPosition3d& pos2)
{
  double p1Ecef[3];
  pos1.getECEF(p1Ecef);
  double p2Ecef[3];
  pos2.getECEF(p2Ecef);

  double distance =
    sqrt(pow(p1Ecef[0] - p2Ecef[0], 2.0) + pow(p1Ecef[1] - p2Ecef[1], 2.0) +
         pow(p1Ecef[2] - p2Ecef[2], 2.0));
  return distance;
}

}  // namespace pnt_integrity
