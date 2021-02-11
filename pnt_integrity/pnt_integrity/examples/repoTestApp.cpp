//============================================================================//
//------------------------ pnt_integrity/repoTestApp.cpp -------*- C++ -*-----//
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
//  Test application for the integrity observalbes repo
//  Josh Clanton <josh.clanton@is4s.com>
//  May 28, 2019
//============================================================================//
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "pnt_integrity/AngleOfArrivalCheck.hpp"
#include "pnt_integrity/IntegrityDataRepository.hpp"
#include "pnt_integrity/IntegrityMonitor.hpp"
#include "pnt_integrity/RepositoryEntry.hpp"

using namespace pnt_integrity;
using namespace pnt_integrity::data;

static bool stop_signal_called = false;
void        sig_int_handler(int)
{
  stop_signal_called = true;
}

void logHandler(const std::string& logMessage, const std::string& logLevel)
{
  std::cout << "[" << logLevel << "]" << logMessage << std::endl;
}

int main(int argc, char** argv)
{
  // //===================Checks ObservableEntry=============
  // // Define an observable histroy entry for the local node
  //  RepositoryEntry localEntry;
  //  localEntry.setLogMessageHandler(std::bind(logHandler,
  //                                            std::placeholders::_1,
  //                                            std::placeholders::_2));
  //
  //  // Add pseudoranges to the local entry
  //  GNSSObservable obs1;
  //  obs1.pseudorange_ = 10;
  //  localEntry.addEntry(1,  obs1);
  //  localEntry.addEntry(2,  obs1);
  //
  //  // Add an RF range to the remote entry
  //  RepositoryEntry remoteEntry(DataLocaleType::Remote,"node2");
  //  RfRange rfRng1;
  //  remoteEntry.addEntry(rfRng1);
  //
  //  GNSSObservableMap gnssObsMap;
  //  gnssObsMap[1] = GNSSObservable();
  //  gnssObsMap[2] = GNSSObservable();
  //  localEntry.addEntry(gnssObsMap);
  //  // add entry and clear the existing map
  //  localEntry.addEntry(gnssObsMap,true);
  //
  //  // ====================Checks addObservalbe (RF range)===========
  //  IntegrityDataRepository::getInstance().setLogMessageHandler(
  //        std::bind(logHandler,std::placeholders::_1,std::placeholders::_2));
  //
  //  RfRange rfRange1;
  //  rfRange1.range = 1.0;
  //  IntegrityDataRepository::getInstance().addEntry(0.0, "node1", rfRange1);
  //
  //  RfRange range;
  //  if (IntegrityDataRepository::getInstance().getData(0.0, "node1", range))
  //  {
  //    std::cout << "returned range value = " << range.range << std::endl;
  //  }
  //
  //  //=================Checks addEntry (pseudorange)===========
  //  // local
  //  IntegrityDataRepository::getInstance().addEntry(0.0, gnssObsMap,false);
  //  IntegrityDataRepository::getInstance().addEntry(0.0, 1, obs1);
  //  IntegrityDataRepository::getInstance().addEntry(0.0, 2, obs1);
  //
  //  // remote
  //  IntegrityDataRepository::getInstance().addEntry(0.0, "node1", 1, obs1);
  //  IntegrityDataRepository::getInstance().addEntry(0.0, "node2", 2, obs1);
  //
  //  GNSSObservable returnedObs;
  //  returnedObs.pseudorange_ = 5;
  //  IntegrityDataRepository::getInstance().getData(0.0, 2, returnedObs);
  //  std::cout << obs1.pseudorange_ << std::endl;
  //  std::cout << returnedObs.pseudorange_ << std::endl;
  //
  //  IntegrityDataRepository::getInstance().getData(0.0,"node3",1,
  //  returnedObs); std::cout << returnedObs.pseudorange_ << std::endl;
  //
  //  //==============checks repo history management========
  //  for(size_t ii = 0; ii < 100; ii++)
  //  {
  //    IntegrityDataRepository::getInstance().addEntry(ii, gnssObsMap,true);
  //
  //    std::cout << "repo size: " <<
  //    IntegrityDataRepository::getInstance().getRepoSize()
  //    << std::endl;
  //  }

  //================AngleOfArrivalCheck===========
  // initialize / instantiate the repo by setting the log handler
  //  IntegrityDataRepository::getInstance().setLogMessageHandler(
  //                                        std::bind(logHandler,
  //                                                  std::placeholders::_1,
  //                                                  std::placeholders::_2));

  //-----setup dummy data-------
  GNSSObservable obsNode1(0,
                          SatelliteSystem::GPS,
                          CodeType::SigC,
                          FrequencyBand::Band1,
                          AssuranceLevel::Unavailable,
                          45.0,
                          true,
                          3e7,
                          10);
  GNSSObservable obsNode2 = obsNode1;
  obsNode2.pseudorange += 10;  // adding a 10m offset to the first range

  GNSSObservableMap obsMapNode1;
  GNSSObservableMap obsMapNode2;

  for (int ii = 1; ii <= 12; ++ii)
  {
    obsNode1.prn    = ii;
    obsMapNode1[ii] = obsNode1;

    obsNode2.prn    = ii;
    obsMapNode2[ii] = obsNode2;
  }

  // create the integrity monitor
  IntegrityMonitor integrityMonitor;
  // bind the monitor's log handling function
  // integrityMonitor.setLogMessageHandler(std::bind(logHandler,
  //                                                 std::placeholders::_1,
  //                                                 std::placeholders::_2));
  // initialize and register the AOA check
  std::string                          aoaCheckName = "aoa_check";
  std::shared_ptr<AngleOfArrivalCheck> aoaCheckPtr =
    std::make_shared<AngleOfArrivalCheck>(
      aoaCheckName, AoaCheckData::UsePseudorange, 5.0, 5);

  integrityMonitor.registerCheck(aoaCheckName, aoaCheckPtr.get());

  double leapSeconds = 19;
  double curTime     = 0.0;
  size_t msgSeq      = 0;
  while (!stop_signal_called)
  {
    Timestamp timestamp(curTime, 0, 0);
    Header    header1(msgSeq, timestamp, timestamp, "local");
    GNSSTime  gpsTime(0, curTime + leapSeconds, TimeSystem::GPS);

    // create the local observables
    GNSSObservables localObs(header1, gpsTime, obsMapNode1);
    integrityMonitor.handleGnssObservables(localObs);

    // create the remote observables
    GNSSObservables remoteObs = localObs;
    remoteObs.header.deviceId = "node2";
    // delete a prn from the remote for testing
    remoteObs.observables.erase(remoteObs.observables.find(1));

    integrityMonitor.handleGnssObservables(remoteObs, false);

    MultiPrnAssuranceMap prnLevels =
      integrityMonitor.getMultiPrnAssuranceData();

    for (auto it = prnLevels.begin(); it != prnLevels.end(); ++it)
    {
      std::cout << "level for prn " << it->first << ": " << (int)it->second
                << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    curTime += 1.0;
    msgSeq++;
  }
  return 0;
}
