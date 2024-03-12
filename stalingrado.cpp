#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

#include "../DynamicalSystemFramework/src/dsm/dsm.hpp"

// COMPATIBLE ONLY WITH DSM version 1.0.1 and newer.

using unit = uint32_t;

using Graph = dsm::Graph<unit, unit>;
using Itinerary = dsm::Itinerary<unit>;
using Dynamics = dsm::FirstOrderDynamics<unit, unit, unit>;
using TrafficLight = dsm::TrafficLight<unit, unit, unit>;
using Street = dsm::Street<unit, unit>;

void printLoadingBar(int const i, int const n) {
  std::cout << "Loading: " << std::setprecision(2) << std::fixed
            << (i * 100. / n) << "%" << '\r';
  std::cout.flush();
}

int main() {
  // Import input data
  std::ifstream ifs{"./data/input.txt"};
  unit timeUnit{60};
  std::vector<unit> vehiclesToInsert{};
  while (!ifs.eof()) {
    unit vehicleId{0};
    ifs >> vehicleId;
    vehiclesToInsert.push_back(vehicleId);
  }
  const unit MAX_TIME{static_cast<unit>(timeUnit * vehiclesToInsert.size())};

  // Create the graph
  TrafficLight tl1{1}, tl2{2}, tl3{3}, tl4{4}, tl5{5};
  Street s01{1, 1, 12., std::make_pair(0, 1)};
  s01.setCapacity(50);
  s01.setLength(326);

  // Street(StreetId, Capacity, Length, vMax, (from, to))
  Street s12{6, 32, 171., 12., std::make_pair(1, 2)};
  Street s23{11, 38, 209., 12., std::make_pair(2, 3)};
  Street s34{16, 35, 75., 12., std::make_pair(3, 4)};

  tl1.setDelay(std::make_pair(40, 70));
  tl1.setCapacity(1);
  tl1.addStreetPriority(s01.id());

  tl2.setDelay(std::make_pair(50, 75));
  tl2.setCapacity(1);
  tl2.addStreetPriority(s12.id());

  tl3.setDelay(std::make_pair(40, 70));
  tl3.setCapacity(1);
  tl3.addStreetPriority(s23.id());

  tl4.setDelay(std::make_pair(50, 75));
  tl4.setCapacity(1);
  tl4.addStreetPriority(s34.id());
  auto tl1_ptr = std::make_shared<TrafficLight>(tl1);
  auto tl2_ptr = std::make_shared<TrafficLight>(tl2);
  auto tl3_ptr = std::make_shared<TrafficLight>(tl3);
  auto tl4_ptr = std::make_shared<TrafficLight>(tl4);

  Graph graph;
  graph.addNode(tl1_ptr);
  graph.addNode(tl2_ptr);
  graph.addNode(tl3_ptr);
  graph.addNode(tl4_ptr);
  graph.addStreets(s01, s12, s23, s34);
  graph.buildAdj();

  // print nodes and streets
  std::cout << "Nodes: " << graph.nodeSet().size() << '\n';
  std::cout << "Streets: " << graph.streetSet().size() << '\n';

  // Create the dynamics
  Dynamics dynamics{graph};
  dynamics.setSeed(69);
  dynamics.setErrorProbability(0.);
  dynamics.setMinSpeedRateo(0.1);
  Itinerary itinerary{0, 4};
  dynamics.addItinerary(itinerary);
  dynamics.updatePaths();

  // Evolution
  int previousAgentNumber{0};
  auto it = vehiclesToInsert.begin();
  std::ofstream ofs{"./stalingrado_output.csv"};
  // print two columns, time and vehicles
  ofs << "time;vehicle_flux;" << '\n';

  // tl2_ptr->setPhase(67);
  // tl3_ptr->setPhase(25);
  // tl4_ptr->setPhase(11);
  int curr_phase = -1;
  for (uint32_t i = 0; i < MAX_TIME; ++i) {
    if (i % 60 == 0) {
      if (i != 0) {
        ++it;
      }
      const int agentNumber = dynamics.agents().size();
      if (i % 300 == 0) {
        ofs << i << ";" << tl4_ptr->agentCounter() << ';' << '\n';
        previousAgentNumber = 0;

        if (i == 0 || i == 79200) {
          tl2_ptr->setPhase(83);
          tl3_ptr->setPhase(4);
          tl4_ptr->setPhase(36);
        } else if (i == 25200) {
          tl2_ptr->setPhase(67);
          tl3_ptr->setPhase(25);
          tl4_ptr->setPhase(11);
        }
      }
      previousAgentNumber += agentNumber;
      dynamics.addAgents(0, *it, 0);
    }
    dynamics.evolve(false);
  }

  std::cout << dynamics.meanTravelTime().mean;
  return 0;
}