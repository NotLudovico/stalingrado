#include <array>
#include <chrono>
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

struct Parent {
  std::vector<int> values;
  double perf;
};
int main() {
  // Import input data
  std::ifstream ifs{"./data/medium.txt"};
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

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine gen(seed);
  std::uniform_real_distribution<double> r(0, 1);

  constexpr int ITERATIONS = 1000;
  constexpr int MAX_WAIT = 110;

  std::vector<Parent> parents = {};
  double weak_perf = 0;
  int weak_index = 0;
  // Generation
  for (int i = 0; i < 10; i++) {
    std::vector<int> values = {static_cast<int>(r(gen) * MAX_WAIT),
                               static_cast<int>(r(gen) * MAX_WAIT),
                               static_cast<int>(r(gen) * MAX_WAIT)

    };
    int previousAgentNumber{0};
    auto it = vehiclesToInsert.begin();
    tl2_ptr->setPhase(values[0]);
    tl3_ptr->setPhase(values[1]);
    tl4_ptr->setPhase(values[2]);
    for (uint32_t i = 0; i < MAX_TIME; ++i) {
      if (i % 60 == 0) {
        if (i != 0) {
          ++it;
        }
        const int agentNumber = dynamics.agents().size();
        if (i % 300 == 0) {
          previousAgentNumber = 0;
        }
        previousAgentNumber += agentNumber;
        dynamics.addAgents(0, *it, 0);
      }
      dynamics.evolve(false);
    }

    double perf = dynamics.meanTravelTime().mean;

    if (perf > weak_perf) {
      weak_index = i;
      weak_perf = perf;
    }
    parents.push_back(Parent{values, perf});
    std::cout << "\nParent{std::vector<int>{"
              << std::format("{},{},{}", values[0], values[1], values[2])
              << "}, " << perf << "},";
  }

  // Genetic
  for (int k = 0; k < 1000; k++) {
    Parent p1 = parents[static_cast<int>(r(gen) * parents.size() - 1)];
    Parent p2 = parents[static_cast<int>(r(gen) * parents.size() - 1)];

    int crossover = static_cast<int>(r(gen) * 2) + 1;

    std::vector<int> values = {};

    for (int j = 0; j < 3; j++) {
      if (j < crossover) {
        values.push_back(p1.values[j]);
      } else {
        values.push_back(p2.values[j]);
      }
    }

    // Evolution
    int previousAgentNumber{0};
    auto it = vehiclesToInsert.begin();
    int curr_phase = -1;
    tl2_ptr->setPhase(values[0]);
    tl3_ptr->setPhase(values[1]);
    tl4_ptr->setPhase(values[2]);
    for (uint32_t i = 0; i < MAX_TIME; ++i) {
      if (i % 60 == 0) {
        if (i != 0) {
          ++it;
        }
        const int agentNumber = dynamics.agents().size();
        if (i % 300 == 0) {
          previousAgentNumber = 0;
        }
        previousAgentNumber += agentNumber;
        dynamics.addAgents(0, *it, 0);
      }
      dynamics.evolve(false);
    }

    double perf = dynamics.meanTravelTime().mean;
    if (perf < weak_perf) {
      bool isDifferent = true;
      for (size_t j = 0; j < parents.size(); j++) {
        if (parents[j].values == values) {
          isDifferent = false;
          break;
        }
      }

      if (isDifferent) {
        parents[weak_index] = Parent{values, perf};

        weak_perf = parents[0].perf;
        weak_index = 0;
        for (int j = 1; j < parents.size(); j++) {
          if (parents[j].perf > weak_perf) {
            weak_perf = parents[j].perf;
            weak_index = j;
          }
        }
      }
    }
  }

  std::cout << "\n\nResults: ";
  for (int i = 0; i < parents.size(); i++) {
    std::cout << "\nParent{std::vector<int>{"
              << std::format("{},{},{}", parents[i].values[0],
                             parents[i].values[1], parents[i].values[2])
              << "}, " << parents[i].perf << "},";
  }

  return 0;
}