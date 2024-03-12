
#ifndef UTILITY
#define UTILITY
#include <iomanip>

#include "../DynamicalSystemFramework/src/dsm/dsm.hpp"
using unit = uint32_t;

using Graph = dsm::Graph<unit, unit>;
using Itinerary = dsm::Itinerary<unit>;
using Dynamics = dsm::FirstOrderDynamics<unit, unit, unit>;
using TrafficLight = dsm::TrafficLight<unit, unit, unit>;
using Street = dsm::Street<unit, unit>;

struct Parent {
  std::vector<int> values;
  double perf;
};

void printLoadingBar(int const i, int const n) {
  std::cout << "Loading: " << std::setprecision(2) << std::fixed
            << (i * 100. / n) << "%" << '\r';
  std::cout.flush();
}
void evolve_stal(TrafficLight* tl1_ptr, TrafficLight* tl2_ptr,
                 TrafficLight* tl3_ptr, TrafficLight* tl4_ptr,
                 std::vector<unit>& vehiclesToInsert, Dynamics& dynamics,
                 bool phase_opt, bool print, std::vector<int>& phases) {
  std::cout << "Simulating...\n";

  std::string filename = "./output";
  if (phase_opt) {
    filename += "_opt.csv";
  } else {
    filename += ".csv";
  }
  if (print) {
    std::cout << "Writing to file: " << filename << '\n';
  }
  std::ofstream ofs{filename};

  const unit MAX_TIME{static_cast<unit>(60 * vehiclesToInsert.size())};

  int previousAgentNumber{0};
  auto it = vehiclesToInsert.begin();
  int curr_phase = -1;
  for (uint32_t i = 0; i < MAX_TIME; ++i) {
    if (print) {
      // printLoadingBar(i, MAX_TIME);
    }
    if (i % 60 == 0) {
      if (i != 0) {
        ++it;
      }
      const int agentNumber = dynamics.agents().size();
      if (i % 300 == 0) {
        if (phase_opt) {
          if (vehiclesToInsert[i / 60] < 10 && curr_phase != 0) {  // LOW
            tl1_ptr->setPhase(phases[0]);
            tl2_ptr->setPhase(phases[1]);
            tl3_ptr->setPhase(phases[2]);
            tl4_ptr->setPhase(phases[3]);
            curr_phase = 0;
          } else if (vehiclesToInsert[i / 60] >= 10 &&
                     curr_phase != 1) {  // MEDIUM-HIGH
            tl1_ptr->setPhase(phases[4]);
            tl2_ptr->setPhase(phases[5]);
            tl3_ptr->setPhase(phases[6]);
            tl4_ptr->setPhase(phases[7]);
            curr_phase = 1;
          }
        }
        if (print) {
          ofs << i << ";" << tl4_ptr->agentCounter() << ';' << '\n';
        }
        previousAgentNumber = 0;
      }
      previousAgentNumber += agentNumber;
      dynamics.addAgents(0, *it, 0);
    }
    dynamics.evolve(false);
  }
}

void read_input(std::string filename, std::vector<unit>& vehiclesToInsert) {
  std::cout << "Reading input...\n";
  std::ifstream ifs{"./input.txt"};
  vehiclesToInsert.reserve(1440);

  while (!ifs.eof()) {
    unit vehicleId{0};
    ifs >> vehicleId;
    vehiclesToInsert.push_back(vehicleId);
  }
  ifs.close();
  std::cout << "Done with input reading\n";
}

Dynamics setup_dynamics(Graph graph) {
  Dynamics dynamics{graph};
  dynamics.setSeed(69);
  dynamics.setErrorProbability(0.);
  dynamics.setMinSpeedRateo(0.1);
  Itinerary itinerary{0, 4};
  dynamics.addItinerary(itinerary);
  dynamics.updatePaths();
  return dynamics;
}

void setup_tl(TrafficLight& tl1, TrafficLight& tl2, TrafficLight& tl3,
              TrafficLight& tl4, Street& s01, Street& s12, Street& s23,
              Street& s34) {
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
}
#endif