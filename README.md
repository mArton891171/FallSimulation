# FallSimulation
# FallSimulation

A realistic freefall and parachute simulation in C++ with air resistance, atmospheric modeling, and Reynolds number-adjusted drag.

![Simulation Preview](docs/fall_simulation.gif)  <!-- Ha van GIF, ide linkeld -->

---

## Features
- Simulates objects in freefall or upward throw
- Supports multiple object types (skydiver, diving ball, custom)
- Parachute deployment with adjustable parameters
- Realistic atmospheric density and gravity modeling
- Reynolds number-adjusted drag coefficient for higher accuracy
- CSV output for detailed analysis

---

## Getting Started

### Requirements
- C++17 compatible compiler (GCC, Clang, MSVC)
- CMake >= 3.10
- Optional: Python/Excel for analyzing CSV results

### Build Instructions
```bash
git clone https://github.com/YourUsername/FallSimulation.git
cd FallSimulation
mkdir build
cd build
cmake ..
make

