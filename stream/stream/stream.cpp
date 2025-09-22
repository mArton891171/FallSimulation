/**
 * @file FallSimulation.cpp
 * @brief Realistic freefall and parachute simulation with air resistance and atmospheric modeling.
 *
 * Supports multiple objects (skydiver, diving ball, custom), parachute deployment,
 * Reynolds number-adjusted drag, and CSV logging.
 *
 * Author: Your Name
 * Date: 2025-09-22
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

 // --- Physical and atmospheric constants ---
namespace Constants {
    constexpr double SEA_LEVEL_G = 9.80665;         // m/s^2
    constexpr double EARTH_RADIUS = 6371000.0;      // m
    constexpr double SEA_LEVEL_PRESSURE = 101325.0; // Pa
    constexpr double SEA_LEVEL_TEMP = 288.15;       // K
    constexpr double TEMP_LAPSE_RATE = 0.0065;      // K/m
    constexpr double GAS_CONSTANT = 8.31446;        // J/(mol·K)
    constexpr double AIR_MOLAR_MASS = 0.0289644;    // kg/mol
    constexpr double AIR_VISCOSITY = 1.81e-5;       // kg/(m·s)
    constexpr double REYNOLDS_CRITICAL = 2.5e5;
}

// --- Object and simulation state ---
struct PhysicalObject {
    std::string name;
    double mass = 0.0;                       // kg
    double drag_coefficient = 0.0;           // freefall
    double area = 0.0;                        // m^2
    double parachute_drag_coefficient = 0.0;
    double parachute_area = 0.0;
    double moment_of_inertia = 0.0;
};

struct State {
    double time = 0.0;
    double height = 0.0;
    double velocity = 0.0;      // Positive = up
    double acceleration = 0.0;
    double g_force = 1.0;
    double reynolds = 0.0;
};

// --- Fall Simulation Class ---
class FallSimulation {
private:
    PhysicalObject object;
    double start_height = 0.0;
    double initial_velocity = 0.0;
    double dt = 0.01;
    double parachute_deploy_height = -1.0;

    std::vector<State> log;

    // Gravity as a function of altitude
    double get_gravity_at_height(double h) const {
        return Constants::SEA_LEVEL_G * std::pow(Constants::EARTH_RADIUS / (Constants::EARTH_RADIUS + h), 2);
    }

    // Air density (International Standard Atmosphere)
    double get_air_density_at_height(double h) const {
        double T = Constants::SEA_LEVEL_TEMP - Constants::TEMP_LAPSE_RATE * h;
        if (T < 0) return 0.0001;
        double P = Constants::SEA_LEVEL_PRESSURE *
            std::pow(1 - Constants::TEMP_LAPSE_RATE * h / Constants::SEA_LEVEL_TEMP,
                Constants::SEA_LEVEL_G * Constants::AIR_MOLAR_MASS / (Constants::GAS_CONSTANT * Constants::TEMP_LAPSE_RATE));
        return (P * Constants::AIR_MOLAR_MASS) / (Constants::GAS_CONSTANT * T);
    }

    double get_reynolds_number(double velocity, double characteristic_length, double air_density) const {
        return (std::abs(velocity) * characteristic_length * air_density) / Constants::AIR_VISCOSITY;
    }

    double get_adjusted_drag_coefficient(double base_cd, double reynolds) const {
        if (reynolds < Constants::REYNOLDS_CRITICAL)
            return base_cd * (1.0 + 0.15 * std::pow(reynolds / Constants::REYNOLDS_CRITICAL, 0.687));
        return base_cd * 0.9;
    }

    static double get_double(const std::string& prompt, double min_val, double max_val) {
        double value;
        while (true) {
            std::cout << prompt;
            std::cin >> value;
            if (std::cin.good() && value >= min_val && value <= max_val) {
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                return value;
            }
            std::cout << "Error: Enter a number between " << min_val << " and " << max_val << ".\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

public:
    FallSimulation() = default;

    void setup() {
        std::cout << "\n=== Simulation Setup ===\n";
        setup_object();
        start_height = get_double("Starting height (m): ", 1.0, 40000.0);
        int mode = get_double("Mode (1: freefall, 2: upward throw): ", 1, 2);
        initial_velocity = (mode == 2) ? get_double("Initial upward velocity (m/s): ", 1.0, 1000.0) : 0.0;

        if (object.parachute_area > 0) {
            parachute_deploy_height = get_double("Parachute deploy height (m, 0 = disabled): ", 0, start_height - 1);
        }

        dt = get_double("Time step (s, recommended 0.05): ", 0.001, 1.0);
    }

    void setup_object() {
        std::cout << "Choose an object:\n1 - Skydiver\n2 - Diving Ball\n3 - Custom\n";
        int choice = get_double("Choice: ", 1, 3);
        switch (choice) {
        case 1: object = { "Skydiver", 85.0, 1.0, 0.7, 1.4, 35.0, 11 }; break;
        case 2: object = { "DivingBall", 7.0, 0.47, 0.04, 0.0, 0.0, 0.0 }; break;
        case 3:
            object.name = "Custom";
            object.mass = get_double("Mass (kg): ", 0.1, 10000.0);
            object.drag_coefficient = get_double("Drag coefficient (freefall): ", 0.0, 2.0);
            object.area = get_double("Cross-sectional area (m^2): ", 0.01, 100.0);
            object.parachute_drag_coefficient = get_double("Drag coefficient (parachute): ", 0.0, 2.0);
            object.parachute_area = get_double("Parachute area (m^2): ", 0.0, 200.0);
            break;
        }
    }

    void run() {
        log.clear();
        bool parachute_open = false;
        State current_state = { 0.0, start_height, initial_velocity, -get_gravity_at_height(start_height), 1.0, 0.0 };
        log.push_back(current_state);

        while (current_state.height > 0) {
            double g = get_gravity_at_height(current_state.height);
            double rho = get_air_density_at_height(current_state.height);

            if (!parachute_open && parachute_deploy_height > 0 && current_state.velocity < 0 &&
                current_state.height <= parachute_deploy_height) {
                parachute_open = true;
                std::cout << "Parachute deployed at " << current_state.height << " m, time " << current_state.time << " s\n";
            }

            double area = parachute_open ? object.parachute_area : object.area;
            double reynolds = get_reynolds_number(current_state.velocity, std::sqrt(4 * area / M_PI), rho);
            double cd = get_adjusted_drag_coefficient(parachute_open ? object.parachute_drag_coefficient : object.drag_coefficient, reynolds);

            double Fg = -object.mass * g;
            double Fd = 0.5 * rho * cd * area * current_state.velocity * std::abs(current_state.velocity);
            double a = (Fg - Fd) / object.mass;

            current_state.acceleration = a;
            current_state.g_force = -a / Constants::SEA_LEVEL_G;
            current_state.velocity += a * dt;
            current_state.height += current_state.velocity * dt;
            current_state.time += dt;
            current_state.reynolds = reynolds;

            log.push_back(current_state);

            if (log.size() > 500000) {
                std::cout << "Max steps reached, stopping simulation.\n";
                break;
            }
        }
    }

    void print_summary() const {
        if (log.empty()) return;

        const auto& final_state = log.back();
        double max_g = std::max_element(log.begin(), log.end(),
            [](const State& a, const State& b) { return a.g_force < b.g_force; })->g_force;

        std::cout << "\n=== Summary: " << object.name << " ===\n";
        std::cout << "Total time: " << final_state.time << " s\n";
        std::cout << "Impact velocity: " << std::abs(final_state.velocity) << " m/s\n";
        std::cout << "Max G-force: " << max_g << " G\n";
    }

    void save_log_to_csv(const std::string& filename) const {
        std::ofstream out(filename);
        out << "Time,Height,Velocity,Acceleration,G-force,Reynolds\n";
        for (const auto& s : log)
            out << s.time << "," << s.height << "," << s.velocity << "," << s.acceleration
            << "," << s.g_force << "," << s.reynolds << "\n";
        std::cout << "Data saved to " << filename << "\n";
    }
};

// --- Main ---
int main() {
    FallSimulation sim;
    sim.setup();
    std::cout << "\nStarting simulation...\n";
    sim.run();
    sim.print_summary();

    char save_choice;
    std::cout << "Save to CSV? (y/n): ";
    std::cin >> save_choice;
    if (save_choice == 'y' || save_choice == 'Y') sim.save_log_to_csv("fall_output_real.csv");

    return 0;
}
