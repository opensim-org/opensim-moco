#ifndef MOCO_BHARGAVA2004METABOLICS_H
#define MOCO_BHARGAVA2004METABOLICS_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: Bhargava2004Metabolics.h                                     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Antoine Falisse                                                 *
 * Contributors: Tim Dorn, Thomas Uchida, Christopher Dembia                  *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "../osimMocoDLL.h"
#include <unordered_map>

#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/Muscle.h>

namespace OpenSim {

/// Object class that holds the metabolic parameters required to calculate
/// metabolic power for a single muscle.
class OSIMMOCO_API Bhargava2004Metabolics_MuscleParameters :
        public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            Bhargava2004Metabolics_MuscleParameters, Component);
public:
    OpenSim_DECLARE_PROPERTY(specific_tension, double,
        "The specific tension of the muscle (Pascals (N/m^2), default is "
        "0.25e6).");
    OpenSim_DECLARE_PROPERTY(density, double,
        "The density of the muscle (kg/m^3, default is 1059.7).");
    OpenSim_DECLARE_PROPERTY(ratio_slow_twitch_fibers, double,
        "Ratio of slow twitch fibers in the muscle "
        "(must be between 0 and 1, default is 0.5).");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(provided_muscle_mass, double,
        "The user specified muscle mass (kg, default is NaN. When this "
        "property is set to NaN, the muscle mass is calculated as follows: "
        "(volume *density) / specific_tension) where "
        "volume = maximal_isometric_force * optimal_fiber_length).");
    OpenSim_DECLARE_PROPERTY(activation_constant_slow_twitch, double,
        "Activation constant for slow twitch fibers (W/kg, default is 40.0).");
    OpenSim_DECLARE_PROPERTY(activation_constant_fast_twitch, double,
        "Activation constant for fast twitch fibers (W/kg, default is "
        "133.0).");
    OpenSim_DECLARE_PROPERTY(maintenance_constant_slow_twitch, double,
        "Maintenance constant for slow twitch fibers (W/kg, default is "
        "74.0).");
    OpenSim_DECLARE_PROPERTY(maintenance_constant_fast_twitch, double,
        "Maintenance constant for fast twitch fibers (W/kg, default is "
        "111.0).");

    OpenSim_DECLARE_SOCKET(muscle, Muscle,
            "The muscle to which the Bhargava2004Metabolics is connected.");

    Bhargava2004Metabolics_MuscleParameters();

    double getMuscleMass() const { return muscleMass; }
    void setMuscleMass();

    const Muscle& getMuscle() const { return getConnectee<Muscle>("muscle"); }

private:
    void constructProperties();
    mutable double muscleMass;
};

/// Metabolic energy model from Bhargava et al (2004).
/// https://doi.org/10.1016/s0021-9290(03)00239-2
class OSIMMOCO_API Bhargava2004Metabolics : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            Bhargava2004Metabolics, ModelComponent);

public:
    OpenSim_DECLARE_PROPERTY(enforce_minimum_heat_rate_per_muscle, bool,
            "Specify whether the total heat rate for a muscle will be clamped "
            "to a minimum value of 1.0 W/kg (default is true).");
    OpenSim_DECLARE_PROPERTY(use_force_dependent_shortening_prop_constant,
            bool, "Specify whether to use a force dependent shortening "
            "proportionality constant (default is false).");
    OpenSim_DECLARE_PROPERTY(basal_coefficient, double, "Basal metabolic "
            "coefficient (default is 1.2).");
    OpenSim_DECLARE_PROPERTY(basal_exponent, double, "Basal metabolic "
            "exponent (default is 1).");
    OpenSim_DECLARE_PROPERTY(muscle_effort_scaling_factor, double,
            "Scale the excitation and activation values to compensate for "
            "solutions with excessive coactivation (e.g., when a suboptimal "
            "tracking strategy is used) (default is 1).");
    OpenSim_DECLARE_PROPERTY(include_negative_mechanical_work, bool,
            "Specify whether negative mechanical work will be included in "
            "mechanicalWorkRate (default is true).");
    OpenSim_DECLARE_PROPERTY(forbid_negative_total_power, bool,
            "Specify whether the total power for each muscle must remain  "
            "positive (default is true).");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(use_smoothing, bool,
            "An optional flag that allows the user to explicitly specify "
            "whether a smooth approximation of the metabolic energy model "
            "should be used (default is false).");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(velocity_smoothing, double,
            "The parameter that determines the smoothness of the transition "
            "of the tanh used to smooth the conditions related to contraction "
            "type (concentric or eccentric). The larger the steeper the "
            "transition but the worse for optimization (default is 10).");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(power_smoothing, double,
            "The parameter that determines the smoothness of the transition "
            "of the tanh used to smooth the condition enforcing non-negative "
            "total power. The larger the steeper the transition but the worse "
            "for optimization (default is 10).");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(heat_rate_smoothing, double,
            "The parameter that determines the smoothness of the transition "
            "of the tanh used to smooth the condition enforcing total heat "
            "rate larger than 1 (W/kg) for a give muscle. The larger the "
            "steeper the transition but the worse for optimization (default "
            "is 10).");

    OpenSim_DECLARE_LIST_PROPERTY(
            muscle_parameters, Bhargava2004Metabolics_MuscleParameters,
            "Metabolic parameters for each muscle.");

    OpenSim_DECLARE_OUTPUT(total_metabolic_rate, double, getTotalMetabolicRate,
            SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(total_activation_rate, double,
            getTotalActivationRate, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(total_maintenance_rate, double,
            getTotalMaintenanceRate, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(total_shortening_rate, double,
            getTotalShorteningRate, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(total_mechanical_work_rate, double,
            getTotalMechanicalWorkRate, SimTK::Stage::Dynamics);

    OpenSim_DECLARE_LIST_OUTPUT(muscle_metabolic_rate, double,
            getMuscleMetabolicRate, SimTK::Stage::Dynamics);

    Bhargava2004Metabolics();

    int getNumMetabolicMuscles() const;

    void addMuscle(const std::string& name, const Muscle& muscle,
            double muscle_mass = SimTK::NaN);

    void addMuscle(const std::string& name, const Muscle& muscle,
            double ratio_slow_twitch_fibers, double specific_tension,
            double muscle_mass = SimTK::NaN);

    void addMuscle(const std::string& name, const Muscle& muscle,
            double ratio_slow_twitch_fibers, double specific_tension,
            double activation_constant_slow_twitch,
            double activation_constant_fast_twitch,
            double maintenance_constant_slow_twitch,
            double maintenance_constant_fast_twitch,
            double muscle_mass = SimTK::NaN);

    double getTotalMetabolicRate(const SimTK::State& s) const;
    double getTotalActivationRate(const SimTK::State& s) const;
    double getTotalMaintenanceRate(const SimTK::State& s) const;
    double getTotalShorteningRate(const SimTK::State& s) const;
    double getTotalMechanicalWorkRate(const SimTK::State& s) const;
    double getMuscleMetabolicRate(
            const SimTK::State& s, const std::string& channel) const;

    double tanhSmoothing(const double x, double smoothing_threshold,
            double smoothing_constant) const;

private:
    void constructProperties();
    void extendFinalizeFromProperties() override;
    void extendRealizeTopology(SimTK::State&) const override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void calcMetabolicRateForCache(const SimTK::State& s) const;
    const SimTK::Vector& getMetabolicRate(const SimTK::State& s) const;
    const SimTK::Vector& getActivationRate(const SimTK::State& s) const;
    const SimTK::Vector& getMaintenanceRate(const SimTK::State& s) const;
    const SimTK::Vector& getShorteningRate(const SimTK::State& s) const;
    const SimTK::Vector& getMechanicalWorkRate(const SimTK::State& s) const;
    void calcMetabolicRate(const SimTK::State& s,
            SimTK::Vector& totalRatesForMuscles,
            SimTK::Vector& activationRatesForMuscles,
            SimTK::Vector& maintenanceRatesForMuscles,
            SimTK::Vector& shorteningRatesForMuscles,
            SimTK::Vector& mechanicalWorkRatesForMuscles) const;
    mutable std::unordered_map<std::string, int> m_muscleIndices;
    using ConditionalFunction =
            double(const double&, const double&, const double&, const double&);
    mutable std::function<ConditionalFunction> m_conditional;
};

} // namespace OpenSim

#endif // MOCO_BHARGAVA2004METABOLICS_H
