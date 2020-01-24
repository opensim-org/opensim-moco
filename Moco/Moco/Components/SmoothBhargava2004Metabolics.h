#ifndef MOCO_SMOOTHBHARGAVA2004METABOLICS_H
#define MOCO_SMOOTHBHARGAVA2004METABOLICS_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: SmoothBhargava2004Metabolics.                                *
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
class OSIMMOCO_API SmoothBhargava2004Metabolics_MuscleParameters :
        public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            SmoothBhargava2004Metabolics_MuscleParameters, Component);
public:
    OpenSim_DECLARE_PROPERTY(specific_tension, double,
        "The specific tension of the muscle (Pascals (N/m^2)).");
    OpenSim_DECLARE_PROPERTY(density, double,
        "The density of the muscle (kg/m^3).");
    OpenSim_DECLARE_PROPERTY(ratio_slow_twitch_fibers, double,
        "Ratio of slow twitch fibers in the muscle "
        "(must be between 0 and 1).");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(use_provided_muscle_mass, bool,
        "An optional flag that allows the user to explicitly specify a muscle "
        "mass. If set to true, the 'provided_muscle_mass' property must be "
        "specified.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(provided_muscle_mass, double,
        "The user specified muscle mass (kg).");
    OpenSim_DECLARE_PROPERTY(activation_constant_slow_twitch, double,
        "Activation constant for slow twitch fibers (W/kg).");
    OpenSim_DECLARE_PROPERTY(activation_constant_fast_twitch, double,
        "Activation constant for fast twitch fibers (W/kg).");
    OpenSim_DECLARE_PROPERTY(maintenance_constant_slow_twitch, double,
        "Maintenance constant for slow twitch fibers (W/kg).");
    OpenSim_DECLARE_PROPERTY(maintenance_constant_fast_twitch, double,
        "Maintenance constant for fast twitch fibers (W/kg).");

    OpenSim_DECLARE_SOCKET(muscle, Muscle,
            "The muscle to which the SmoothBhargava2004Metabolic is "
            "connected.");

    SmoothBhargava2004Metabolics_MuscleParameters();
    //SmoothBhargava2004Metabolics_MuscleParameters(
    //        const std::string& name,
    //        const Muscle& muscle,
    //        double ratio_slow_twitch_fibers,
    //        double specific_tension,
    //        double muscle_mass = SimTK::NaN);
    //SmoothBhargava2004Metabolics_MuscleParameters(
    //    const std::string& name,
    //    const Muscle& muscle,
    //    double ratio_slow_twitch_fibers,
    //    double specific_tension,
    //    double activation_constant_slow_twitch,
    //    double activation_constant_fast_twitch,
    //    double maintenance_constant_slow_twitch,
    //    double maintenance_constant_fast_twitch,
    //    double muscle_mass = SimTK::NaN);

    double getMuscleMass() const { return muscleMass; }
    void setMuscleMass();

    const Muscle& getMuscle() const { return getConnectee<Muscle>("muscle"); }

private:
    void constructProperties();
    mutable double muscleMass;
};

/// Metabolic energy model from Bhargava et al (2004).
/// https://doi.org/10.1016/s0021-9290(03)00239-2
class OSIMMOCO_API SmoothBhargava2004Metabolics : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            SmoothBhargava2004Metabolics, ModelComponent);

public:
    OpenSim_DECLARE_PROPERTY(activation_rate_on, bool, "Specify whether "
        "activation heat rate is to be calculated (default is true).");
    OpenSim_DECLARE_PROPERTY(maintenance_rate_on, bool, "Specify whether "
        "maintenance heat rate is to be calculated (default is true).");
    OpenSim_DECLARE_PROPERTY(shortening_rate_on, bool, "Specify whether "
        "shortening heat rate is to be calculated (default is true).");
    OpenSim_DECLARE_PROPERTY(basal_rate_on, bool, "Specify whether basal heat "
        "rate is to be calculated (default is true).");
    OpenSim_DECLARE_PROPERTY(mechanical_work_rate_on, bool, "Specify whether "
        "mechanical work rate is to be calculated (default is true).");
    OpenSim_DECLARE_PROPERTY(enforce_minimum_heat_rate_per_muscle, bool,
        "Specify whether the total heat rate for a muscle will be clamped to "
        "a minimum value of 1.0 W/kg (default is true).");
    OpenSim_DECLARE_PROPERTY(use_fiber_length_dependence_on_maintenance_rate,
        bool, "Specify whether to use the normalized fiber length dependence "
        "on maintenance rate (default is false).");
    OpenSim_DECLARE_PROPERTY(
        normalized_fiber_length_dependence_on_maintenance_rate,
        PiecewiseLinearFunction,
        "Contains a PiecewiseLinearFunction object that describes the "
        "normalized fiber length dependence on maintenance rate.");
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
        "Specify whether negative mechanical work will be included in Wdot "
        "(default is true).");
    OpenSim_DECLARE_PROPERTY(forbid_negative_total_power, bool,
        "Specify whether the total power for each muscle must remain positive "
        "(default is true).");
    OpenSim_DECLARE_PROPERTY(velocity_smoothing, double,
            "The parameter that determines the smoothness of the transition "
            "of the tanh used to smooth the conditions related to contraction "
            "type (concentric or eccentric). The larger the steeper the "
            "transition but the worse for optimization, default is 10.");
    OpenSim_DECLARE_PROPERTY(power_smoothing, double,
            "The parameter that determines the smoothness of the transition "
            "of the tanh used to smooth the condition enforcing non-negative "
            "total power. The larger the steeper the transition but the worse "
            "for optimization, default is 10.");
    OpenSim_DECLARE_PROPERTY(heat_rate_smoothing, double,
            "The parameter that determines the smoothness of the transition "
            "of the tanh used to smooth the condition enforcing total heat "
            "rate larger than 1 (W/kg) for a give muscle. The larger the "
            "steeper the transition but the worse for optimization, default "
            "is 10.");

    OpenSim_DECLARE_LIST_PROPERTY(
            muscle_parameters, SmoothBhargava2004Metabolics_MuscleParameters,
            "Metabolic parameters for each muscle.");

    OpenSim_DECLARE_OUTPUT(total_metabolic_rate, double, getTotalMetabolicRate,
            SimTK::Stage::Velocity);
    OpenSim_DECLARE_LIST_OUTPUT(muscle_metabolic_rate, double,
            getMuscleMetabolicRate, SimTK::Stage::Velocity);

    SmoothBhargava2004Metabolics();
    SmoothBhargava2004Metabolics(
            const bool activation_rate_on,
            const bool maintenance_rate_on,
            const bool shortening_rate_on,
            const bool basal_rate_on,
            const bool work_rate_on);

    int getNumMetabolicMuscles() const;

    //void addMuscle(
    //        SmoothBhargava2004Metabolics_MuscleParameters muscle_parameters) {
    //    append_muscle_parameters(std::move(muscle_parameters));
    //};

    //void addMuscle(const std::string& name, const Muscle& muscle,
    //        double ratio_slow_twitch_fibers, double specific_tension,
    //        double muscle_mass = SimTK::NaN) {

    //    SmoothBhargava2004Metabolics_MuscleParameters muscle_parameters(name,
    //            muscle, ratio_slow_twitch_fibers, specific_tension,
    //            muscle_mass);
    //    append_muscle_parameters(std::move(muscle_parameters));
    //}

    void addMuscle(const std::string& name, const Muscle& muscle,
            double ratio_slow_twitch_fibers, double specific_tension,
            double muscle_mass = SimTK::NaN) {
        append_muscle_parameters(SmoothBhargava2004Metabolics_MuscleParameters());
        auto& mp = upd_muscle_parameters(getProperty_muscle_parameters().size() - 1);
        mp.setName(name);
        mp.set_ratio_slow_twitch_fibers(ratio_slow_twitch_fibers);
        mp.set_specific_tension(specific_tension);
        if (SimTK::isNaN(muscle_mass)) {
            mp.set_use_provided_muscle_mass(false);
        }
        else {
            mp.set_use_provided_muscle_mass(true);
            mp.set_provided_muscle_mass(muscle_mass);
        }
        mp.connectSocket_muscle(muscle);
    }

    double getTotalMetabolicRate(const SimTK::State& s) const;
    double getMuscleMetabolicRate(
            const SimTK::State& s, const std::string& channel) const;

private:

    void constructProperties();
    void extendRealizeTopology(SimTK::State&) const override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    const SimTK::Vector& getMetabolicRate(const SimTK::State& s) const;
    void calcMetabolicRate(const SimTK::State& s, SimTK::Vector& ratesForMuscles) const;
    mutable std::vector<SimTK::ReferencePtr<const
        SmoothBhargava2004Metabolics_MuscleParameters>> m_muscleParameters;
    mutable std::unordered_map<std::string, int> m_muscleIndices;
};

} // namespace OpenSim

#endif // MOCO_SMOOTHBHARGAVA2004METABOLICS_H
