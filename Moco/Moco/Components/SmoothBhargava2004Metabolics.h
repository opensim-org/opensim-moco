#ifndef MOCO_SMOOTHBHARGAVA2004METABOLICS_H
#define MOCO_SMOOTHBHARGAVA2004METABOLICS_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: SmoothBhargava2004Metabolics.                              *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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
class OSIMMOCO_API SmoothBhargava2004Metabolics_MetabolicMuscleParameters :
        public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            SmoothBhargava2004Metabolics_MetabolicMuscleParameters, Object);
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
    SmoothBhargava2004Metabolics_MetabolicMuscleParameters();
    SmoothBhargava2004Metabolics_MetabolicMuscleParameters(
            const std::string& muscleName,
            double ratio_slow_twitch_fibers,
            double muscle_mass = SimTK::NaN);
    SmoothBhargava2004Metabolics_MetabolicMuscleParameters(
        const std::string& muscleName,
        double ratio_slow_twitch_fibers,
        double activation_constant_slow_twitch,
        double activation_constant_fast_twitch,
        double maintenance_constant_slow_twitch,
        double maintenance_constant_fast_twitch,
        double muscle_mass = SimTK::NaN);

    // TODO
    const double getMuscleMass() const      { return _muscMass; }
    void setMuscleMass();
    // TODO
    const Muscle* getMuscle() const         { return _musc; }
    void setMuscle(Muscle* m)               { _musc = m; }

protected:
    // TODO
    Muscle* _musc;          // Internal pointer to the muscle that corresponds
                            // to these parameters.
    // TODO
    double _muscMass;       // The mass of the muscle (depends on if
                            // <use_provided_muscle_mass> is true or false.

private:
    void constructProperties();
};

/// Object class that holds the set of metabolic parameters required to
/// calculate metabolic power for a single muscle.
class OSIMMOCO_API
    SmoothBhargava2004Metabolics_MetabolicMuscleParameterSet
    : public Set<SmoothBhargava2004Metabolics_MetabolicMuscleParameters>
{
    OpenSim_DECLARE_CONCRETE_OBJECT(
        SmoothBhargava2004Metabolics_MetabolicMuscleParameterSet,
        Set<SmoothBhargava2004Metabolics_MetabolicMuscleParameters>);

public:
    SmoothBhargava2004Metabolics_MetabolicMuscleParameterSet() {}
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

    OpenSim_DECLARE_UNNAMED_PROPERTY(
        SmoothBhargava2004Metabolics_MetabolicMuscleParameterSet,
        "A set containing, for each muscle, the parameters "
        "required to calculate muscle metabolic power.");

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

    const int getNumMetabolicMuscles() const;

    void addMuscle(const std::string& muscleName,
        double ratio_slow_twitch_fibers,
        double muscle_mass);

    void addMuscle(const std::string& muscleName,
        double ratio_slow_twitch_fibers,
        double activation_constant_slow_twitch,
        double activation_constant_fast_twitch,
        double maintenance_constant_slow_twitch,
        double maintenance_constant_fast_twitch,
        double muscle_mass);

    /// Set an existing muscle to use a provided muscle mass.
    void useProvidedMass(const std::string& muscleName, double providedMass);

    /// Set an existing muscle to calculate its own mass.
    void useCalculatedMass(const std::string& muscleName);

    /// Get whether the muscle mass is being explicitly provided.
    /// True means that it is using the property 'provided_muscle_mass'
    /// False means that the muscle mass is being calculated from muscle properties.
    bool isUsingProvidedMass(const std::string& muscleName);

    /// Get the muscle mass used in the metabolic analysis. The value
    /// returned will depend on if the muscle mass is explicitly provided
    /// (i.e. isUsingProvidedMass = true), or if it is being automatically
    /// calculated from muscle data already present in the model
    /// (i.e. isUsingProvidedMass = true).
    const double getMuscleMass(const std::string& muscleName) const;

    /// Get the ratio of slow twitch fibers for an existing muscle.
    const double getRatioSlowTwitchFibers(const std::string& muscleName) const;

    /// Set the ratio of slow twitch fibers for an existing muscle.
    void setRatioSlowTwitchFibers(const std::string& muscleName, const double& ratio);

    /// Get the density for an existing muscle (kg/m^3).
    const double getDensity(const std::string& muscleName) const;

    /// Set the density for an existing muscle (kg/m^3).
    void setDensity(const std::string& muscleName, const double& density);

    /// Get the specific tension for an existing muscle (Pascals (N/m^2)).
    const double getSpecificTension(const std::string& muscleName) const;

    /// Set the specific tension for an existing muscle (Pascals (N/m^2)).
    void setSpecificTension(const std::string& muscleName, const double& specificTension);

    /// Get the activation constant for slow twitch fibers for an existing muscle.
    const double getActivationConstantSlowTwitch(const std::string& muscleName) const;

    /// Set the activation constant for slow twitch fibers for an existing muscle.
    void setActivationConstantSlowTwitch(const std::string& muscleName, const double& c);

    /// Get the activation constant for fast twitch fibers for an existing muscle.
    const double getActivationConstantFastTwitch(const std::string& muscleName) const;

    /// Set the activation constant for fast twitch fibers for an existing muscle.
    void setActivationConstantFastTwitch(const std::string& muscleName, const double& c);

    /// Get the maintenance constant for slow twitch fibers for an existing muscle.
    const double getMaintenanceConstantSlowTwitch(const std::string& muscleName) const;

    /// Set the maintenance constant for slow twitch fibers for an existing muscle.
    void setMaintenanceConstantSlowTwitch(const std::string& muscleName, const double& c);

    /// Get the maintenance constant for fast twitch fibers for an existing muscle.
    const double getMaintenanceConstantFastTwitch(const std::string& muscleName) const;

    /// Set the maintenance constant for fast twitch fibers for an existing muscle.
    void setMaintenanceConstantFastTwitch(const std::string& muscleName, const double& c);

    double getTotalMetabolicRate(const SimTK::State& s) const;
    double getMuscleMetabolicRate(
            const SimTK::State& s, const std::string& channel) const;

private:

    void constructProperties();
    void extendConnectToModel(Model&) override;
    void extendAddToSystem(SimTK::MultibodySystem&) const override;
    const SimTK::Vector& getMetabolicRate(const SimTK::State& s) const;
    void calcMetabolicRate(const SimTK::State& s, SimTK::Vector&) const;
    // TODO
    mutable std::vector<std::pair<SimTK::ReferencePtr<const Muscle>,
        SimTK::ReferencePtr<SmoothBhargava2004Metabolics_MetabolicMuscleParameters>>> m_muscles;
    // TODO
    mutable std::unordered_map<std::string, int> m_muscleIndices;

    const SmoothBhargava2004Metabolics_MetabolicMuscleParameters*
        getMetabolicParameters(const std::string& muscleName) const;
    SmoothBhargava2004Metabolics_MetabolicMuscleParameters*
        updMetabolicParameters(const std::string& muscleName);
};

} // namespace OpenSim

#endif // MOCO_SMOOTHBHARGAVA2004METABOLICS_H
