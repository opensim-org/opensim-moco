#ifndef MOCO_MOCOACTIVEFIBERPOWERGOAL_H
#define MOCO_MOCOACTIVEFIBERPOWERGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoActiveFiberPowerGoal.h                                   *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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

#include "MocoGoal.h"

namespace OpenSim {

class OSIMMOCO_API MocoActiveFiberPowerGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoActiveFiberPowerGoal, MocoGoal);

public:
    MocoActiveFiberPowerGoal();
    MocoActiveFiberPowerGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoActiveFiberPowerGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    /// Set the exponent on the active fiber powers.
    void setExponent(int exponent) { set_exponent(exponent); }
    double getExponent() const { return get_exponent(); }

    /// Set if the goal should be divided by the displacement of the system's
    /// center of mass over the phase.
    void setDivideByDisplacement(bool tf) { set_divide_by_displacement(tf); }
    bool getDivideByDisplacement() const {
        return get_divide_by_displacement();
    }

    void setIgnoreNegativeFiberPower(bool tf) {
        set_ignore_negative_fiber_power(tf);
    }
    bool getIgnoreNegativeFiberPower() const {
        return get_ignore_negative_fiber_power();
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const SimTK::State& state, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;
    //void printDescriptionImpl(std::ostream& stream = std::cout) const override;

private:
    void constructProperties();
    OpenSim_DECLARE_PROPERTY(exponent, int,
            "The exponent on active fiber powers; greater than or equal to "
            "1 (default: 1).");
    OpenSim_DECLARE_PROPERTY(divide_by_displacement, bool,
            "Divide by the model's displacement over the phase (default: "
            "false)");
    OpenSim_DECLARE_PROPERTY(muscle_density, double,
            "The density of all muscles in the model. This is used to compute "
            "the mass of each muscle, which normalizes active fiber power. "
            "Default: 1059.7 [kg/m^3].");
    OpenSim_DECLARE_PROPERTY(specific_tension, double,
            "The specific tension of all muscles in the model. This is used to "
            "compute the mass of each muscle, which normalizes active fiber "
            "power. Default: 600000 [N/m^2].");
    OpenSim_DECLARE_PROPERTY(ignore_negative_fiber_power, double, 
            "Ignore negative muscle fiber powers. If enabled, the smoothed "
            "positive muscle fiber power equation presented in Koelewijn et "
            "al. 2018 is used. Default: false.")
    OpenSim_DECLARE_PROPERTY(positive_fiber_power_smoothing, double,
            "If the property 'ignore_negative_fiber_power' is enabled, this "
            "parameter controls the smoothing used in the positive fiber power "
            "calculation from Koelewijn et al. 2018. Default: 1e-3.")

    mutable std::vector<SimTK::ReferencePtr<const Muscle>> m_muscleRefs;
    mutable std::function<double(const double&)> m_power_function;
    mutable std::vector<double> m_muscleMasses;
    mutable double m_epsilonSquared;
};

} // namespace OpenSim

#endif // MOCO_MOCOACTIVEFIBERPOWERGOAL_H