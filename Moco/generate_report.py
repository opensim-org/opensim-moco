# -------------------------------------------------------------------------- #
# OpenSim Moco: plot_iterate.py                                              #
# -------------------------------------------------------------------------- #
# Copyright (c) 2017 Stanford University and the Authors                     #
#                                                                            #
# Author(s): Christopher Dembia                                              #
#                                                                            #
# Licensed under the Apache License, Version 2.0 (the "License"); you may    #
# not use this file except in compliance with the License. You may obtain a  #
# copy of the License at http://www.apache.org/licenses/LICENSE-2.0          #
#                                                                            #
# Unless required by applicable law or agreed to in writing, software        #
# distributed under the License is distributed on an "AS IS" BASIS,          #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
# See the License for the specific language governing permissions and        #
# limitations under the License.                                             #
# -------------------------------------------------------------------------- #

import os
import math
import opensim as osim
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib.ticker import FormatStrFormatter
import matplotlib.cm as cm
import numpy as np
from collections import defaultdict, OrderedDict
import pdb
import argparse

## Input parsing.
## =============
parser = argparse.ArgumentParser(
    description="Generate a report given a MocoIterate and an associated "
                "OpenSim Model. Optionally, additional reference data "
                "compatible with the MocoIterate may be plotted "
                "simultaneously.")
# Required arguments.
parser.add_argument('model', type=str,
                    help="OpenSim Model file name (including path).")
parser.add_argument('iterate', type=str,
                    help="MocoIterate file name (including path).")
# Optional arguments.
parser.add_argument('--bilateral', action='store_true',
                    help="Plot left and right limb states and controls "
                         "together.")
parser.add_argument('--refs', type=str, nargs='+',
                    help="Paths to reference data files.")
parser.add_argument('--colormap', type=str, 
                    help="Matplotlib colormap from which plot colors are "
                         "sampled from.")
args = parser.parse_args()
# Load the Model and MocoIterate from file.
model = osim.Model(args.model)
iterate_file = args.iterate
iterate = osim.MocoIterate(iterate_file)
# Store 'bilateral' boolean option.
bilateral = args.bilateral
# Get any reference files provided by the user and create a list of NumPy
# arrays to use in plotting.
ref_files = args.refs
refs = list()
for ref_file in ref_files:
    num_header_rows = 1
    with open(ref_file) as f:
        for line in f:
            if not line.startswith('endheader'):
                num_header_rows += 1
            else:
                break
    this_ref = np.genfromtxt(ref_file, names=True, delimiter='\t',
                              skip_header=num_header_rows)
    refs.append(this_ref)
# Load the colormap provided by the user. Use a default colormap ('jet') if 
# not provided. Uniformly sample the colormap based on the number of reference
# data sets, plus one for the MocoIterate. 
colormap = args.colormap
if colormap is None: colormap = 'jet'
cmap_samples = np.linspace(0.1, 0.9, len(refs)+1)
cmap = cm.get_cmap(colormap)

## Legend handles and labels.
# ===========================
# Create legend handles and labels that can be used to create a figure legend
# that is applicable all figures.
legend_handles = list()
legend_labels = list()
ref_files.append(iterate_file)
for sample, file in zip(cmap_samples, ref_files):
    color = cmap(sample)
    if bilateral:
        r = mlines.Line2D([], [], ls='-', color=color, linewidth=3)
        legend_handles.append(r)
        legend_labels.append(file + '(right leg)')
        l = mlines.Line2D([], [], ls='--', color=color, linewidth=3)
        legend_handles.append(l)
        legend_labels.append(file + '(left leg)')
    else:
        h = mlines.Line2D([], [], ls='-', color=color, linewidth=3)
        legend_handles.append(h)
        legend_labels.append(file)

## Helper functions.
# ==================
# Convert a SimTK::Vector to a NumPy array for plotting.
# TODO: put something similar in the bindings.
def convert(simtkVector):
    n = simtkVector.size()
    vec = np.empty(n)
    for elt in range(n):
        vec[elt] = simtkVector[elt]

    return vec

# Get a descriptive string given a OpenSim::Coordinate::MotionType enum. 
def getLabelFromMotionType(motionTypeEnum, level):
    label = ''
    if motionTypeEnum == 1:
        if level == 'value': label = 'angle (rad)'
        elif level == 'speed': label = 'ang. vel. (rad/s)'
        else: label = 'rotate'
        return label
    elif motionTypeEnum == 2:
        if level == 'value': label = 'position (m)'
        elif level == 'speed': label = 'velocity (m/s)'
        else: label = 'translate'
        return label
    elif motionTypeEnum == 3:
        return 'coupled'
    else:
        return 'undefined'

# Given a state or control name with substring identifying either the left or 
# right limb, remove the substring and return the updated name. This function
# also takes the argument 'ls_dict', which is a dictionary of plot linestyles
# corresponding to the right leg (solid line) or left leg (dashed line); it is
# updated here for convenience.
def bilateralize(name, ls_dict):
    if '_r/' in name:
        name = name.replace('_r/', '/')
        ls_dict[name].append('-')
    elif '_l/' in name:
        name = name.replace('_l/', '/')
        ls_dict[name].append('--')
    elif '_r_' in name:
        name = name.replace('_r_', '_')
        ls_dict[name].append('-')
    elif '_l_' in name:
        name = name.replace('_l_', '_')
        ls_dict[name].append('--')
    else:
        ls_dict[name].append('-')

    return name, ls_dict

# Default pyplot settings to be used across all plots.
def setPlotDefaults(plt):
    plt.ticklabel_format(axis='y', style='sci', scilimits=(-3, 3))
    ax = plt.gca()
    ax.get_yaxis().get_offset_text().set_position((-0.15,0))
    ax.get_yaxis().get_offset_text().set_fontsize(6)
    ax.tick_params(direction='in', gridOn=True)
    ax.xaxis.set_major_formatter(
        FormatStrFormatter('%.1f'))

## Generate report.
# =================
plots_per_page = 15.0
num_cols = 3
# Add an extra row to hold the legend and other infromation.
num_rows = (plots_per_page / 3) + 1
with PdfPages('report.pdf') as pdf:

    # Time
    # -----
    # Convert iterate time vector to a plotting-friendly NumPy array.
    time = convert(iterate.getTime())
    # Create a conservative set of x-tick values based on the time vector.
    nexttime = math.ceil(time[0] * 10) / 10
    nexttolast = math.floor(time[-1] * 10) / 10
    timeticks = np.arange(nexttime, nexttolast, 0.2)

	# States
    # ------
    state_names = iterate.getStateNames()
    if len(state_names) > 0:
        # Loop through the model's joints and cooresponding coordinates to 
        # store plotting information.
        state_dict = OrderedDict()
        ls_dict = defaultdict(list)
        label_dict = dict()
        jointSet = model.getJointSet()
        for j in range(jointSet.getSize()):
            joint = jointSet.get(j)
            numCoords = joint.numCoordinates()
            for c in range(numCoords):
                coord = joint.get_coordinates(c)
                coordName = coord.getName()
                coordPath = coord.getAbsolutePathString()
                coordMotType = coord.getMotionType()

                # Append suffixes to create names for position and speed state
                # variables.
                valueName = coordName + '/value'
                speedName = coordName + '/speed'
                if bilateral:
                    # If the --bilateral flag was set by the user, remove
                    # substrings that indicate the body side and update the
                    # linestyle dict. 
                    valueName, ls_dict = bilateralize(valueName, ls_dict)      
                    speedName, ls_dict = bilateralize(speedName, ls_dict)
                else:
                    ls_dict[valueName].append('-')
                    ls_dict[speedName].append('-')

                if not valueName in state_dict:
                    state_dict[valueName] = list()
                # If --bilateral was set, the 'valueName' key will correspond
                # to a list containing paths for both sides of the model.
                state_dict[valueName].append(coordPath + '/value')
                label_dict[coordPath + '/value'] = \
                    getLabelFromMotionType(coordMotType, 'value')

                if not speedName in state_dict:
                    state_dict[speedName] = list()
                # If --bilateral was set, the 'speedName' key will correspond
                # to a list containing paths for both sides of the model.
                state_dict[speedName].append(coordPath + '/speed')
                label_dict[coordPath + '/speed'] = \
                    getLabelFromMotionType(coordMotType, 'speed')


        # Loop through all keys in the dictionary we created from the
        # coordinate information and plot all state variables.
        p = 1 # Counter to keep track of number of plots per page.
        for i, key in enumerate(state_dict.keys()):
            # If this is first key or if we filled up the previous page with 
            # plots, create a new figure that will become the next page.
            if p % plots_per_page == 1:
                fig = plt.figure(figsize=(8.5, 11))

            plt.subplot(num_rows, num_cols, p + 3)
            # Loop through all the state variable paths for this key.
            for statePath, ls in zip(state_dict[key], ls_dict[key]):
                state = convert(iterate.getState(statePath))
                # If any reference data was provided, that has columns matching
                # the current state path, then plot them first.
                for r, ref in enumerate(refs):
                    # Column names for reference data are read in with no
                    # slashes.
                    statePathNoSlashes = statePath.replace('/', '')
                    if statePathNoSlashes in ref.dtype.names:
                        plt.plot(ref['time'], ref[statePathNoSlashes], ls=ls, 
                                 color=cmap(cmap_samples[r]),
                                 linewidth=2.5)

                # Plot the state values from the MocoIterate.
                plt.plot(time, state, ls=ls, color=cmap(
                         cmap_samples[len(refs)]),
                         linewidth=1.5)

            # Plot labels and settings.
            plt.title(key, fontsize=10)
            plt.xlabel('time (s)', fontsize=8)                
            plt.ylabel(label_dict[statePath], fontsize=8)
            plt.xticks(timeticks, fontsize=6)
            plt.yticks(fontsize=6)
            plt.xlim(time[0], time[-1])
            # TODO plt.ylim()
            setPlotDefaults(plt)
                
            # If we filled up the current figure or ran out of keys, add this
            # figure as a new page to the PDF. Otherwise, increment the plot
            # counter and keep going.
            if (p % plots_per_page == 0) or (i == len(state_dict.keys())-1):
                fig.tight_layout()
                plt.figlegend(legend_handles, legend_labels, 
                    loc='upper center', bbox_to_anchor=(0.5, 0.97),
                    fancybox=True, shadow=True) 
                pdf.savefig(fig)
                plt.close() 
                p = 1
            else:
                p += 1

	# Controls 
    # --------
    control_names = iterate.getControlNames()
    if len(control_names) > 0:

        control_dict = OrderedDict()
        ls_dict = defaultdict(list)
        for control_name in control_names:
            if bilateral:
                # If the --bilateral flag was set by the user, remove
                # substrings that indicate the body side and update the
                # linestyle dict. 
                valueName, ls_dict = bilateralize(valueName, ls_dict)      
                speedName, ls_dict = bilateralize(speedName, ls_dict)
            else:
                ls_dict[valueName].append('-')
                ls_dict[speedName].append('-')

        pdb.set_trace()





        fig = plt.figure(figsize=(8, 8))
        side =  math.ceil(math.sqrt(len(control_names)))
        for i in range(len(control_names)):
            plt.subplot(side, side, i+1)
            control = convert(iterate.getControl(control_names[i]))
            plt.plot(time, control)
            plt.title(control_names[i].replace('/',''), fontsize=8)
            plt.xlabel('time (s)')
            plt.ticklabel_format(axis='y', style='sci', scilimits=(0, 4))



        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()

	# Multipliers


	# Derivatives



	# Slacks



	# Parameters
