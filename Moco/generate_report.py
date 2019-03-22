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
from matplotlib.ticker import FormatStrFormatter
import matplotlib.cm as cm
import numpy as np
from collections import defaultdict, OrderedDict
import pdb
import argparse

parser = argparse.ArgumentParser(
    description="Generate a report given 1 or more MocoIterates.")
parser.add_argument('model', type=str,
                    help="Model file name.")
parser.add_argument('file', type=str,
                    help="Paths to MocoIterate files.")
parser.add_argument('--bilateral', action='store_true',
                    help="Plot left and right limb coordinates together.")
parser.add_argument('--refs', type=str, nargs='+',
                    help="Paths to reference data files.")

args = parser.parse_args()

model_fname = args.model
model = osim.Model(model_fname)

iterate_fname = args.file
iterate = osim.MocoIterate(iterate_fname)

bilateral = args.bilateral

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

cmap_samples = np.linspace(0.0, 1.0, len(refs)+1)
cmap = cm.get_cmap('jet')

# TODO put this in the bindings
def convert(simtkVector):
    n = simtkVector.size()
    vec = np.empty(n)
    for elt in range(n):
        vec[elt] = simtkVector[elt]

    return vec

def getMotionTypeString(motionTypeEnum):
    if motionTypeEnum == 1:
        return 'rotate'
    elif motionTypeEnum == 2:
        return 'translate'
    elif motionTypeEnum == 3:
        return 'coupled'
    else:
        return 'undefined'

right_ls = '_'
left_ls = '-'
def removeSide(name, ls_dict):
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

plots_per_page = 18.0
num_cols = 3
num_rows = plots_per_page / 3
with PdfPages('report.pdf') as pdf:

    time = convert(iterate.getTime())
    starttime = time[0]
    nexttime = math.ceil(time[0] * 10) / 10
    endtime = time[-1]
    nexttolast = math.ceil(time[-1] * 10) / 10
    timeticks = np.arange(nexttime, nexttolast, 0.2)

	# States
    state_names = iterate.getStateNames()
    if len(state_names) > 0:

        state_dict = OrderedDict()
        ls_dict = defaultdict(list)
        coord_motion_types = dict()

        jointSet = model.getJointSet()
        for j in range(jointSet.getSize()):
            joint = jointSet.get(j)
            numCoords = joint.numCoordinates()
            for c in range(numCoords):
                coord = joint.get_coordinates(c)
                coordName = coord.getName()
                absPath = coord.getAbsolutePathString()
                motionType = getMotionTypeString(coord.getMotionType())

                valueName = coordName + '/value'
                speedName = coordName + '/speed'
                if bilateral:
                    valueName, ls_dict = removeSide(valueName, ls_dict)      
                    speedName, ls_dict = removeSide(speedName, ls_dict)
                else:
                    ls_dict[valueName].append('-')
                    ls_dict[speedName].append('-')

                valueAbsPath = absPath + '/value'
                if not state_dict.has_key(valueName):
                    state_dict[valueName] = list()
                state_dict[valueName].append(valueAbsPath)
                coord_motion_types[valueAbsPath] = motionType

                speedAbsPath = absPath + '/speed'
                if not state_dict.has_key(speedName):
                    state_dict[speedName] = list()
                state_dict[speedName].append(speedAbsPath)
                coord_motion_types[speedAbsPath] = motionType


        p = 1
        side = math.ceil(math.sqrt(len(state_names)))
        for i, key in enumerate(state_dict.keys()):
            if p % plots_per_page == 1:
                fig = plt.figure(figsize=(8.5, 11))

            plt.subplot(num_rows, num_cols, p)
            for statePath, ls in zip(state_dict[key], ls_dict[key]):
                state = convert(iterate.getState(statePath))
                for r, ref in enumerate(refs):
                    # Column names are read in with slashes
                    statePathNoSlashes = statePath.replace('/', '')
                    if statePathNoSlashes in ref.dtype.names:
                        plt.plot(ref['time'], ref[statePathNoSlashes], ls=ls, 
                                 color=cmap(cmap_samples[r]))

                plt.plot(time, state, ls=ls, color=cmap(
                         cmap_samples[len(refs)]))

                plt.title(key, fontsize=10)
                plt.xlabel('time (s)', fontsize=8)
                motionType = coord_motion_types[statePath]
                if motionType == 'rotate':
                    if 'value' in statePath:
                        ylabel = 'angle (rad)'
                    elif 'speed' in statePath:
                        ylabel = 'ang. vel. (rad/s)'
                elif motionType == 'translate':
                    if 'value' in statePath:
                        ylabel = 'position (m)'
                    elif 'speed' in statePath:
                        ylabel = 'velocity (m/s)'
                else:
                    ylabel = motionType   

                plt.ylabel(ylabel, fontsize=8)
                plt.xticks(timeticks, fontsize=6)
                plt.yticks(fontsize=6)
                plt.xlim(time[0], time[-1])
                plt.ticklabel_format(axis='y', style='sci', scilimits=(-3, 3))
                ax = plt.gca()
                ax.get_yaxis().get_offset_text().set_position((-0.15,0))
                ax.get_yaxis().get_offset_text().set_fontsize(6)
                ax.tick_params(direction='in', gridOn=True)
                ax.xaxis.set_major_formatter(
                    FormatStrFormatter('%.1f'))



            if (p % plots_per_page == 0) or (i == len(state_names)-1):
                fig.tight_layout()
                pdf.savefig(fig)
                plt.close() 
                p = 1
            else:
                p += 1

	# Controls 
    control_names = iterate.getControlNames()
    if len(control_names) > 0:
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
