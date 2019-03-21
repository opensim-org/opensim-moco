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
import numpy as np
from collections import defaultdict

import argparse

parser = argparse.ArgumentParser(
    description="Generate a report given 1 or more MocoIterates.")
parser.add_argument('model', type=str,
                    help="Model file name.")
parser.add_argument('file', type=str, nargs='+',
                    help="Paths to MocoIterate files.")
parser.add_argument('--bilateral', action='store_true',
                    help="Plot left and right limb coordinates together.")

args = parser.parse_args()

model = args.model

datafiles = args.file

bilateral = args.bilateral


import pdb

model = osim.Model(model)
iterate = osim.MocoIterate(datafiles[0])



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

def removeSide(name):
    if '_r/' in name:
        name = name.replace('_r/', '/')
    elif '_l/' in name:
        name = name.replace('_l/', '/')
    elif '_r_' in name:
        name = name.replace('_r_', '_')
    elif '_l_' in name:
        name = name.replace('_l_', '_')

    return name

plots_per_page = 18.0
num_cols = 3
num_rows = plots_per_page / 3
with PdfPages('report.pdf') as pdf:

    time = convert(iterate.getTime())
    starttime = time[0]
    endtime = time[-1]

	# States
    state_names = iterate.getStateNames()
    if len(state_names) > 0:

        state_dict = defaultdict(list)
        coord_motion_types = dict()

        coordSet = model.updCoordinateSet()
        for i in range(coordSet.getSize()):
            coord = coordSet.get(i)
            coordName = coord.getName()
            absPath = coord.getAbsolutePathString()
            motionType = getMotionTypeString(coord.getMotionType())

            valueName = coordName + '/value'
            speedName = coordName + '/speed'
            if bilateral:
                valueName = removeSide(valueName)
                speedName = removeSide(speedName)

            valueAbsPath = absPath + '/value'
            state_dict[valueName].append(valueAbsPath)
            coord_motion_types[valueAbsPath] = motionType

            speedAbsPath = absPath + '/speed'
            state_dict[speedName].append(speedAbsPath)
            coord_motion_types[speedAbsPath] = motionType


        p = 1
        side = math.ceil(math.sqrt(len(state_names)))
        for i, key in enumerate(state_dict.keys()):
            if p % plots_per_page == 1:
                fig = plt.figure(figsize=(8.5, 11))

            plt.subplot(num_rows, num_cols, p)
            for statePath in state_dict[key]:
                state = convert(iterate.getState(statePath))
                plt.plot(time, state)
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
                plt.xticks(fontsize=8)
                plt.yticks(fontsize=8)
                plt.xlim(time[0], time[-1])
                plt.ticklabel_format(axis='y', style='sci', scilimits=(0, 4))
                ax = plt.gca()
                ax.get_yaxis().get_offset_text().set_position((-0.15,0))
                ax.get_yaxis().get_offset_text().set_fontsize(8)
                ax.tick_params(direction='in', gridOn=True)




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
