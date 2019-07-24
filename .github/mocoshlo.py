
#!/usr/bin/env python3

import os
import sys
import yaml
import argparse
import datetime
from pathlib import Path

parser = argparse.ArgumentParser(
        description="Submit a Moco job to the Stanford Sherlock computing cluster.")
parser.add_argument('--directory', type=str, default="",
        help="Location of input files for running a job on Sherlock.")
parser.add_argument('--build-docker', metavar='gitref:dockertag', type=str, 
        dest='build_docker', default="",
        help="Build a new Docker container of opensim-moco using the git "
        "commit/tag gitref, upload the container to DockerHub using tag "
        "dockertag, and download the container to Sherlock as a Singularity "
        "container.")
parser.add_argument('--duration', type=str, default="00:30:00",
        help="Maximum duration for the job in HH:MM:SS.")
parser.add_argument('--name', type=str, default="",
        help="A name for the job (default: directory name).")
parser.add_argument('--note', type=str, default="",
        help="A note to save to the directory (as note.txt).")
parser.add_argument('--tag', type=str, default="latest",
        help="The tag of the Singularity container to use.")

parser.add_argument('--sshmaster', dest='sshmaster', action='store_true',
        help="Start master SSH session (default).")
parser.add_argument('--no-sshmaster', dest='sshmaster', action='store_false',
        help="Do not start master SSH session (if you already started one).")
parser.add_argument('--sshexit', dest='sshexit', action='store_true',
        help="Exit the master session after submitting the job (default).")
parser.add_argument('--no-sshexit', dest='sshexit', action='store_false',
        help=("Do not exit the master session after submitting (if you plan to "
            "submit more jobs."))
parser.set_defaults(sshmaster=True, sshexit=True)

# TODO windows
# TODO initial configuring gdrive on Sherlock.
# TODO building a docker container from a branch (as a separate step?).
# TODO allow customizing where files are saved in Google Drive.


args = parser.parse_args()

if not args.directory and not args.build_docker:
    raise Exception("Must provide either --directory or --build-docker.")
if args.directory and args.build_docker:
    raise Exception("Must provide either --directory or --build-docker, not "
            "both.")

directory = args.directory
if args.name != "":
    name = args.name
else:
    name = Path(directory).name
duration = args.duration
note = args.note
sshmaster = args.sshmaster
sshexit = args.sshexit

if ' ' in name:
    raise Exception("Cannot have spaces in name.")

with open('config.yaml') as f:
    config = yaml.safe_load(f)

sunetid = config['sunetid']

if directory:
    # Check that the directory contains setup.omoco.
    if not os.path.exists(os.path.join(directory, 'setup.omoco')):
        raise Exception(f"setup.omoco is missing from {directory}.")

    # if note:
    #     with open(os.path.join(directory, 'note.txt'), 'w') as f:
    #         f.write(note)

#docker build -- build-arg GITHUBTOKEN --build-arg MOCOBRANCH=... --tag
#chrisdembia/opensim-moco:TODO .
#docker push chrisdembia/opensim-moco:TODO
#singularity pull docker://chrisdembia/opensim-moco:TODO



home = str(Path.home()) # Should work on Windows and UNIX.
if not os.path.exists(f'{home}/.ssh/controlmasters/'):
    os.makedirs(f'{home}/.ssh/controlmasters')
# Create master (-M) SSH session in the background (-f), without running
# a command (-N), with 
# https://unix.stackexchange.com/questions/83806/how-to-kill-ssh-session-that-was-started-with-the-f-option-run-in-background
control_path = "~/.ssh/controlmasters/%C"
server = f"{sunetid}@login.sherlock.stanford.edu"
now = datetime.datetime.now()
date = now.strftime('%Y-%m-%d')
time = '%s.%i' % (now.strftime('%Y-%m-%dT%H%M%S'), now.microsecond)
job_directory = '%s-%s' % (time, name)
print(f"Submitting {job_directory}")
mocojobs_dir = f"~/nmbl/mocojobs/"
server_job_dir = f"{mocojobs_dir}{job_directory}"
if sshmaster:
    os.system(f'ssh -M -f -N -S {control_path} {server}')

batch = f"""#!/bin/bash
#SBATCH --job-name={name}
#SBATCH --output={name}.out
#SBATCH --error={name}.err
#SBATCH --time={duration}
#SBATCH --mail-type=END
#SBATCH --mail-user={sunetid}@stanford.edu
#SBATCH --nodes=1
#SBATCH --partition=owners,normal
module load gcc/8.1.0

container=$GROUP_HOME/opensim-moco/opensim-moco_latest.sif
singularity exec $container /opensim-moco-install/bin/opensim-moco run-tool setup.omoco

# TODO: generate report.
singularity exec $container opensim-moco-generate-report MODEL SOLUTION


# Upload results to Google Drive.
module load system gdrive

# TODO: What happens when we hit max files to list (--max)?
opensim_moco_folder_id=$(gdrive list --absolute | grep 'opensim-moco ' | cut -d" " -f1)
if [[ -z "$opensim_moco_folder_id" ]]; then
    echo "Creating opensim-moco folder."
    opensim_moco_folder_id=$(gdrive mkdir opensim-moco | cut -d" " -f2)
else
    echo "opensim-moco folder exists."
fi

# date_folder_id=$(gdrive list | grep 'opensim-moco/{date}' | cut -d" " -f1)
# if [[ -z "$date_folder_id" ]]; then
#     date_folder_id=$(gdrive mkdir --parent $opensim_moco_folder_id {date} | cut -d" " -f2)
# fi

# Copy results.
# gdrive upload --recursive --parent $date_folder_id {server_job_dir}
gdrive upload --recursive --parent $opensim_moco_folder_id {server_job_dir}

"""

with open(f'{directory}/{name}.batch', 'w') as f:
    f.write(batch)

# Re-use existing SSH tunnel.
# Recursively make the job directory.
os.system(f'ssh -S {control_path} {server} "mkdir -p {mocojobs_dir}"')
os.system(f"scp -o ControlPath={control_path} -r '{directory}/' {server}:{server_job_dir}")

os.system(f'ssh -S {control_path} {server} "cd {server_job_dir} && echo \"{note}\" > note.txt && sbatch {name}.batch"')

if sshexit:
    os.system(f'ssh -S {control_path} -O exit {server}')
