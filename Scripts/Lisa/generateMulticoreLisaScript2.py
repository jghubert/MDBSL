import math
import os

# Filename of the scripts to launch
SETTINGS = [13, 14]
nbruns = len(SETTINGS)
nbrrep=22

experimentScriptName_PREFIX = 'launchGecco'
experimentName = 'ParameterSetting'
experimentsDirectory = 'results'
# Filename of the generated script
outputScriptName_PREFIX = 'launchMultiCoreGecco'
cwd = os.getcwd()
sh = 'sh'


#Script parameters
nodesPerJob = 1
coresPerJob = 1
coresPerNode = 16
maxJobPerNode = 8
jobsPerNode = min(int(math.floor(coresPerNode / coresPerJob)), maxJobPerNode)
nodesNeeded = int(math.ceil(float(nbruns) / jobsPerNode))

# Cluster parameters
pbs = []
pbs.append('walltime=80:00:00')

script_filenames = []
for setting in SETTINGS:
	script_filenames.append(experimentScriptName_PREFIX + '_ParameterSetting' + str(setting) + '.sh')


index = 0
for n in range(nodesNeeded):

    filename = experimentsDirectory + '/' + outputScriptName_PREFIX + '_' + str(n) + '.sh'
    print('Creating file {}...'.format(filename))
    with open(filename, 'w') as script:
        script.write('#!/bin/sh\n')
        script.write('#PBS -l')
        for p in pbs:
            script.write(' ' + p)
        script.write('\n')
        script.write('#PBS -lnodes={}:ppn={}\n'.format(nodesPerJob, jobsPerNode))
        script.write('\n')

        script.write('cd ' + cwd + '/' + experimentsDirectory + '\n\n')

        nbjobs = min(jobsPerNode, nbruns)
        for j in range(nbruns):
            filename = script_filenames[index]
            script.write(sh + ' ' +  outputScriptName_PREFIX + '/run' + str(j) + '/'+ filename + ' &\n\n')
            index += 1
        nbruns -= nbjobs

        script.write('wait\n')
