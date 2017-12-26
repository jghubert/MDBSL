import math
import os

# Filename of the scripts to launch
SETTINGS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109]
nbrset = len(SETTINGS)
nbrrep = [50, 6, 30, 4, 50, 10, 10, 30, 30, 30, 50, 10, 5, 30, 30, 30, 30, 10, 30, 10, 50, 10, 10, 30, 50, 50, 30, 10, 30, 30, 50, 50, 30, 30, 10, 20, 30]
nbrrepmax = 30

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
maxJobPerNode = 16
jobsPerNode = min(int(math.floor(coresPerNode / coresPerJob)), maxJobPerNode)
nodesNeeded = int(math.ceil(float(nbrset*nbrrepmax) / jobsPerNode))

# Cluster parameters
pbs = []
pbs.append('walltime=100:00:00')

script_filenames = []
for setting in SETTINGS:
	script_filenames.append(experimentScriptName_PREFIX + '_ParameterSetting' + str(setting) + '.sh')


index = 0
setting= 0
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

        #nbjobs = min(jobsPerNode, nbruns)
        for j in range(jobsPerNode):
            filename = script_filenames[setting]
            script.write(sh + ' ' +  'ParameterSetting' + str(SETTINGS[setting]) + '/' + str(index) + '/'+ filename + ' &\n\n')
            index += 1
	    if index == nbrrepmax:
		index = 0
		setting += 1
	    if setting == len(SETTINGS):
		break 
       #nbruns -= nbjobs

        script.write('wait\n')
