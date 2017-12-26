#!/usr/bin/python
import os
import shutil

# CONFIGURATION
EXEC_PREFIX = 'ParameterSetting'
EXECFILENAME = 'dream_single'
CONFIG_PREFIX = 'ParameterSetting'
SETTINGS = [12]
OUTPUTSCRIPT_PREFIX = 'launchExperiment'

sourceDirectory = 'jars'
experimentsDirectory = 'results'
launchScript = True
pbscmd = 'qsub'
postParameters = '1>ZZ_out.txt 2>ZZ_err.txt'

cwd = os.getcwd()

#Script parameters
nbruns = 30
#DO NOT INCREASE THIS PARAMETER. More nodes will do nothing.
nodesPerJob = 1
jobsPerNode = 4
coresPerNode = 16


# Cluster parameters
pbs = []
pbs.append('walltime=20:00:00')

def testAndCreateDirectory(path):
    if os.path.exists(path):
        answer = raw_input('The directory "{}" already exists. Do you want to continue (y/n)? '.format(path))
        if answer == 'n' or answer == 'N':
            quit()
    else:
        os.makedirs(path)


def copyFiles(sourceFile, destPath):
    shutil.copy(sourceFile, destPath)

def linkFiles(sourceFile, destPath):
    os.symlink(sourceFile, destPath)

for setting in SETTINGS:
        # Copy the files
        # JAR file
        experimentName = EXEC_PREFIX + str(setting)
        executableName = EXECFILENAME
        configurationFilename = CONFIG_PREFIX + str(setting) + '.conf'

        expPath = experimentsDirectory + '/' + experimentName
        testAndCreateDirectory(expPath)

        srcfile = sourceDirectory + '/' + executableName
        copyFiles(srcfile, expPath)
        # CONF file
        srcfile = sourceDirectory + '/' + configurationFilename
        copyFiles(srcfile, expPath)

        nbjobs=0
        runs=nbruns
        while (runs>0):
          nbjobs+=1
          runs-=jobsPerNode
        for job in range(nbjobs):
            runPath = experimentsDirectory + '/' + experimentName + '/run_' + str(job)
            testAndCreateDirectory(runPath)

            linkFiles(expPath + '/' + executableName, runPath + '/' + executableName)
            linkFiles(expPath + '/' + configurationFilename, runPath + '/' + configurationFilename)
            outputScriptName = OUTPUTSCRIPT_PREFIX + '_ParameterSetting' + str(setting) + '.sh'
            # create the LISA script
            with open(runPath + '/' + outputScriptName, 'w') as script:
                script.write('#!/bin/sh\n')
                script.write('#PBS -l')
                for p in pbs:
                    script.write(' ' + p)
                script.write('\n')
                script.write('#PBS -lnodes={}:ppn={}\n'.format(nodesPerJob, jobsPerNode))

                script.write('\n')
                script.write('module load gcc/6.3.0\n')
                script.write('module load fann/2.2.0\n')
                script.write('module load boost/1.61\n')
                script.write('export LDFLAGS="-L$HOME/usr/lib -L$HOME/usr/local/lib $LDFLAGS"\n')
                script.write('export LD_LIBRARY_PATH="$HOME/usr/lib:$HOME/usr/local/lib:$LD_LIBRARY_PATH"\n')


                for run in range(job*jobsPerNode,min((job+1)*jobsPerNode,nbruns)):
                    script.write('\n')
                    script.write('\n')
                    script.write('cd $TMPDIR\n')
                    script.write('mkdir ' + experimentName + '_run' + str(run) + '\n')
                    script.write('cd ' + experimentName + '_run' + str(run) + '\n')
                    script.write('cp "' + cwd + '/' + expPath + '/' + executableName + '" .\n')
                    script.write('cp "' + cwd + '/' + expPath + '/' + configurationFilename + '" .\n')

                # script.write('cd "' + cwd + '/' + destPath + '"\n\n')
                    script.write(javacmd + ' -Xmx' + str(javaHeapSize) + 'm' + ' -jar ' + executableName + ' ' + configurationFilename)
                    script.write()
                    postParameters = '1>ZZ_out_setting' + str(setting) + '.txt 2>ZZ_err_setting' + str(setting) + '.txt'
                    script.write(' ' + postParameters + '\n')
                    script.write('\n')

                # Let's copy the data back
                    script.write('tar czf ' + experimentName + '_run' + str(run) + '.tgz ' + experimentName + ' ZZ_* *.conf *.jar\n')
                    script.write('cp ' + experimentName + '_run' + str(run) + '.tgz "' + cwd + '/' + runPath + '"\n')
                    script.write('cp ZZ* "' + cwd + '/' + runPath + '"\n\n')
                    script.write(') &\n')
                
                # Wait for all processes to finish
                script.write('wait')

            if launchScript:
                os.system(pbscmd + ' ' + runPath + '/' + outputScriptName)

