import os
import shutil

# CONFIGURATION
JARFILE_PREFIX = 'ParameterSetting'
JARFILENAME = 'RandomSeed.jar'
CONFIG_PREFIX = 'ParameterSetting'
SETTINGS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109]
OUTPUTSCRIPT_PREFIX = 'launchGecco'

sourceDirectory = 'jars'
experimentsDirectory = 'results'
launchScript = False
pbscmd = 'qsub'
javacmd = 'java'
postParameters = '1>ZZ_out.txt 2>ZZ_err.txt'

cwd = os.getcwd()

# Java parameters
javaHeapSize = 4096

#Script parameters
nbruns = 40
nodesPerJob = 1
jobsPerNode = 1
coresPerNode = 1


# Cluster parameters
pbs = []
pbs.append('walltime=30:00:00')

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
        experimentName = JARFILE_PREFIX + str(setting)
        executableName = JARFILENAME
        configurationFilename = CONFIG_PREFIX + str(setting) + '.conf'

        expPath = experimentsDirectory + '/' + experimentName
        testAndCreateDirectory(expPath)

        srcfile = sourceDirectory + '/' + executableName
        copyFiles(srcfile, expPath)
        # CONF file
        srcfile = sourceDirectory + '/' + configurationFilename
        copyFiles(srcfile, expPath)

        for run in range(nbruns):
            runPath = experimentsDirectory + '/' + experimentName + '/' + str(run)
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

                # Need the correct Java version
                script.write('\n')
                script.write('module load java\n')

                script.write('\n')
                script.write('cd $TMPDIR\n')
                script.write('mkdir ' + experimentName + '_run' + str(run) + '\n')
                script.write('cd ' + experimentName + '_run' + str(run) + '\n')
                script.write('cp "' + cwd + '/' + expPath + '/' + executableName + '" .\n')
                script.write('cp "' + cwd + '/' + expPath + '/' + configurationFilename + '" .\n')

                # script.write('cd "' + cwd + '/' + destPath + '"\n\n')
                script.write(javacmd + ' -Xmx' + str(javaHeapSize) + 'm' + ' -jar ' + executableName + ' ' + configurationFilename)
                postParameters = '1>ZZ_out_setting' + str(setting) + '.txt 2>ZZ_err_setting' + str(setting) + '.txt' 
                script.write(' ' + postParameters + '\n')
                script.write('\n')

                # Let's copy the data back
                script.write('tar czf ' + experimentName + '_run' + str(run) + '.tgz ' + experimentName + ' ZZ_* *.conf\n')
                script.write('cp ' + experimentName + '_run' + str(run) + '.tgz "' + cwd + '/' + runPath + '"\n')
                script.write('cp ZZ* "' + cwd + '/' + runPath + '"\n\n')


            if launchScript:
                os.system(pbscmd + ' ' + runPath + '/' + outputScriptName)


