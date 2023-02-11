import os, warnings

removeCopiedFiles = True
removeCommonFiles = True

expdataSource = './ExpData_blackbike/'
targetDirectory = '/media/sda1/storageBikeProject/ExpData/'
csvSourceFileList = os.listdir(expdataSource)
csvTargetFileList = os.listdir(targetDirectory)

newExpFileList = list(set(csvSourceFileList).difference(set(csvTargetFileList)))
print('The following ' + str(len(newExpFileList)) + ' files will be copied to -> ' + targetDirectory)
print(newExpFileList)

commonExpFileList = list(set(csvSourceFileList).intersection(set(csvTargetFileList)))


for ind in range(len(newExpFileList)):
    fileName = newExpFileList[ind]
    cmdLine = 'cp ' + expdataSource + fileName + ' ' + targetDirectory + fileName
    print(cmdLine)
    os.system(cmdLine)

if removeCopiedFiles:
    warnings.warn('Going to delete the Copied local files, Are u sure?')
    decision = input(
        'Type N or n to abort')
    if decision is not ('N' or 'n'):
        for ind in range(len(newExpFileList)):
            fileName = newExpFileList[ind]
            cmdLineDelete = 'rm ' + expdataSource + fileName
            print(cmdLineDelete)
            os.system(cmdLineDelete)

if removeCommonFiles:
    warnings.warn('Going to delete the Common local files, Are u sure?')
    decision = input(
        'Type N or n to abort')
    if decision is not ('N' or 'n'):
        for ind in range(len(commonExpFileList)):
            fileName = commonExpFileList[ind]
            cmdLineDelete = 'rm ' + expdataSource + fileName
            print(cmdLineDelete)
            os.system(cmdLineDelete)



