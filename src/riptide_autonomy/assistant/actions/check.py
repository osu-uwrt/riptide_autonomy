import os

from actions.reconfigure import doReconfigure
from ament_index_python import get_package_prefix
from util import (BtNodeType, askConfirmation, autonomyIncludeLocation,
                  autonomyTestLocation, categorizedGlob, createNodeFile,
                  createTestFile, fileNameNoExt, info, prompt)


def checkFiles(args, autonomyRootLoc: str):
    skipAllPrompts = args.force
    
    #gives a prompt to the user as long as skipAllPrompts is False. Returns the user-selected result as an integer
    def conditionalPrompt(msg: str, options: 'list[str]', default: int = 1):
        if not skipAllPrompts:
            return prompt(args, msg, options, default)
    
    
    #analyzes files and their tests to figure out if something is missing. The goal is for every file to have its own test.
    #returns true if changes were made to the workspace that need to be built. (false otherwise)
    def analyzeFiles(files: 'list[str]', tests: 'list[str]', nodeType: BtNodeType):
        nonlocal skipAllPrompts
        
        #check bt node type
        if not isinstance(nodeType, BtNodeType):
            raise TypeError("Parameter nodeType must be a member of the BtNodeType enum.")
        
        madeChanges = False
        
        #names of files
        fileNames = [fileNameNoExt(file) for file in files]
        testNames = [fileNameNoExt(test) for test in tests]
        
        #convention is for every file (say XYZ) in fileNames, there should be a test file named TestXYZ
        filesWithoutTests = [file for file in fileNames if "Test{}".format(file) not in testNames]
        testsWithoutFiles = [test for test in testNames if test.replace("Test", "") not in fileNames]
        
        #print summary
        info(args, "Found {} files.".format(len(fileNames)))
        info(args, "Found {} tests.".format(len(testNames)))
        info(args, "Found {} files without tests.".format(len(filesWithoutTests)))
        info(args, "Found {} useless tests.\n".format(len(testsWithoutFiles)))
        
        # #ask user what they want to do about files without tests
        for file in filesWithoutTests:
            response = conditionalPrompt(
                "BT Node {} has a file definition but no test. What would you like to do?".format(file),
                [
                    "Create a test file",
                    "Delete the file",
                    "Do nothing",
                    "Do nothing for all"
                ],
                2 #default is do nothing
            )
            
            if response == 0: #create test file
                createTestFile(args, file, nodeType)
                madeChanges = True
            elif response == 1: #delete file
                if askConfirmation(args, "Do you really want to delete {}?".format(file)):
                    pathToFile = "{0}/bt_{1}s/{2}.bt{1}.hpp".format(autonomyIncludeLocation(autonomyRootLoc), nodeType.name.lower(), file)
                    os.remove(pathToFile)
                    info(args, "Removed {} {}.".format(nodeType.name.lower(), file))
                    madeChanges = True
            #3 is do nothing
            elif response == 3: #do nothing for all
                skipAllPrompts = True
            
        
        for test in testsWithoutFiles:
            response = conditionalPrompt(
                "Test file {} does not correspond with an existing BT node. What would you like to do?".format(test),
                [
                    "Create a Node file",
                    "Delete the test",
                    "Do nothing",
                    "Do nothing for all"
                ],
                2 #default is do nothing
            )
            
            if response == 0: #create
                createNodeFile(args, test, nodeType)
                madeChanges = True
            elif response == 1: #delete
                if askConfirmation(args, "Are you sure you want to delete the test?"):
                    pathToFile = "{}/bt_{}s/{}.cpp".format(autonomyTestLocation(autonomyRootLoc), nodeType.name.lower(), test)
                    os.remove(pathToFile)
                    info(args, "Removed test {}".format(test))
                    madeChanges = True
            #3 is do nothing
            elif response == 3: #do nothing for all
                skipAllPrompts = True
        
        return madeChanges
    
    #
    # ACTUAL FILE CHECK FUNCTION STUFF STARTS HERE
    #
    
    #get list of bt node and test files
    actionFiles, conditionFiles, decoratorFiles = categorizedGlob(autonomyIncludeLocation(autonomyRootLoc), "*.hpp")
    actionTests, conditionTests, decoratorTests = categorizedGlob(autonomyTestLocation(autonomyRootLoc), "*.cpp")
        
    info(args, "BT Actions")
    actionsHadChanges = analyzeFiles(actionFiles, actionTests, BtNodeType.ACTION)
    
    info(args, "BT Conditions")
    conditionsHadChanges = analyzeFiles(conditionFiles, conditionTests, BtNodeType.CONDITION)
    
    info(args, "BT Decorators")
    decoratorsHadChanges = analyzeFiles(decoratorFiles, decoratorTests, BtNodeType.DECORATOR)
    
    #if changes were made, ask the user to rebuild them
    if actionsHadChanges or conditionsHadChanges or decoratorsHadChanges:
        if askConfirmation(args, "Changes were made to the colcon workspace that need to be built. Rebuild autonomy now?"):
            doReconfigure(args, autonomyRootLoc, False, False)
        else:
            info(args, "Not rebuilding autonomy. Remember to rebuild it yourself before running it so your changes can take effect")
    
    return True
    
#invokes the checkBT executable.
#returns True on success, False on fail.
def checkBT():
    return not os.system("ros2 run riptide_autonomy2 checkBT")


#invokes the test_nodes executable
def checkNodes(args, autonomyRootLoc: str):
    testExecPath = os.path.join(get_package_prefix("riptide_autonomy2"), "lib", "riptide_autonomy2", "test_nodes")
    
    #reconfigure and rebuild autonomy with tests enabled
    info(args, "Building autonomy with tests enabled")
    success = doReconfigure(args, autonomyRootLoc, True, False)
    
    if not success:
        info(args, "Error rebuilding autonomy.")
        return
        
    #directly invoke test_nodes executable
    return not os.system(testExecPath)


#runs all checks and summarizes results
def checkSystem(args, autonomyRootLoc: str):
    fileCheck = checkFiles(args, autonomyRootLoc)
    btCheck   = checkBT()
    nodeCheck = checkNodes(args, autonomyRootLoc)
    
    info(args, "\n\nCHECK SUMMARY:")
    info(args, f"Files: {'good' if fileCheck else 'bad'}")
    info(args, f"BehaviorTree workspace: {'good' if btCheck else 'bad'}")
    info(args, f"Nodes: {'good' if nodeCheck else 'bad'}")
    
    return fileCheck and btCheck and nodeCheck


#called when the check task is invoked
def onCheck(args, autonomyRootLoc: str):
    success = False
    
    if args.module == "files":
        success = checkFiles(args, autonomyRootLoc)
    elif args.module == "behaviortree":
        success = checkBT()
    elif args.module == "nodes":
        success = checkNodes(args, autonomyRootLoc)
    elif args.module == "system":
        success = checkSystem(args, autonomyRootLoc)
    else:
        info(args, f"Fatal: cannot check module {args.module}. Check is undefined")

    info(args, "\nCheck completed.")
    info(args, f"Check was {'good' if success else 'bad'}.")
