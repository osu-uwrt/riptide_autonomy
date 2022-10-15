#! /usr/bin/env python3

import argparse
from enum import Enum
import os
import sys
from glob import glob

HOME_DIR = os.getenv("HOME")
if HOME_DIR is None:
    print("Could not get value of HOME environment variable!", file=sys.stderr)
    exit(1)

AUTONOMY_SRC_LOCATION = "{}/osu-uwrt/riptide_software/src/riptide_autonomy/src/riptide_autonomy".format(HOME_DIR)
AUTONOMY_INCLUDE_LOCATION = "{}/osu-uwrt/riptide_software/src/riptide_autonomy/include/riptide_autonomy".format(HOME_DIR)
AUTONOMY_TEST_LOCATION = "{}/osu-uwrt/riptide_software/src/riptide_autonomy/test/riptide_autonomy".format(HOME_DIR)


class BtNodeType(Enum):
    ACTION = "UWRTActionNode"
    CONDITION = "UWRTConditionNode"
    DECORATOR = "UWRTDecoratorNode"


#
# UTIL FUNCTIONS
#

#prints the given info message if the silent flag was not given to the program.
def info(args, msg):
    if not args.silent:
        print(msg)


#asks user for confirmation of an action with a y/n unless the --force flag was specified.
def askConfirmation(args, msg):
    if not args.force:
        res = input("{} [y/n]: ".format(msg))
        return res.upper() == "Y"

    return True #user forced


def prompt(args, msg: str, options: 'list[str]', default: int = 1):
    if not args.force:
        print(msg)
        for i in range(0, len(options)):
            print("    {}: {}".format(i + 1, options[i]))
        
        response = input("[{}-{}]? ".format(1, len(options)))
        while not response.isnumeric() or int(response) < 1 or int(response) > len(options):
            response = input("Invalid response: please enter a number from {} to {}: ".format(1, len(options)))
        
        return int(response) - 1
            
    else:
        return default
        

def categorizedGlob(baseDir: str, pattern: str) -> 'tuple[list[str]]':
    actionFiles = glob("{}/bt_actions/{}".format(baseDir, pattern))
    conditionFiles = glob("{}/bt_conditions/{}".format(baseDir, pattern))
    decoratorFiles = glob("{}/bt_decorators/{}".format(baseDir, pattern))
    return actionFiles, conditionFiles, decoratorFiles


def fileNameNoExt(fileName: str):
    base = os.path.basename(fileName)
    noExt = base[ : base.find('.')]
    return noExt


#checks if a directory exists and creates it if it does not
def ensureDirectoryExists(name: str):
    if not os.path.isdir(name):
        os.mkdir(name)
        

#returns the contents of a specified file as a single string.
def readFileToString(name: str):
    f = open(name, 'r')
    contents = "".join(f.readlines())
    f.close()
    
    return contents


#writes the provided string to the file with the provided name.
def writeStringToFile(name: str, contents: str):
    f = open(name, 'w')
    f.write(contents)
    f.close()


#reads the template file at templatePath, subs in the args, and writes the file to targetPath.
#args are handled as such. Any <1>'s in the template file will be replaced by the 0th arg, <2>'s are replaced by the 1st arg, and so on.
def createFileFromTemplate(templatePath: str, targetPath: str, args: 'list[str]'):
    contents = readFileToString(templatePath)
    
    for i in range(0, len(args)):
        contents = contents.replace("<{}>".format(i + 1), args[i])
    
    writeStringToFile(targetPath, contents)


#creates a node file for a node with name nodeName with type nodeType
def createNodeFile(args, nodeName: str, nodeType: BtNodeType):
    #check node type
    if not isinstance(nodeType, BtNodeType):
        raise TypeError("Parameter nodeType must be a member of the BtNodeType enum.")
    
    template = "actionnode_hpp_template" if nodeType == BtNodeType.ACTION else "nonactionnode_hpp_template"
        
    templatePath = "{}/assistant/templates/{}".format(AUTONOMY_SRC_LOCATION, template)    
    newFileName = "{0}/bt_{1}s/{2}.bt{1}.hpp".format(AUTONOMY_INCLUDE_LOCATION, nodeType.name.lower(), nodeName)
    
    if os.path.exists(newFileName):
        if not askConfirmation(args, "File with name {} already exists. Overwrite it?".format(newFileName)):
            info(args, "Not creating file.") #user said no
            return
    
    createFileFromTemplate(templatePath, newFileName, [nodeName, nodeType.value])
    info(args, "Created a new {} with name {}.".format(nodeType.value, nodeName))


#creates a test file for a node with name nodeName of type nodeType.
def createTestFile(args, nodeName: str, nodeType: BtNodeType):
    #check node type
    if not isinstance(nodeType, BtNodeType):
        raise TypeError("Parameter nodeType must be a member of the BtNodeType enum.")
    
    templatePath = "{}/assistant/templates/test_template".format(AUTONOMY_SRC_LOCATION)
    newFileName = "{0}/bt_{1}s/Test{2}.cpp".format(AUTONOMY_TEST_LOCATION, nodeType.name.lower(), nodeName)

    if os.path.exists(newFileName):
        if not askConfirmation(args, "File with name {} already exists. Overwrite it?".format(newFileName)):
            info(args, "Not creating file.") #user said no
            return
    
    createFileFromTemplate(templatePath, newFileName, [nodeName])
    info(args, "Created test file for {} {}.".format(nodeType.name.lower(), nodeName))

#
# CALLBACKS
#

#called when the rebuild_autonomy task is invoked
def onReconfigureAutonomy(args):
    info(args, "Reconfiguring and building the riptide_autonomy2 package...")
    
    workspaceDir = "{}/osu-uwrt/riptide_software".format(HOME_DIR)
    os.chdir(workspaceDir)
    
    #run command to reconfigure and build
    res = os.system("colcon build --cmake-clean-cache --packages-select riptide_autonomy2")
    
    if res:
        info(args, "Error reconfiguring autonomy.")


#called when the check task is invoked
def onCheck(args):
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
        
        fileNames = [fileNameNoExt(file) for file in files]
        testNames = [fileNameNoExt(test).replace("Test", "") for test in tests] # ex. "TestActuate.cpp" becomes Actuate
        
        filesWithoutTests = [file for file in fileNames if file not in testNames]
        testsWithoutFiles = [test for test in testNames if test not in fileNames]
        
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
                    pathToFile = "{0}/bt_{1}s/{2}.bt{1}.hpp".format(AUTONOMY_INCLUDE_LOCATION, nodeType.name.lower(), file)
                    os.remove(pathToFile)
                    info(args, "Removed {} {}.".format(nodeType.name.lower(), file))
                    madeChanges = True
            #3 is do nothing
            elif response == 3: #do nothing for all
                skipAllPrompts = True
            
        
        for test in testsWithoutFiles:
            response = conditionalPrompt(
                "Test file Test{} does not correspond with an existing BT node. What would you like to do?".format(test),
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
                    pathToFile = "{}/bt_{}s/Test{}.cpp".format(AUTONOMY_TEST_LOCATION, nodeType.name.lower(), test)
                    os.remove(pathToFile)
                    info(args, "Removed test for {} {}".format(nodeType.name.lower(), test))
                    madeChanges = True
            #3 is do nothing
            elif response == 3: #do nothing for all
                skipAllPrompts = True
        
        return madeChanges
            
    #ACTUAL CHECK FUNCTION STUFF STARTS HERE
    
    #get list of bt node and test files
    actionFiles, conditionFiles, decoratorFiles = categorizedGlob(AUTONOMY_INCLUDE_LOCATION, "*.hpp")
    actionTests, conditionTests, decoratorTests = categorizedGlob(AUTONOMY_TEST_LOCATION, "*.cpp")
        
    info(args, "BT Actions")
    actionsHadChanges = analyzeFiles(actionFiles, actionTests, BtNodeType.ACTION)
    
    info(args, "BT Conditions")
    conditionsHadChanges = analyzeFiles(conditionFiles, conditionTests, BtNodeType.CONDITION)
    
    info(args, "BT Decorators")
    decoratorsHadChanges = analyzeFiles(decoratorFiles, decoratorTests, BtNodeType.DECORATOR)
    
    #if changes were made, ask the user to rebuild them
    if actionsHadChanges or conditionsHadChanges or decoratorsHadChanges:
        if askConfirmation(args, "Changes were made to the workspace that need to be built. Rebuild autonomy now?"):
            onReconfigureAutonomy(args)
        else:
            info(args, "Not rebuilding autonomy. Remember to rebuild it yourself before running it so your changes can take effect")
    
    info(args, "Check complete.")
    


#called when the create task is invoked.
def onCreate(args):
    nodeType = None
    try:
        #select enum value by name, which is the user type specification but uppercase.
        #this statement must populate nodeType or else catch block will fire
        nodeType = BtNodeType[args.type.upper()] 
    except KeyError:
        print("Node Type {} not recognized/implemented!".format(args.type), file=sys.stderr)
        exit(1)
    
    #create file
    createNodeFile(args, args.name, nodeType)
    createTestFile(args, args.name, nodeType)
    
    #rebuild autonomy to force cmake to configure and generate appropriate headers
    if args.no_rebuild:
        info(args, "You need to completely rebuild riptide_autonomy2 for the changes to take effect.")
    else:
        onReconfigureAutonomy(args)


def onGenerateRegistrators(args):
    location = args.directory
    
    #resolve general node file names    
    generalActionFiles, generalConditionFiles, generalDecoratorFiles = categorizedGlob(AUTONOMY_INCLUDE_LOCATION, "*.hpp")
    
    #resolve simple node file names
    simpleActionFiles, simpleConditionFiles, simpleDecoratorFiles = categorizedGlob(AUTONOMY_SRC_LOCATION, "*.simple.cpp")
    
    #creates a single registrator
    def createRegistrator(generalFiles: 'list[str]', simpleFiles: 'list[str]', location: str):
        registrations = ""
        headers = ""
        for file in generalFiles:
            # perform general registration on all other hpps
            registrations += "    factory.registerNodeType<{0}>(\"{0}\");\n".format(fileNameNoExt(file))
            
            #figure out node type (its in the name between the second to last and last dots)
            lastDot = file.rfind(".")
            secondLastDot = file.rfind(".", 0, lastDot - 1)
                            
            #not checking dots because files need to have two to be globbed
            nodeType = file[secondLastDot + 1 : lastDot]
            
            nodeDir = ""
            if(nodeType == "btaction"):
                nodeDir = "bt_actions"
            elif(nodeType == "btcondition"):
                nodeDir = "bt_conditions"
            elif(nodeType == "btdecorator"):
                nodeDir = "bt_decorators"
            
            headers += "#include \"riptide_autonomy/{}/{}\"\n".format(nodeDir, os.path.basename(file))
        
        for file in simpleFiles:
            registrations += "    {0}::bulkRegister(factory);\n".format(fileNameNoExt(file))
        
        templatePath = "{}/assistant/templates/plugin_registrator_template".format(AUTONOMY_SRC_LOCATION)
        createFileFromTemplate(templatePath, location, [headers, registrations])
    
    createRegistrator(generalActionFiles, simpleActionFiles, "{}/registerActions.cpp".format(location))
    createRegistrator(generalConditionFiles, simpleConditionFiles, "{}/registerConditions.cpp".format(location))
    createRegistrator(generalDecoratorFiles, simpleDecoratorFiles, "{}/registerDecorators.cpp".format(location))
    
    info(args, "Generated registrators in directory {}".format(os.path.abspath(location)))
        
        
#
# ARGPARSE TASK DEFINITIONS
#

#task struct containing name, callback, and description. arguments are custom
class Task:
    class Argument:
        def __init__(self, name: str, options: 'list[str]' = None, optional: bool = False):
            self.name = name
            self.options = options
            self.optional = optional
        
    def __init__(self, callback, description: str, args: 'list[Argument]'):
        self.callback = callback
        self.description = description
        self.args = args
        
#
# map of task options that this program supports
#
taskOptions = {
    "reconfigure_autonomy" : Task(
        onReconfigureAutonomy,
        "Clean and re-build the riptide_autonomy2 package.",
        []
    ),
    
    "check" : Task(
        onCheck,
        "Check the status of the autonomy package and recommend steps to keep it in an ideal state. Using this with the -f option will skip all user prompts and make no changes to the workspace.",
        []
    ),
    
    "create" : Task(
        onCreate, 
        "Create a new custom action, decorator, or condition node.", 
        [
            Task.Argument("type", options=["action", "condition", "decorator"]),
            Task.Argument('name'),
            Task.Argument("--no-rebuild", optional=True)
        ]
    ),
    
    "generate_registrators" : Task(
        onGenerateRegistrators,
        "Generate C++ source code for plugin registrators.",
        [
            Task.Argument("directory")
        ]
    )
}

#
# ARGPARSE AND MAIN
# 

def createArgParser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    
    #optional silent flag
    parser.add_argument("-s", "--silent", action="store_true", help="Silence relgular terminal output. Errors will still print")
    parser.add_argument("-f", "--force", action="store_true", help="Force the assistant to attempt to complete the task without asking user for confirmation of some acitons like file overwrites.")
    
    subparsers = parser.add_subparsers(dest="task", help="The task to complete", required=True)
    for taskName in taskOptions.keys():
        task = taskOptions[taskName]
        subparser = subparsers.add_parser(taskName, help=task.description)
        
        for arg in task.args:
            if arg.optional:
                subparser.add_argument(arg.name, action="store_true")
            else:
                subparser.add_argument(arg.name, choices=arg.options)

    
    return parser


def main():
    parser = createArgParser()
    args = parser.parse_args()
    
    #run desired task
    taskOptions[args.task].callback(args)


if __name__ == "__main__":
    main()
