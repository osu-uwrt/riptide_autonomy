#! /usr/bin/env python3

import argparse
import os
import sys
from glob import glob

HOME_DIR = os.getenv("HOME")
if HOME_DIR is None:
    print("Could not get value of HOME environment variable!", file=sys.stderr)
    exit(1)

AUTONOMY_SRC_LOCATION = "{}/osu-uwrt/riptide_software/src/riptide_autonomy/src/riptide_autonomy".format(HOME_DIR)
AUTONOMY_INCLUDE_LOCATION = "{}/osu-uwrt/riptide_software/src/riptide_autonomy/include/riptide_autonomy".format(HOME_DIR)

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


#called when the create task is invoked.
def onCreate(args):
    #figure out node superclass and template
    nodeType = "UWRTBtNode" #placeholder. if this ends up in the file then something was wonky
    template = "nonactionnode_hpp_template"
    if(args.type == "action"):
        nodeType = "UWRTActionNode"
        template = "actionnode_hpp_template"
    elif(args.type == "condition"):
        nodeType = "UWRTConditionNode"
    elif(args.type == "decorator"):
        nodeType = "UWRTDecoratorNode"
        
    
    templateName = "{}/assistant/templates/{}".format(AUTONOMY_SRC_LOCATION, template)    
    newFileName = "{0}/bt_{1}s/{2}.bt{1}.hpp".format(AUTONOMY_INCLUDE_LOCATION, args.type, args.name)
    
    if os.path.exists(newFileName):
        if not askConfirmation(args, "File with name {} already exists. Overwrite it?".format(newFileName)):
            info(args, "Not creating file.") #user said no
            return
    
    createFileFromTemplate(templateName, newFileName, [args.name, nodeType, args.type.lower()])
    info(args, "Created a new {} with name {}.".format(args.type, args.name))
    
    #rebuild autonomy to force cmake to configure and generate appropriate headers
    if args.no_rebuild:
        info(args, "You need to completely rebuild riptide_autonomy2 for the changes to take effect.")
    else:
        onReconfigureAutonomy(args)
        

def onGenerateRegistrators(args):
    location = args.directory
    
    #resolve general node file names
    generalActionFiles = glob("{}/bt_actions/*.btaction.hpp".format(AUTONOMY_INCLUDE_LOCATION))
    generalConditionFiles = glob("{}/bt_conditions/*.btcondition.hpp".format(AUTONOMY_INCLUDE_LOCATION))
    generalDecoratorFiles = glob("{}/bt_decorators/*.btdecorator.hpp".format(AUTONOMY_INCLUDE_LOCATION))
    
    #resolve simple node file names
    simpleActionFiles = glob("{}/bt_actions/*.simple.cpp".format(AUTONOMY_SRC_LOCATION))
    simpleConditionFiles = glob("{}/bt_conditions/*.simple.cpp".format(AUTONOMY_SRC_LOCATION))
    simpleDecoratorFiles = glob("{}/bt_decorators/*.simple.cpp".format(AUTONOMY_SRC_LOCATION))
    
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
