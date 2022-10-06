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
SHELL_ASSISTANT_FILE  = "{}/assistant/shellassistant.sh".format(AUTONOMY_SRC_LOCATION)

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
    res = os.system("colcon build --cmake-force-configure")
    
    if res:
        info(args, "Error reconfiguring autonomy.")


#called when the create task is invoked.
def onCreate(args):
    templateName = "{}/assistant/templates/node_src_template".format(AUTONOMY_SRC_LOCATION, args.type)    
    newFileName = "{0}/bt_{1}s/{2}.bt{1}.cpp".format(AUTONOMY_SRC_LOCATION, args.type, args.name)
    
    if os.path.exists(newFileName):
        if not askConfirmation(args, "File with name {} already exists. Overwrite it?".format(newFileName)):
            info(args, "Not creating file.") #user said no
            return
    
    createFileFromTemplate(templateName, newFileName, [args.name, args.type.lower()])
    info(args, "Created a new {} with name {}.".format(args.type, args.name))
    
    #rebuild autonomy to force cmake to configure and generate appropriate headers
    if args.no_rebuild:
        info(args, "You need to completely rebuild riptide_autonomy2 for the changes to take effect.")
    else:
        onReconfigureAutonomy(args)
    
    
#called when the generate headers action is invoked.
def onGenerateHeaders(args):
    absdir = os.path.abspath(args.target_directory)
    info(args, "Generating headers in directory {}".format(absdir))
    
    #ensure necessary directories exist
    ensureDirectoryExists(args.target_directory)
    ensureDirectoryExists("{}/{}".format(args.target_directory, "bt_actions"))
    ensureDirectoryExists("{}/{}".format(args.target_directory, "bt_conditions"))
    ensureDirectoryExists("{}/{}".format(args.target_directory, "bt_decorators"))
    
    #resolve source file names
    actionFiles = glob("{}/bt_actions/*.btaction.cpp".format(AUTONOMY_SRC_LOCATION))
    conditionFiles = glob("{}/bt_conditions/*.btcondition.cpp".format(AUTONOMY_SRC_LOCATION))
    decoratorFiles = glob("{}/bt_decorators/*.btdecorator.cpp".format(AUTONOMY_SRC_LOCATION))
    
    #resolve template names
    headerTemplate = "{}/assistant/templates/node_header_template".format(AUTONOMY_SRC_LOCATION)
    autonomyTemplate = "{}/assistant/templates/autonomy_header_template".format(AUTONOMY_SRC_LOCATION)
    simpleTemplate = "{}/assistant/templates/simple_header_template".format(AUTONOMY_SRC_LOCATION)
    
    #populates header files from templates with two arguments (first is uppercase name, second is titlecase name)
    #returns a list containing all populated header files
    def populateHeadersFromTemplates(directory: str, files: 'list[str]', headerSuperclass: str):
        headersList = []
        for filePath in files:
            #get actual name of file (without extension)
            fileName = fileNameNoExt(filePath)
            info(args, "Creating header file for {}".format(fileName))

            # handle the bulk condition
            if(".simple." in filePath):
                #install file from template
                installFileName = "{}/{}/{}.h".format(args.target_directory, directory, fileName)
                createFileFromTemplate(simpleTemplate, installFileName, [fileName.upper(), fileName, headerSuperclass])
                headersList.append("{}/{}.h".format(directory, fileName))

            else:
                #install file from template
                installFileName = "{}/{}/{}.h".format(args.target_directory, directory, fileName)
                createFileFromTemplate(headerTemplate, installFileName, [fileName.upper(), fileName, headerSuperclass])
                headersList.append("{}/{}.h".format(directory, fileName))
        
        return headersList
        
    actionHeaders    = populateHeadersFromTemplates("bt_actions", actionFiles, "UWRTSyncActionNode")
    conditionHeaders = populateHeadersFromTemplates("bt_conditions", conditionFiles, "BT::ConditionNode")
    decoratorHeaders = populateHeadersFromTemplates("bt_decorators", decoratorFiles, "BT::DecoratorNode")
    
    #create autonomy.h file. only need to fill in the #include's for all custom nodes
    includes = ""
    for header in (actionHeaders + conditionHeaders + decoratorHeaders):
        includes += "#include \"{}\"\n".format(header) # ex. #include "Actuate.h"\n
    
    autonomyHeaderLocation = "{}/autonomy.h".format(args.target_directory)
    createFileFromTemplate(autonomyTemplate, autonomyHeaderLocation, [includes])
    

def onGenerateRegistrator(args):
    location = args.file_location
    
    #resolve source file names
    actionFiles = glob("{}/bt_actions/*.btaction.cpp".format(AUTONOMY_SRC_LOCATION))
    conditionFiles = glob("{}/bt_conditions/*.btcondition.cpp".format(AUTONOMY_SRC_LOCATION))
    decoratorFiles = glob("{}/bt_decorators/*.btdecorator.cpp".format(AUTONOMY_SRC_LOCATION))
    
    #string containing code that registers the nodes
    registrations = ""
    for file in (actionFiles + conditionFiles + decoratorFiles):
        # perform bulk registration
        if(".simple." in file):
            registrations += "    {0}::bulkRegister(factory);\n".format(fileNameNoExt(file))
        # perform general registration
        else:
            registrations += "    factory.registerNodeType<{0}>(\"{0}\");\n".format(fileNameNoExt(file))
    
    template = "{}/assistant/templates/node_registrator_template".format(AUTONOMY_SRC_LOCATION)
    createFileFromTemplate(template, location, [registrations])
    info(args, "Generated registrator source code at {}".format(os.path.abspath(location)))
        
    
#
# ARGPARSE TASK DEFINITIONS
#

#task struct containing name, callback, and description. arguments are custom
class Task:
    class Argument:
        def __init__(self, name, options=None, optional=False):
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
    
    "generate_headers" : Task(
        onGenerateHeaders,
        "Generate headers for custom nodes in the specified target directory.",
        [
            Task.Argument("target_directory")
        ]
    ),
    
    "generate_registrator" : Task(
        onGenerateRegistrator,
        "Generate C++ source code for the registerCustomNodes() function in autonomy.h.",
        [
            Task.Argument("file_location")
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