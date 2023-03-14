import os
from enum import Enum
from glob import glob

#
# Util functions and other things for riptide_autonomy's btassistant executable.
#

class BtNodeType(Enum):
    ACTION = "UWRTActionNode"
    CONDITION = "UWRTConditionNode"
    DECORATOR = "UWRTDecoratorNode"
    
def autonomySrcLocation(root: str):
    return f"{root}/src/riptide_autonomy"

def autonomyIncludeLocation(root: str):
    return f"{root}/include/riptide_autonomy"

def autonomyTestLocation(root: str):
    return f"{root}/test/riptide_autonomy"

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
def createNodeFile(args, nodeName: str, nodeType: BtNodeType, srcLoc: str, includeLoc: str):
    #check node type
    if not isinstance(nodeType, BtNodeType):
        raise TypeError("Parameter nodeType must be a member of the BtNodeType enum.")
    
    template = "actionnode_hpp_template" if nodeType == BtNodeType.ACTION else "nonactionnode_hpp_template"
        
    templatePath = "{}/assistant/templates/{}".format(srcLoc, template)    
    newFileName = "{0}/bt_{1}s/{2}.bt{1}.hpp".format(includeLoc, nodeType.name.lower(), nodeName)
    
    if os.path.exists(newFileName):
        if not askConfirmation(args, "File with name {} already exists. Overwrite it?".format(newFileName)):
            info(args, "Not creating file.") #user said no
            return
    
    createFileFromTemplate(templatePath, newFileName, [nodeName, nodeType.value])
    info(args, "Created a new {} with name {}.".format(nodeType.value, nodeName))


#creates a test file for a node with name nodeName of type nodeType.
def createTestFile(args, nodeName: str, nodeType: BtNodeType, srcLoc: str, testLoc: str):
    #check node type
    if not isinstance(nodeType, BtNodeType):
        raise TypeError("Parameter nodeType must be a member of the BtNodeType enum.")
    
    templatePath = "{}/assistant/templates/test_template".format(srcLoc)
    newFileName = "{0}/bt_{1}s/Test{2}.cpp".format(testLoc, nodeType.name.lower(), nodeName)

    if os.path.exists(newFileName):
        if not askConfirmation(args, "File with name {} already exists. Overwrite it?".format(newFileName)):
            info(args, "Not creating file.") #user said no
            return
    
    createFileFromTemplate(templatePath, newFileName, [nodeName])
    info(args, "Created test file for {} {}.".format(nodeType.name.lower(), nodeName))

