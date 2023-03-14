#! /usr/bin/env python3

import argparse
import os
import sys
from enum import Enum
from glob import glob

from actions.check import onCheck
from actions.create import onCreate
from actions.generateRegistrators import onGenerateRegistrators
from actions.reconfigure import onReconfigureAutonomy
from util import info, autonomySrcLocation, autonomyIncludeLocation, autonomyTestLocation

#
# VERIFY AUTONOMY SRC PACKAGE LOCATION BEFORE COMMANDS ARE RUN
# 

#find the location of this file and use it to locate the riptide_autonomy source package root
FILE_LOC = os.path.abspath(__file__)
AUTONOMY_ROOT_LOCATION = os.path.join(FILE_LOC[0 : FILE_LOC.find("/riptide_autonomy/")], "riptide_autonomy")

#if the root location is in an install directory, change it to the source directory
if AUTONOMY_ROOT_LOCATION.find("/install/") >= 0:
    AUTONOMY_ROOT_LOCATION = os.path.join(AUTONOMY_ROOT_LOCATION[0 : AUTONOMY_ROOT_LOCATION.find("/install/")], "src", "riptide_autonomy")



#check that paths exist
def assertPathExists(name, pth):
    if not os.path.exists(pth):
        print(f"FATAL: Could not verify existence of {name} at {pth}. Is this file running within the riptide_autonomy package?", file=sys.stderr)
        exit()

assertPathExists("autonomy source path", autonomySrcLocation(AUTONOMY_ROOT_LOCATION))
assertPathExists("autonomy include path", autonomyIncludeLocation(AUTONOMY_ROOT_LOCATION))
assertPathExists("autonomy test path", autonomyTestLocation(AUTONOMY_ROOT_LOCATION))        
        
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
        [
            Task.Argument("module", options=["files", "behaviortree", "nodes", "system"])
        ]
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
    
    #print some information about the command
    info(args, "Running subcommand {} on the package at {}".format(args.task, AUTONOMY_ROOT_LOCATION))
    
    #run desired task
    taskOptions[args.task].callback(args, AUTONOMY_ROOT_LOCATION)


if __name__ == "__main__":
    main()
