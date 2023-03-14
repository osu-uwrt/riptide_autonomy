import sys

from util import BtNodeType, createNodeFile, createTestFile, info
from actions.reconfigure import onReconfigureAutonomy

#called when the create task is invoked.
def onCreate(args, autonomyRootLoc: str):
    nodeType = None
    try:
        #select enum value by name, which is the user type specification but uppercase.
        #this statement must populate nodeType or else catch block will fire
        nodeType = BtNodeType[args.type.upper()] 
    except KeyError:
        print("Node Type {} not recognized/implemented!".format(args.type), file=sys.stderr)
        exit(1)
    
    #create file
    createNodeFile(args, args.name, nodeType, autonomyRootLoc)
    createTestFile(args, args.name, nodeType, autonomyRootLoc)
    
    #rebuild autonomy to force cmake to configure and generate appropriate headers
    if args.no_rebuild:
        info(args, "You need to completely rebuild riptide_autonomy2 for the changes to take effect.")
    else:
        onReconfigureAutonomy(args, autonomyRootLoc)

