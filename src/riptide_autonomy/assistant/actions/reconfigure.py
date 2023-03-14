import os
from util import info

#called when the rebuild_autonomy task is invoked
def onReconfigureAutonomy(args, autonomyRootLoc: str):
    info(args, "Reconfiguring and building the riptide_autonomy2 package...")
    
    #back out two directories for workspace path (autonomy root is <workspace>/src/riptide_autonomy)
    workspaceDir = os.path.join(autonomyRootLoc, "../..")
    os.chdir(workspaceDir)
    
    info(args, "Building package in location {}".format(os.path.abspath(workspaceDir)))
    
    #run command to reconfigure and build
    res = os.system("colcon build --cmake-clean-cache --packages-select riptide_autonomy2")
    
    if res:
        info(args, "Error reconfiguring autonomy.")

