import os

from util import info

def doReconfigure(args, autonomyRootLoc: str, buildTests: bool, buildDebug: bool):
    info(args, "Reconfiguring and building the riptide_autonomy2 package...")
    
    #back out two directories for workspace path (autonomy root is <workspace>/src/riptide_autonomy)
    workspaceDir = os.path.join(autonomyRootLoc, "../..")
    os.chdir(workspaceDir)
    
    info(args, "Building package in location {}".format(os.path.abspath(workspaceDir)))
    
    #run command to reconfigure and build
    cmd = "colcon build --cmake-clean-cache --packages-select riptide_autonomy2"
    if buildTests or buildDebug:
        cmd += " --cmake-args"
        
        if buildTests:
            info(args, "Building riptide_autonomy with tests")
            cmd += " -DAUTONOMY_BUILD_TESTS=ON"
        
        if buildDebug:
            info(args, "Building riptide_autonomy in debug mode")
            cmd += " -DCMAKE_BUILD_TYPE=DEBUG"
    
    return not os.system(cmd) #return True on nonzero return code



#called when the rebuild_autonomy task is invoked
def onReconfigureAutonomy(args, autonomyRootLoc: str):
    success = doReconfigure(args, autonomyRootLoc, args.with_tests, args.debug)
    
    if not success: #command returned nonzero value
        info(args, "Error reconfiguring autonomy.")
