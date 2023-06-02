# riptide_autonomy
This package contains OSU UWRT's competition task code. UWRT uses a behaviortree-based autonomy system, featuring the [BehaviorTree.CPP](https://behaviortree.dev) library, and uses [its fork of Groot](https://github.com/osu-uwrt/Groot) to edit its behavior trees. This package contains the following parts of the system:

- All actions, conditions, and decorators used in the behavior trees
- All behavior trees used on the robot
- An action server to run the behavior trees
- An assistant to streamline maintainance on the system
- Utilties for testing the health of the system

## Package Structure
The package is split up into five main parts:

- Custom node *definitions* or *headers*, which are located in *include/riptide_autonomy* and split up by node type (bt_actions, bt_conditions, and bt_decorators). This is the C++ code that defines the behaviors of the package's custom BT nodes.
- Custom node *tests*, which are located in *test/riptide_autonomy* and split up by node type. This is the C++ code that tests the custom node definitions. All tests are written using the GTest framework. Header files for test utilities can be found in *include/autonomy_test*.
- The **doTask** executable, which spins up an action server that runs behavior tree XML files on the system.
- The BT XML, located in *trees*, stores the XML which is run by the **doTask** executable. The XML workspace is located in *trees/.groot/workspace.xml* and contains all custom node declarations and subtree definitions. This file cannot actually be run because there is no main tree, but it is used by UWRTs [Groot](https://github.com/osu-uwrt/Groot) editor to ensure that each executable behavior tree in the *trees* directory shares the same custom nodes and subtrees. 
- The BT assistant, located in *src/riptide_autonomy/assistant* is a useful tool for performing tedious tasks related to the package.

## Running BehaviorTrees
### Starting the action server
The **doTask** node contains an action server which allows for easy running of behavior trees stored in the *trees* directory. Launch it, along with other important nodes, with the command:

```ros2 launch riptide_autonomy2 autonomy.launch.py robot:=<robot name>```

The action server should appear as ```<robot name>/autonomy/run_tree``` and ingests the ```riptide_msgs2/action/ExecuteTree``` action type (see [riptide_core](https://github.com/osu-uwrt/riptide_core) for more information). 

### Running using RViz
Behavior trees can also be run from UWRTs MissionPanel RViz plugin (see riptide_rviz in UWRTs [riptide_gui](https://github.com/osu-uwrt/riptide_gui) repo for more information). So long as the action server is running, the MissionPanel can be used to select behavior trees found in the *trees* directory and run them with the press of a button.

### LED Status Patterns
UWRT's AUV has strips of LEDs in both cages to indicate robot status to swimmers and operators on the surface. riptide_autonomy commands these LEDs to display colors and patterns to indicate what the behavior is commanding the robot to do. States are set using the SetStatus action. The following table matches status names (which go into the "name" port on the SetStatus node) to their LED behaviors and descriptions.
| Status     | LED behavior      | Description                 |
|---------   |------------------ |-----------------------------|
| undefined  | Solid white       | Invalid status received.    |
| waiting    | Solid red         | Waiting for kill switch     |
| starting   | Slow blink blue   | Kill inserted, starting run |
| moving     | Solid green       | Positioning for next task   |
| searching  | Solid yellow      | Searching for next task     |
| aligning   | Slow blink purple | Aligning with task          |
| performing | Fast blink blue   | Performing task             |
| success    | Pulsing green     | Tree ended with SUCCESS     |
| failure    | Pulsing red       | Tree ended with FAILURE     |
| panic      | Fast blink red    | Tree crashed                |

## Editing BehaviorTrees
Use UWRTs fork of [Groot](https://github.com/osu-uwrt/Groot) to edit behavior trees used by the system.

## Using the BT assistant
The BehaviorTree assistant is a tool that streamlines system-related tasks such as creating nodes, checking system health, rebuilding the project, and generating the C++ sources that register the custom nodes with the BT.CPP library. The tool can be invoked through ROS with the command:

```ros2 run riptide_autonomy2 btassistant.py [-h] [-s] [-f]```

The tool can also be directly invoked by running the Python script located at:

*src/riptide_autonomy/assistant/btassistant.py*

The optional flags (-h, -s, -f) are detailed as such:
- **-h**: Display a help menu and exit
- **-s**: "silent" mode: only display prompts and output from invoked processes (like colcon). 
- **-f**: "force" mode: assume yes on all prompts.

The tasks the BT assistant can perform are detailed in the below sections.

### Creating Nodes (**btassistant.py create**)
By using the **create** subcommand, the BT assistant can automatically create new custom nodes and register them with the BT.CPP library. The usage of this subcommand is:

```ros2 run riptide_autonomy2 btassistant.py create <action type> <node name> [--no-rebuild]```

where **action type** is the type of action to create, and can be any of the following BT types:

- **action** - BT Action
- **condition** - BT Condition
- **decorator** - BT Decorator

and **node name** is the name of the new node being created.

Upon generating the necessary files, the command will invoke colcon and rebuild the package unless the **--no-rebuild** flag is specified.

### Checking System Health (**btassistant.py check**)
The **check** subcommand evaluates the health of different aspects of the system, ranging from ensuring that every custom node is part of the XML workspace, to invoking the custom nodes' test code. The usage of this subcommand is:

```ros2 run riptide_autonomy2 btassistant.py check <module>```

where **module** can be any of the following:

- **files** - Checks the package's file tree to ensure that every custom node has a "definition" in *include/riptide_autonomy* and a "test" in *test/riptide_autonomy*. If the assistant finds a custom node with a definition but no test, or a test but no definition, it will notify the user and prompt them to delete the existing file, add the missing one, or ignore the problem. If changes were made to the file tree at the end of execution, the user will be asked if they would like to rebuild the package.

- **behaviortree** - The same as running ```ros2 run riptide_autonomy checkBT```. Checks that every custom node in the package is defined in the TreeNodesModel of the XML workspace. This also checks that all ports reported in the XML workspace match those defined by the custom nodes. If any discrepancies are found, the assistant will offer to fix the problem by modifying the XML workspace. This tool will NOT modify any source code. It is up to the user to determine whether modifying the workspace is the best fix, or if it is better to modify source code. 

- **nodes** - This subcommand invokes the tests defined in *test/riptide_autonomy*. It is roughly equivalent to running the command ```ros2 run riptide_autonomy testBT```, however, because tests are not built by default (for quicker compilation), this subcommand will rebuild autonomy with tests enabled if no **testBT** executable is detected.

- **system** - This subcommand invokes the above three checks and reports if any of them failed. This is an effective health check for the entire package.

### Reconfiguring The Package (**btassistant.py reconfigure_autonomy**)
The **reconfigure_autonomy** subcommand cleans the CMake cache and then rebuilds the project. This forces any new custom nodes to be built if they haven't been already (however, if you use the ```btassistant.py create``` subcommand **without** the --no-rebuild option, you should be fine). This subcommand also provides optional flags to enable building with tests or building in debug mode (needed if running code with breakpoints). The usage of the subcommand is:

```ros2 run riptide_autonomy btassistant.py reconfigure_autonomy [--with-tests] [--debug]```

where the optional flags do the following:

- **--with-tests**: Builds the project with tests enabled (-DAUTONOMY_BUILD_TESTS=ON). Use this flag if you wish to invoke the ```testBT``` executable.
- **--debug**: Builds the project with debug mode enabled (-DCMAKE_BUILD_TYPE=DEBUG).

This subcommand is roughly equivalent to running ```colcon build --packages-select riptide_autonomy``` with the necessary CMake flags set.

### Generating The Registration Sources (**btassistant.py generate_registrators**)
This subcommand is invoked during the CMake configure step to generate the C++ source files that are compiled into the shared libaries. Realistically, there is no reason to manually invoke this subcommand.

## Repo Information
|            |              |
|------------|--------------|
| OS Distro  | Ubuntu 22.04 |
| ROS Distro | ROS2 Humble  |
| BT.CPP     | BT-CPP-v3
