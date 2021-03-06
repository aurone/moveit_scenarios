TODO: This is the dream for the generic test app

Tests are specified via hierarchical config. A single test configuration is
built by traversing a hierarchy of directories where each child directory
overrides pieces of the configuration specified in its ancestor directories.

The SCENARIO_ROOT environment variable must be set to the path to the directory
containing the top-level configuration file, scenario.yaml. The top-level
scenario.yaml file contains the base definitions for the motion plan request and
for the planning options that compose a move group action goal.

If SCENARIO_ROOT specifies a relative path, the complete path is interpreted
relative to the current working directory.

Subdirectories of $SCENARIO_ROOT may override the base configuration by
redefining parameters within another file named scenario.yaml. Parameters that
are not specified are not overridden

The test executable is given the SCENARIO_ROOT environment variable and the path
to a scenario.yaml file that is within a subdirectoriy of $SCENARIO_ROOT

Example:

$SCENARIO_ROOT/
    scenario.yaml
    pr2/
        scenario.yaml
        warehouse/
            scenario.yaml
        kitchen/
            scenario.yaml
        tabletop/
            scenario.yaml
    ur5/
        scenario.yaml
        apc/
            scenario.yaml

The test executable will first build the final configuration, and then send the
move group goal to move_group via the actionlib interface. The result will be
written to stdout in yaml format (via operator<< for the result message type)
