/**
 * Application Constants
 * distinct from project-specific configuration
 */

const path = require('path');

// Pixi Environment Constants
const PIXI_WS_PATH = 'C:\\pixi_ws';
const PIXI_ENV_NAME = 'default';
const PIXI_HOME = path.join(PIXI_WS_PATH, '.pixi', 'envs', PIXI_ENV_NAME);

// Constructed Paths
const PIXI_SCRIPTS_PATH = path.join(PIXI_HOME, 'Scripts');
const PIXI_LIBRARY_BIN_PATH = path.join(PIXI_HOME, 'Library', 'bin');

// Exported Executable Paths
const ROS2_EXE = path.join(PIXI_LIBRARY_BIN_PATH, 'ros2.exe');
const COLCON_EXE = path.join(PIXI_SCRIPTS_PATH, 'colcon.exe');

// Exported Environment Path String
// This is required for ros2/colcon to find DLLs/libs without running inside 'pixi shell'
// Matches the full PATH set by pixi shell for complete compatibility
const PIXI_LIBRARY = path.join(PIXI_HOME, 'Library');
const PIXI_ENV_PATH_STRING = [
    // RViz2 OGRE vendor paths (critical for RViz2!)
    path.join(PIXI_LIBRARY, 'opt', 'rviz_ogre_vendor', 'lib'),
    path.join(PIXI_LIBRARY, 'opt', 'rviz_ogre_vendor', 'bin'),
    // Core pixi paths
    PIXI_HOME,
    // MinGW and usr paths
    path.join(PIXI_LIBRARY, 'mingw-w64', 'bin'),
    path.join(PIXI_LIBRARY, 'usr', 'bin'),
    path.join(PIXI_LIBRARY, 'bin'),
    // Scripts
    PIXI_SCRIPTS_PATH,
    path.join(PIXI_HOME, 'bin')
].join(';') + ';'; // Trailing semicolon for appending to existing PATH

module.exports = {
    PIXI_PATHS: {
        ROS2_EXE,
        COLCON_EXE,
        PIXI_ENV_PATH_STRING,
        PIXI_WS_PATH
    }
};
