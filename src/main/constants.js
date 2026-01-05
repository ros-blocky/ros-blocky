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
const PIXI_ENV_PATH_STRING = [
    PIXI_HOME,
    PIXI_SCRIPTS_PATH,
    PIXI_LIBRARY_BIN_PATH
].join(';') + ';'; // Trailing semicolon for appending to existing PATH

module.exports = {
    PIXI_PATHS: {
        ROS2_EXE,
        COLCON_EXE,
        PIXI_ENV_PATH_STRING,
        PIXI_WS_PATH
    }
};
