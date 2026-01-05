/**
 * ROS Service - Handles ROS2 command execution
 * Responsible for running nodes, URDF publishers, and launch files
 */

const { spawn } = require('child_process');
const path = require('path');
const fs = require('fs');
const os = require('os');
const { BrowserWindow } = require('electron');
const { isValidName, isValidTopicName, createValidationError } = require('../helpers/validationUtil');
const { PIXI_PATHS } = require('../constants');

// Reference to packageService (set during initialization)
let packageService = null;

// Track running processes
const runningProcesses = new Map();

/**
 * Initialize the ROS service with dependencies
 * @param {Object} pkgService - The package service instance
 */
function init(pkgService) {
    packageService = pkgService;
    console.log('[ROS Service] Initialized');
}

/**
 * Run a Python node using ros2 run
 * @param {string} packageName - Name of the ROS package
 * @param {string} nodeName - Name of the node file (with or without .py)
 * @returns {Promise<Object>} Result with success status and PID
 */
async function runNode(packageName, nodeName) {
    // Validate inputs to prevent command injection
    if (!isValidName(packageName)) {
        return createValidationError('package name', packageName);
    }
    if (!isValidName(nodeName.replace(/\.py$/, ''))) {
        return createValidationError('node name', nodeName);
    }

    const nodeNameWithoutExt = nodeName.replace(/\.py$/, '');
    const args = ['run', packageName, nodeNameWithoutExt];
    return await spawnRosProcess(PIXI_PATHS.ROS2_EXE, args, `node:${packageName}/${nodeName}`);
}

/**
 * Run robot_state_publisher with a URDF/XACRO file
 * Uses pixi shell -> source ros2 -> python -c with xacro and robot_state_publisher
 * @param {string} packageName - Name of the ROS package
 * @param {string} fileName - Name of the URDF/XACRO file
 * @returns {Promise<Object>} Result with success status and PID
 */
async function runRobotPublisher(packageName, fileName) {
    // Validate inputs to prevent command injection
    if (!isValidName(packageName)) {
        return createValidationError('package name', packageName);
    }
    if (!isValidName(fileName)) {
        return createValidationError('file name', fileName);
    }

    const projectPath = getProjectPath();
    if (!projectPath) {
        return { success: false, error: 'No project loaded' };
    }

    const xacroPath = path.join(projectPath, 'src', packageName, 'urdf', fileName);

    // For URDF, we use python to run a script that runs xacro and then robot_state_publisher
    // This is complex because we need to capture xacro output.
    // We'll run python directly from the environment.

    // Construct the python script to run
    const pythonScript = `
import subprocess
import os
import sys

# Add PIXI paths to environment for subprocesses
env = os.environ.copy()
env["PATH"] = r"${PIXI_PATHS.PIXI_ENV_PATH_STRING}" + env["PATH"]

try:
    # Run xacro
    result = subprocess.run([r"${PIXI_PATHS.ROS2_EXE}", "run", "xacro", "xacro", r"${xacroPath}"], capture_output=True, text=True, env=env)
    
    if result.returncode != 0:
        print(f"Xacro Error: {result.stderr}", file=sys.stderr)
        sys.exit(1)

    # Run robot_state_publisher
    subprocess.run([r"${PIXI_PATHS.ROS2_EXE}", "run", "robot_state_publisher", "robot_state_publisher", "--ros-args", "-p", f"robot_description:={result.stdout}"], env=env)

except Exception as e:
    print(f"Error: {e}", file=sys.stderr)
    sys.exit(1)
`;

    return await spawnRosProcess('python', ['-c', pythonScript], `urdf:${packageName}/${fileName}`);
}

/**
 * Run a launch file using ros2 launch
 * @param {string} packageName - Name of the ROS package
 * @param {string} fileName - Name of the launch file
 * @returns {Promise<Object>} Result with success status and PID
 */
async function runLaunch(packageName, fileName) {
    // Validate inputs to prevent command injection
    if (!isValidName(packageName)) {
        return createValidationError('package name', packageName);
    }
    if (!isValidName(fileName)) {
        return createValidationError('file name', fileName);
    }

    const args = ['launch', packageName, fileName];
    return await spawnRosProcess(PIXI_PATHS.ROS2_EXE, args, `launch:${packageName}/${fileName}`);
}

/**
 * Run RViz2 visualization tool
 * @returns {Promise<Object>} Result with success status and PID
 */
async function runRviz() {
    return await spawnRosProcess(PIXI_PATHS.ROS2_EXE, ['run', 'rviz2', 'rviz2'], 'rviz2');
}

/**
 * Run Joint State Publisher GUI
 * @returns {Promise<Object>} Result with success status and PID
 */
async function runJointStatePublisherGui() {
    return await spawnRosProcess(PIXI_PATHS.ROS2_EXE, ['run', 'joint_state_publisher_gui', 'joint_state_publisher_gui'], 'joint_state_publisher_gui');
}

/**
 * Run TurtleSim node
 * @returns {Promise<Object>} Result with success status and PID
 */
async function runTurtlesim() {
    return await spawnRosProcess(PIXI_PATHS.ROS2_EXE, ['run', 'turtlesim', 'turtlesim_node'], 'turtlesim');
}

/**
 * Get list of available ROS 2 topics
 * @returns {Promise<Object>} Result with topics array
 */
async function getTopicList() {
    return new Promise((resolve) => {
        const ros2Path = PIXI_PATHS.ROS2_EXE;
        const envPath = PIXI_PATHS.PIXI_ENV_PATH_STRING;

        let output = '';
        let errorOutput = '';

        console.log('[ROS Service] Getting topic list via direct ros2 execution');

        const child = spawn(ros2Path, ['topic', 'list'], {
            windowsHide: true,
            env: {
                ...process.env,
                PATH: `${envPath};${process.env.PATH}`,
                PYTHONUNBUFFERED: '1'
            }
        });

        child.stdout.on('data', (data) => {
            output += data.toString();
        });

        child.stderr.on('data', (data) => {
            errorOutput += data.toString();
        });

        child.on('close', (code) => {
            if (code !== 0 && errorOutput) {
                console.error('[ROS Service] Error getting topic list:', errorOutput);
                resolve({ success: false, error: errorOutput, topics: [] });
                return;
            }

            // Parse topics from output - each topic is on its own line starting with /
            const lines = output.split('\n')
                .map(line => line.trim())
                .filter(line => line.startsWith('/'));

            console.log('[ROS Service] Topics found:', lines);
            resolve({ success: true, topics: lines });
        });

        child.on('error', (error) => {
            console.error('[ROS Service] Spawn error getting topic list:', error);
            resolve({ success: false, error: error.message, topics: [] });
        });

        // Timeout after 10 seconds
        setTimeout(() => {
            if (child.exitCode === null) {
                child.kill();
                resolve({ success: false, error: 'Timeout getting topics', topics: [] });
            }
        }, 10000);
    });
}

/**
 * Get detailed information about a specific ROS 2 topic
 * @param {string} topicName - Name of the topic (e.g., /turtle1/cmd_vel)
 * @returns {Promise<Object>} Result with topic info (type, publishers, subscribers)
 */
async function getTopicInfo(topicName) {
    // Validate topic name to prevent command injection
    if (!isValidTopicName(topicName)) {
        return { success: false, error: `Invalid topic name: "${topicName}". Topic names must start with / and contain only alphanumeric characters, underscores, and forward slashes.` };
    }

    return new Promise((resolve) => {
        const ros2Path = PIXI_PATHS.ROS2_EXE;
        const envPath = PIXI_PATHS.PIXI_ENV_PATH_STRING;

        let output = '';
        let errorOutput = '';

        const child = spawn(ros2Path, ['topic', 'info', topicName, '-v'], {
            windowsHide: true,
            env: {
                ...process.env,
                PATH: `${envPath};${process.env.PATH}`,
                PYTHONUNBUFFERED: '1'
            }
        });

        child.stdout.on('data', (data) => {
            output += data.toString();
        });

        child.stderr.on('data', (data) => {
            errorOutput += data.toString();
        });

        child.on('close', (code) => {
            if (code !== 0 && errorOutput) {
                resolve({ success: false, error: errorOutput });
                return;
            }

            // Parse topic info from output
            const info = {
                topic: topicName,
                type: '',
                publisherCount: 0,
                subscriberCount: 0,
                publishers: [],
                subscribers: []
            };

            const lines = output.split('\n');
            let currentSection = '';

            for (const line of lines) {
                const trimmed = line.trim();

                // Get message type
                if (trimmed.startsWith('Type:')) {
                    info.type = trimmed.replace('Type:', '').trim();
                }
                // Get publisher count
                else if (trimmed.startsWith('Publisher count:')) {
                    info.publisherCount = parseInt(trimmed.replace('Publisher count:', '').trim()) || 0;
                    currentSection = 'publishers';
                }
                // Get subscriber count
                else if (trimmed.startsWith('Subscription count:')) {
                    info.subscriberCount = parseInt(trimmed.replace('Subscription count:', '').trim()) || 0;
                    currentSection = 'subscribers';
                }
                // Parse node names (lines starting with "Node name:")
                else if (trimmed.startsWith('Node name:')) {
                    const nodeName = trimmed.replace('Node name:', '').trim();
                    if (currentSection === 'publishers') {
                        info.publishers.push(nodeName);
                    } else if (currentSection === 'subscribers') {
                        info.subscribers.push(nodeName);
                    }
                }
            }

            console.log('[ROS Service] Topic info:', info);
            resolve({ success: true, info });
        });

        child.on('error', (error) => {
            resolve({ success: false, error: error.message });
        });

        // Timeout after 10 seconds
        setTimeout(() => {
            if (child.exitCode === null) {
                child.kill();
                resolve({ success: false, error: 'Timeout getting topic info' });
            }
        }, 10000);
    });
}

/**
 * Start echoing a topic (streaming output)
 * @param {string} topicName - Name of the topic to echo
 * @returns {Promise<Object>} Result with success status and process key
 */
async function startTopicEcho(topicName) {
    // Validate topic name to prevent command injection
    if (!isValidTopicName(topicName)) {
        return { success: false, error: `Invalid topic name: "${topicName}". Topic names must start with / and contain only alphanumeric characters, underscores, and forward slashes.` };
    }

    const processKey = `echo:${topicName}`;

    // Stop existing echo for this topic if running
    if (runningProcesses.has(processKey)) {
        await stopProcess(processKey);
    }

    return new Promise((resolve) => {
        const ros2Path = PIXI_PATHS.ROS2_EXE;
        const envPath = PIXI_PATHS.PIXI_ENV_PATH_STRING;

        const child = spawn(ros2Path, ['topic', 'echo', topicName], {
            windowsHide: true,
            env: {
                ...process.env,
                PATH: `${envPath};${process.env.PATH}`,
                PYTHONUNBUFFERED: '1'
            }
        });

        runningProcesses.set(processKey, child);

        // Helper to send to renderer
        const sendEchoOutput = (data, key) => {
            const mainWindow = BrowserWindow.getAllWindows()[0];
            if (mainWindow) {
                mainWindow.webContents.send('echo-output', data, key);
            }
        };

        // Stream stdout to renderer
        child.stdout.on('data', (data) => {
            const message = data.toString();
            sendEchoOutput(message, processKey);
        });

        child.stderr.on('data', (data) => {
            const message = data.toString();
            sendEchoOutput(message, processKey);
        });

        child.on('close', (code) => {
            runningProcesses.delete(processKey);
            const mainWindow = BrowserWindow.getAllWindows()[0];
            if (mainWindow) {
                mainWindow.webContents.send('echo-stopped', { code }, processKey);
            }
        });

        child.on('error', (error) => {
            const mainWindow = BrowserWindow.getAllWindows()[0];
            if (mainWindow) {
                mainWindow.webContents.send('echo-error', error.message, processKey);
            }
            runningProcesses.delete(processKey);
        });

        console.log(`[ROS Service] Started echo for ${topicName}, PID: ${child.pid}`);

        resolve({
            success: true,
            pid: child.pid,
            processKey: processKey
        });
    });
}

/**
 * Build all packages in the workspace using colcon build
 * @param {Function} onStatus - Callback for status updates
 * @param {string} projectPathOverride - Optional project path (for new projects not yet in packageService)
 * @returns {Promise<Object>} Result with success status
 */
async function buildAllPackages(onStatus, projectPathOverride) {
    const projectPath = projectPathOverride || getProjectPath();
    if (!projectPath) {
        return { success: false, error: 'No project loaded' };
    }

    try {
        if (onStatus) onStatus('Starting colcon build...');

        // Run colcon build in the project directory
        const result = await runColconBuild(projectPath, null, onStatus);
        return result;
    } catch (error) {
        console.error('[ROS Service] Build all packages error:', error);
        return { success: false, error: error.message };
    }
}

/**
 * Build a single package using colcon build --packages-select
 * @param {string} packageName - Name of the package to build
 * @param {Function} onStatus - Callback for status updates
 * @returns {Promise<Object>} Result with success status
 */
async function buildPackage(packageName, onStatus) {
    // Validate package name to prevent command injection
    if (!isValidName(packageName)) {
        return createValidationError('package name', packageName);
    }

    const projectPath = getProjectPath();
    if (!projectPath) {
        return { success: false, error: 'No project loaded' };
    }

    try {
        if (onStatus) onStatus(`Building package "${packageName}"...`);

        // Run colcon build with --packages-select
        const result = await runColconBuild(projectPath, packageName, onStatus);
        return result;
    } catch (error) {
        console.error(`[ROS Service] Build package ${packageName} error:`, error);
        return { success: false, error: error.message };
    }
}

/**
 * Run colcon build command
 * @param {string} projectPath - Path to the project
 * @param {string|null} packageName - Package name for --packages-select, or null for all
 * @param {Function} onStatus - Callback for status updates
 * @returns {Promise<Object>} Result with success status
 */
async function runColconBuild(projectPath, packageName, onStatus) {
    return new Promise((resolve) => {
        console.log(`[ROS Service] Running colcon build in ${projectPath}${packageName ? ` for package ${packageName}` : ''}`);

        // Call colcon.exe directly with pixi environment PATH
        const colconPath = PIXI_PATHS.COLCON_EXE;
        const pixiEnvPath = PIXI_PATHS.PIXI_ENV_PATH_STRING;

        const args = [
            'build',
            '--base-paths', projectPath,
            '--symlink-install',
            '--parallel-workers', '16',
            '--cmake-args', '-G', 'Ninja', '-DCMAKE_BUILD_TYPE=Release'
        ];

        if (packageName) {
            args.push('--packages-select', packageName);
        }

        const child = spawn(colconPath, args, {
            cwd: projectPath,
            windowsHide: true,
            env: {
                ...process.env,
                PATH: `${pixiEnvPath};${process.env.PATH}`,
                // Disable notifications and progress bars
                COLCON_LOG_LEVEL: 'error',
                PYTHONUNBUFFERED: '1',
                NO_COLOR: '1'
            }
        });

        let output = '';
        let resolved = false;

        child.stdout.on('data', (data) => {
            const text = data.toString();
            output += text;
            console.log('[Build Output]', text);

            // Resolve IMMEDIATELY when we see Summary: (build is done)
            if (!resolved && text.includes('Summary:')) {
                resolved = true;
                console.log('[ROS Service] Summary detected - resolving immediately!');
                resolve({ success: true, output });
            }

            if (onStatus) {
                const lines = text.split('\n').filter(l => l.trim());
                if (lines.length > 0) onStatus(lines[lines.length - 1].trim());
            }
        });

        child.stderr.on('data', (data) => {
            output += data.toString();
            console.error('[Build Error]', data.toString());
        });

        // Fallback for builds without Summary output
        child.on('exit', (code) => {
            if (resolved) return; // Already resolved
            console.log(`[ROS Service] Build exited with code: ${code}`);
            if (code === 0) {
                resolve({ success: true, output });
            } else {
                resolve({ success: false, error: `Build failed with code ${code}`, output });
            }
        });

        child.on('error', (error) => {
            if (resolved) return;
            console.error('[ROS Service] Build error:', error);
            resolve({ success: false, error: error.message });
        });
    });
}

/**
 * Stop a running process
 * @param {string} processKey - The key of the process to stop
 * @returns {boolean} True if process was stopped
 */
function stopProcess(processKey) {
    const proc = runningProcesses.get(processKey);

    // Check if this is a URDF process - we need to kill robot_state_publisher regardless
    const isUrdfProcess = processKey && processKey.startsWith('urdf:');

    if (proc) {
        try {
            console.log(`[ROS Runner] Stopping process: ${processKey}, PID: ${proc.pid}`);

            // Kill the process tree on Windows
            spawn('taskkill', ['/pid', proc.pid.toString(), '/f', '/t'], {
                shell: true,
                windowsHide: true
            });

            runningProcesses.delete(processKey);
        } catch (error) {
            console.error(`[ROS Runner] Error stopping process by PID:`, error);
        }
    } else {
        console.log(`[ROS Runner] Process not found in map: ${processKey}`);
    }

    // For URDF processes, ALWAYS try to kill robot_state_publisher by name
    // This handles cases where the process exited but robot_state_publisher is still running
    if (isUrdfProcess) {
        console.log(`[ROS Runner] Killing robot_state_publisher and related processes...`);

        // Kill robot_state_publisher by name
        spawn('taskkill', ['/im', 'robot_state_publisher.exe', '/f'], {
            shell: true,
            windowsHide: true
        });

        // Kill python processes that might be running xacro
        spawn('taskkill', ['/im', 'python.exe', '/f', '/fi', 'WINDOWTITLE eq pixi*'], {
            shell: true,
            windowsHide: true
        });
    }

    // ALWAYS notify renderer that process stopped (even if not found in map)
    // This ensures the button state is reset
    sendToRenderer('status', 'stopped', processKey);
    console.log(`[ROS Runner] Stopped process: ${processKey}`);

    return true;
}

/**
 * Check if a process is running
 * @param {string} processKey - The key of the process
 * @returns {boolean} True if running
 */
function isProcessRunning(processKey) {
    return runningProcesses.has(processKey);
}

/**
 * Stop all running processes (called on app quit)
 * Terminates all tracked processes and common ROS executables
 */
function stopAllProcesses() {
    console.log('[ROS Service] Stopping all running processes...');

    // Stop all tracked processes
    for (const [processKey, proc] of runningProcesses) {
        try {
            console.log(`[ROS Service] Stopping: ${processKey}, PID: ${proc.pid}`);
            spawn('taskkill', ['/pid', proc.pid.toString(), '/f', '/t'], {
                shell: true,
                windowsHide: true
            });
        } catch (error) {
            console.error(`[ROS Service] Error stopping ${processKey}:`, error);
        }
    }

    // Also kill common ROS processes that might have been spawned
    const processesToKill = [
        'robot_state_publisher.exe',
        'rviz2.exe',
        'ros2.exe',
        'python.exe'  // Nodes run as Python processes
    ];

    for (const processName of processesToKill) {
        try {
            spawn('taskkill', ['/im', processName, '/f'], {
                shell: true,
                windowsHide: true
            });
            console.log(`[ROS Service] Killed: ${processName}`);
        } catch (error) {
            // Ignore errors - process might not exist
        }
    }

    runningProcesses.clear();
    console.log('[ROS Service] All processes stopped');
}

/**
 * Get the current project path from packageService
 * @returns {string|null} The project path or null
 */
function getProjectPath() {
    if (!packageService) {
        console.error('[ROS Service] Not initialized - packageService is null');
        return null;
    }
    return packageService.getProjectPath();
}

/**
 * Send log message to renderer
 * @param {string} type - 'log', 'error', 'status'
 * @param {string} message - The message
 * @param {string} processKey - The process key
 */
function sendToRenderer(type, message, processKey) {
    const mainWindow = BrowserWindow.getAllWindows()[0];
    if (mainWindow) {
        mainWindow.webContents.send('ros-output', { type, message, processKey });
    }
}

/**
 * Execute commands inside pixi shell in background
 * Captures output and sends to renderer
 * @param {string[]} commands - Array of commands to run inside pixi shell
 * @param {string} processKey - Unique key for this process
 * @returns {Promise<Object>} Result with success status and PID
 */
/**
 * Spawn a ROS process directly (replacing executeInPixiShell)
 * @param {string} executable - Path to executable (or command name if in PATH)
 * @param {string[]} args - Arguments for the command
 * @param {string} processKey - Unique key for tracking
 * @returns {Promise<Object>} Result with success status and PID
 */
async function spawnRosProcess(executable, args, processKey) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) {
            return { success: false, error: 'No project loaded' };
        }

        if (runningProcesses.has(processKey)) {
            return { success: false, error: 'Process already running', isRunning: true };
        }

        console.log(`[ROS Runner] Spawning: ${executable} ${args.join(' ')}`);

        // Construct Pixi environment paths
        const pixiHome = path.join(PIXI_PATHS.PIXI_WS_PATH, '.pixi', 'envs', 'default');
        const pixiLibrary = path.join(pixiHome, 'Library'); // ROS 2 packages are installed here
        const installDir = path.join(projectPath, 'install');

        // Setup Environment with Pixi Paths and ROS variables
        const env = {
            ...process.env,
            // Full pixi environment PATH (includes rviz_ogre_vendor, mingw, etc.)
            PATH: `${PIXI_PATHS.PIXI_ENV_PATH_STRING}${process.env.PATH}`,

            // Critical: AMENT_PREFIX_PATH tells ROS 2 where to find packages/plugins
            // For conda-forge ROS 2 on Windows, packages are in Library folder
            AMENT_PREFIX_PATH: `${pixiLibrary};${installDir}`,

            // Critical: PYTHONPATH for Python nodes
            PYTHONPATH: `${path.join(pixiHome, 'Lib', 'site-packages')};${path.join(installDir, 'Lib', 'site-packages')}`,

            // Required by python_qt_binding and other conda packages
            CONDA_PREFIX: pixiHome,

            // Qt plugins path for RViz2 and other Qt-based apps
            QT_PLUGIN_PATH: path.join(pixiLibrary, 'plugins'),

            // OGRE plugins for RViz2's 3D rendering
            OGRE_PLUGIN_PATH: path.join(pixiLibrary, 'bin', 'OGRE'),

            // Other useful vars
            PYTHONUNBUFFERED: '1',
            NO_COLOR: '1'
        };

        const child = spawn(executable, args, {
            cwd: projectPath,
            windowsHide: true,
            env: env
        });

        // Store process
        runningProcesses.set(processKey, {
            pid: child.pid,
            process: child,
            startTime: Date.now()
        });

        // Notify renderer
        sendToRenderer('status', 'starting', processKey);

        // Capture stdout
        child.stdout.on('data', (data) => {
            const output = data.toString();
            console.log(`[ROS Output ${processKey}] ${output}`);
            sendToRenderer('log', output, processKey);

            if (output.includes('Robot initialized') ||
                (processKey.startsWith('node:') && output.includes('[INFO]'))) {
                sendToRenderer('status', 'running', processKey);
            }
        });

        // Capture stderr
        child.stderr.on('data', (data) => {
            const output = data.toString();
            console.error(`[ROS Error ${processKey}] ${output}`); // Uncommented for debugging
            sendToRenderer('error', output, processKey);

            if (output.includes('Robot initialized') ||
                (processKey.startsWith('node:') && output.includes('[INFO]'))) {
                sendToRenderer('status', 'running', processKey);
            }
        });

        // Handle exit
        child.on('close', (code) => {
            console.log(`[ROS Runner] ${processKey} exited with code: ${code}`);
            runningProcesses.delete(processKey);
            sendToRenderer('status', 'stopped', processKey);
        });

        child.on('error', (error) => {
            console.error(`[ROS Runner] ${processKey} error:`, error);
            runningProcesses.delete(processKey);
            sendToRenderer('error', error.message, processKey);
            sendToRenderer('status', 'error', processKey);
        });

        return { success: true, pid: child.pid, processKey };

    } catch (error) {
        console.error(`[ROS Runner] Spawn error:`, error);
        return { success: false, error: error.message };
    }
}

module.exports = {
    init,
    runNode,
    runRobotPublisher,
    runLaunch,
    runRviz,
    runJointStatePublisherGui,
    runTurtlesim,
    getTopicList,
    getTopicInfo,
    startTopicEcho,
    buildAllPackages,
    buildPackage,
    stopProcess,
    stopAllProcesses,
    isProcessRunning
};
