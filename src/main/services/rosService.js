/**
 * ROS Service - Handles ROS2 command execution
 * Responsible for running nodes, URDF publishers, and launch files
 */

const { spawn } = require('child_process');
const path = require('path');
const fs = require('fs');
const os = require('os');
const { BrowserWindow } = require('electron');

// Reference to packageService (set during initialization)
let packageService = null;

// Track running processes
const runningProcesses = new Map();

// Pixi workspace path (where pixi.toml is located)
const PIXI_WS_PATH = 'C:\\pixi_ws';

// ROS2 environment setup script path
const ROS2_SETUP_SCRIPT = 'ros2-windows\\local_setup.bat';

/**
 * Initialize the ROS service with dependencies
 * @param {Object} pkgService - The package service instance
 */
function init(pkgService) {
    packageService = pkgService;
    console.log('[ROS Service] Initialized');
    console.log('[ROS Service] Pixi workspace:', PIXI_WS_PATH);
}

/**
 * Run a Python node using ros2 run
 * @param {string} packageName - Name of the ROS package
 * @param {string} nodeName - Name of the node file (with or without .py)
 * @returns {Promise<Object>} Result with success status and PID
 */
async function runNode(packageName, nodeName) {
    const nodeNameWithoutExt = nodeName.replace(/\.py$/, '');
    const commands = [
        `call ${ROS2_SETUP_SCRIPT}`,
        `ros2 run ${packageName} ${nodeNameWithoutExt}`
    ];

    return await executeInPixiShell(commands, `node:${packageName}/${nodeName}`);
}

/**
 * Run robot_state_publisher with a URDF/XACRO file
 * Uses pixi shell -> source ros2 -> python -c with xacro and robot_state_publisher
 * @param {string} packageName - Name of the ROS package
 * @param {string} fileName - Name of the URDF/XACRO file
 * @returns {Promise<Object>} Result with success status and PID
 */
async function runRobotPublisher(packageName, fileName) {
    const projectPath = getProjectPath();
    if (!projectPath) {
        return { success: false, error: 'No project loaded' };
    }

    const xacroPath = path.join(projectPath, 'src', packageName, 'urdf', fileName);

    // Commands to run inside pixi shell
    const commands = [
        `call ${ROS2_SETUP_SCRIPT}`,
        `python -c "import subprocess; result=subprocess.run(['ros2', 'run', 'xacro', 'xacro', r'${xacroPath}'], capture_output=True, text=True); subprocess.run(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args', '-p', f'robot_description:={result.stdout}'])"`
    ];

    return await executeInPixiShell(commands, `urdf:${packageName}/${fileName}`);
}

/**
 * Run a launch file using ros2 launch
 * @param {string} packageName - Name of the ROS package
 * @param {string} fileName - Name of the launch file
 * @returns {Promise<Object>} Result with success status and PID
 */
async function runLaunch(packageName, fileName) {
    const commands = [
        `call ${ROS2_SETUP_SCRIPT}`,
        `ros2 launch ${packageName} ${fileName}`
    ];

    return await executeInPixiShell(commands, `launch:${packageName}/${fileName}`);
}

/**
 * Run RViz2 visualization tool
 * @returns {Promise<Object>} Result with success status and PID
 */
async function runRviz() {
    const commands = [
        `call ${ROS2_SETUP_SCRIPT}`,
        `ros2 run rviz2 rviz2`
    ];

    return await executeInPixiShell(commands, 'rviz2');
}

/**
 * Stop a running process
 * @param {string} processKey - The key of the process to stop
 * @returns {boolean} True if process was stopped
 */
function stopProcess(processKey) {
    const proc = runningProcesses.get(processKey);
    if (proc) {
        try {
            console.log(`[ROS Runner] Stopping process: ${processKey}, PID: ${proc.pid}`);

            // Kill the process tree on Windows
            const kill1 = spawn('taskkill', ['/pid', proc.pid.toString(), '/f', '/t'], {
                shell: true,
                windowsHide: true
            });

            // Also kill robot_state_publisher by name (it might have spawned separately)
            const kill2 = spawn('taskkill', ['/im', 'robot_state_publisher.exe', '/f'], {
                shell: true,
                windowsHide: true
            });

            // Also kill python processes that might be running xacro
            const kill3 = spawn('taskkill', ['/im', 'python.exe', '/f', '/fi', 'WINDOWTITLE eq pixi*'], {
                shell: true,
                windowsHide: true
            });

            runningProcesses.delete(processKey);

            // Notify renderer that process stopped
            sendToRenderer('status', 'stopped', processKey);

            console.log(`[ROS Runner] Stopped process: ${processKey}`);
            return true;
        } catch (error) {
            console.error(`[ROS Runner] Error stopping process:`, error);
            return false;
        }
    }
    console.log(`[ROS Runner] Process not found: ${processKey}`);
    return false;
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
        'ros2.exe'
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
async function executeInPixiShell(commands, processKey) {
    try {
        const projectPath = getProjectPath();

        if (!projectPath) {
            return { success: false, error: 'No project loaded' };
        }

        // Check if already running
        if (runningProcesses.has(processKey)) {
            return { success: false, error: 'Process already running', isRunning: true };
        }

        console.log(`[ROS Runner] Commands:`, commands);
        console.log(`[ROS Runner] Process key: ${processKey}`);

        // Create temp files for commands
        const tempDir = os.tmpdir();
        const commandsFile = path.join(tempDir, `ros_cmds_${Date.now()}.txt`);
        const commandsContent = commands.join('\n') + '\n';
        fs.writeFileSync(commandsFile, commandsContent, { encoding: 'utf8' });

        // Create batch file that pipes commands to pixi shell
        const tempBatch = path.join(tempDir, `ros_run_${Date.now()}.bat`);
        const batchContent = `@echo off
cd /d ${PIXI_WS_PATH}
type "${commandsFile}" | pixi shell
`;
        fs.writeFileSync(tempBatch, batchContent, { encoding: 'utf8' });

        // Notify renderer that process is starting
        sendToRenderer('status', 'starting', processKey);

        // Spawn hidden process with output capture
        const child = spawn('cmd', ['/c', tempBatch], {
            cwd: PIXI_WS_PATH,
            windowsHide: true,  // Hide the window
            stdio: ['ignore', 'pipe', 'pipe']
        });

        // Store the process
        runningProcesses.set(processKey, {
            pid: child.pid,
            process: child,
            startTime: Date.now()
        });

        // Capture stdout
        child.stdout.on('data', (data) => {
            const output = data.toString();
            console.log(`[ROS Output] ${output}`);
            sendToRenderer('log', output, processKey);

            // Check for "Robot initialized" to notify UI
            if (output.includes('Robot initialized')) {
                sendToRenderer('status', 'running', processKey);
            }
        });

        // Capture stderr (ROS often logs to stderr)
        child.stderr.on('data', (data) => {
            const output = data.toString();
            console.error(`[ROS Error] ${output}`);
            sendToRenderer('error', output, processKey);

            // Check for "Robot initialized" in stderr too (ROS logs go to stderr)
            if (output.includes('Robot initialized')) {
                sendToRenderer('status', 'running', processKey);
            }
        });

        // Handle process exit
        child.on('close', (code) => {
            console.log(`[ROS Runner] Process exited with code: ${code}`);
            runningProcesses.delete(processKey);
            sendToRenderer('status', 'stopped', processKey);

            // Clean up temp files
            try {
                fs.unlinkSync(commandsFile);
                fs.unlinkSync(tempBatch);
            } catch (e) { /* ignore cleanup errors */ }
        });

        child.on('error', (error) => {
            console.error(`[ROS Runner] Process error:`, error);
            runningProcesses.delete(processKey);
            sendToRenderer('error', error.message, processKey);
            sendToRenderer('status', 'error', processKey);
        });

        console.log(`[ROS Runner] Started process with PID: ${child.pid}`);

        return {
            success: true,
            pid: child.pid,
            processKey: processKey
        };

    } catch (error) {
        console.error(`[ROS Runner] Error:`, error);
        return { success: false, error: error.message };
    }
}

module.exports = {
    init,
    runNode,
    runRobotPublisher,
    runLaunch,
    runRviz,
    stopProcess,
    stopAllProcesses,
    isProcessRunning
};
