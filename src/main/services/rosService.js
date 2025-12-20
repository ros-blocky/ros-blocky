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
        `ros2 run rviz2 rviz2`
    ];

    return await executeInPixiShell(commands, 'rviz2');
}

/**
 * Run Joint State Publisher GUI
 * @returns {Promise<Object>} Result with success status and PID
 */
async function runJointStatePublisherGui() {
    const commands = [
        `ros2 run joint_state_publisher_gui joint_state_publisher_gui`
    ];

    return await executeInPixiShell(commands, 'joint_state_publisher_gui');
}

/**
 * Run TurtleSim node
 * @returns {Promise<Object>} Result with success status and PID
 */
async function runTurtlesim() {
    const commands = [
        `ros2 run turtlesim turtlesim_node`
    ];

    return await executeInPixiShell(commands, 'turtlesim');
}

/**
 * Get list of available ROS 2 topics
 * @returns {Promise<Object>} Result with topics array
 */
async function getTopicList() {
    return new Promise((resolve) => {
        const tempDir = os.tmpdir();
        const commandsFile = path.join(tempDir, `topics_cmds_${Date.now()}.txt`);
        const commandsContent = 'ros2 topic list\n';

        fs.writeFileSync(commandsFile, commandsContent, { encoding: 'utf8' });

        const tempBatch = path.join(tempDir, `topics_run_${Date.now()}.bat`);
        const batchContent = `@echo off
cd /d ${PIXI_WS_PATH}
type "${commandsFile}" | pixi shell
`;
        fs.writeFileSync(tempBatch, batchContent, { encoding: 'utf8' });

        let output = '';
        let errorOutput = '';

        const child = spawn('cmd', ['/c', tempBatch], {
            cwd: PIXI_WS_PATH,
            windowsHide: true
        });

        child.stdout.on('data', (data) => {
            output += data.toString();
        });

        child.stderr.on('data', (data) => {
            errorOutput += data.toString();
        });

        child.on('close', (code) => {
            // Clean up temp files
            try {
                fs.unlinkSync(commandsFile);
                fs.unlinkSync(tempBatch);
            } catch (e) { /* ignore cleanup errors */ }

            if (code !== 0 && errorOutput) {
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
    return new Promise((resolve) => {
        const tempDir = os.tmpdir();
        const commandsFile = path.join(tempDir, `topic_info_${Date.now()}.txt`);
        const commandsContent = `ros2 topic info ${topicName} -v\n`;

        fs.writeFileSync(commandsFile, commandsContent, { encoding: 'utf8' });

        const tempBatch = path.join(tempDir, `topic_info_run_${Date.now()}.bat`);
        const batchContent = `@echo off
cd /d ${PIXI_WS_PATH}
type "${commandsFile}" | pixi shell
`;
        fs.writeFileSync(tempBatch, batchContent, { encoding: 'utf8' });

        let output = '';
        let errorOutput = '';

        const child = spawn('cmd', ['/c', tempBatch], {
            cwd: PIXI_WS_PATH,
            windowsHide: true
        });

        child.stdout.on('data', (data) => {
            output += data.toString();
        });

        child.stderr.on('data', (data) => {
            errorOutput += data.toString();
        });

        child.on('close', (code) => {
            // Clean up temp files
            try {
                fs.unlinkSync(commandsFile);
                fs.unlinkSync(tempBatch);
            } catch (e) { /* ignore cleanup errors */ }

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
    const processKey = `echo:${topicName}`;

    // Stop existing echo for this topic if running
    if (runningProcesses.has(processKey)) {
        await stopProcess(processKey);
    }

    return new Promise((resolve) => {
        const tempDir = os.tmpdir();
        const commandsFile = path.join(tempDir, `echo_cmds_${Date.now()}.txt`);
        const commandsContent = `ros2 topic echo ${topicName}\n`;

        fs.writeFileSync(commandsFile, commandsContent, { encoding: 'utf8' });

        const tempBatch = path.join(tempDir, `echo_run_${Date.now()}.bat`);
        const batchContent = `@echo off
cd /d ${PIXI_WS_PATH}
type "${commandsFile}" | pixi shell
`;
        fs.writeFileSync(tempBatch, batchContent, { encoding: 'utf8' });

        const child = spawn('cmd', ['/c', tempBatch], {
            cwd: PIXI_WS_PATH,
            windowsHide: true
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

            // Clean up temp files
            try {
                fs.unlinkSync(commandsFile);
                fs.unlinkSync(tempBatch);
            } catch (e) { /* ignore */ }
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
 * @returns {Promise<Object>} Result with success status
 */
async function buildAllPackages(onStatus) {
    const projectPath = getProjectPath();
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
        // Build the colcon command
        // Use --symlink-install for single package builds (faster for Python)
        let colconCmd = 'colcon build';
        if (packageName) {
            colconCmd += ` --symlink-install --packages-select ${packageName}`;
        }

        console.log(`[ROS Service] Running: ${colconCmd} in ${projectPath}`);

        // Create temp files for commands - run colcon in project directory
        const tempDir = os.tmpdir();
        const commandsFile = path.join(tempDir, `build_cmds_${Date.now()}.txt`);
        const commandsContent = [
            `cd /d "${projectPath}"`,
            colconCmd
        ].join('\n') + '\n';

        fs.writeFileSync(commandsFile, commandsContent, { encoding: 'utf8' });

        // Create batch file that pipes commands to pixi shell
        const tempBatch = path.join(tempDir, `build_run_${Date.now()}.bat`);
        const batchContent = `@echo off
cd /d ${PIXI_WS_PATH}
type "${commandsFile}" | pixi shell
`;
        fs.writeFileSync(tempBatch, batchContent, { encoding: 'utf8' });

        // Spawn the build process
        const child = spawn('cmd', ['/c', tempBatch], {
            cwd: PIXI_WS_PATH,
            windowsHide: true,
            stdio: ['ignore', 'pipe', 'pipe']
        });

        let output = '';
        let hasError = false;

        child.stdout.on('data', (data) => {
            const text = data.toString();
            output += text;
            console.log('[Build Output]', text);

            // Update status with meaningful lines
            const lines = text.split('\n').filter(l => l.trim());
            if (lines.length > 0 && onStatus) {
                const lastLine = lines[lines.length - 1].trim();
                if (lastLine) {
                    onStatus(lastLine);
                }
            }
        });

        child.stderr.on('data', (data) => {
            const text = data.toString();
            output += text;
            console.error('[Build Error]', text);

            // Only flag as error for actual build failures, not warnings
            // Check for specific failure patterns that indicate real errors
            const lowerText = text.toLowerCase();
            if (lowerText.includes('fatal error') ||
                lowerText.includes('cmake error') ||
                lowerText.includes('failed to build') ||
                lowerText.includes('colcon build failed')) {
                hasError = true;
            }
        });

        child.on('close', (code) => {
            console.log(`[ROS Service] Build completed with code: ${code}`);

            // Clean up temp files
            try {
                fs.unlinkSync(commandsFile);
                fs.unlinkSync(tempBatch);
            } catch (e) { /* ignore */ }

            // Success is based primarily on exit code (0 = success)
            // hasError is only for critical build failures
            if (code === 0) {
                resolve({ success: true, output });
            } else {
                resolve({ success: false, error: `Build failed with code ${code}`, output });
            }
        });

        child.on('error', (error) => {
            console.error('[ROS Service] Build process error:', error);
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

        // Build the full command list with workspace sourcing
        const installDir = path.join(projectPath, 'install');
        const localSetupScript = path.join(installDir, 'local_setup.bat');

        // Prepare commands - add workspace sourcing if install directory exists
        let fullCommands = [...commands];
        if (fs.existsSync(localSetupScript)) {
            // Add workspace sourcing at the beginning, before the actual command
            fullCommands.splice(0, 0, `call "${localSetupScript}"`);
            console.log(`[ROS Runner] Added workspace sourcing: ${localSetupScript}`);
        } else {
            console.log(`[ROS Runner] No workspace install found, skipping sourcing`);
        }

        // Create temp files for commands
        const tempDir = os.tmpdir();
        const commandsFile = path.join(tempDir, `ros_cmds_${Date.now()}.txt`);
        const commandsContent = fullCommands.join('\n') + '\n';
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

            // Check for "Robot initialized" to notify UI (for URDF)
            if (output.includes('Robot initialized')) {
                sendToRenderer('status', 'running', processKey);
            }
            // For nodes, detect when ros2 run command is executed or when they start logging
            if (processKey.startsWith('node:') && (output.includes('[INFO]') || output.includes('ros2 run'))) {
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
            // For nodes, detect when they start logging (ROS INFO pattern)
            if (processKey.startsWith('node:') && output.includes('[INFO]')) {
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
