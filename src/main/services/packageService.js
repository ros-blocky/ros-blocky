const { dialog, BrowserWindow } = require('electron');
const fs = require('fs').promises;
const path = require('path');
const { spawn } = require('child_process');

/**
 * Package Service - Handles ROS2 package operations
 */
class PackageService {
    constructor() {
        this.currentProjectPath = null;
    }

    /**
     * Set the current project path
     * @param {string} projectPath 
     */
    setProjectPath(projectPath) {
        this.currentProjectPath = projectPath;
    }

    /**
     * Prompt user for package name
     * @returns {Promise<string|null>}
     */
    /**
     * Prompt user for package name
     * @returns {Promise<string|null>}
     */
    async promptPackageName() {
        return new Promise((resolve) => {
            const promptWindow = new BrowserWindow({
                width: 450,
                height: 220,
                modal: true,
                frame: false,
                resizable: false,
                show: false,
                parent: BrowserWindow.getFocusedWindow(),
                webPreferences: {
                    preload: path.join(__dirname, '../../preload/dialogPreload.js'),
                    nodeIntegration: false,
                    contextIsolation: true
                }
            });

            // Load HTML file
            promptWindow.loadFile(path.join(__dirname, '../../renderer/dialogs/packageNamePrompt.html'));

            // Show when ready to prevent white flash
            promptWindow.once('ready-to-show', () => {
                promptWindow.show();
            });

            const { ipcMain } = require('electron');
            const handler = (event, name) => {
                promptWindow.close();
                ipcMain.removeListener('package-name-result', handler);
                resolve(name);
            };

            ipcMain.on('package-name-result', handler);

            promptWindow.on('closed', () => {
                ipcMain.removeListener('package-name-result', handler);
                resolve(null);
            });
        });
    }

    /**
     * Create a new ROS2 package
     * @param {string} packageName - Name of the package to create
     * @returns {Promise<{success: boolean, message: string, packagePath?: string}>}
     */
    async createPackage(packageName) {
        let loadingWindow = null;

        try {
            if (!this.currentProjectPath) {
                return {
                    success: false,
                    message: 'No project is currently loaded. Please open or create a project first.'
                };
            }

            // Validate package name
            if (!packageName || packageName.trim() === '') {
                return { success: false, message: 'Package name cannot be empty' };
            }

            // ROS2 package naming rules: lowercase letters, numbers, underscores
            const validNamePattern = /^[a-z][a-z0-9_]*$/;
            if (!validNamePattern.test(packageName)) {
                return {
                    success: false,
                    message: 'Invalid package name. Must start with a letter and contain only lowercase letters, numbers, and underscores.'
                };
            }

            const srcPath = path.join(this.currentProjectPath, 'src');
            const packagePath = path.join(srcPath, packageName);

            // Check if package already exists
            try {
                await fs.access(packagePath);
                return {
                    success: false,
                    message: `Package "${packageName}" already exists in this project.`
                };
            } catch {
                // Package doesn't exist, continue
            }

            // Show loading dialog
            loadingWindow = new BrowserWindow({
                width: 400,
                height: 280,
                modal: true,
                frame: false,
                resizable: false,
                show: false,
                parent: BrowserWindow.getFocusedWindow(),
                webPreferences: {
                    preload: path.join(__dirname, '../../preload/dialogPreload.js'),
                    nodeIntegration: false,
                    contextIsolation: true
                }
            });
            loadingWindow.loadFile(path.join(__dirname, '../../renderer/dialogs/packageCreationLoading.html'));

            // Show when ready to prevent white flash
            loadingWindow.once('ready-to-show', () => {
                loadingWindow.show();
            });

            // Update status
            loadingWindow.webContents.on('did-finish-load', () => {
                loadingWindow.webContents.send('package-creation-status', `Creating package "${packageName}"...`);
            });

            // Ensure src directory exists
            await fs.mkdir(srcPath, { recursive: true });

            // Create package: source ROS2 setup then use pixi run
            const result = await this.runPixiCommand(
                `ros2 pkg create ${packageName} --build-type ament_python`,
                srcPath
            );

            // Close loading dialog
            if (loadingWindow && !loadingWindow.isDestroyed()) {
                loadingWindow.close();
            }

            if (result.success || result.output?.includes('going to create')) {
                return {
                    success: true,
                    message: `Package "${packageName}" created successfully!`,
                    packagePath
                };
            } else {
                return {
                    success: false,
                    message: `Failed to create package: ${result.error}`
                };
            }

        } catch (error) {
            // Close loading dialog on error
            if (loadingWindow && !loadingWindow.isDestroyed()) {
                loadingWindow.close();
            }
            console.error('Error creating package:', error);
            return {
                success: false,
                message: `Error creating package: ${error.message}`
            };
        }
    }

    /**
     * Show delete confirmation dialog
     * @param {string} packageName - Name of the package to delete
     * @returns {Promise<boolean>} - True if user confirmed, false otherwise
     */
    promptDeletePackage(packageName) {
        return new Promise((resolve) => {
            const confirmWindow = new BrowserWindow({
                width: 450,
                height: 380,
                modal: true,
                frame: false,
                resizable: false,
                show: false,
                parent: BrowserWindow.getFocusedWindow(),
                webPreferences: {
                    preload: path.join(__dirname, '../../preload/dialogPreload.js'),
                    nodeIntegration: false,
                    contextIsolation: true
                }
            });

            confirmWindow.loadFile(path.join(__dirname, '../../renderer/dialogs/deletePackageConfirm.html'));

            // Show when ready to prevent white flash
            confirmWindow.once('ready-to-show', () => {
                confirmWindow.show();
            });

            // Send package name after load
            confirmWindow.webContents.on('did-finish-load', () => {
                confirmWindow.webContents.send('set-package-name', packageName);
            });

            const { ipcMain } = require('electron');
            const handler = (event, confirmedName) => {
                confirmWindow.close();
                ipcMain.removeListener('delete-package-confirmed', handler);
                resolve(confirmedName === packageName);
            };

            ipcMain.on('delete-package-confirmed', handler);

            confirmWindow.on('closed', () => {
                ipcMain.removeListener('delete-package-confirmed', handler);
                resolve(false);
            });
        });
    }

    /**
     * Delete a ROS2 package
     * @param {string} packageName - Name of the package to delete
     * @returns {Promise<{success: boolean, message: string}>}
     */
    async deletePackage(packageName) {
        try {
            if (!this.currentProjectPath) {
                return {
                    success: false,
                    message: 'No project is currently loaded.'
                };
            }

            // Show confirmation dialog
            const confirmed = await this.promptDeletePackage(packageName);
            if (!confirmed) {
                return {
                    success: false,
                    message: 'Deletion cancelled'
                };
            }

            const packagePath = path.join(this.currentProjectPath, 'src', packageName);

            // Check if package exists
            try {
                await fs.access(packagePath);
            } catch {
                return {
                    success: false,
                    message: `Package "${packageName}" not found.`
                };
            }

            // Delete the package folder recursively
            await fs.rm(packagePath, { recursive: true, force: true });

            return {
                success: true,
                message: `Package "${packageName}" deleted successfully.`
            };

        } catch (error) {
            console.error('Error deleting package:', error);
            return {
                success: false,
                message: `Error deleting package: ${error.message}`
            };
        }
    }

    /**
     * Show node name prompt dialog
     * @param {string} packageName - Name of the package
     * @returns {Promise<string|null>} - Node name or null if cancelled
     */
    promptNodeName(packageName) {
        return new Promise((resolve) => {
            const promptWindow = new BrowserWindow({
                width: 450,
                height: 280,
                modal: true,
                frame: false,
                resizable: false,
                show: false,
                parent: BrowserWindow.getFocusedWindow(),
                webPreferences: {
                    preload: path.join(__dirname, '../../preload/dialogPreload.js'),
                    nodeIntegration: false,
                    contextIsolation: true
                }
            });

            promptWindow.loadFile(path.join(__dirname, '../../renderer/dialogs/nodeNamePrompt.html'));

            // Show when ready to prevent white flash
            promptWindow.once('ready-to-show', () => {
                promptWindow.show();
            });

            // Send package name after load
            promptWindow.webContents.on('did-finish-load', () => {
                promptWindow.webContents.send('set-package-name', packageName);
            });

            const { ipcMain } = require('electron');
            const handler = (event, nodeName) => {
                promptWindow.close();
                ipcMain.removeListener('node-name-result', handler);
                resolve(nodeName);
            };

            ipcMain.on('node-name-result', handler);

            promptWindow.on('closed', () => {
                ipcMain.removeListener('node-name-result', handler);
                resolve(null);
            });
        });
    }

    /**
     * Create a new node in a package
     * @param {string} packageName - Name of the package
     * @param {string} nodeName - Name of the node
     * @returns {Promise<{success: boolean, message: string, nodePath?: string}>}
     */
    async createNode(packageName, nodeName) {
        try {
            if (!this.currentProjectPath) {
                return {
                    success: false,
                    message: 'No project is currently loaded.'
                };
            }

            // Validate node name
            if (!nodeName || nodeName.trim() === '') {
                return { success: false, message: 'Node name cannot be empty' };
            }

            const validNamePattern = /^[a-z][a-z0-9_]*$/;
            if (!validNamePattern.test(nodeName)) {
                return {
                    success: false,
                    message: 'Invalid node name. Must start with a letter and contain only lowercase letters, numbers, and underscores.'
                };
            }

            const packagePath = path.join(this.currentProjectPath, 'src', packageName);
            const nodeDir = path.join(packagePath, packageName);
            const nodePath = path.join(nodeDir, `${nodeName}.py`);

            // Check if package exists
            try {
                await fs.access(packagePath);
            } catch {
                return {
                    success: false,
                    message: `Package "${packageName}" not found.`
                };
            }

            // Check if node already exists
            try {
                await fs.access(nodePath);
                return {
                    success: false,
                    message: `Node "${nodeName}" already exists in package "${packageName}".`
                };
            } catch {
                // Node doesn't exist, continue
            }

            // Create node Python file
            const nodeContent = `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class ${this.toPascalCase(nodeName)}(Node):
    """${nodeName} node."""

    def __init__(self):
        super().__init__('${nodeName}')
        self.get_logger().info('${nodeName} node started')


def main(args=None):
    rclpy.init(args=args)
    node = ${this.toPascalCase(nodeName)}()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
`;

            await fs.writeFile(nodePath, nodeContent);

            // Update setup.py to add entry point
            await this.addNodeEntryPoint(packageName, nodeName);

            return {
                success: true,
                message: `Node "${nodeName}" created successfully!`,
                nodePath
            };

        } catch (error) {
            console.error('Error creating node:', error);
            return {
                success: false,
                message: `Error creating node: ${error.message}`
            };
        }
    }

    /**
     * Convert snake_case to PascalCase
     */
    toPascalCase(str) {
        return str.split('_').map(word =>
            word.charAt(0).toUpperCase() + word.slice(1)
        ).join('');
    }

    /**
     * Add node entry point to setup.py
     */
    async addNodeEntryPoint(packageName, nodeName) {
        const setupPyPath = path.join(this.currentProjectPath, 'src', packageName, 'setup.py');

        try {
            let content = await fs.readFile(setupPyPath, 'utf8');

            // Find the console_scripts section and add the entry point
            const entryPoint = `'${nodeName} = ${packageName}.${nodeName}:main'`;

            // Check if entry point already exists
            if (content.includes(entryPoint)) {
                return;
            }

            // Find console_scripts array and add entry
            const consoleScriptsMatch = content.match(/'console_scripts':\s*\[\s*\]/);
            if (consoleScriptsMatch) {
                // Empty array - add first entry
                content = content.replace(
                    /'console_scripts':\s*\[\s*\]/,
                    `'console_scripts': [\n            ${entryPoint},\n        ]`
                );
            } else {
                // Array has entries - add to existing
                const match = content.match(/'console_scripts':\s*\[([^\]]*)\]/);
                if (match) {
                    const existingEntries = match[1].trim();
                    if (existingEntries) {
                        content = content.replace(
                            /'console_scripts':\s*\[([^\]]*)\]/,
                            `'console_scripts': [${existingEntries}\n            ${entryPoint},\n        ]`
                        );
                    }
                }
            }

            await fs.writeFile(setupPyPath, content);
        } catch (error) {
            console.error('Error updating setup.py:', error);
        }
    }

    /**
     * List nodes in a package
     * @param {string} packageName - Name of the package
     * @returns {Promise<string[]>}
     */
    async listPackageNodes(packageName) {
        try {
            const nodeDir = path.join(this.currentProjectPath, 'src', packageName, packageName);
            const files = await fs.readdir(nodeDir);

            // Filter .py files, exclude __init__.py
            return files
                .filter(f => f.endsWith('.py') && f !== '__init__.py')
                .map(f => f.replace('.py', ''));
        } catch {
            return [];
        }
    }

    /**
     * Run a command in the pixi environment
     * @param {string} command - Command to run
     * @param {string} cwd - Working directory
     * @returns {Promise<{success: boolean, output?: string, error?: string}>}
     */
    runPixiCommand(command, cwd) {
        return new Promise((resolve) => {
            // Build full command: source ROS2 setup, then run pixi
            const fullCommand = `call C:\\pixi_ws\\ros2-windows\\local_setup.bat && pixi run ${command}`;

            const pixiProcess = spawn('cmd.exe', ['/c', fullCommand], {
                cwd,
                shell: false
            });

            let stdout = '';
            let stderr = '';

            pixiProcess.stdout.on('data', (data) => {
                stdout += data.toString();
                console.log('Output:', data.toString());
            });

            pixiProcess.stderr.on('data', (data) => {
                stderr += data.toString();
                console.log('Stderr:', data.toString());
            });

            pixiProcess.on('close', (code) => {
                console.log('pixi run closed with code:', code);
                if (code === 0 || stdout.includes('going to create')) {
                    resolve({ success: true, output: stdout });
                } else {
                    resolve({ success: false, error: stderr || stdout });
                }
            });

            pixiProcess.on('error', (error) => {
                resolve({ success: false, error: error.message });
            });
        });
    }

    /**
     * List all packages in the current project
     * @returns {Promise<string[]>}
     */
    async listPackages() {
        try {
            if (!this.currentProjectPath) {
                return [];
            }

            const srcPath = path.join(this.currentProjectPath, 'src');

            // Check if src directory exists
            try {
                await fs.access(srcPath);
            } catch {
                return [];
            }

            // Read all directories in src
            const entries = await fs.readdir(srcPath, { withFileTypes: true });
            const packages = entries
                .filter(entry => entry.isDirectory())
                .map(entry => entry.name);

            return packages;

        } catch (error) {
            console.error('Error listing packages:', error);
            return [];
        }
    }
}

module.exports = new PackageService();
