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
                parent: BrowserWindow.getFocusedWindow(),
                webPreferences: {
                    nodeIntegration: true,
                    contextIsolation: false
                }
            });

            // Load HTML file
            promptWindow.loadFile(path.join(__dirname, '../../renderer/dialogs/packageNamePrompt.html'));

            const { ipcMain } = require('electron');
            const handler = (event, name) => {
                promptWindow.close();
                ipcMain.removeListener('package-name-response', handler);
                resolve(name);
            };

            ipcMain.on('package-name-response', handler);

            promptWindow.on('closed', () => {
                ipcMain.removeListener('package-name-response', handler);
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

            // Ensure src directory exists
            await fs.mkdir(srcPath, { recursive: true });

            // Create package using ros2 pkg create command
            const result = await this.runPixiCommand(
                `ros2 pkg create ${packageName} --build-type ament_python`,
                srcPath
            );

            if (result.success) {
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
            console.error('Error creating package:', error);
            return {
                success: false,
                message: `Error creating package: ${error.message}`
            };
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
            const pixiProcess = spawn('pixi', ['run', ...command.split(' ')], {
                cwd,
                shell: true
            });

            let stdout = '';
            let stderr = '';

            pixiProcess.stdout.on('data', (data) => {
                stdout += data.toString();
            });

            pixiProcess.stderr.on('data', (data) => {
                stderr += data.toString();
            });

            pixiProcess.on('close', (code) => {
                if (code === 0) {
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
