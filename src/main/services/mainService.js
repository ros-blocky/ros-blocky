const { dialog, BrowserWindow, ipcMain } = require('electron');
const fs = require('fs').promises;
const path = require('path');
const { spawn } = require('child_process');

/**
 * Main Service - Handles business logic for project operations
 */
class MainService {
    /**
     * Create a new ROS2 project
     * @returns {Promise<{success: boolean, message: string, projectPath?: string}>}
     */
    async createNewProject() {
        try {
            // Step 1: Get project name from user
            const projectName = await this.promptProjectName();

            if (!projectName) {
                return { success: false, message: 'Project creation cancelled' };
            }

            // Step 2: Show folder picker to select parent location
            const result = await dialog.showOpenDialog({
                title: `Select Location for "${projectName}"`,
                buttonLabel: 'Create Project Here',
                properties: ['openDirectory', 'createDirectory']
            });

            if (result.canceled) {
                return { success: false, message: 'Project creation cancelled' };
            }

            const parentPath = result.filePaths[0];
            const projectPath = path.join(parentPath, projectName);

            // **Notify renderer that dialogs are done - show loading screen now!**
            const focusedWindow = BrowserWindow.getFocusedWindow();
            if (focusedWindow) {
                focusedWindow.webContents.send('project-dialogs-complete', 'Creating');
            }

            // Step 3: Check if folder already exists
            try {
                await fs.access(projectPath);
                return {
                    success: false,
                    message: `A folder named "${projectName}" already exists in this location.\nPlease choose a different name or location.`
                };
            } catch {
                // Folder doesn't exist - good, continue
            }

            // Step 4: Create project folder and src subfolder
            await fs.mkdir(projectPath, { recursive: true });
            await fs.mkdir(path.join(projectPath, 'src'), { recursive: true });

            // Step 5: Run colcon build
            return await this.runColconBuild(projectPath);

        } catch (error) {
            console.error('Error creating project:', error);
            return { success: false, message: error.message };
        }
    }

    /**
     * Prompt user for project name
     * @returns {Promise<string|null>}
     */
    async promptProjectName() {
        return new Promise((resolve) => {
            const promptWindow = new BrowserWindow({
                width: 450,
                height: 180,
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
            promptWindow.loadFile(path.join(__dirname, '../../renderer/dialogs/projectNamePrompt.html'));

            // Show when ready to prevent white flash
            promptWindow.once('ready-to-show', () => {
                promptWindow.show();
            });

            const handler = (event, name) => {
                promptWindow.close();
                ipcMain.removeListener('project-name-result', handler);
                resolve(name);
            };

            ipcMain.on('project-name-result', handler);

            promptWindow.on('closed', () => {
                ipcMain.removeListener('project-name-result', handler);
                resolve(null);
            });
        });
    }

    /**
     * Open an existing ROS2 project
     * @returns {Promise<{success: boolean, message: string, projectPath?: string}>}
     */
    async openExistingProject() {
        try {
            const result = await dialog.showOpenDialog({
                title: 'Open ROS2 Project',
                buttonLabel: 'Open Project',
                properties: ['openDirectory']
            });

            if (result.canceled) {
                return { success: false, message: 'Project opening cancelled' };
            }

            const projectPath = result.filePaths[0];

            // **Notify renderer that dialog is done - show loading screen now!**
            const focusedWindow = BrowserWindow.getFocusedWindow();
            if (focusedWindow) {
                focusedWindow.webContents.send('project-dialogs-complete', 'Opening');
            }

            const isValid = await this.isValidROS2Workspace(projectPath);

            if (!isValid) {
                return {
                    success: false,
                    message: 'Selected folder is not a valid ROS2 workspace.\n\nA valid workspace must have:\n- src/ folder\n- At least one of: build/, install/, or log/ folders'
                };
            }

            // Set project path in package service
            const packageService = require('./packageService');
            packageService.setProjectPath(projectPath);

            // Notify renderer that project is loaded
            if (focusedWindow) {
                focusedWindow.webContents.send('project-loaded', projectPath);
            }

            return {
                success: true,
                message: 'Project opened successfully',
                projectPath: projectPath
            };
        } catch (error) {
            console.error('Error opening project:', error);
            return { success: false, message: error.message };
        }
    }

    /**
     * Validate ROS2 workspace
     */
    async isValidROS2Workspace(projectPath) {
        try {
            const srcPath = path.join(projectPath, 'src');
            const hasSrc = await fs.access(srcPath).then(() => true).catch(() => false);
            if (!hasSrc) return false;

            const buildPath = path.join(projectPath, 'build');
            const installPath = path.join(projectPath, 'install');
            const logPath = path.join(projectPath, 'log');

            const hasBuild = await fs.access(buildPath).then(() => true).catch(() => false);
            const hasInstall = await fs.access(installPath).then(() => true).catch(() => false);
            const hasLog = await fs.access(logPath).then(() => true).catch(() => false);

            return hasSrc && (hasBuild || hasInstall || hasLog);
        } catch (error) {
            console.error('Error validating workspace:', error);
            return false;
        }
    }

    /**
     * Run colcon build
     */
    runColconBuild(projectPath) {
        return new Promise((resolve) => {
            console.log('Starting colcon build for project:', projectPath);

            const shell = spawn('cmd.exe', [], {
                cwd: 'c:\\pixi_ws',
                windowsHide: true
            });

            let output = '';
            let errorOutput = '';
            let buildCompleted = false;

            shell.stdout.on('data', (data) => {
                const text = data.toString();
                output += text;
                console.log('Output:', text);
                if (text.includes('Summary:') || text.includes('Finished')) {
                    buildCompleted = true;
                }
            });

            shell.stderr.on('data', (data) => {
                const text = data.toString();
                errorOutput += text;
                console.error('Error:', text);
            });

            shell.on('close', (code) => {
                console.log('Shell closed with code:', code);
                if (buildCompleted || code === 0) {
                    // Set project path in package service
                    const packageService = require('./packageService');
                    packageService.setProjectPath(projectPath);

                    // Notify renderer that project is loaded
                    const focusedWindow = BrowserWindow.getFocusedWindow();
                    if (focusedWindow) {
                        focusedWindow.webContents.send('project-loaded', projectPath);
                    }

                    resolve({
                        success: true,
                        message: 'Project created successfully',
                        projectPath: projectPath
                    });
                } else {
                    resolve({
                        success: false,
                        message: `colcon build failed: ${errorOutput || 'Unknown error'}`
                    });
                }
            });

            shell.on('error', (error) => {
                console.error('Shell error:', error);
                resolve({
                    success: false,
                    message: `Failed to execute commands: ${error.message}`
                });
            });

            setTimeout(() => {
                console.log('Sending commands to shell...');
                shell.stdin.write('cd c:\\pixi_ws\n');
                shell.stdin.write('pixi shell\n');
                setTimeout(() => {
                    shell.stdin.write(`cd /d "${projectPath}"\n`);
                    shell.stdin.write('colcon build\n');
                    setTimeout(() => {
                        shell.stdin.write('exit\n');
                        shell.stdin.write('exit\n');
                    }, 10000);
                }, 2000);
            }, 500);
        });
    }
}

module.exports = new MainService();
