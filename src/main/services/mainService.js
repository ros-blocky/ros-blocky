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
            console.log('[MainService] Project name received:', projectName);

            if (!projectName) {
                console.log('[MainService] No project name, returning cancelled');
                return { success: false, message: 'Project creation cancelled' };
            }

            // Step 2: Show folder picker to select parent location
            // Get the main window - getFocusedWindow might return null after dialog closes
            const allWindows = BrowserWindow.getAllWindows();
            const mainWindow = allWindows.find(w => !w.isDestroyed() && w.webContents.getURL().includes('index.html')) || BrowserWindow.getFocusedWindow();
            console.log('[MainService] Opening folder picker, mainWindow:', mainWindow ? 'found' : 'null');

            const result = await dialog.showOpenDialog(mainWindow, {
                title: `Select Location for "${projectName}"`,
                buttonLabel: 'Create Project Here',
                properties: ['openDirectory', 'createDirectory']
            });
            console.log('[MainService] Folder picker result:', result);

            if (result.canceled) {
                console.log('[MainService] Folder picker cancelled');
                return { success: false, message: 'Project creation cancelled' };
            }

            const parentPath = result.filePaths[0];
            const projectPath = path.join(parentPath, projectName);
            console.log('[MainService] Project path:', projectPath);

            // **Notify renderer that dialogs are done - show loading screen now!**
            if (mainWindow && !mainWindow.isDestroyed()) {
                mainWindow.webContents.send('project-dialogs-complete', 'Creating');
            }

            // Step 3: Check if folder already exists
            try {
                await fs.access(projectPath);
                console.log('[MainService] Folder already exists');
                return {
                    success: false,
                    message: `A folder named "${projectName}" already exists in this location.\nPlease choose a different name or location.`
                };
            } catch {
                // Folder doesn't exist - good, continue
                console.log('[MainService] Folder does not exist, proceeding');
            }

            // Step 4: Create project folder and src subfolder
            console.log('[MainService] Creating project folders...');
            await fs.mkdir(projectPath, { recursive: true });
            await fs.mkdir(path.join(projectPath, 'src'), { recursive: true });

            // Step 5: Run colcon build
            console.log('[MainService] Running colcon build...');
            return await this.runColconBuild(projectPath);

        } catch (error) {
            console.error('[MainService] Error creating project:', error);
            return { success: false, message: error.message };
        }
    }

    /**
     * Prompt user for project name
     * @returns {Promise<string|null>}
     */
    async promptProjectName() {
        return new Promise((resolve) => {
            let resolved = false; // Flag to prevent double resolution
            console.log('[MainService] Opening project name prompt dialog...');

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
                    contextIsolation: true,
                    webSecurity: true,
                    sandbox: true
                }
            });

            // Load HTML file
            const htmlPath = path.join(__dirname, '../../renderer/dialogs/projectNamePrompt.html');
            console.log('[MainService] Loading dialog from:', htmlPath);
            promptWindow.loadFile(htmlPath);

            // Show when ready to prevent white flash
            promptWindow.once('ready-to-show', () => {
                console.log('[MainService] Dialog ready to show');
                promptWindow.show();
            });

            // Debug: Open DevTools for the dialog (disabled - was blocking interaction)
            // promptWindow.webContents.openDevTools({ mode: 'detach' });

            const handler = (event, name) => {
                console.log('[MainService] Received project-name-result:', name);
                if (resolved) {
                    console.log('[MainService] Already resolved, ignoring');
                    return;
                }
                resolved = true;
                ipcMain.removeListener('project-name-result', handler);
                promptWindow.close();
                resolve(name);
            };

            ipcMain.on('project-name-result', handler);

            promptWindow.on('closed', () => {
                console.log('[MainService] Dialog closed, resolved=', resolved);
                ipcMain.removeListener('project-name-result', handler);
                if (!resolved) {
                    console.log('[MainService] Resolving with null (cancelled)');
                    resolved = true;
                    resolve(null);
                }
            });
        });
    }

    /**
     * Open an existing ROS2 project
     * @returns {Promise<{success: boolean, message: string, projectPath?: string}>}
     */
    async openExistingProject() {
        try {
            // Pass focused window as parent to make dialog modal (blocks main window)
            const parentWindow = BrowserWindow.getFocusedWindow();
            const result = await dialog.showOpenDialog(parentWindow, {
                title: 'Open ROS2 Project',
                buttonLabel: 'Open Project',
                properties: ['openDirectory']
            });

            if (result.canceled) {
                return { success: false, message: 'Project opening cancelled' };
            }

            const projectPath = result.filePaths[0];

            // **Notify renderer that dialog is done - show loading screen now!**
            // Use parentWindow consistently to avoid race condition if window loses focus
            if (parentWindow && !parentWindow.isDestroyed()) {
                parentWindow.webContents.send('project-dialogs-complete', 'Opening');
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
            // Use parentWindow consistently to avoid race condition
            if (parentWindow && !parentWindow.isDestroyed()) {
                parentWindow.webContents.send('project-loaded', projectPath);
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

                    // Find main window explicitly - don't rely on getFocusedWindow
                    // which can be null during fast transitions
                    const allWindows = BrowserWindow.getAllWindows();
                    const mainWindow = allWindows.find(w =>
                        !w.isDestroyed() &&
                        w.webContents.getURL().includes('index.html')
                    );

                    if (mainWindow && !mainWindow.isDestroyed()) {
                        console.log('[MainService] Sending project-loaded event');
                        mainWindow.webContents.send('project-loaded', projectPath);
                    } else {
                        console.warn('[MainService] Main window not found for project-loaded event');
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
