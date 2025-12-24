/**
 * Package Service - Handles ROS2 package operations
 * Refactored to delegate to specialized services for better maintainability
 */

const { BrowserWindow } = require('electron');
const fs = require('fs').promises;
const path = require('path');
const { spawn } = require('child_process');

// Import specialized services
const dialogService = require('./dialogService');
const setupPyService = require('./setupPyService');
const nodeService = require('./nodeService');
const fileService = require('./fileService');

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

        // Initialize child services with project path getter
        const getProjectPath = () => this.currentProjectPath;
        setupPyService.init(getProjectPath);
        nodeService.init(getProjectPath);
        fileService.init(getProjectPath);
    }

    /**
     * Get the current project path
     * @returns {string|null}
     */
    getProjectPath() {
        return this.currentProjectPath;
    }

    // ============== Dialog Delegates ==============

    promptPackageName() {
        return dialogService.promptPackageName();
    }

    showConfirmDialog(message) {
        return dialogService.showConfirmDialog(message);
    }

    showLoadingDialog(config) {
        return dialogService.showLoadingDialog(config);
    }

    promptDeletePackage(packageName) {
        return dialogService.promptDeletePackage(packageName);
    }

    promptItemName(itemType, packageName) {
        return dialogService.promptItemName(itemType, packageName);
    }

    promptNodeName(packageName) {
        return dialogService.promptNodeName(packageName);
    }

    promptUrdfName(packageName) {
        return dialogService.promptUrdfName(packageName);
    }

    promptConfigName(packageName) {
        return dialogService.promptConfigName(packageName);
    }

    promptLaunchName(packageName) {
        return dialogService.promptLaunchName(packageName);
    }

    promptGenericFileName(packageName, title, prompt) {
        return dialogService.promptGenericFileName(packageName, title, prompt);
    }

    // ============== Node Delegates ==============

    createNode(packageName, nodeName) {
        return nodeService.createNode(packageName, nodeName);
    }

    saveNodeFile(packageName, fileName, content) {
        return nodeService.saveNodeFile(packageName, fileName, content);
    }

    saveNodeBlockState(packageName, fileName, blockXml) {
        return nodeService.saveNodeBlockState(packageName, fileName, blockXml);
    }

    loadNodeBlockState(packageName, fileName) {
        return nodeService.loadNodeBlockState(packageName, fileName);
    }

    listPackageNodes(packageName) {
        return nodeService.listPackageNodes(packageName);
    }

    toPascalCase(str) {
        return nodeService.toPascalCase(str);
    }

    // ============== File Delegates ==============

    createUrdf(packageName, urdfName) {
        return fileService.createUrdf(packageName, urdfName);
    }

    saveUrdfFile(packageName, fileName, content) {
        return fileService.saveUrdfFile(packageName, fileName, content);
    }

    saveBlockState(packageName, fileName, blockXml) {
        return fileService.saveBlockState(packageName, fileName, blockXml);
    }

    loadBlockState(packageName, fileName) {
        return fileService.loadBlockState(packageName, fileName);
    }

    listPackageUrdfs(packageName) {
        return fileService.listPackageUrdfs(packageName);
    }

    createConfig(packageName, configName) {
        return fileService.createConfig(packageName, configName);
    }

    listPackageConfigs(packageName) {
        return fileService.listPackageConfigs(packageName);
    }

    createLaunch(packageName, launchName) {
        return fileService.createLaunch(packageName, launchName);
    }

    listPackageLaunches(packageName) {
        return fileService.listPackageLaunches(packageName);
    }

    importMeshFiles(packageName) {
        return fileService.importMeshFiles(packageName);
    }

    listPackageMeshes(packageName) {
        return fileService.listPackageMeshes(packageName);
    }

    deleteSectionFiles(packageName, sectionType) {
        return fileService.deleteSectionFiles(packageName, sectionType);
    }

    deleteSectionFile(packageName, sectionType, fileName) {
        return fileService.deleteSectionFile(packageName, sectionType, fileName);
    }

    createSectionFile(packageName, folderName, fileName) {
        return fileService.createSectionFile(packageName, folderName, fileName);
    }

    // ============== Setup.py Delegates ==============

    addNodeEntryPoint(packageName, nodeName) {
        return setupPyService.addNodeEntryPoint(packageName, nodeName);
    }

    removeNodeEntryPoint(packageName, nodeName) {
        return setupPyService.removeNodeEntryPoint(packageName, nodeName);
    }

    addDataFilesEntry(packageName, folderName) {
        return setupPyService.addDataFilesEntry(packageName, folderName);
    }

    removeDataFilesEntry(packageName, folderName) {
        return setupPyService.removeDataFilesEntry(packageName, folderName);
    }

    // ============== Package Operations (kept here) ==============

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
                    contextIsolation: true,
                    webSecurity: true,
                    sandbox: true
                }
            });
            loadingWindow.loadFile(path.join(__dirname, '../../renderer/dialogs/packageCreationLoading.html'));

            loadingWindow.once('ready-to-show', () => {
                loadingWindow.show();
            });

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
     * List all packages in the current project
     * @returns {Promise<string[]>}
     */
    async listPackages() {
        try {
            if (!this.currentProjectPath) {
                return [];
            }

            const srcPath = path.join(this.currentProjectPath, 'src');

            try {
                await fs.access(srcPath);
            } catch {
                return [];
            }

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

    /**
     * Run a command in the pixi environment using pixi shell
     * @param {string} command - Command to run
     * @param {string} targetDir - Target directory for the command
     * @returns {Promise<{success: boolean, output?: string, error?: string}>}
     */
    runPixiCommand(command, targetDir) {
        const pixiUtil = require('./pixiUtil');
        return pixiUtil.executeInPixiShell([command], { targetDir });
    }
}

module.exports = new PackageService();

