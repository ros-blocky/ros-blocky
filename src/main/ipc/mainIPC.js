const { ipcMain } = require('electron');
const mainService = require('../services/mainService');

/**
 * Register IPC handlers for project management
 * This module only handles IPC communication and delegates to mainService
 */
function registerProjectIPC() {
    /**
     * Handle create-project IPC request
     */
    ipcMain.handle('create-project', async (event) => {
        return await mainService.createNewProject();
    });

    /**
     * Handle open-project IPC request
     */
    ipcMain.handle('open-project', async (event) => {
        return await mainService.openExistingProject();
    });

    /**
     * Handle create-package IPC request
     */
    const packageService = require('../services/packageService');
    ipcMain.handle('create-package', async (event, packageName) => {
        return await packageService.createPackage(packageName);
    });

    /**
     * Handle prompt-package-name IPC request
     */
    ipcMain.handle('prompt-package-name', async (event) => {
        return await packageService.promptPackageName();
    });

    /**
     * Handle list-packages IPC request
     */
    ipcMain.handle('list-packages', async (event) => {
        return await packageService.listPackages();
    });

    /**
     * Handle delete-package IPC request
     */
    ipcMain.handle('delete-package', async (event, packageName) => {
        return await packageService.deletePackage(packageName);
    });

    /**
     * Handle prompt-node-name IPC request
     */
    ipcMain.handle('prompt-node-name', async (event, packageName) => {
        return await packageService.promptNodeName(packageName);
    });

    /**
     * Handle create-node IPC request
     */
    ipcMain.handle('create-node', async (event, packageName, nodeName) => {
        return await packageService.createNode(packageName, nodeName);
    });

    /**
     * Handle list-package-nodes IPC request
     */
    ipcMain.handle('list-package-nodes', async (event, packageName) => {
        return await packageService.listPackageNodes(packageName);
    });

    // ========== URDF Operations ==========

    /**
     * Handle prompt-urdf-name IPC request
     */
    ipcMain.handle('prompt-urdf-name', async (event, packageName) => {
        return await packageService.promptUrdfName(packageName);
    });

    /**
     * Handle create-urdf IPC request
     */
    ipcMain.handle('create-urdf', async (event, packageName, urdfName) => {
        return await packageService.createUrdf(packageName, urdfName);
    });

    /**
     * Handle list-package-urdfs IPC request
     */
    ipcMain.handle('list-package-urdfs', async (event, packageName) => {
        return await packageService.listPackageUrdfs(packageName);
    });

    // ========== Config Operations ==========

    /**
     * Handle prompt-config-name IPC request
     */
    ipcMain.handle('prompt-config-name', async (event, packageName) => {
        return await packageService.promptConfigName(packageName);
    });

    /**
     * Handle create-config IPC request
     */
    ipcMain.handle('create-config', async (event, packageName, configName) => {
        return await packageService.createConfig(packageName, configName);
    });

    /**
     * Handle list-package-configs IPC request
     */
    ipcMain.handle('list-package-configs', async (event, packageName) => {
        return await packageService.listPackageConfigs(packageName);
    });

    // ========== Launch Operations ==========

    /**
     * Handle prompt-launch-name IPC request
     */
    ipcMain.handle('prompt-launch-name', async (event, packageName) => {
        return await packageService.promptLaunchName(packageName);
    });

    /**
     * Handle create-launch IPC request
     */
    ipcMain.handle('create-launch', async (event, packageName, launchName) => {
        return await packageService.createLaunch(packageName, launchName);
    });

    /**
     * Handle list-package-launches IPC request
     */
    ipcMain.handle('list-package-launches', async (event, packageName) => {
        return await packageService.listPackageLaunches(packageName);
    });

    // ========== Generic File Operations ==========

    /**
     * Handle prompt-file-name IPC request (generic prompt for urdf, config, launch)
     */
    ipcMain.handle('prompt-file-name', async (event, packageName, title, prompt) => {
        return await packageService.promptGenericFileName(packageName, title, prompt);
    });

    /**
     * Handle create-section-file IPC request (create file in urdf, config, launch folders)
     */
    ipcMain.handle('create-section-file', async (event, packageName, folderName, fileName) => {
        return await packageService.createSectionFile(packageName, folderName, fileName);
    });
}

module.exports = { registerProjectIPC };
