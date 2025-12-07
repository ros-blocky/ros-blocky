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
}

module.exports = { registerProjectIPC };
