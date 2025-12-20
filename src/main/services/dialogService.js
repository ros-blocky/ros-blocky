/**
 * Dialog Service - Handles all UI dialogs and prompts
 * Extracted from packageService.js for better separation of concerns
 */

const { BrowserWindow, ipcMain } = require('electron');
const path = require('path');

/**
 * Prompt user for package name
 * @returns {Promise<string|null>}
 */
function promptPackageName() {
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

        promptWindow.loadFile(path.join(__dirname, '../../renderer/dialogs/packageNamePrompt.html'));

        promptWindow.once('ready-to-show', () => {
            promptWindow.show();
        });

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
 * Show generic confirm dialog with Yes/No buttons
 * @param {string} message - Message to display
 * @returns {Promise<boolean>} - True if user clicked Yes
 */
function showConfirmDialog(message) {
    return new Promise((resolve) => {
        const confirmWindow = new BrowserWindow({
            width: 400,
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

        confirmWindow.loadFile(path.join(__dirname, '../../renderer/dialogs/confirmDialog.html'));

        confirmWindow.once('ready-to-show', () => {
            confirmWindow.show();
        });

        confirmWindow.webContents.on('did-finish-load', () => {
            confirmWindow.webContents.send('set-confirm-config', { message });
        });

        const handler = (event, confirmed) => {
            confirmWindow.close();
            ipcMain.removeListener('confirm-result', handler);
            resolve(confirmed);
        };

        ipcMain.on('confirm-result', handler);

        confirmWindow.on('closed', () => {
            ipcMain.removeListener('confirm-result', handler);
            resolve(false);
        });
    });
}

/**
 * Show generic loading dialog
 * @param {Object} config - { type: 'create'|'build', target: 'all'|'packageName' }
 * @returns {Promise<BrowserWindow>} The loading window
 */
function showLoadingDialog(config) {
    return new Promise((resolve) => {
        const loadingWindow = new BrowserWindow({
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

        loadingWindow.loadFile(path.join(__dirname, '../../renderer/dialogs/loadingDialog.html'));

        loadingWindow.once('ready-to-show', () => {
            loadingWindow.show();
        });

        loadingWindow.webContents.on('did-finish-load', () => {
            loadingWindow.webContents.send('loading-config', config);
        });

        resolve(loadingWindow);
    });
}

/**
 * Show delete package confirmation dialog
 * @param {string} packageName - Name of the package to delete
 * @returns {Promise<boolean>} - True if user confirmed
 */
function promptDeletePackage(packageName) {
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

        confirmWindow.once('ready-to-show', () => {
            confirmWindow.show();
        });

        confirmWindow.webContents.on('did-finish-load', () => {
            // Small delay to ensure renderer IPC listener is ready
            setTimeout(() => {
                confirmWindow.webContents.send('set-package-name', packageName);
            }, 100);
        });

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
 * Show generic item name prompt dialog
 * @param {string} itemType - Type of item: 'node', 'urdf', 'config', 'launch'
 * @param {string} packageName - Name of the package
 * @returns {Promise<{name: string, type: string}|null>}
 */
function promptItemName(itemType, packageName) {
    return new Promise((resolve) => {
        const promptWindow = new BrowserWindow({
            width: 450,
            height: 300,
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

        promptWindow.loadFile(path.join(__dirname, '../../renderer/dialogs/itemNamePrompt.html'));

        promptWindow.once('ready-to-show', () => {
            promptWindow.show();
        });

        promptWindow.webContents.on('did-finish-load', () => {
            promptWindow.webContents.send('set-item-config', {
                type: itemType,
                packageName: packageName
            });
        });

        const handler = (event, result) => {
            promptWindow.close();
            ipcMain.removeListener('item-name-result', handler);
            resolve(result);
        };

        ipcMain.on('item-name-result', handler);

        promptWindow.on('closed', () => {
            ipcMain.removeListener('item-name-result', handler);
            resolve(null);
        });
    });
}

// Convenience methods
async function promptNodeName(packageName) {
    const result = await promptItemName('node', packageName);
    return result ? result.name : null;
}

async function promptUrdfName(packageName) {
    const result = await promptItemName('urdf', packageName);
    return result ? result.name : null;
}

async function promptConfigName(packageName) {
    const result = await promptItemName('config', packageName);
    return result ? result.name : null;
}

async function promptLaunchName(packageName) {
    const result = await promptItemName('launch', packageName);
    return result ? result.name : null;
}

async function promptGenericFileName(packageName, title, prompt) {
    let itemType = 'node';
    if (title.toLowerCase().includes('urdf')) {
        itemType = 'urdf';
    } else if (title.toLowerCase().includes('config')) {
        itemType = 'config';
    } else if (title.toLowerCase().includes('launch')) {
        itemType = 'launch';
    }

    const result = await promptItemName(itemType, packageName);
    return result ? result.name : null;
}

module.exports = {
    promptPackageName,
    showConfirmDialog,
    showLoadingDialog,
    promptDeletePackage,
    promptItemName,
    promptNodeName,
    promptUrdfName,
    promptConfigName,
    promptLaunchName,
    promptGenericFileName
};
