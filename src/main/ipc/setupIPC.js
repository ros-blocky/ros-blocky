const { ipcMain } = require('electron');
const setupService = require('../services/setupService');

function registerSetupIPC() {
    ipcMain.handle('check-internet', async () => {
        return await setupService.checkInternetConnection();
    });

    ipcMain.on('start-installation', (event) => {
        console.log('========================================');
        console.log('Received start-installation event from renderer');
        console.log('Event sender ID:', event.sender.id);
        console.log('========================================');

        // Update the setup window reference in case it changed or wasn't set
        const BrowserWindow = require('electron').BrowserWindow;
        const window = BrowserWindow.fromWebContents(event.sender);
        if (window) {
            setupService.setSetupWindow(window);
        }

        setupService.handleInstallation();
    });

    ipcMain.on('cancel-installation', () => {
        setupService.cancelInstallation();
    });
}

module.exports = { registerSetupIPC };
