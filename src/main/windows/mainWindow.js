const { BrowserWindow } = require('electron');
const path = require('path');

let mainWindow = null;

/**
 * Creates the main application window
 * @returns {BrowserWindow} The created window
 */
function createMainWindow() {
    // Create the browser window.
    mainWindow = new BrowserWindow({
        width: 1920,
        height: 1080,
        frame: false,
        title: 'ROS Block Code',
        webPreferences: {
            preload: path.join(__dirname, '../../preload/mainPreload.js'),
            contextIsolation: true,
            nodeIntegration: false,
            webSecurity: true,
            sandbox: true
        }
    });

    // Maximize the window to fullscreen
    mainWindow.maximize();

    // and load the index.html of the app.
    mainWindow.loadFile(path.join(__dirname, '../../renderer/main/index.html'));

    // Open the DevTools.
    // mainWindow.webContents.openDevTools()

    mainWindow.on('closed', () => {
        mainWindow = null;
    });

    // Window control handlers
    const { ipcMain } = require('electron');

    ipcMain.on('window-minimize', () => {
        if (mainWindow) mainWindow.minimize();
    });

    ipcMain.on('window-maximize', () => {
        if (mainWindow) {
            if (mainWindow.isMaximized()) {
                mainWindow.unmaximize();
            } else {
                mainWindow.maximize();
            }
        }
    });

    ipcMain.on('window-close', () => {
        if (mainWindow) mainWindow.close();
    });

    return mainWindow;
}

module.exports = { createMainWindow };
