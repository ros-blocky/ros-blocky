const { BrowserWindow } = require('electron');
const path = require('path');
const { createMainWindow } = require('./mainWindow');
const setupService = require('../services/setupService');
const { registerSetupIPC } = require('../ipc/setupIPC');
// Register IPC handlers
registerSetupIPC();

/**
 * Creates and returns the setup window
 * @returns {BrowserWindow} The setup window instance
 */
function createSetupWindow() {
    console.log('Creating setup window...');

    const setupWindow = new BrowserWindow({
        width: 900,
        height: 300,
        resizable: false,
        frame: false,
        autoHideMenuBar: true,
        backgroundColor: '#ffffff',
        webPreferences: {
            preload: path.join(__dirname, '../../preload/setupPreload.js'),
            contextIsolation: true,
            nodeIntegration: false,
            sandbox: false
        }
    });

    setupWindow.loadFile(path.join(__dirname, '../../renderer/setup/setup.html'));

    // Log when page is loaded
    setupWindow.webContents.on('did-finish-load', () => {
        console.log('Setup window finished loading');
    });

    // Log console messages from renderer
    setupWindow.webContents.on('console-message', (event, level, message, line, sourceId) => {
        console.log(`[Renderer Console]: ${message}`);
    });

    // Open DevTools for debugging
    // setupWindow.webContents.openDevTools();

    console.log('Setup window created');
    return setupWindow;
}

/**
 * Initializes the application by checking environment status
 * and opening the appropriate window.
 */
function initialize() {
    // Configure setup service with the ability to create main window
    setupService.setMainWindowCreator(createMainWindow);

    if (setupService.isEnvironmentReady()) {
        console.log('Environment is ready - opening main window');
        createMainWindow();
    } else {
        console.log('Environment not ready - opening setup window');
        const setupWindow = createSetupWindow();
        setupService.setSetupWindow(setupWindow);
    }
}

module.exports = { createSetupWindow, initialize };
