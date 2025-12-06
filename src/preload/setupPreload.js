const { contextBridge, ipcRenderer } = require('electron');

/**
 * Preload script for the setup window
 * Exposes secure IPC APIs to the renderer process
 */
contextBridge.exposeInMainWorld('setupAPI', {
    /**
     * Start the installation process
     */
    startInstallation: () => {
        ipcRenderer.send('start-installation');
    },

    /**
     * Listen for log messages from the installation process
     * @param {Function} callback - Called with each log line
     */
    onLogMessage: (callback) => {
        ipcRenderer.on('installation-log', (event, data) => {
            callback(data);
        });
    },

    /**
     * Listen for installation completion
     * @param {Function} callback - Called with success status
     */
    onInstallComplete: (callback) => {
        ipcRenderer.on('installation-complete', (event, success) => {
            callback(success);
        });
    },

    /**
     * Listen for error messages
     * @param {Function} callback - Called with error message
     */
    onError: (callback) => {
        ipcRenderer.on('installation-error', (event, errorMessage) => {
            callback(errorMessage);
        });
    },

    /**
     * Listen for progress updates
     * @param {Function} callback - Called with progress data {current, total, percentage, message}
     */
    onInstallationProgress: (callback) => {
        ipcRenderer.on('installation-progress', (event, data) => {
            callback(data);
        });
    },

    /**
     * Check for internet connection
     * @returns {Promise<boolean>} True if connected
     */
    checkInternet: () => {
        return ipcRenderer.invoke('check-internet');
    },

    /**
     * Cancel the installation
     */
    cancelInstallation: () => {
        ipcRenderer.send('cancel-installation');
    }
});
