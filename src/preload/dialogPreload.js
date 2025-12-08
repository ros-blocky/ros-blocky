const { contextBridge, ipcRenderer } = require('electron');

/**
 * Preload script for dialog windows
 * Exposes only necessary APIs via contextBridge for security
 */
contextBridge.exposeInMainWorld('dialogAPI', {
    // Project name prompt
    sendProjectNameResult: (projectName) => ipcRenderer.send('project-name-result', projectName),

    // Package name prompt
    sendPackageNameResult: (packageName) => ipcRenderer.send('package-name-result', packageName),

    // Node name prompt
    onSetPackageName: (callback) => ipcRenderer.on('set-package-name', (event, name) => callback(name)),
    sendNodeNameResult: (nodeName) => ipcRenderer.send('node-name-result', nodeName),

    // Delete package confirm
    sendDeletePackageConfirmed: (packageName) => ipcRenderer.send('delete-package-confirmed', packageName),

    // Package creation loading
    onPackageCreationStatus: (callback) => ipcRenderer.on('package-creation-status', (event, status) => callback(status)),

    // Generic item prompt (node, urdf, config, launch)
    onSetItemConfig: (callback) => ipcRenderer.on('set-item-config', (event, config) => callback(config)),
    sendItemNameResult: (result) => ipcRenderer.send('item-name-result', result),

    // Generic confirm dialog
    onSetConfirmConfig: (callback) => ipcRenderer.on('set-confirm-config', (event, config) => callback(config)),
    sendConfirmResult: (confirmed) => ipcRenderer.send('confirm-result', confirmed)
});
