const { contextBridge, ipcRenderer } = require('electron');

/**
 * The preload script runs before `index.html` is loaded
 * in the renderer. It has access to web APIs as well as
 * Electron's renderer process modules and some polyfilled
 * Node.js functions.
 *
 * https://www.electronjs.org/docs/latest/tutorial/sandbox
 */

// Expose protected methods that allow the renderer process to use
// the ipcRenderer without exposing the entire object
contextBridge.exposeInMainWorld('electronAPI', {
  minimizeWindow: () => ipcRenderer.send('window-minimize'),
  maximizeWindow: () => ipcRenderer.send('window-maximize'),
  closeWindow: () => ipcRenderer.send('window-close'),
  createProject: () => ipcRenderer.invoke('create-project'),
  openProject: () => ipcRenderer.invoke('open-project'),
  onProjectDialogsComplete: (callback) => ipcRenderer.on('project-dialogs-complete', (event, type) => callback(type)),
  onProjectLoaded: (callback) => ipcRenderer.on('project-loaded', (event, projectPath) => callback(projectPath))
});
