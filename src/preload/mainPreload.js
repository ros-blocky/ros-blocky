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
  onProjectLoaded: (callback) => ipcRenderer.on('project-loaded', (event, projectPath) => callback(projectPath)),

  // Package management
  listPackages: () => ipcRenderer.invoke('list-packages'),
  createPackage: (packageName) => ipcRenderer.invoke('create-package', packageName),
  promptPackageName: () => ipcRenderer.invoke('prompt-package-name'),
  deletePackage: (packageName) => ipcRenderer.invoke('delete-package', packageName),

  // Node management
  promptNodeName: (packageName) => ipcRenderer.invoke('prompt-node-name', packageName),
  createNode: (packageName, nodeName) => ipcRenderer.invoke('create-node', packageName, nodeName),
  listPackageNodes: (packageName) => ipcRenderer.invoke('list-package-nodes', packageName),

  // URDF management
  promptUrdfName: (packageName) => ipcRenderer.invoke('prompt-urdf-name', packageName),
  createUrdf: (packageName, urdfName) => ipcRenderer.invoke('create-urdf', packageName, urdfName),
  listPackageUrdfs: (packageName) => ipcRenderer.invoke('list-package-urdfs', packageName),

  // Config management
  promptConfigName: (packageName) => ipcRenderer.invoke('prompt-config-name', packageName),
  createConfig: (packageName, configName) => ipcRenderer.invoke('create-config', packageName, configName),
  listPackageConfigs: (packageName) => ipcRenderer.invoke('list-package-configs', packageName),

  // Launch file management
  promptLaunchName: (packageName) => ipcRenderer.invoke('prompt-launch-name', packageName),
  createLaunch: (packageName, launchName) => ipcRenderer.invoke('create-launch', packageName, launchName),
  listPackageLaunches: (packageName) => ipcRenderer.invoke('list-package-launches', packageName),

  // Generic file operations (used by UI for urdf, config, launch)
  promptFileName: (packageName, title, prompt) => ipcRenderer.invoke('prompt-file-name', packageName, title, prompt),
  createSectionFile: (packageName, folderName, fileName) => ipcRenderer.invoke('create-section-file', packageName, folderName, fileName)
});
