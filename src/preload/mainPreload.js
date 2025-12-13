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
  saveUrdfFile: (packageName, fileName, content) => ipcRenderer.invoke('save-urdf-file', packageName, fileName, content),
  saveBlockState: (packageName, fileName, blockXml) => ipcRenderer.invoke('save-block-state', packageName, fileName, blockXml),
  loadBlockState: (packageName, fileName) => ipcRenderer.invoke('load-block-state', packageName, fileName),

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
  createSectionFile: (packageName, folderName, fileName) => ipcRenderer.invoke('create-section-file', packageName, folderName, fileName),

  // Delete operations
  deleteSectionFiles: (packageName, sectionType) => ipcRenderer.invoke('delete-section-files', packageName, sectionType),
  deleteSectionFile: (packageName, sectionType, fileName) => ipcRenderer.invoke('delete-section-file', packageName, sectionType, fileName),

  // Dialogs
  showConfirmDialog: (message) => ipcRenderer.invoke('show-confirm-dialog', message),

  // Mesh operations
  importMeshFiles: (packageName) => ipcRenderer.invoke('import-mesh-files', packageName),
  listPackageMeshes: (packageName) => ipcRenderer.invoke('list-package-meshes', packageName),

  // Run operations (execute ROS commands)
  runNode: (packageName, nodeName) => ipcRenderer.invoke('run-node', packageName, nodeName),
  runUrdf: (packageName, fileName) => ipcRenderer.invoke('run-urdf', packageName, fileName),
  runLaunch: (packageName, fileName) => ipcRenderer.invoke('run-launch', packageName, fileName),
  runRviz: () => ipcRenderer.invoke('run-rviz'),

  // Process control
  stopRosProcess: (processKey) => ipcRenderer.invoke('stop-ros-process', processKey),
  isRosRunning: (processKey) => ipcRenderer.invoke('is-ros-running', processKey),

  // ROS output listener
  onRosOutput: (callback) => ipcRenderer.on('ros-output', (event, data) => callback(data)),
  removeRosOutputListener: () => ipcRenderer.removeAllListeners('ros-output')
});
