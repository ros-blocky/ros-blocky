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

    /**
     * Handle prompt-node-name IPC request
     */
    ipcMain.handle('prompt-node-name', async (event, packageName) => {
        return await packageService.promptNodeName(packageName);
    });

    /**
     * Handle create-node IPC request
     */
    ipcMain.handle('create-node', async (event, packageName, nodeName) => {
        return await packageService.createNode(packageName, nodeName);
    });

    /**
     * Handle list-package-nodes IPC request
     */
    ipcMain.handle('list-package-nodes', async (event, packageName) => {
        return await packageService.listPackageNodes(packageName);
    });

    // ========== URDF Operations ==========

    /**
     * Handle prompt-urdf-name IPC request
     */
    ipcMain.handle('prompt-urdf-name', async (event, packageName) => {
        return await packageService.promptUrdfName(packageName);
    });

    /**
     * Handle create-urdf IPC request
     */
    ipcMain.handle('create-urdf', async (event, packageName, urdfName) => {
        return await packageService.createUrdf(packageName, urdfName);
    });

    /**
     * Handle list-package-urdfs IPC request
     */
    ipcMain.handle('list-package-urdfs', async (event, packageName) => {
        return await packageService.listPackageUrdfs(packageName);
    });

    /**
     * Handle save-urdf-file IPC request
     */
    ipcMain.handle('save-urdf-file', async (event, packageName, fileName, content) => {
        return await packageService.saveUrdfFile(packageName, fileName, content);
    });

    /**
     * Handle save-block-state IPC request (saves .blocks sidecar file)
     */
    ipcMain.handle('save-block-state', async (event, packageName, fileName, blockXml) => {
        return await packageService.saveBlockState(packageName, fileName, blockXml);
    });

    /**
     * Handle load-block-state IPC request (loads .blocks sidecar file)
     */
    ipcMain.handle('load-block-state', async (event, packageName, fileName) => {
        return await packageService.loadBlockState(packageName, fileName);
    });

    // ========== Node File Operations ==========

    /**
     * Handle save-node-file IPC request
     */
    ipcMain.handle('save-node-file', async (event, packageName, fileName, content) => {
        return await packageService.saveNodeFile(packageName, fileName, content);
    });

    /**
     * Handle save-node-block-state IPC request (saves .nodeblocks sidecar file)
     */
    ipcMain.handle('save-node-block-state', async (event, packageName, fileName, blockXml) => {
        return await packageService.saveNodeBlockState(packageName, fileName, blockXml);
    });

    /**
     * Handle load-node-block-state IPC request (loads .nodeblocks sidecar file)
     */
    ipcMain.handle('load-node-block-state', async (event, packageName, fileName) => {
        return await packageService.loadNodeBlockState(packageName, fileName);
    });

    // ========== Config Operations ==========

    /**
     * Handle prompt-config-name IPC request
     */
    ipcMain.handle('prompt-config-name', async (event, packageName) => {
        return await packageService.promptConfigName(packageName);
    });

    /**
     * Handle create-config IPC request
     */
    ipcMain.handle('create-config', async (event, packageName, configName) => {
        return await packageService.createConfig(packageName, configName);
    });

    /**
     * Handle list-package-configs IPC request
     */
    ipcMain.handle('list-package-configs', async (event, packageName) => {
        return await packageService.listPackageConfigs(packageName);
    });

    // ========== Launch Operations ==========

    /**
     * Handle prompt-launch-name IPC request
     */
    ipcMain.handle('prompt-launch-name', async (event, packageName) => {
        return await packageService.promptLaunchName(packageName);
    });

    /**
     * Handle create-launch IPC request
     */
    ipcMain.handle('create-launch', async (event, packageName, launchName) => {
        return await packageService.createLaunch(packageName, launchName);
    });

    /**
     * Handle list-package-launches IPC request
     */
    ipcMain.handle('list-package-launches', async (event, packageName) => {
        return await packageService.listPackageLaunches(packageName);
    });

    // ========== Generic File Operations ==========

    /**
     * Handle prompt-file-name IPC request (generic prompt for urdf, config, launch)
     */
    ipcMain.handle('prompt-file-name', async (event, packageName, title, prompt) => {
        return await packageService.promptGenericFileName(packageName, title, prompt);
    });

    /**
     * Handle create-section-file IPC request (create file in urdf, config, launch folders)
     */
    ipcMain.handle('create-section-file', async (event, packageName, folderName, fileName) => {
        return await packageService.createSectionFile(packageName, folderName, fileName);
    });

    // ========== Delete Operations ==========

    /**
     * Handle delete-section-files IPC request (delete all files in a section)
     */
    ipcMain.handle('delete-section-files', async (event, packageName, sectionType) => {
        return await packageService.deleteSectionFiles(packageName, sectionType);
    });

    /**
     * Handle delete-section-file IPC request (delete a single file from a section)
     */
    ipcMain.handle('delete-section-file', async (event, packageName, sectionType, fileName) => {
        return await packageService.deleteSectionFile(packageName, sectionType, fileName);
    });

    /**
     * Handle show-confirm-dialog IPC request (show generic Yes/No dialog)
     */
    ipcMain.handle('show-confirm-dialog', async (event, message) => {
        return await packageService.showConfirmDialog(message);
    });

    // ========== Mesh Operations ==========

    /**
     * Handle import-mesh-files IPC request
     */
    ipcMain.handle('import-mesh-files', async (event, packageName) => {
        return await packageService.importMeshFiles(packageName);
    });

    /**
     * Handle list-package-meshes IPC request
     */
    ipcMain.handle('list-package-meshes', async (event, packageName) => {
        return await packageService.listPackageMeshes(packageName);
    });

    // ========== Run Operations ==========

    // Initialize ROS service with packageService
    const rosService = require('../services/rosService');
    rosService.init(packageService);

    /**
     * Handle run-node IPC request
     * Runs a ROS 2 Python node
     */
    ipcMain.handle('run-node', async (event, packageName, nodeName) => {
        return await rosService.runNode(packageName, nodeName);
    });

    /**
     * Handle run-urdf IPC request
     * Runs robot_state_publisher with the URDF file
     */
    ipcMain.handle('run-urdf', async (event, packageName, fileName) => {
        return await rosService.runRobotPublisher(packageName, fileName);
    });

    /**
     * Handle run-launch IPC request
     * Runs ros2 launch with the launch file
     */
    ipcMain.handle('run-launch', async (event, packageName, fileName) => {
        return await rosService.runLaunch(packageName, fileName);
    });

    /**
     * Handle stop-ros-process IPC request
     * Stops a running ROS process
     */
    ipcMain.handle('stop-ros-process', async (event, processKey) => {
        return rosService.stopProcess(processKey);
    });

    /**
     * Handle is-ros-running IPC request
     * Checks if a ROS process is running
     */
    ipcMain.handle('is-ros-running', async (event, processKey) => {
        return rosService.isProcessRunning(processKey);
    });

    /**
     * Handle run-rviz IPC request
     * Runs RViz2 visualization tool
     */
    ipcMain.handle('run-rviz', async (event) => {
        return await rosService.runRviz();
    });

    /**
     * Handle run-joint-state-publisher-gui IPC request
     * Runs Joint State Publisher GUI tool
     */
    ipcMain.handle('run-joint-state-publisher-gui', async (event) => {
        return await rosService.runJointStatePublisherGui();
    });

    /**
     * Handle run-turtlesim IPC request
     * Runs TurtleSim node
     */
    ipcMain.handle('run-turtlesim', async (event) => {
        return await rosService.runTurtlesim();
    });

    // ========== Build Operations ==========

    /**
     * Handle build-all-packages IPC request
     * Builds all packages using colcon build
     */
    ipcMain.handle('build-all-packages', async (event) => {
        const { BrowserWindow } = require('electron');

        // Show loading dialog
        const loadingWindow = await packageService.showLoadingDialog({
            type: 'build',
            target: 'all'
        });

        try {
            const result = await rosService.buildAllPackages((status) => {
                // Send status updates to loading dialog
                if (loadingWindow && !loadingWindow.isDestroyed()) {
                    loadingWindow.webContents.send('loading-status', status);
                }
            });

            // Close loading dialog
            if (loadingWindow && !loadingWindow.isDestroyed()) {
                loadingWindow.close();
            }

            // Send build result to main window for toast notification
            const mainWindow = BrowserWindow.getAllWindows().find(w => !w.isDestroyed() && w.webContents.getURL().includes('index.html'));
            if (mainWindow) {
                mainWindow.webContents.send('build-result', {
                    success: result.success,
                    target: 'all',
                    error: result.error
                });
            }

            return result;
        } catch (error) {
            if (loadingWindow && !loadingWindow.isDestroyed()) {
                loadingWindow.close();
            }
            return { success: false, error: error.message };
        }
    });

    /**
     * Handle build-package IPC request
     * Builds a single package using colcon build --packages-select
     */
    ipcMain.handle('build-package', async (event, packageName) => {
        const { BrowserWindow } = require('electron');

        // Show loading dialog
        const loadingWindow = await packageService.showLoadingDialog({
            type: 'build',
            target: packageName
        });

        try {
            const result = await rosService.buildPackage(packageName, (status) => {
                // Send status updates to loading dialog
                if (loadingWindow && !loadingWindow.isDestroyed()) {
                    loadingWindow.webContents.send('loading-status', status);
                }
            });

            // Close loading dialog
            if (loadingWindow && !loadingWindow.isDestroyed()) {
                loadingWindow.close();
            }

            // Send build result to main window for toast notification
            const mainWindow = BrowserWindow.getAllWindows().find(w => !w.isDestroyed() && w.webContents.getURL().includes('index.html'));
            if (mainWindow) {
                mainWindow.webContents.send('build-result', {
                    success: result.success,
                    target: packageName,
                    error: result.error
                });
            }

            return result;
        } catch (error) {
            if (loadingWindow && !loadingWindow.isDestroyed()) {
                loadingWindow.close();
            }
            return { success: false, error: error.message };
        }
    });
}

module.exports = { registerProjectIPC };

