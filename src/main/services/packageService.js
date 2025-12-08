const { dialog, BrowserWindow } = require('electron');
const fs = require('fs').promises;
const path = require('path');
const { spawn } = require('child_process');

/**
 * Package Service - Handles ROS2 package operations
 */
class PackageService {
    constructor() {
        this.currentProjectPath = null;
    }

    /**
     * Set the current project path
     * @param {string} projectPath 
     */
    setProjectPath(projectPath) {
        this.currentProjectPath = projectPath;
    }

    /**
     * Prompt user for package name
     * @returns {Promise<string|null>}
     */
    /**
     * Prompt user for package name
     * @returns {Promise<string|null>}
     */
    async promptPackageName() {
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

            // Load HTML file
            promptWindow.loadFile(path.join(__dirname, '../../renderer/dialogs/packageNamePrompt.html'));

            // Show when ready to prevent white flash
            promptWindow.once('ready-to-show', () => {
                promptWindow.show();
            });

            const { ipcMain } = require('electron');
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
     * Create a new ROS2 package
     * @param {string} packageName - Name of the package to create
     * @returns {Promise<{success: boolean, message: string, packagePath?: string}>}
     */
    async createPackage(packageName) {
        let loadingWindow = null;

        try {
            if (!this.currentProjectPath) {
                return {
                    success: false,
                    message: 'No project is currently loaded. Please open or create a project first.'
                };
            }

            // Validate package name
            if (!packageName || packageName.trim() === '') {
                return { success: false, message: 'Package name cannot be empty' };
            }

            // ROS2 package naming rules: lowercase letters, numbers, underscores
            const validNamePattern = /^[a-z][a-z0-9_]*$/;
            if (!validNamePattern.test(packageName)) {
                return {
                    success: false,
                    message: 'Invalid package name. Must start with a letter and contain only lowercase letters, numbers, and underscores.'
                };
            }

            const srcPath = path.join(this.currentProjectPath, 'src');
            const packagePath = path.join(srcPath, packageName);

            // Check if package already exists
            try {
                await fs.access(packagePath);
                return {
                    success: false,
                    message: `Package "${packageName}" already exists in this project.`
                };
            } catch {
                // Package doesn't exist, continue
            }

            // Show loading dialog
            loadingWindow = new BrowserWindow({
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
            loadingWindow.loadFile(path.join(__dirname, '../../renderer/dialogs/packageCreationLoading.html'));

            // Show when ready to prevent white flash
            loadingWindow.once('ready-to-show', () => {
                loadingWindow.show();
            });

            // Update status
            loadingWindow.webContents.on('did-finish-load', () => {
                loadingWindow.webContents.send('package-creation-status', `Creating package "${packageName}"...`);
            });

            // Ensure src directory exists
            await fs.mkdir(srcPath, { recursive: true });

            // Create package: source ROS2 setup then use pixi run
            const result = await this.runPixiCommand(
                `ros2 pkg create ${packageName} --build-type ament_python`,
                srcPath
            );

            // Close loading dialog
            if (loadingWindow && !loadingWindow.isDestroyed()) {
                loadingWindow.close();
            }

            if (result.success || result.output?.includes('going to create')) {
                return {
                    success: true,
                    message: `Package "${packageName}" created successfully!`,
                    packagePath
                };
            } else {
                return {
                    success: false,
                    message: `Failed to create package: ${result.error}`
                };
            }

        } catch (error) {
            // Close loading dialog on error
            if (loadingWindow && !loadingWindow.isDestroyed()) {
                loadingWindow.close();
            }
            console.error('Error creating package:', error);
            return {
                success: false,
                message: `Error creating package: ${error.message}`
            };
        }
    }

    /**
     * Show generic confirm dialog with Yes/No buttons
     * @param {string} message - Message to display
     * @returns {Promise<boolean>} - True if user clicked Yes, false otherwise
     */
    showConfirmDialog(message) {
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

            // Show when ready to prevent white flash
            confirmWindow.once('ready-to-show', () => {
                confirmWindow.show();
            });

            // Send config after load
            confirmWindow.webContents.on('did-finish-load', () => {
                confirmWindow.webContents.send('set-confirm-config', { message });
            });

            const { ipcMain } = require('electron');
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
     * Show delete confirmation dialog
     * @param {string} packageName - Name of the package to delete
     * @returns {Promise<boolean>} - True if user confirmed, false otherwise
     */
    promptDeletePackage(packageName) {
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

            // Show when ready to prevent white flash
            confirmWindow.once('ready-to-show', () => {
                confirmWindow.show();
            });

            // Send package name after load
            confirmWindow.webContents.on('did-finish-load', () => {
                confirmWindow.webContents.send('set-package-name', packageName);
            });

            const { ipcMain } = require('electron');
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
     * Delete a ROS2 package
     * @param {string} packageName - Name of the package to delete
     * @returns {Promise<{success: boolean, message: string}>}
     */
    async deletePackage(packageName) {
        try {
            if (!this.currentProjectPath) {
                return {
                    success: false,
                    message: 'No project is currently loaded.'
                };
            }

            // Show confirmation dialog
            const confirmed = await this.promptDeletePackage(packageName);
            if (!confirmed) {
                return {
                    success: false,
                    message: 'Deletion cancelled'
                };
            }

            const packagePath = path.join(this.currentProjectPath, 'src', packageName);

            // Check if package exists
            try {
                await fs.access(packagePath);
            } catch {
                return {
                    success: false,
                    message: `Package "${packageName}" not found.`
                };
            }

            // Delete the package folder recursively
            await fs.rm(packagePath, { recursive: true, force: true });

            return {
                success: true,
                message: `Package "${packageName}" deleted successfully.`
            };

        } catch (error) {
            console.error('Error deleting package:', error);
            return {
                success: false,
                message: `Error deleting package: ${error.message}`
            };
        }
    }

    /**
     * Show generic item name prompt dialog
     * @param {string} itemType - Type of item: 'node', 'urdf', 'config', 'launch'
     * @param {string} packageName - Name of the package
     * @returns {Promise<{name: string, type: string}|null>} - Item info or null if cancelled
     */
    promptItemName(itemType, packageName) {
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

            // Show when ready to prevent white flash
            promptWindow.once('ready-to-show', () => {
                promptWindow.show();
            });

            // Send item config after load
            promptWindow.webContents.on('did-finish-load', () => {
                promptWindow.webContents.send('set-item-config', {
                    type: itemType,
                    packageName: packageName
                });
            });

            const { ipcMain } = require('electron');
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

    /**
     * Show node name prompt dialog (convenience method)
     * @param {string} packageName - Name of the package
     * @returns {Promise<string|null>} - Node name or null if cancelled
     */
    async promptNodeName(packageName) {
        const result = await this.promptItemName('node', packageName);
        return result ? result.name : null;
    }

    /**
     * Create a new node in a package
     * @param {string} packageName - Name of the package
     * @param {string} nodeName - Name of the node
     * @returns {Promise<{success: boolean, message: string, nodePath?: string}>}
     */
    async createNode(packageName, nodeName) {
        try {
            if (!this.currentProjectPath) {
                return {
                    success: false,
                    message: 'No project is currently loaded.'
                };
            }

            // Validate node name
            if (!nodeName || nodeName.trim() === '') {
                return { success: false, message: 'Node name cannot be empty' };
            }

            const validNamePattern = /^[a-z][a-z0-9_]*$/;
            if (!validNamePattern.test(nodeName)) {
                return {
                    success: false,
                    message: 'Invalid node name. Must start with a letter and contain only lowercase letters, numbers, and underscores.'
                };
            }

            const packagePath = path.join(this.currentProjectPath, 'src', packageName);
            const nodeDir = path.join(packagePath, packageName);
            const nodePath = path.join(nodeDir, `${nodeName}.py`);

            // Check if package exists
            try {
                await fs.access(packagePath);
            } catch {
                return {
                    success: false,
                    message: `Package "${packageName}" not found.`
                };
            }

            // Check if node already exists
            try {
                await fs.access(nodePath);
                return {
                    success: false,
                    message: `Node "${nodeName}" already exists in package "${packageName}".`
                };
            } catch {
                // Node doesn't exist, continue
            }

            // Create node Python file
            const nodeContent = `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class ${this.toPascalCase(nodeName)}(Node):
    """${nodeName} node."""

    def __init__(self):
        super().__init__('${nodeName}')
        self.get_logger().info('${nodeName} node started')


def main(args=None):
    rclpy.init(args=args)
    node = ${this.toPascalCase(nodeName)}()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
`;

            await fs.writeFile(nodePath, nodeContent);

            // Update setup.py to add entry point
            await this.addNodeEntryPoint(packageName, nodeName);

            return {
                success: true,
                message: `Node "${nodeName}" created successfully!`,
                nodePath
            };

        } catch (error) {
            console.error('Error creating node:', error);
            return {
                success: false,
                message: `Error creating node: ${error.message}`
            };
        }
    }

    /**
     * Convert snake_case to PascalCase
     */
    toPascalCase(str) {
        return str.split('_').map(word =>
            word.charAt(0).toUpperCase() + word.slice(1)
        ).join('');
    }

    /**
     * Add node entry point to setup.py
     */
    async addNodeEntryPoint(packageName, nodeName) {
        const setupPyPath = path.join(this.currentProjectPath, 'src', packageName, 'setup.py');

        try {
            let content = await fs.readFile(setupPyPath, 'utf8');

            // Find the console_scripts section and add the entry point
            const entryPoint = `'${nodeName} = ${packageName}.${nodeName}:main'`;

            // Check if entry point already exists
            if (content.includes(entryPoint)) {
                return;
            }

            // Find console_scripts array and add entry
            const consoleScriptsMatch = content.match(/'console_scripts':\s*\[\s*\]/);
            if (consoleScriptsMatch) {
                // Empty array - add first entry
                content = content.replace(
                    /'console_scripts':\s*\[\s*\]/,
                    `'console_scripts': [\n            ${entryPoint},\n        ]`
                );
            } else {
                // Array has entries - add to existing
                const match = content.match(/'console_scripts':\s*\[([^\]]*)\]/);
                if (match) {
                    const existingEntries = match[1].trim();
                    if (existingEntries) {
                        content = content.replace(
                            /'console_scripts':\s*\[([^\]]*)\]/,
                            `'console_scripts': [${existingEntries}\n            ${entryPoint},\n        ]`
                        );
                    }
                }
            }

            await fs.writeFile(setupPyPath, content);
        } catch (error) {
            console.error('Error updating setup.py:', error);
        }
    }

    /**
     * List nodes in a package
     * @param {string} packageName - Name of the package
     * @returns {Promise<string[]>}
     */
    async listPackageNodes(packageName) {
        try {
            const nodeDir = path.join(this.currentProjectPath, 'src', packageName, packageName);
            const files = await fs.readdir(nodeDir);

            // Filter .py files, exclude __init__.py
            return files
                .filter(f => f.endsWith('.py') && f !== '__init__.py')
                .map(f => f.replace('.py', ''));
        } catch {
            return [];
        }
    }

    /**
     * Prompt for URDF name (convenience method)
     * @param {string} packageName - Name of the package
     * @returns {Promise<string|null>}
     */
    async promptUrdfName(packageName) {
        const result = await this.promptItemName('urdf', packageName);
        return result ? result.name : null;
    }

    /**
     * Create a new URDF file in a package
     * @param {string} packageName - Name of the package
     * @param {string} urdfName - Name of the URDF file
     * @returns {Promise<{success: boolean, message: string, urdfPath?: string}>}
     */
    async createUrdf(packageName, urdfName) {
        try {
            if (!this.currentProjectPath) {
                return {
                    success: false,
                    message: 'No project is currently loaded.'
                };
            }

            // Validate name
            if (!urdfName || urdfName.trim() === '') {
                return { success: false, message: 'URDF name cannot be empty' };
            }

            const validNamePattern = /^[a-z][a-z0-9_]*$/;
            if (!validNamePattern.test(urdfName)) {
                return {
                    success: false,
                    message: 'Invalid URDF name. Must start with a letter and contain only lowercase letters, numbers, and underscores.'
                };
            }

            const packagePath = path.join(this.currentProjectPath, 'src', packageName);
            const urdfDir = path.join(packagePath, 'urdf');
            const urdfPath = path.join(urdfDir, `${urdfName}.xacro`);

            // Check if package exists
            try {
                await fs.access(packagePath);
            } catch {
                return {
                    success: false,
                    message: `Package "${packageName}" not found.`
                };
            }

            // Create urdf directory if it doesn't exist
            await fs.mkdir(urdfDir, { recursive: true });

            // Check if URDF already exists
            try {
                await fs.access(urdfPath);
                return {
                    success: false,
                    message: `URDF "${urdfName}" already exists in package "${packageName}".`
                };
            } catch {
                // URDF doesn't exist, continue
            }

            // Create URDF xacro file
            const urdfContent = `<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="${urdfName}">
    <!-- ${urdfName} URDF description -->
    
    <!-- Materials -->
    <material name="white">
        <color rgba="1 1 1 1"/>
    </material>
    <material name="blue">
        <color rgba="0 0 0.8 1"/>
    </material>
    
    <!-- Base Link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.1 0.1 0.1"/>
            </geometry>
            <material name="blue"/>
        </visual>
        <collision>
            <geometry>
                <box size="0.1 0.1 0.1"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
        </inertial>
    </link>
    
</robot>
`;

            await fs.writeFile(urdfPath, urdfContent);

            return {
                success: true,
                message: `URDF "${urdfName}" created successfully!`,
                urdfPath
            };

        } catch (error) {
            console.error('Error creating URDF:', error);
            return {
                success: false,
                message: `Error creating URDF: ${error.message}`
            };
        }
    }

    /**
     * List URDFs in a package
     * @param {string} packageName - Name of the package
     * @returns {Promise<string[]>}
     */
    async listPackageUrdfs(packageName) {
        try {
            const urdfDir = path.join(this.currentProjectPath, 'src', packageName, 'urdf');
            const files = await fs.readdir(urdfDir);
            return files.filter(f => f.endsWith('.xacro') || f.endsWith('.urdf'));
        } catch {
            return [];
        }
    }

    /**
     * Prompt for config name (convenience method)
     * @param {string} packageName - Name of the package
     * @returns {Promise<string|null>}
     */
    async promptConfigName(packageName) {
        const result = await this.promptItemName('config', packageName);
        return result ? result.name : null;
    }

    /**
     * Create a new config file in a package
     * @param {string} packageName - Name of the package
     * @param {string} configName - Name of the config file
     * @returns {Promise<{success: boolean, message: string, configPath?: string}>}
     */
    async createConfig(packageName, configName) {
        try {
            if (!this.currentProjectPath) {
                return {
                    success: false,
                    message: 'No project is currently loaded.'
                };
            }

            // Validate name
            if (!configName || configName.trim() === '') {
                return { success: false, message: 'Config name cannot be empty' };
            }

            const validNamePattern = /^[a-z][a-z0-9_]*$/;
            if (!validNamePattern.test(configName)) {
                return {
                    success: false,
                    message: 'Invalid config name. Must start with a letter and contain only lowercase letters, numbers, and underscores.'
                };
            }

            const packagePath = path.join(this.currentProjectPath, 'src', packageName);
            const configDir = path.join(packagePath, 'config');
            const configPath = path.join(configDir, `${configName}.yaml`);

            // Check if package exists
            try {
                await fs.access(packagePath);
            } catch {
                return {
                    success: false,
                    message: `Package "${packageName}" not found.`
                };
            }

            // Create config directory if it doesn't exist
            await fs.mkdir(configDir, { recursive: true });

            // Check if config already exists
            try {
                await fs.access(configPath);
                return {
                    success: false,
                    message: `Config "${configName}" already exists in package "${packageName}".`
                };
            } catch {
                // Config doesn't exist, continue
            }

            // Create config yaml file
            const configContent = `# ${configName} configuration file
# Package: ${packageName}

/**:
  ros__parameters:
    # Add your parameters here
    example_param: "value"
    example_number: 1.0
    example_list:
      - item1
      - item2
`;

            await fs.writeFile(configPath, configContent);

            return {
                success: true,
                message: `Config "${configName}" created successfully!`,
                configPath
            };

        } catch (error) {
            console.error('Error creating config:', error);
            return {
                success: false,
                message: `Error creating config: ${error.message}`
            };
        }
    }

    /**
     * List configs in a package
     * @param {string} packageName - Name of the package
     * @returns {Promise<string[]>}
     */
    async listPackageConfigs(packageName) {
        try {
            const configDir = path.join(this.currentProjectPath, 'src', packageName, 'config');
            const files = await fs.readdir(configDir);
            return files.filter(f => f.endsWith('.yaml') || f.endsWith('.yml'));
        } catch {
            return [];
        }
    }

    /**
     * Prompt for launch file name (convenience method)
     * @param {string} packageName - Name of the package
     * @returns {Promise<string|null>}
     */
    async promptLaunchName(packageName) {
        const result = await this.promptItemName('launch', packageName);
        return result ? result.name : null;
    }

    /**
     * Create a new launch file in a package
     * @param {string} packageName - Name of the package
     * @param {string} launchName - Name of the launch file
     * @returns {Promise<{success: boolean, message: string, launchPath?: string}>}
     */
    async createLaunch(packageName, launchName) {
        try {
            if (!this.currentProjectPath) {
                return {
                    success: false,
                    message: 'No project is currently loaded.'
                };
            }

            // Validate name
            if (!launchName || launchName.trim() === '') {
                return { success: false, message: 'Launch file name cannot be empty' };
            }

            const validNamePattern = /^[a-z][a-z0-9_]*$/;
            if (!validNamePattern.test(launchName)) {
                return {
                    success: false,
                    message: 'Invalid launch name. Must start with a letter and contain only lowercase letters, numbers, and underscores.'
                };
            }

            const packagePath = path.join(this.currentProjectPath, 'src', packageName);
            const launchDir = path.join(packagePath, 'launch');
            const launchPath = path.join(launchDir, `${launchName}.py`);

            // Check if package exists
            try {
                await fs.access(packagePath);
            } catch {
                return {
                    success: false,
                    message: `Package "${packageName}" not found.`
                };
            }

            // Create launch directory if it doesn't exist
            await fs.mkdir(launchDir, { recursive: true });

            // Check if launch file already exists
            try {
                await fs.access(launchPath);
                return {
                    success: false,
                    message: `Launch file "${launchName}" already exists in package "${packageName}".`
                };
            } catch {
                // Launch doesn't exist, continue
            }

            // Create launch Python file
            const launchContent = `#!/usr/bin/env python3
"""${launchName} launch file for ${packageName} package."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    
    # Declare launch arguments
    # use_sim_time = LaunchConfiguration('use_sim_time')
    
    # declare_use_sim_time = DeclareLaunchArgument(
    #     'use_sim_time',
    #     default_value='false',
    #     description='Use simulation time'
    # )
    
    # Define nodes
    # example_node = Node(
    #     package='${packageName}',
    #     executable='your_node_name',
    #     name='your_node_name',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )
    
    return LaunchDescription([
        # declare_use_sim_time,
        # example_node,
    ])
`;

            await fs.writeFile(launchPath, launchContent);

            return {
                success: true,
                message: `Launch file "${launchName}" created successfully!`,
                launchPath
            };

        } catch (error) {
            console.error('Error creating launch file:', error);
            return {
                success: false,
                message: `Error creating launch file: ${error.message}`
            };
        }
    }

    /**
     * List launch files in a package
     * @param {string} packageName - Name of the package
     * @returns {Promise<string[]>}
     */
    async listPackageLaunches(packageName) {
        try {
            const launchDir = path.join(this.currentProjectPath, 'src', packageName, 'launch');
            const files = await fs.readdir(launchDir);
            return files.filter(f => f.endsWith('.py'));
        } catch {
            return [];
        }
    }

    /**
     * Generic file name prompt (used by UI for urdf, config, launch)
     * Uses the generic item prompt dialog
     * @param {string} packageName - Name of the package
     * @param {string} title - Dialog title
     * @param {string} prompt - Prompt text
     * @returns {Promise<string|null>}
     */
    async promptGenericFileName(packageName, title, prompt) {
        // Determine the item type based on the title
        let itemType = 'node'; // default
        if (title.toLowerCase().includes('urdf')) {
            itemType = 'urdf';
        } else if (title.toLowerCase().includes('config')) {
            itemType = 'config';
        } else if (title.toLowerCase().includes('launch')) {
            itemType = 'launch';
        }

        const result = await this.promptItemName(itemType, packageName);
        return result ? result.name : null;
    }

    /**
     * Create a file in a section folder (urdf, config, launch)
     * @param {string} packageName - Name of the package
     * @param {string} folderName - Folder name (urdf, config, launch)
     * @param {string} fileName - Full file name with extension
     * @returns {Promise<{success: boolean, message: string, filePath?: string}>}
     */
    async createSectionFile(packageName, folderName, fileName) {
        try {
            if (!this.currentProjectPath) {
                return {
                    success: false,
                    message: 'No project is currently loaded.'
                };
            }

            // Extract name without extension
            const baseName = fileName.replace(/\.[^/.]+$/, '');

            // Route to specific create method based on folder
            switch (folderName) {
                case 'urdf':
                    return await this.createUrdf(packageName, baseName);
                case 'config':
                    return await this.createConfig(packageName, baseName);
                case 'launch':
                    return await this.createLaunch(packageName, baseName);
                default:
                    return {
                        success: false,
                        message: `Unknown folder type: ${folderName}`
                    };
            }

        } catch (error) {
            console.error('Error creating section file:', error);
            return {
                success: false,
                message: `Error creating file: ${error.message}`
            };
        }
    }

    /**
     * Run a command in the pixi environment
     * @param {string} command - Command to run
     * @param {string} cwd - Working directory
     * @returns {Promise<{success: boolean, output?: string, error?: string}>}
     */
    runPixiCommand(command, cwd) {
        return new Promise((resolve) => {
            // Build full command: source ROS2 setup, then run pixi
            const fullCommand = `call C:\\pixi_ws\\ros2-windows\\local_setup.bat && pixi run ${command}`;

            const pixiProcess = spawn('cmd.exe', ['/c', fullCommand], {
                cwd,
                shell: false
            });

            let stdout = '';
            let stderr = '';

            pixiProcess.stdout.on('data', (data) => {
                stdout += data.toString();
                console.log('Output:', data.toString());
            });

            pixiProcess.stderr.on('data', (data) => {
                stderr += data.toString();
                console.log('Stderr:', data.toString());
            });

            pixiProcess.on('close', (code) => {
                console.log('pixi run closed with code:', code);
                if (code === 0 || stdout.includes('going to create')) {
                    resolve({ success: true, output: stdout });
                } else {
                    resolve({ success: false, error: stderr || stdout });
                }
            });

            pixiProcess.on('error', (error) => {
                resolve({ success: false, error: error.message });
            });
        });
    }

    /**
     * List all packages in the current project
     * @returns {Promise<string[]>}
     */
    async listPackages() {
        try {
            if (!this.currentProjectPath) {
                return [];
            }

            const srcPath = path.join(this.currentProjectPath, 'src');

            // Check if src directory exists
            try {
                await fs.access(srcPath);
            } catch {
                return [];
            }

            // Read all directories in src
            const entries = await fs.readdir(srcPath, { withFileTypes: true });
            const packages = entries
                .filter(entry => entry.isDirectory())
                .map(entry => entry.name);

            return packages;

        } catch (error) {
            console.error('Error listing packages:', error);
            return [];
        }
    }

    /**
     * Delete all files in a section (nodes, urdf, config, launch)
     * @param {string} packageName - Name of the package
     * @param {string} sectionType - Type of section: 'nodes', 'urdf', 'config', 'launch'
     * @returns {Promise<{success: boolean, message: string}>}
     */
    async deleteSectionFiles(packageName, sectionType) {
        try {
            if (!this.currentProjectPath) {
                return {
                    success: false,
                    message: 'No project is currently loaded.'
                };
            }

            // Map section type to folder name
            const folderMap = {
                'nodes': packageName,  // nodes are in package/package_name folder
                'urdf': 'urdf',
                'config': 'config',
                'launch': 'launch'
            };

            const folderName = folderMap[sectionType];
            if (!folderName) {
                return {
                    success: false,
                    message: `Unknown section type: ${sectionType}`
                };
            }

            let sectionPath;
            if (sectionType === 'nodes') {
                // Nodes are in package/package_name/ folder (the Python module)
                sectionPath = path.join(this.currentProjectPath, 'src', packageName, packageName);
            } else {
                sectionPath = path.join(this.currentProjectPath, 'src', packageName, folderName);
            }

            // Check if section exists
            try {
                await fs.access(sectionPath);
            } catch {
                return {
                    success: false,
                    message: `Section "${sectionType}" not found in package "${packageName}".`
                };
            }

            if (sectionType === 'nodes') {
                // For nodes, we delete all .py files except __init__.py
                const files = await fs.readdir(sectionPath);
                const nodeFiles = files.filter(f => f.endsWith('.py') && f !== '__init__.py');

                for (const file of nodeFiles) {
                    await fs.unlink(path.join(sectionPath, file));
                }

                // Also need to update setup.py to remove entry points
                // For simplicity, we'll just delete the files for now

                return {
                    success: true,
                    message: `Deleted ${nodeFiles.length} node(s) from package "${packageName}".`
                };
            } else {
                // For other sections, delete the entire folder and recreate it empty
                await fs.rm(sectionPath, { recursive: true, force: true });

                return {
                    success: true,
                    message: `Deleted all ${sectionType} files from package "${packageName}".`
                };
            }

        } catch (error) {
            console.error('Error deleting section files:', error);
            return {
                success: false,
                message: `Error deleting ${sectionType} files: ${error.message}`
            };
        }
    }

    /**
     * Delete a single file from a section
     * @param {string} packageName - Name of the package
     * @param {string} sectionType - Type of section: 'nodes', 'urdf', 'config', 'launch'
     * @param {string} fileName - Name of the file to delete
     * @returns {Promise<{success: boolean, message: string}>}
     */
    async deleteSectionFile(packageName, sectionType, fileName) {
        try {
            if (!this.currentProjectPath) {
                return {
                    success: false,
                    message: 'No project is currently loaded.'
                };
            }

            // Map section type to folder name
            const folderMap = {
                'nodes': packageName,
                'urdf': 'urdf',
                'config': 'config',
                'launch': 'launch'
            };

            const folderName = folderMap[sectionType];
            if (!folderName) {
                return {
                    success: false,
                    message: `Unknown section type: ${sectionType}`
                };
            }

            let filePath;
            if (sectionType === 'nodes') {
                filePath = path.join(this.currentProjectPath, 'src', packageName, packageName, fileName);
            } else {
                filePath = path.join(this.currentProjectPath, 'src', packageName, folderName, fileName);
            }

            // Check if file exists
            try {
                await fs.access(filePath);
            } catch {
                return {
                    success: false,
                    message: `File "${fileName}" not found.`
                };
            }

            // Delete the file
            await fs.unlink(filePath);

            // For urdf/config/launch, check if folder is now empty and delete it
            if (sectionType !== 'nodes') {
                const folderPath = path.join(this.currentProjectPath, 'src', packageName, folderName);
                try {
                    const remainingFiles = await fs.readdir(folderPath);
                    if (remainingFiles.length === 0) {
                        await fs.rmdir(folderPath);
                        console.log(`Deleted empty folder: ${folderPath}`);
                    }
                } catch (err) {
                    console.log('Could not check/delete folder:', err.message);
                }
            }

            return {
                success: true,
                message: `File "${fileName}" deleted successfully.`
            };

        } catch (error) {
            console.error('Error deleting file:', error);
            return {
                success: false,
                message: `Error deleting file: ${error.message}`
            };
        }
    }
}

module.exports = new PackageService();
