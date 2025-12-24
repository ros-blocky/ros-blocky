/**
 * File Service - Handles URDF, config, launch, and mesh file operations
 * Extracted from packageService.js for better separation of concerns
 */

const { dialog, BrowserWindow } = require('electron');
const fs = require('fs').promises;
const path = require('path');
const setupPyService = require('./setupPyService');
const dialogService = require('./dialogService');

// Reference to get project path (set by packageService)
let getProjectPathFn = null;

/**
 * Initialize the service with a project path getter
 * @param {Function} getProjectPath - Function to get current project path
 */
function init(getProjectPath) {
    getProjectPathFn = getProjectPath;
}

/**
 * Get the current project path
 * @returns {string|null}
 */
function getProjectPath() {
    return getProjectPathFn ? getProjectPathFn() : null;
}

/**
 * Notify renderer that a file was deleted so it can close any open tabs
 * @param {string} packageName - Package name
 * @param {string} fileName - File name that was deleted
 */
function notifyFileDeleted(packageName, fileName) {
    try {
        const mainWindow = BrowserWindow.getAllWindows().find(w => !w.isDestroyed());
        if (mainWindow) {
            mainWindow.webContents.send('file-deleted', { packageName, fileName });
            console.log(`[FileService] Notified renderer of file deletion: ${packageName}/${fileName}`);
        }
    } catch (error) {
        console.error('[FileService] Error notifying file deletion:', error);
    }
}

// ============== URDF Operations ==============

/**
 * Create a new URDF file in a package
 * @param {string} packageName - Name of the package
 * @param {string} urdfName - Name of the URDF file
 * @returns {Promise<{success: boolean, message: string, urdfPath?: string}>}
 */
async function createUrdf(packageName, urdfName) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) {
            return { success: false, message: 'No project is currently loaded.' };
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

        const packagePath = path.join(projectPath, 'src', packageName);
        const urdfDir = path.join(packagePath, 'urdf');
        const urdfPath = path.join(urdfDir, `${urdfName}.xacro`);

        // Check if package exists
        try {
            await fs.access(packagePath);
        } catch {
            return { success: false, message: `Package "${packageName}" not found.` };
        }

        // Create urdf directory if it doesn't exist
        await fs.mkdir(urdfDir, { recursive: true });

        // Check if URDF already exists
        try {
            await fs.access(urdfPath);
            return { success: false, message: `URDF "${urdfName}" already exists in package "${packageName}".` };
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

        // Add urdf folder to setup.py data_files
        await setupPyService.addDataFilesEntry(packageName, 'urdf');

        return {
            success: true,
            message: `URDF "${urdfName}" created successfully!`,
            urdfPath
        };

    } catch (error) {
        console.error('Error creating URDF:', error);
        return { success: false, message: `Error creating URDF: ${error.message}` };
    }
}

/**
 * Save URDF file content
 * @param {string} packageName - Package name
 * @param {string} fileName - File name (e.g. robot.urdf)
 * @param {string} content - File content
 * @returns {Promise<{success: boolean, message: string}>}
 */
async function saveUrdfFile(packageName, fileName, content) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) {
            return { success: false, message: 'No project is currently loaded.' };
        }

        const urdfPath = path.join(projectPath, 'src', packageName, 'urdf', fileName);
        await fs.writeFile(urdfPath, content, 'utf8');

        return { success: true, message: `File "${fileName}" saved successfully.` };
    } catch (error) {
        console.error('Error saving URDF file:', error);
        return { success: false, message: `Error saving file: ${error.message}` };
    }
}

/**
 * Save block state to a sidecar .blocks file
 * @param {string} packageName - Package name
 * @param {string} fileName - Original file name (e.g. robot.urdf)
 * @param {string} blockXml - Blockly XML serialization
 * @returns {Promise<{success: boolean, message: string}>}
 */
async function saveBlockState(packageName, fileName, blockXml) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) {
            return { success: false, message: 'No project is currently loaded.' };
        }

        const blocksFileName = `${fileName}.blocks`;
        const blocksPath = path.join(projectPath, 'src', packageName, 'urdf', blocksFileName);

        await fs.writeFile(blocksPath, blockXml, 'utf8');

        console.log(`[FileService] Block state saved to ${blocksFileName}`);
        return { success: true, message: `Block state saved successfully.` };
    } catch (error) {
        console.error('Error saving block state:', error);
        return { success: false, message: `Error saving block state: ${error.message}` };
    }
}

/**
 * Load block state from a sidecar .blocks file
 * @param {string} packageName - Package name
 * @param {string} fileName - Original file name (e.g. robot.urdf)
 * @returns {Promise<{success: boolean, blockXml?: string, message?: string}>}
 */
async function loadBlockState(packageName, fileName) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) {
            return { success: false, message: 'No project is currently loaded.' };
        }

        const blocksFileName = `${fileName}.blocks`;
        const blocksPath = path.join(projectPath, 'src', packageName, 'urdf', blocksFileName);

        try {
            await fs.access(blocksPath);
        } catch {
            return { success: false, message: 'No block state file found.' };
        }

        const blockXml = await fs.readFile(blocksPath, 'utf8');

        console.log(`[FileService] Block state loaded from ${blocksFileName}`);
        return { success: true, blockXml };
    } catch (error) {
        console.error('Error loading block state:', error);
        return { success: false, message: `Error loading block state: ${error.message}` };
    }
}

/**
 * List URDFs in a package
 * @param {string} packageName - Name of the package
 * @returns {Promise<string[]>}
 */
async function listPackageUrdfs(packageName) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) return [];

        const urdfDir = path.join(projectPath, 'src', packageName, 'urdf');
        const files = await fs.readdir(urdfDir);
        return files.filter(f => f.endsWith('.xacro') || f.endsWith('.urdf'));
    } catch {
        return [];
    }
}

// ============== Config Operations ==============

/**
 * Create a new config file in a package
 * @param {string} packageName - Name of the package
 * @param {string} configName - Name of the config file
 * @returns {Promise<{success: boolean, message: string, configPath?: string}>}
 */
async function createConfig(packageName, configName) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) {
            return { success: false, message: 'No project is currently loaded.' };
        }

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

        const packagePath = path.join(projectPath, 'src', packageName);
        const configDir = path.join(packagePath, 'config');
        const configPath = path.join(configDir, `${configName}.yaml`);

        try {
            await fs.access(packagePath);
        } catch {
            return { success: false, message: `Package "${packageName}" not found.` };
        }

        await fs.mkdir(configDir, { recursive: true });

        try {
            await fs.access(configPath);
            return { success: false, message: `Config "${configName}" already exists in package "${packageName}".` };
        } catch {
            // Config doesn't exist, continue
        }

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
        await setupPyService.addDataFilesEntry(packageName, 'config');

        return { success: true, message: `Config "${configName}" created successfully!`, configPath };

    } catch (error) {
        console.error('Error creating config:', error);
        return { success: false, message: `Error creating config: ${error.message}` };
    }
}

/**
 * List configs in a package
 * @param {string} packageName - Name of the package
 * @returns {Promise<string[]>}
 */
async function listPackageConfigs(packageName) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) return [];

        const configDir = path.join(projectPath, 'src', packageName, 'config');
        const files = await fs.readdir(configDir);
        return files.filter(f => f.endsWith('.yaml') || f.endsWith('.yml'));
    } catch {
        return [];
    }
}

// ============== Launch Operations ==============

/**
 * Create a new launch file in a package
 * @param {string} packageName - Name of the package
 * @param {string} launchName - Name of the launch file
 * @returns {Promise<{success: boolean, message: string, launchPath?: string}>}
 */
async function createLaunch(packageName, launchName) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) {
            return { success: false, message: 'No project is currently loaded.' };
        }

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

        const packagePath = path.join(projectPath, 'src', packageName);
        const launchDir = path.join(packagePath, 'launch');
        const launchPath = path.join(launchDir, `${launchName}.py`);

        try {
            await fs.access(packagePath);
        } catch {
            return { success: false, message: `Package "${packageName}" not found.` };
        }

        await fs.mkdir(launchDir, { recursive: true });

        try {
            await fs.access(launchPath);
            return { success: false, message: `Launch file "${launchName}" already exists in package "${packageName}".` };
        } catch {
            // Launch doesn't exist, continue
        }

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
        await setupPyService.addDataFilesEntry(packageName, 'launch');

        return { success: true, message: `Launch file "${launchName}" created successfully!`, launchPath };

    } catch (error) {
        console.error('Error creating launch file:', error);
        return { success: false, message: `Error creating launch file: ${error.message}` };
    }
}

/**
 * List launch files in a package
 * @param {string} packageName - Name of the package
 * @returns {Promise<string[]>}
 */
async function listPackageLaunches(packageName) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) return [];

        const launchDir = path.join(projectPath, 'src', packageName, 'launch');
        const files = await fs.readdir(launchDir);
        return files.filter(f => f.endsWith('.py'));
    } catch {
        return [];
    }
}

// ============== Mesh Operations ==============

/**
 * Import mesh files (.stl, .dae) into a package's meshes folder
 * @param {string} packageName - Name of the package
 * @returns {Promise<{success: boolean, message: string, filesCopied?: number}>}
 */
async function importMeshFiles(packageName) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) {
            return { success: false, message: 'No project is currently loaded.' };
        }

        // Open file dialog for mesh files
        const result = await dialog.showOpenDialog({
            title: 'Select Mesh Files',
            filters: [
                { name: 'Mesh Files', extensions: ['stl', 'STL', 'dae', 'DAE'] }
            ],
            properties: ['openFile', 'multiSelections']
        });

        if (result.canceled || result.filePaths.length === 0) {
            return { success: false, message: 'No files selected' };
        }

        const meshesDir = path.join(projectPath, 'src', packageName, 'meshes');

        // Create meshes folder if it doesn't exist
        try {
            await fs.access(meshesDir);
        } catch {
            await fs.mkdir(meshesDir, { recursive: true });
            console.log('Created meshes folder:', meshesDir);
        }

        // Check for duplicate files
        const duplicates = [];
        for (const filePath of result.filePaths) {
            const fileName = path.basename(filePath);
            const destPath = path.join(meshesDir, fileName);
            try {
                await fs.access(destPath);
                duplicates.push(fileName);
            } catch {
                // File doesn't exist, no duplicate
            }
        }

        // If duplicates found, ask for confirmation
        if (duplicates.length > 0) {
            const duplicateList = duplicates.length <= 3
                ? duplicates.join(', ')
                : `${duplicates.slice(0, 3).join(', ')} and ${duplicates.length - 3} more`;
            const confirmOverwrite = await dialogService.showConfirmDialog(
                `The following files already exist:\n${duplicateList}\n\nDo you want to overwrite them?`
            );

            if (!confirmOverwrite) {
                return { success: false, message: 'Import cancelled' };
            }
        }

        // Copy each selected file to meshes folder
        let filesCopied = 0;
        for (const filePath of result.filePaths) {
            const fileName = path.basename(filePath);
            const destPath = path.join(meshesDir, fileName);
            await fs.copyFile(filePath, destPath);
            filesCopied++;
            console.log(`Copied mesh file: ${fileName}`);
        }

        // Add meshes folder to setup.py data_files
        await setupPyService.addDataFilesEntry(packageName, 'meshes');

        return {
            success: true,
            message: `Successfully imported ${filesCopied} mesh file(s).`,
            filesCopied
        };

    } catch (error) {
        console.error('Error importing mesh files:', error);
        return { success: false, message: `Error importing mesh files: ${error.message}` };
    }
}

/**
 * List mesh files in a package
 * @param {string} packageName - Name of the package
 * @returns {Promise<string[]>}
 */
async function listPackageMeshes(packageName) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) return [];

        const meshesDir = path.join(projectPath, 'src', packageName, 'meshes');
        const files = await fs.readdir(meshesDir);
        return files.filter(f =>
            f.toLowerCase().endsWith('.stl') ||
            f.toLowerCase().endsWith('.dae')
        );
    } catch {
        return [];
    }
}

// ============== Generic File Operations ==============

/**
 * Delete all files in a section (nodes, urdf, config, launch)
 * @param {string} packageName - Name of the package
 * @param {string} sectionType - Type of section: 'nodes', 'urdf', 'config', 'launch'
 * @returns {Promise<{success: boolean, message: string}>}
 */
async function deleteSectionFiles(packageName, sectionType) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) {
            return { success: false, message: 'No project is currently loaded.' };
        }

        const folderMap = {
            'nodes': packageName,
            'meshes': 'meshes',
            'urdf': 'urdf',
            'config': 'config',
            'launch': 'launch'
        };

        const folderName = folderMap[sectionType];
        if (!folderName) {
            return { success: false, message: `Unknown section type: ${sectionType}` };
        }

        let sectionPath;
        if (sectionType === 'nodes') {
            sectionPath = path.join(projectPath, 'src', packageName, packageName);
        } else {
            sectionPath = path.join(projectPath, 'src', packageName, folderName);
        }

        try {
            await fs.access(sectionPath);
        } catch {
            return { success: false, message: `Section "${sectionType}" not found in package "${packageName}".` };
        }

        if (sectionType === 'nodes') {
            const files = await fs.readdir(sectionPath);
            const nodeFiles = files.filter(f => f.endsWith('.py') && f !== '__init__.py');

            for (const file of nodeFiles) {
                await fs.unlink(path.join(sectionPath, file));
                const nodeName = file.replace('.py', '');
                await setupPyService.removeNodeEntryPoint(packageName, nodeName);

                // Also delete the .nodeblocks sidecar file if it exists
                try {
                    await fs.unlink(path.join(sectionPath, `${file}.nodeblocks`));
                } catch { /* sidecar doesn't exist */ }

                // Notify renderer to close open tab
                notifyFileDeleted(packageName, file);
            }

            return {
                success: true,
                message: `Deleted ${nodeFiles.length} node(s) from package "${packageName}".`
            };
        } else {
            // For urdf, notify for each file before deleting the whole folder
            if (sectionType === 'urdf') {
                try {
                    const urdfFiles = await fs.readdir(sectionPath);
                    for (const file of urdfFiles) {
                        if (file.endsWith('.xacro') || file.endsWith('.urdf')) {
                            notifyFileDeleted(packageName, file);
                        }
                    }
                } catch { /* folder might not exist */ }
            }

            await fs.rm(sectionPath, { recursive: true, force: true });
            await setupPyService.removeDataFilesEntry(packageName, sectionType);

            return {
                success: true,
                message: `Deleted all ${sectionType} files from package "${packageName}".`
            };
        }

    } catch (error) {
        console.error('Error deleting section files:', error);
        return { success: false, message: `Error deleting ${sectionType} files: ${error.message}` };
    }
}

/**
 * Delete a single file from a section
 * @param {string} packageName - Name of the package
 * @param {string} sectionType - Type of section: 'nodes', 'urdf', 'config', 'launch'
 * @param {string} fileName - Name of the file to delete
 * @returns {Promise<{success: boolean, message: string}>}
 */
async function deleteSectionFile(packageName, sectionType, fileName) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) {
            return { success: false, message: 'No project is currently loaded.' };
        }

        const folderMap = {
            'nodes': packageName,
            'meshes': 'meshes',
            'urdf': 'urdf',
            'config': 'config',
            'launch': 'launch'
        };

        const folderName = folderMap[sectionType];
        if (!folderName) {
            return { success: false, message: `Unknown section type: ${sectionType}` };
        }

        let filePath;
        if (sectionType === 'nodes') {
            filePath = path.join(projectPath, 'src', packageName, packageName, fileName);
        } else {
            filePath = path.join(projectPath, 'src', packageName, folderName, fileName);
        }

        try {
            await fs.access(filePath);
        } catch {
            return { success: false, message: `File "${fileName}" not found.` };
        }

        await fs.unlink(filePath);

        // For nodes, also remove entry point from setup.py and delete sidecar
        if (sectionType === 'nodes') {
            const nodeName = fileName.replace('.py', '');
            await setupPyService.removeNodeEntryPoint(packageName, nodeName);

            // Delete the .nodeblocks sidecar file if it exists
            try {
                await fs.unlink(path.join(projectPath, 'src', packageName, packageName, `${fileName}.nodeblocks`));
                console.log(`[FileService] Deleted sidecar: ${fileName}.nodeblocks`);
            } catch { /* sidecar doesn't exist */ }
        }

        // For urdf, also delete the .blocks sidecar file
        if (sectionType === 'urdf') {
            try {
                await fs.unlink(path.join(projectPath, 'src', packageName, 'urdf', `${fileName}.blocks`));
                console.log(`[FileService] Deleted sidecar: ${fileName}.blocks`);
            } catch { /* sidecar doesn't exist */ }
        }

        // For urdf/config/launch/meshes, check if folder is now empty and delete it
        if (sectionType !== 'nodes') {
            const folderPath = path.join(projectPath, 'src', packageName, folderName);
            try {
                const remainingFiles = await fs.readdir(folderPath);
                if (remainingFiles.length === 0) {
                    await fs.rmdir(folderPath);
                    console.log(`Deleted empty folder: ${folderPath}`);
                    await setupPyService.removeDataFilesEntry(packageName, sectionType);
                }
            } catch (err) {
                console.log('Could not check/delete folder:', err.message);
            }
        }

        // Notify renderer to close any open tabs for this file
        notifyFileDeleted(packageName, fileName);

        return { success: true, message: `File "${fileName}" deleted successfully.` };

    } catch (error) {
        console.error('Error deleting file:', error);
        return { success: false, message: `Error deleting file: ${error.message}` };
    }
}

/**
 * Create a file in a section folder (urdf, config, launch)
 * @param {string} packageName - Name of the package
 * @param {string} folderName - Folder name (urdf, config, launch)
 * @param {string} fileName - Full file name with extension
 * @returns {Promise<{success: boolean, message: string, filePath?: string}>}
 */
async function createSectionFile(packageName, folderName, fileName) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) {
            return { success: false, message: 'No project is currently loaded.' };
        }

        // Extract name without extension
        const baseName = fileName.replace(/\.[^/.]+$/, '');

        // Route to specific create method based on folder
        switch (folderName) {
            case 'urdf':
                return await createUrdf(packageName, baseName);
            case 'config':
                return await createConfig(packageName, baseName);
            case 'launch':
                return await createLaunch(packageName, baseName);
            default:
                return { success: false, message: `Unknown folder type: ${folderName}` };
        }

    } catch (error) {
        console.error('Error creating section file:', error);
        return { success: false, message: `Error creating file: ${error.message}` };
    }
}

module.exports = {
    init,
    // URDF
    createUrdf,
    saveUrdfFile,
    saveBlockState,
    loadBlockState,
    listPackageUrdfs,
    // Config
    createConfig,
    listPackageConfigs,
    // Launch
    createLaunch,
    listPackageLaunches,
    // Mesh
    importMeshFiles,
    listPackageMeshes,
    // Generic
    deleteSectionFiles,
    deleteSectionFile,
    createSectionFile
};
