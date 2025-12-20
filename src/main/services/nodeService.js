/**
 * Node Service - Handles Python node file operations
 * Extracted from packageService.js for better separation of concerns
 */

const fs = require('fs').promises;
const path = require('path');
const setupPyService = require('./setupPyService');

// Reference to get project path (set by packageService)
let getProjectPathFn = null;

/**
 * Initialize the service with a project path getter
 * @param {Function} getProjectPath - Function to get current project path
 */
function init(getProjectPath) {
    getProjectPathFn = getProjectPath;
    setupPyService.init(getProjectPath);
}

/**
 * Get the current project path
 * @returns {string|null}
 */
function getProjectPath() {
    return getProjectPathFn ? getProjectPathFn() : null;
}

/**
 * Convert snake_case to PascalCase
 * @param {string} str - String to convert
 * @returns {string}
 */
function toPascalCase(str) {
    return str.split('_').map(word =>
        word.charAt(0).toUpperCase() + word.slice(1)
    ).join('');
}

/**
 * Create a new node in a package
 * @param {string} packageName - Name of the package
 * @param {string} nodeName - Name of the node
 * @returns {Promise<{success: boolean, message: string, nodePath?: string}>}
 */
async function createNode(packageName, nodeName) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) {
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

        const packagePath = path.join(projectPath, 'src', packageName);
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


class ${toPascalCase(nodeName)}(Node):
    """${nodeName} node."""

    def __init__(self):
        super().__init__('${nodeName}')
        self.get_logger().info('${nodeName} node started')


def main(args=None):
    rclpy.init(args=args)
    node = ${toPascalCase(nodeName)}()
    
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
        await setupPyService.addNodeEntryPoint(packageName, nodeName);

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
 * Save Node Python file content
 * @param {string} packageName - Package name
 * @param {string} fileName - File name (e.g. my_node.py)
 * @param {string} content - Python code content
 * @returns {Promise<{success: boolean, message: string}>}
 */
async function saveNodeFile(packageName, fileName, content) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) {
            return {
                success: false,
                message: 'No project is currently loaded.'
            };
        }

        // Node files are in src/package_name/package_name/
        const nodePath = path.join(projectPath, 'src', packageName, packageName, fileName);

        // Write content to file
        await fs.writeFile(nodePath, content, 'utf8');

        console.log(`[NodeService] Node file saved: ${nodePath}`);
        return {
            success: true,
            message: `Node file "${fileName}" saved successfully.`
        };
    } catch (error) {
        console.error('Error saving Node file:', error);
        return {
            success: false,
            message: `Error saving Node file: ${error.message}`
        };
    }
}

/**
 * Save Node block state to a sidecar .nodeblocks file
 * @param {string} packageName - Package name
 * @param {string} fileName - Original file name (e.g. my_node.py)
 * @param {string} blockXml - Blockly XML serialization
 * @returns {Promise<{success: boolean, message: string}>}
 */
async function saveNodeBlockState(packageName, fileName, blockXml) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) {
            return {
                success: false,
                message: 'No project is currently loaded.'
            };
        }

        // Create .nodeblocks sidecar file path (in same folder as node)
        const blocksFileName = `${fileName}.nodeblocks`;
        const blocksPath = path.join(projectPath, 'src', packageName, packageName, blocksFileName);

        // Write block state to file
        await fs.writeFile(blocksPath, blockXml, 'utf8');

        console.log(`[NodeService] Node block state saved to ${blocksFileName}`);
        return {
            success: true,
            message: `Node block state saved successfully.`
        };
    } catch (error) {
        console.error('Error saving Node block state:', error);
        return {
            success: false,
            message: `Error saving Node block state: ${error.message}`
        };
    }
}

/**
 * Load Node block state from a sidecar .nodeblocks file
 * @param {string} packageName - Package name
 * @param {string} fileName - Original file name (e.g. my_node.py)
 * @returns {Promise<{success: boolean, blockXml?: string, message?: string}>}
 */
async function loadNodeBlockState(packageName, fileName) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) {
            return {
                success: false,
                message: 'No project is currently loaded.'
            };
        }

        // Create .nodeblocks sidecar file path
        const blocksFileName = `${fileName}.nodeblocks`;
        const blocksPath = path.join(projectPath, 'src', packageName, packageName, blocksFileName);

        // Check if file exists
        try {
            await fs.access(blocksPath);
        } catch {
            // File doesn't exist - this is normal for new files
            return {
                success: false,
                message: 'No node block state file found.'
            };
        }

        // Read block state from file
        const blockXml = await fs.readFile(blocksPath, 'utf8');

        console.log(`[NodeService] Node block state loaded from ${blocksFileName}`);
        return {
            success: true,
            blockXml: blockXml
        };
    } catch (error) {
        console.error('Error loading Node block state:', error);
        return {
            success: false,
            message: `Error loading Node block state: ${error.message}`
        };
    }
}

/**
 * List nodes in a package
 * @param {string} packageName - Name of the package
 * @returns {Promise<string[]>}
 */
async function listPackageNodes(packageName) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) return [];

        const nodeDir = path.join(projectPath, 'src', packageName, packageName);
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
 * Delete a node file
 * @param {string} packageName - Package name
 * @param {string} fileName - Node file name
 * @returns {Promise<{success: boolean, message: string}>}
 */
async function deleteNode(packageName, fileName) {
    try {
        const projectPath = getProjectPath();
        if (!projectPath) {
            return {
                success: false,
                message: 'No project is currently loaded.'
            };
        }

        const filePath = path.join(projectPath, 'src', packageName, packageName, fileName);

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

        // Remove entry point from setup.py
        const nodeName = fileName.replace('.py', '');
        await setupPyService.removeNodeEntryPoint(packageName, nodeName);

        // Also delete the .nodeblocks sidecar file if it exists
        const blocksPath = path.join(projectPath, 'src', packageName, packageName, `${fileName}.nodeblocks`);
        try {
            await fs.access(blocksPath);
            await fs.unlink(blocksPath);
            console.log(`[NodeService] Deleted sidecar file: ${fileName}.nodeblocks`);
        } catch {
            // Sidecar doesn't exist, that's fine
        }

        return {
            success: true,
            message: `Node "${fileName}" deleted successfully.`
        };
    } catch (error) {
        console.error('Error deleting node:', error);
        return {
            success: false,
            message: `Error deleting node: ${error.message}`
        };
    }
}

module.exports = {
    init,
    toPascalCase,
    createNode,
    saveNodeFile,
    saveNodeBlockState,
    loadNodeBlockState,
    listPackageNodes,
    deleteNode
};
