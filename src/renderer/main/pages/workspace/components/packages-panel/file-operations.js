/**
 * Packages Panel - File Operations
 * Create/delete packages, nodes, files, and folders
 */

import { FILE_SECTIONS, SECTION_DISPLAY_NAMES } from './config.js';
import * as api from './api.js';

// Will be set by panel-ui.js to avoid circular dependency
let refreshPackageListFn = null;

export function setRefreshFunction(fn) {
    refreshPackageListFn = fn;
}

async function refreshUI() {
    if (refreshPackageListFn) {
        await refreshPackageListFn();
    }
}

// ============================================
// Package Operations
// ============================================

/**
 * Create a new package
 */
export async function createNewPackage() {
    try {
        const packageName = await api.promptPackageName();
        if (packageName) {
            const result = await api.createPackage(packageName);
            if (result.success) {
                await refreshUI();
            } else {
                alert('Error: ' + result.message);
            }
        }
    } catch (error) {
        console.error('Error creating package:', error);
        alert('Error creating package: ' + error.message);
    }
}

/**
 * Delete a package
 */
export async function deletePackage(packageName) {
    try {
        const result = await api.deletePackage(packageName);
        if (result.success) {
            await refreshUI();
        } else {
            if (!result.message.includes('cancelled')) {
                alert('Error: ' + result.message);
            }
        }
    } catch (error) {
        console.error('Error deleting package:', error);
        alert('Error deleting package: ' + error.message);
    }
}

// ============================================
// Node Operations
// ============================================

/**
 * Add a node to a package
 */
export async function addNodeToPackage(packageName) {
    try {
        const nodeName = await api.promptNodeName(packageName);
        if (nodeName) {
            const result = await api.createNode(packageName, nodeName);
            if (result.success) {
                console.log('Node created:', result.nodePath);
                await refreshUI();
            } else {
                alert('Error: ' + result.message);
            }
        }
    } catch (error) {
        console.error('Error creating node:', error);
        alert('Error creating node: ' + error.message);
    }
}

/**
 * Delete a node from a package
 */
export async function deleteNode(packageName, nodeName) {
    try {
        const message = `Are you sure you want to delete node "${nodeName}" from package "${packageName}"?`;
        const confirmed = await api.showConfirmDialog(message);

        if (!confirmed) return;

        const result = await api.deleteSectionFile(packageName, 'nodes', nodeName + '.py');
        if (result.success) {
            console.log('Node deleted successfully');
            await refreshUI();
        } else {
            alert('Error: ' + result.message);
        }
    } catch (error) {
        console.error('Error deleting node:', error);
        alert('Error deleting node: ' + error.message);
    }
}

// ============================================
// File Section Operations
// ============================================

/**
 * Import mesh files to a package
 */
export async function importMeshes(packageName) {
    try {
        const result = await api.importMeshFiles(packageName);
        if (result.success) {
            console.log(`Imported ${result.filesCopied} mesh file(s)`);
            await refreshUI();
        } else {
            if (result.message !== 'No files selected') {
                alert('Error: ' + result.message);
            }
        }
    } catch (error) {
        console.error('Error importing mesh files:', error);
        alert('Error importing mesh files: ' + error.message);
    }
}

/**
 * Add files to a section (urdf, config, launch, meshes)
 */
export async function addFilesToSection(packageName, folderName) {
    try {
        const config = FILE_SECTIONS[folderName];
        if (!config) {
            console.error(`No configuration found for folder: ${folderName}`);
            return false;
        }

        // Check mode: create new file or import existing files
        if (config.mode === 'create') {
            // Create new file with name prompt
            const fileName = await api.promptFileName(
                packageName,
                `Create ${config.label} File`,
                `Enter ${config.label.toLowerCase()} file name (without extension)`
            );
            if (!fileName) return false;

            // Determine file extension based on folder type
            let extension = config.extensions[0];
            const fullFileName = fileName + extension;

            // Call backend to create the file
            const result = await api.createSectionFile(packageName, folderName, fullFileName);

            if (result.success) {
                console.log(`Created ${fullFileName} in ${folderName}`);
                await refreshUI();
                return true;
            } else {
                alert('Error: ' + result.message);
                return false;
            }
        } else {
            // Import existing files (for meshes)
            const result = await api.selectFolderFiles(
                packageName,
                folderName,
                config.extensions,
                config.fileTypeLabel
            );

            // Check if duplicates were found
            if (result.message === 'duplicates_found') {
                const confirmed = await api.showConfirmDialog(
                    `The selected files already exist. Do you want to overwrite them?`
                );

                if (confirmed) {
                    const copyResult = await api.copyFolderFilesForce(
                        packageName,
                        folderName,
                        result.selectedFiles
                    );
                    if (copyResult.success) {
                        console.log(`${copyResult.filesCopied} ${folderName} file(s) copied`);
                        await refreshUI();
                        return true;
                    } else {
                        alert('Error: ' + copyResult.message);
                        return false;
                    }
                } else {
                    return false;
                }
            } else if (result.success) {
                console.log(`${result.filesCopied} ${folderName} file(s) copied`);
                await refreshUI();
                return true;
            } else {
                if (result.message !== 'No files selected') {
                    alert('Error: ' + result.message);
                }
                return false;
            }
        }
    } catch (error) {
        console.error(`Error adding ${folderName} files:`, error);
        alert(`Error adding ${folderName} files: ` + error.message);
        return false;
    }
}

/**
 * Delete a single file from a section
 */
export async function deleteFile(packageName, sectionType, fileName) {
    try {
        const message = `Are you sure you want to delete "${fileName}" from package "${packageName}"?`;
        const confirmed = await api.showConfirmDialog(message);

        if (!confirmed) return;

        const result = await api.deleteSectionFile(packageName, sectionType, fileName);
        if (result.success) {
            console.log(`File ${fileName} deleted successfully`);
            await refreshUI();
        } else {
            alert('Error: ' + result.message);
        }
    } catch (error) {
        console.error('Error deleting file:', error);
        alert('Error deleting file: ' + error.message);
    }
}

/**
 * Delete all files in a section (folder)
 */
export async function deleteFolder(packageName, sectionType) {
    try {
        const displayName = SECTION_DISPLAY_NAMES[sectionType] || sectionType;
        const message = `Are you sure you want to delete all ${displayName} from package "${packageName}"?`;
        const confirmed = await api.showConfirmDialog(message);

        if (!confirmed) return;

        const result = await api.deleteSectionFiles(packageName, sectionType);
        if (result.success) {
            console.log(`${sectionType} folder deleted successfully`);
            await refreshUI();
        } else {
            alert('Error: ' + result.message);
        }
    } catch (error) {
        console.error('Error deleting folder:', error);
        alert('Error deleting folder: ' + error.message);
    }
}
