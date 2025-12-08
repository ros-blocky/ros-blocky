/**
 * Packages Panel - Context Menu
 * Context menu setup and visibility rules
 */

import { contextState, resetContextState } from './config.js';
import * as fileOps from './file-operations.js';

/**
 * Setup context menu for the packages panel
 */
export function setupContextMenu() {
    const explorerContent = document.getElementById('explorer-panel-content');
    const contextMenu = document.getElementById('explorer-context-menu');
    const addPackageItem = document.getElementById('context-add-package');
    const addNodeItem = document.getElementById('context-add-node');
    const addMeshesItem = document.getElementById('context-add-meshes');
    const addUrdfItem = document.getElementById('context-add-urdf');
    const addConfigItem = document.getElementById('context-add-config');
    const addLaunchItem = document.getElementById('context-add-launch');
    const deletePackageItem = document.getElementById('context-delete-package');
    const deleteNodeItem = document.getElementById('context-delete-node');
    const deleteFileItem = document.getElementById('context-delete-file');
    const deleteFolderItem = document.getElementById('context-delete-folder');

    if (!explorerContent || !contextMenu) {
        console.error('Context menu elements not found');
        return;
    }

    // Right-click handler
    explorerContent.addEventListener('contextmenu', (e) => {
        const nodeItem = e.target.closest('.node-item');
        const fileItem = e.target.closest('.file-item');
        const folderHeader = e.target.closest('.file-header');
        const nodesHeader = e.target.closest('.nodes-header');
        const packageHeader = e.target.closest('.package-header');

        // Check if clicked on + button (don't show context menu)
        if (e.target.classList.contains('section-add-btn')) {
            return;
        }

        if (folderHeader) {
            // Right-clicked on folder header (meshes, urdf, config, launch)
            handleFolderHeaderContext(e, folderHeader, {
                addPackageItem, addNodeItem, addMeshesItem, addUrdfItem,
                addConfigItem, addLaunchItem, deletePackageItem, deleteNodeItem,
                deleteFileItem, deleteFolderItem
            });
        } else if (nodesHeader) {
            // Right-clicked on nodes header
            handleNodesHeaderContext(e, nodesHeader, {
                addPackageItem, addNodeItem, addMeshesItem, addUrdfItem,
                addConfigItem, addLaunchItem, deletePackageItem, deleteNodeItem,
                deleteFileItem, deleteFolderItem
            });
        } else if (fileItem) {
            // Right-clicked on a file
            handleFileContext(e, fileItem, {
                addPackageItem, addNodeItem, addMeshesItem, addUrdfItem,
                addConfigItem, addLaunchItem, deletePackageItem, deleteNodeItem,
                deleteFileItem, deleteFolderItem
            });
        } else if (nodeItem) {
            // Right-clicked on a node
            handleNodeContext(e, nodeItem, {
                addPackageItem, addNodeItem, addMeshesItem, addUrdfItem,
                addConfigItem, addLaunchItem, deletePackageItem, deleteNodeItem,
                deleteFileItem, deleteFolderItem
            });
        } else if (packageHeader) {
            // Right-clicked on a package
            handlePackageContext(e, packageHeader, {
                addPackageItem, addNodeItem, addMeshesItem, addUrdfItem,
                addConfigItem, addLaunchItem, deletePackageItem, deleteNodeItem,
                deleteFileItem, deleteFolderItem
            });
        } else {
            // Right-clicked on empty area
            handleEmptyContext(e, {
                addPackageItem, addNodeItem, addMeshesItem, addUrdfItem,
                addConfigItem, addLaunchItem, deletePackageItem, deleteNodeItem,
                deleteFileItem, deleteFolderItem
            });
        }

        // Position context menu at click location
        const rect = explorerContent.getBoundingClientRect();
        contextMenu.style.left = (e.clientX - rect.left) + 'px';
        contextMenu.style.top = (e.clientY - rect.top) + 'px';
        contextMenu.classList.remove('hidden');
    });

    // Close context menu on click
    document.addEventListener('click', () => {
        contextMenu.classList.add('hidden');
    });

    // Setup menu item click handlers
    setupMenuItemHandlers(contextMenu, {
        addPackageItem, addNodeItem, addMeshesItem, addUrdfItem,
        addConfigItem, addLaunchItem, deletePackageItem, deleteNodeItem,
        deleteFileItem, deleteFolderItem
    });
}

/**
 * Handle folder header context menu
 */
function handleFolderHeaderContext(e, folderHeader, items) {
    e.preventDefault();
    const packageItem = folderHeader.closest('.package-item');
    const fileSection = folderHeader.closest('.file-section');

    contextState.currentPackageContext = packageItem.dataset.packageName;
    contextState.currentNodeContext = null;
    contextState.currentFileContext = { folder: null, fileName: null };
    contextState.currentFolderContext = fileSection.dataset.folder;

    hideAllItems(items);
    if (items.deleteFolderItem) items.deleteFolderItem.classList.remove('hidden');
}

/**
 * Handle nodes header context menu
 */
function handleNodesHeaderContext(e, nodesHeader, items) {
    e.preventDefault();
    const packageItem = nodesHeader.closest('.package-item');

    contextState.currentPackageContext = packageItem.dataset.packageName;
    contextState.currentNodeContext = null;
    contextState.currentFileContext = { folder: null, fileName: null };
    contextState.currentFolderContext = 'nodes';

    hideAllItems(items);
    if (items.deleteFolderItem) items.deleteFolderItem.classList.remove('hidden');
}

/**
 * Handle file context menu
 */
function handleFileContext(e, fileItem, items) {
    e.preventDefault();
    const packageItem = fileItem.closest('.package-item');
    const fileSection = fileItem.closest('.file-section');

    contextState.currentPackageContext = packageItem.dataset.packageName;
    contextState.currentNodeContext = null;
    contextState.currentFileContext.folder = fileSection.dataset.folder;
    contextState.currentFileContext.fileName = fileItem.querySelector('.file-name').textContent;
    contextState.currentFolderContext = null;

    hideAllItems(items);
    if (items.deleteFileItem) items.deleteFileItem.classList.remove('hidden');
}

/**
 * Handle node context menu
 */
function handleNodeContext(e, nodeItem, items) {
    e.preventDefault();
    const packageItem = nodeItem.closest('.package-item');

    contextState.currentPackageContext = packageItem.dataset.packageName;
    contextState.currentNodeContext = nodeItem.querySelector('.node-name').textContent;
    contextState.currentFileContext = { folder: null, fileName: null };
    contextState.currentFolderContext = null;

    hideAllItems(items);
    if (items.deleteNodeItem) items.deleteNodeItem.classList.remove('hidden');
}

/**
 * Handle package context menu
 */
function handlePackageContext(e, packageHeader, items) {
    e.preventDefault();
    const packageItem = packageHeader.closest('.package-item');

    contextState.currentPackageContext = packageItem.dataset.packageName;
    contextState.currentNodeContext = null;
    contextState.currentFileContext = { folder: null, fileName: null };
    contextState.currentFolderContext = null;

    hideAllItems(items);
    if (items.addNodeItem) items.addNodeItem.classList.remove('hidden');
    if (items.addMeshesItem) items.addMeshesItem.classList.remove('hidden');
    if (items.addUrdfItem) items.addUrdfItem.classList.remove('hidden');
    if (items.addConfigItem) items.addConfigItem.classList.remove('hidden');
    if (items.addLaunchItem) items.addLaunchItem.classList.remove('hidden');
    if (items.deletePackageItem) items.deletePackageItem.classList.remove('hidden');
}

/**
 * Handle empty area context menu
 */
function handleEmptyContext(e, items) {
    e.preventDefault();
    resetContextState();

    hideAllItems(items);
    if (items.addPackageItem) items.addPackageItem.classList.remove('hidden');
}

/**
 * Hide all context menu items
 */
function hideAllItems(items) {
    Object.values(items).forEach(item => {
        if (item) item.classList.add('hidden');
    });
}

/**
 * Setup menu item click handlers
 */
function setupMenuItemHandlers(contextMenu, items) {
    const { addPackageItem, addNodeItem, addMeshesItem, addUrdfItem,
        addConfigItem, addLaunchItem, deletePackageItem, deleteNodeItem,
        deleteFileItem, deleteFolderItem } = items;

    if (addPackageItem) {
        addPackageItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            await fileOps.createNewPackage();
        });
    }

    if (addNodeItem) {
        addNodeItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            if (contextState.currentPackageContext) {
                await fileOps.addNodeToPackage(contextState.currentPackageContext);
            }
        });
    }

    if (addMeshesItem) {
        addMeshesItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            if (contextState.currentPackageContext) {
                await fileOps.importMeshes(contextState.currentPackageContext);
            }
        });
    }

    if (addUrdfItem) {
        addUrdfItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            if (contextState.currentPackageContext) {
                await fileOps.addFilesToSection(contextState.currentPackageContext, 'urdf');
            }
        });
    }

    if (addConfigItem) {
        addConfigItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            if (contextState.currentPackageContext) {
                await fileOps.addFilesToSection(contextState.currentPackageContext, 'config');
            }
        });
    }

    if (addLaunchItem) {
        addLaunchItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            if (contextState.currentPackageContext) {
                await fileOps.addFilesToSection(contextState.currentPackageContext, 'launch');
            }
        });
    }

    if (deletePackageItem) {
        deletePackageItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            if (contextState.currentPackageContext) {
                await fileOps.deletePackage(contextState.currentPackageContext);
            }
        });
    }

    if (deleteNodeItem) {
        deleteNodeItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            if (contextState.currentPackageContext && contextState.currentNodeContext) {
                await fileOps.deleteNode(contextState.currentPackageContext, contextState.currentNodeContext);
            }
        });
    }

    if (deleteFileItem) {
        deleteFileItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            const { folder, fileName } = contextState.currentFileContext;
            if (contextState.currentPackageContext && folder && fileName) {
                await fileOps.deleteFile(contextState.currentPackageContext, folder, fileName);
            }
        });
    }

    if (deleteFolderItem) {
        deleteFolderItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            if (contextState.currentPackageContext && contextState.currentFolderContext) {
                await fileOps.deleteFolder(contextState.currentPackageContext, contextState.currentFolderContext);
            }
        });
    }
}
