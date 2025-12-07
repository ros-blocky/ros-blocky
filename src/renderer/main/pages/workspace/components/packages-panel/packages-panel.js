/**
 * Packages Panel Component
 * Handles package list display, context menu, and package operations
 */

let currentPackageContext = null; // Store package name for context menu
let currentNodeContext = null; // Store node name for context menu
let currentFileContext = { folder: null, fileName: null }; // Store file info for deletion
let currentFolderContext = null; // Store folder name for folder deletion

// File sections configuration (URDF, CONFIG, LAUNCH, MESHES)
const FILE_SECTIONS = {
    meshes: {
        label: 'MESHES',
        icon: '🦾',
        extensions: ['.stl', '.dae', '.STL', '.DAE'],
        fileTypeLabel: 'Mesh files',
        mode: 'import' // Import existing files
    },
    urdf: {
        label: 'URDF',
        icon: '🤖',
        extensions: ['.urdf', '.xacro'],
        fileTypeLabel: 'URDF files',
        mode: 'create' // Create new file
    },
    config: {
        label: 'CONFIG',
        icon: '⚙️',
        extensions: ['.yaml', '.json', '.yml'],
        fileTypeLabel: 'Config files',
        mode: 'create' // Create new file
    },
    launch: {
        label: 'LAUNCH',
        icon: '🚀',
        extensions: ['.py', '.launch'],
        fileTypeLabel: 'Launch files',
        mode: 'create' // Create new file
    }
};

/**
 * Initialize the packages panel
 */
export function initPackagesPanel() {
    console.log('Packages panel initialized');
    setupPanelResize();
    setupCreatePackageButton();
    setupContextMenu();

    // Refresh package list immediately (project might already be loaded)
    refreshPackageList();

    // Also refresh when project-loaded event fires
    window.electronAPI.onProjectLoaded((projectPath) => {
        console.log('Project loaded in packages panel:', projectPath);
        refreshPackageList();
    });
}

/**
 * Setup panel resize functionality
 */
function setupPanelResize() {
    const panel = document.querySelector('.project-structure-panel');
    const resizeHandle = document.querySelector('.resize-handle');
    if (!panel || !resizeHandle) return;

    let isResizing = false;
    let startX = 0;
    let startWidth = 0;

    resizeHandle.addEventListener('mousedown', (e) => {
        isResizing = true;
        startX = e.clientX;
        startWidth = panel.offsetWidth;
        document.body.style.cursor = 'ew-resize';
        document.body.style.userSelect = 'none';
        e.preventDefault();
    });

    document.addEventListener('mousemove', (e) => {
        if (!isResizing) return;
        const diff = startX - e.clientX;
        const newWidth = startWidth + diff;

        if (newWidth < 50) {
            panel.style.width = '30px';
            panel.classList.add('collapsed');
        } else if (newWidth >= 50 && newWidth <= 500) {
            panel.classList.remove('collapsed');
            panel.style.width = newWidth >= 200 ? `${newWidth}px` : '200px';
        }
    });

    document.addEventListener('mouseup', () => {
        if (isResizing) {
            isResizing = false;
            document.body.style.cursor = '';
            document.body.style.userSelect = '';
        }
    });
}

/**
 * Setup create package button
 */
function setupCreatePackageButton() {
    const createPackageBtn = document.getElementById('create-package-btn');
    const headerAddBtn = document.getElementById('header-add-package-btn');

    if (createPackageBtn) {
        createPackageBtn.classList.remove('hidden');
        createPackageBtn.addEventListener('click', createNewPackage);
    }
    if (headerAddBtn) {
        headerAddBtn.addEventListener('click', createNewPackage);
    }
}

/**
 * Setup context menu
 */
function setupContextMenu() {
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
            e.preventDefault();
            const packageItem = folderHeader.closest('.package-item');
            const fileSection = folderHeader.closest('.file-section');

            currentPackageContext = packageItem.dataset.packageName;
            currentNodeContext = null;
            currentFileContext = { folder: null, fileName: null };
            currentFolderContext = fileSection.dataset.folder;

            // Show only "Delete Folder"
            if (addPackageItem) addPackageItem.classList.add('hidden');
            if (addNodeItem) addNodeItem.classList.add('hidden');
            if (addMeshesItem) addMeshesItem.classList.add('hidden');
            if (addUrdfItem) addUrdfItem.classList.add('hidden');
            if (addConfigItem) addConfigItem.classList.add('hidden');
            if (addLaunchItem) addLaunchItem.classList.add('hidden');
            if (deletePackageItem) deletePackageItem.classList.add('hidden');
            if (deleteNodeItem) deleteNodeItem.classList.add('hidden');
            if (deleteFileItem) deleteFileItem.classList.add('hidden');
            if (deleteFolderItem) deleteFolderItem.classList.remove('hidden');
        } else if (nodesHeader) {
            // Right-clicked on nodes header
            e.preventDefault();
            const packageItem = nodesHeader.closest('.package-item');

            currentPackageContext = packageItem.dataset.packageName;
            currentNodeContext = null;
            currentFileContext = { folder: null, fileName: null };
            currentFolderContext = packageItem.dataset.packageName;

            // Show only "Delete Folder"
            if (addPackageItem) addPackageItem.classList.add('hidden');
            if (addNodeItem) addNodeItem.classList.add('hidden');
            if (addMeshesItem) addMeshesItem.classList.add('hidden');
            if (addUrdfItem) addUrdfItem.classList.add('hidden');
            if (addConfigItem) addConfigItem.classList.add('hidden');
            if (addLaunchItem) addLaunchItem.classList.add('hidden');
            if (deletePackageItem) deletePackageItem.classList.add('hidden');
            if (deleteNodeItem) deleteNodeItem.classList.add('hidden');
            if (deleteFileItem) deleteFileItem.classList.add('hidden');
            if (deleteFolderItem) deleteFolderItem.classList.remove('hidden');
        } else if (fileItem) {
            // Right-clicked on a file
            e.preventDefault();
            const packageItem = fileItem.closest('.package-item');
            const fileSection = fileItem.closest('.file-section');

            currentPackageContext = packageItem.dataset.packageName;
            currentNodeContext = null;
            currentFileContext.folder = fileSection.dataset.folder;
            currentFileContext.fileName = fileItem.querySelector('.file-name').textContent;
            currentFolderContext = null;

            // Show only "Delete File"
            if (addPackageItem) addPackageItem.classList.add('hidden');
            if (addNodeItem) addNodeItem.classList.add('hidden');
            if (addMeshesItem) addMeshesItem.classList.add('hidden');
            if (addUrdfItem) addUrdfItem.classList.add('hidden');
            if (addConfigItem) addConfigItem.classList.add('hidden');
            if (addLaunchItem) addLaunchItem.classList.add('hidden');
            if (deletePackageItem) deletePackageItem.classList.add('hidden');
            if (deleteNodeItem) deleteNodeItem.classList.add('hidden');
            if (deleteFileItem) deleteFileItem.classList.remove('hidden');
            if (deleteFolderItem) deleteFolderItem.classList.add('hidden');
        } else if (nodeItem) {
            // Right-clicked on a node
            e.preventDefault();
            const packageItem = nodeItem.closest('.package-item');
            currentPackageContext = packageItem.dataset.packageName;
            currentNodeContext = nodeItem.querySelector('.node-name').textContent;
            currentFileContext = { folder: null, fileName: null };
            currentFolderContext = null;

            // Show only "Delete Node"
            if (addPackageItem) addPackageItem.classList.add('hidden');
            if (addNodeItem) addNodeItem.classList.add('hidden');
            if (addMeshesItem) addMeshesItem.classList.add('hidden');
            if (addUrdfItem) addUrdfItem.classList.add('hidden');
            if (addConfigItem) addConfigItem.classList.add('hidden');
            if (addLaunchItem) addLaunchItem.classList.add('hidden');
            if (deletePackageItem) deletePackageItem.classList.add('hidden');
            if (deleteNodeItem) deleteNodeItem.classList.remove('hidden');
            if (deleteFileItem) deleteFileItem.classList.add('hidden');
            if (deleteFolderItem) deleteFolderItem.classList.add('hidden');
        } else if (packageHeader) {
            // Right-clicked on a package
            e.preventDefault();
            const packageItem = packageHeader.closest('.package-item');
            currentPackageContext = packageItem.dataset.packageName;
            currentNodeContext = null;
            currentFileContext = { folder: null, fileName: null };
            currentFolderContext = null;

            // Show "Add Node", "Add Meshes", and "Delete Package"
            if (addPackageItem) addPackageItem.classList.add('hidden');
            if (addNodeItem) addNodeItem.classList.remove('hidden');

            // Check if meshes folder already exists - just show for now
            if (addMeshesItem) addMeshesItem.classList.remove('hidden');

            // Check and show URDF, CONFIG, LAUNCH options
            if (addUrdfItem) addUrdfItem.classList.remove('hidden');
            if (addConfigItem) addConfigItem.classList.remove('hidden');
            if (addLaunchItem) addLaunchItem.classList.remove('hidden');

            if (deletePackageItem) deletePackageItem.classList.remove('hidden');
            if (deleteNodeItem) deleteNodeItem.classList.add('hidden');
            if (deleteFileItem) deleteFileItem.classList.add('hidden');
            if (deleteFolderItem) deleteFolderItem.classList.add('hidden');
        } else {
            // Right-clicked on empty area
            e.preventDefault();
            currentPackageContext = null;
            currentNodeContext = null;
            currentFileContext = { folder: null, fileName: null };
            currentFolderContext = null;

            // Show only "Add Package"
            if (addPackageItem) addPackageItem.classList.remove('hidden');
            if (addNodeItem) addNodeItem.classList.add('hidden');
            if (addMeshesItem) addMeshesItem.classList.add('hidden');
            if (addUrdfItem) addUrdfItem.classList.add('hidden');
            if (addConfigItem) addConfigItem.classList.add('hidden');
            if (addLaunchItem) addLaunchItem.classList.add('hidden');
            if (deletePackageItem) deletePackageItem.classList.add('hidden');
            if (deleteNodeItem) deleteNodeItem.classList.add('hidden');
            if (deleteFileItem) deleteFileItem.classList.add('hidden');
            if (deleteFolderItem) deleteFolderItem.classList.add('hidden');
        }

        // Position context menu at click location relative to the panel content
        const rect = explorerContent.getBoundingClientRect();
        contextMenu.style.left = (e.clientX - rect.left) + 'px';
        contextMenu.style.top = (e.clientY - rect.top) + 'px';
        contextMenu.classList.remove('hidden');
    });

    document.addEventListener('click', () => {
        contextMenu.classList.add('hidden');
    });

    if (addPackageItem) {
        addPackageItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            await createNewPackage();
        });
    }

    if (addNodeItem) {
        addNodeItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            if (currentPackageContext) {
                await addNodeToPackage(currentPackageContext);
            }
        });
    }

    if (addMeshesItem) {
        addMeshesItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            if (currentPackageContext) {
                await addFolderToPackage(currentPackageContext, 'meshes');
            }
        });
    }

    if (addUrdfItem) {
        addUrdfItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            if (currentPackageContext) {
                await addFilesToSection(currentPackageContext, 'urdf');
            }
        });
    }

    if (addConfigItem) {
        addConfigItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            if (currentPackageContext) {
                await addFilesToSection(currentPackageContext, 'config');
            }
        });
    }

    if (addLaunchItem) {
        addLaunchItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            if (currentPackageContext) {
                await addFilesToSection(currentPackageContext, 'launch');
            }
        });
    }

    if (deletePackageItem) {
        deletePackageItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            if (currentPackageContext) {
                await deletePackage(currentPackageContext);
            }
        });
    }

    if (deleteNodeItem) {
        deleteNodeItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            if (currentPackageContext && currentNodeContext) {
                await deleteNode(currentPackageContext, currentNodeContext);
            }
        });
    }

    if (deleteFileItem) {
        deleteFileItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            if (currentPackageContext && currentFileContext.folder && currentFileContext.fileName) {
                await deleteFile(currentPackageContext, currentFileContext.folder, currentFileContext.fileName);
            }
        });
    }

    if (deleteFolderItem) {
        deleteFolderItem.addEventListener('click', async (e) => {
            e.stopPropagation();
            contextMenu.classList.add('hidden');
            if (currentPackageContext && currentFolderContext) {
                await deleteFolder(currentPackageContext, currentFolderContext);
            }
        });
    }
}

/**
 * Create new package
 */
async function createNewPackage() {
    try {
        const packageName = await window.electronAPI.promptPackageName();
        if (packageName) {
            const result = await window.electronAPI.createPackage(packageName);
            if (result.success) {
                await refreshPackageList();
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
 * Add node to package
 */
async function addNodeToPackage(packageName) {
    try {
        const nodeName = await window.electronAPI.promptNodeName(packageName);
        if (nodeName) {
            const result = await window.electronAPI.createNode(packageName, nodeName);
            if (result.success) {
                console.log('Node created:', result.nodePath);
                await refreshPackageList();
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
 * Add a folder to a package
 * @param {string} packageName 
 * @param {string} folderName 
 */
async function addFolderToPackage(packageName, folderName) {
    try {
        const result = await window.electronAPI.createPackageFolder(packageName, folderName);
        if (result.success) {
            console.log(`Folder ${folderName} created:`, result.folderPath);
            // Refresh the package list to show the new folder
            await refreshPackageList();
        } else {
            alert('Error: ' + result.message);
        }
    } catch (error) {
        console.error(`Error creating folder ${folderName}:`, error);
        alert(`Error creating folder ${folderName}: ` + error.message);
    }
}

/**
 * Add mesh files to package
 * @param {string} packageName 
 * @returns {Promise<boolean>} true if files were added
 */
async function addMeshFilesToPackage(packageName) {
    try {
        const result = await window.electronAPI.selectMeshFiles(packageName);

        // Check if duplicates were found
        if (result.message === 'duplicates_found') {
            const confirmed = await window.electronAPI.confirm({
                message: `The selected files already exist. Do you want to overwrite them?`,
                confirmButtonText: 'Overwrite',
                cancelButtonText: 'Cancel'
            });

            if (confirmed) {
                // User chose to overwrite
                const copyResult = await window.electronAPI.copyMeshFilesForce(packageName, result.selectedFiles);
                if (copyResult.success) {
                    console.log(`${copyResult.filesCopied} mesh file(s) copied`);
                    return true;
                } else {
                    alert('Error: ' + copyResult.message);
                    return false;
                }
            } else {
                return false;
            }
        } else if (result.success) {
            console.log(`${result.filesCopied} mesh file(s) copied`);
            return true;
        } else {
            if (result.message !== 'No files selected') {
                alert('Error: ' + result.message);
            }
            return false;
        }
    } catch (error) {
        console.error('Error adding mesh files:', error);
        alert('Error adding mesh files: ' + error.message);
        return false;
    }
}


/**
 * Delete a package
 */
async function deletePackage(packageName) {
    try {
        const result = await window.electronAPI.deletePackage(packageName);
        if (result.success) {
            await refreshPackageList();
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

/**
 * Delete a node
 */
async function deleteNode(packageName, nodeName) {
    try {
        const result = await window.electronAPI.deleteNode(packageName, nodeName);
        if (result.success) {
            console.log('Node deleted successfully');
            await refreshPackageList();
        } else {
            if (!result.message.includes('cancelled')) {
                alert('Error: ' + result.message);
            }
        }
    } catch (error) {
        console.error('Error deleting node:', error);
        alert('Error deleting node: ' + error.message);
    }
}

/**
 * Delete a file from a section folder (generic for meshes, urdf, config, launch)
 */
async function deleteFile(packageName, folderName, fileName) {
    try {
        // Confirm deletion
        const confirmed = await window.electronAPI.confirm({
            message: `Delete ${fileName} from ${folderName}?`,
            confirmButtonText: 'Delete',
            cancelButtonText: 'Cancel'
        });

        if (!confirmed) return;

        const result = await window.electronAPI.deleteFile(packageName, folderName, fileName);
        if (result.success) {
            console.log(`File ${fileName} deleted successfully`);
            await refreshPackageList();
        } else {
            alert('Error: ' + result.message);
        }
    } catch (error) {
        console.error('Error deleting file:', error);
        alert('Error deleting file: ' + error.message);
    }
}

/**
 * Delete an entire folder (meshes, urdf, config, launch, or package nodes folder)
 */
async function deleteFolder(packageName, folderName) {
    try {
        // Confirm deletion with folder name
        const confirmed = await window.electronAPI.confirm({
            message: `Delete the entire ${folderName} folder and all its contents?`,
            confirmButtonText: 'Delete',
            cancelButtonText: 'Cancel'
        });

        if (!confirmed) return;

        const result = await window.electronAPI.deleteFolder(packageName, folderName);
        if (result.success) {
            console.log(`Folder ${folderName} deleted successfully`);
            await refreshPackageList();
        } else {
            alert('Error: ' + result.message);
        }
    } catch (error) {
        console.error('Error deleting folder:', error);
        alert('Error deleting folder: ' + error.message);
    }
}

/**
 * Refresh the package list
 */
async function refreshPackageList() {
    console.log('refreshPackageList called');
    try {
        const packages = await window.electronAPI.listPackages();
        console.log('Packages received:', packages);

        const emptyStateContainer = document.getElementById('empty-state-container');
        const packagesList = document.getElementById('packages-list');

        if (packages && packages.length > 0) {
            console.log('Showing packages list with', packages.length, 'packages');
            // Hide empty state, show packages list
            emptyStateContainer.classList.add('hidden');
            packagesList.classList.remove('hidden');
            packagesList.innerHTML = '';

            for (const packageName of packages) {
                console.log('Creating element for package:', packageName);
                await createPackageElement(packageName, packagesList);
            }
        } else {
            console.log('No packages found, showing empty state');
            // Show empty state, hide packages list
            emptyStateContainer.classList.remove('hidden');
            packagesList.classList.add('hidden');
        }
    } catch (error) {
        console.error('Error refreshing package list:', error);
    }
}

/**
 * Create package element in the tree
 */
async function createPackageElement(packageName, packagesList) {
    const packageItem = document.createElement('div');
    packageItem.className = 'package-item';
    packageItem.dataset.packageName = packageName;

    // Get nodes for this package
    const nodes = await window.electronAPI.listPackageNodes(packageName);

    // Full package display with collapse arrow and sections
    packageItem.innerHTML = `
        <div class="package-header">
            <span class="collapse-arrow ${nodes.length > 0 ? '' : 'hidden'}">▶</span>
            <span class="package-icon">📦</span>
            <span class="package-name">${packageName}</span>
        </div>
        <div class="package-content hidden">
            <div class="nodes-section">
                <div class="nodes-header">
                    <span class="section-collapse-arrow">▶</span>
                    <span class="section-icon">⚙️</span>
                    <span class="section-name">NODES</span>
                    <button class="section-add-btn" title="Add Node">+</button>
                </div>
                <div class="nodes-list hidden"></div>
            </div>
        </div>
    `;

    // Setup collapse behavior
    const collapseArrow = packageItem.querySelector('.collapse-arrow');
    const packageContent = packageItem.querySelector('.package-content');

    if (collapseArrow && !collapseArrow.classList.contains('hidden')) {
        collapseArrow.addEventListener('click', (e) => {
            e.stopPropagation();
            collapseArrow.classList.toggle('expanded');
            packageContent.classList.toggle('hidden');
        });
    }

    // Setup nodes section collapse
    const nodesHeader = packageItem.querySelector('.nodes-header');
    const sectionArrow = packageItem.querySelector('.section-collapse-arrow');
    const nodesList = packageItem.querySelector('.nodes-list');

    nodesHeader.addEventListener('click', (e) => {
        if (e.target.classList.contains('section-add-btn')) return;
        e.stopPropagation();
        sectionArrow.classList.toggle('expanded');
        nodesList.classList.toggle('hidden');
    });

    // Setup add node button
    const addNodeBtn = packageItem.querySelector('.section-add-btn');
    addNodeBtn.addEventListener('click', async (e) => {
        e.stopPropagation();
        await addNodeToPackage(packageName);
    });

    // Populate nodes
    if (nodes.length > 0) {
        nodes.forEach(nodeName => {
            const nodeItem = document.createElement('div');
            nodeItem.className = 'node-item';
            nodeItem.dataset.nodeName = nodeName;
            nodeItem.innerHTML = `
                <span class="node-icon">🐍</span>
                <span class="node-name">${nodeName}</span>
            `;
            nodesList.appendChild(nodeItem);
        });
        // Auto-expand if has nodes
        collapseArrow.classList.add('expanded');
        packageContent.classList.remove('hidden');
        sectionArrow.classList.add('expanded');
        nodesList.classList.remove('hidden');
    }

    packagesList.appendChild(packageItem);
}

/**
 * Setup collapse/expand for package with nodes
 */
function setupPackageCollapse(packageItem, packageName, nodes) {
    const packageHeader = packageItem.querySelector('.package-header');
    const packageContent = packageItem.querySelector('.package-content');
    const collapseArrow = packageItem.querySelector('.collapse-arrow');
    const nodesHeader = packageItem.querySelector('.nodes-header');
    const nodesList = packageItem.querySelector('.nodes-list');

    let nodesLoaded = false;
    const fileSectionsLoaded = {}; // Track which file sections have been loaded

    // Package header click - expand/collapse entire package
    packageHeader.addEventListener('click', async (e) => {
        if (e.target.closest('.context-menu')) return;
        const isExpanded = !packageContent.classList.contains('hidden');

        if (isExpanded) {
            packageContent.classList.add('hidden');
            collapseArrow.textContent = '▸';
        } else {
            packageContent.classList.remove('hidden');
            collapseArrow.textContent = '▾';

            // Load nodes if not loaded
            if (nodes && nodes.length > 0 && !nodesLoaded) {
                loadNodes(nodesList, nodes);
                nodesLoaded = true;
            }
        }
    });

    // Nodes section collapse
    if (nodesHeader) {
        nodesHeader.addEventListener('click', (e) => {
            // Don't collapse if clicking the add button
            if (e.target.classList.contains('section-add-btn')) {
                e.stopPropagation();
                const pkgName = e.target.dataset.package;
                addNodeToPackage(pkgName);
                return;
            }

            e.stopPropagation();
            const collapseArrowSmall = nodesHeader.querySelector('.collapse-arrow-small');
            const isExpanded = !nodesList.classList.contains('hidden');

            if (isExpanded) {
                nodesList.classList.add('hidden');
                collapseArrowSmall.textContent = '▸';
            } else {
                nodesList.classList.remove('hidden');
                collapseArrowSmall.textContent = '▾';
            }
        });
    }

    // Generic file sections collapse (meshes, urdf, config, launch)
    const fileSections = packageItem.querySelectorAll('.file-section');
    fileSections.forEach(section => {
        const folderName = section.dataset.folder;
        const fileHeader = section.querySelector('.file-header');
        const fileList = section.querySelector('.file-list');

        if (fileHeader && fileList) {
            fileHeader.addEventListener('click', async (e) => {
                // Don't collapse if clicking the add button
                if (e.target.classList.contains('section-add-btn')) {
                    e.stopPropagation();
                    const pkgName = e.target.dataset.package;
                    const folder = e.target.dataset.folder;
                    await addFilesToSection(pkgName, folder);
                    return;
                }

                e.stopPropagation();
                const collapseArrowSmall = fileHeader.querySelector('.collapse-arrow-small');
                const isExpanded = !fileList.classList.contains('hidden');

                if (isExpanded) {
                    fileList.classList.add('hidden');
                    collapseArrowSmall.textContent = '▸';
                } else {
                    fileList.classList.remove('hidden');
                    collapseArrowSmall.textContent = '▾';

                    // Load files if not loaded
                    if (!fileSectionsLoaded[folderName]) {
                        await loadSectionFiles(fileList, packageName, folderName);
                        fileSectionsLoaded[folderName] = true;
                    }
                }
            });
        }
    });
}

/**
 * Load nodes into the list
 */
function loadNodes(nodesList, nodes) {
    nodesList.innerHTML = '';
    nodes.forEach(nodeName => {
        const nodeItem = document.createElement('div');
        nodeItem.className = 'node-item';
        const displayName = nodeName.replace(/\.py$/, '');
        nodeItem.innerHTML = `
            <span class="node-icon">◉</span>
            <span class="node-name">${displayName}</span>
        `;
        nodesList.appendChild(nodeItem);
    });
}

/**
 * Load mesh files into the list
 */
/**
 * Generic function to load files in a file section (meshes, urdf, config, launch)
 */
async function loadSectionFiles(fileListElement, packageName, folderName) {
    try {
        const config = FILE_SECTIONS[folderName];
        if (!config) {
            console.error(`No configuration found for folder: ${folderName}`);
            return;
        }

        const files = await window.electronAPI.listFolderFiles(packageName, folderName, config.extensions);
        fileListElement.innerHTML = '';

        if (files && files.length > 0) {
            files.forEach(fileName => {
                const fileItem = document.createElement('div');
                fileItem.className = 'file-item';
                fileItem.innerHTML = `
                    <span class="file-icon">📄</span>
                    <span class="file-name">${fileName}</span>
                `;
                fileListElement.appendChild(fileItem);
            });
        } else {
            fileListElement.innerHTML = `<div class="no-files">No ${folderName} files</div>`;
        }
    } catch (error) {
        console.error(`Error loading ${folderName} files:`, error);
        fileListElement.innerHTML = '<div class="no-files">Error loading files</div>';
    }
}

/**
 * Generic function to add files to a file section (meshes, urdf, config, launch)
 */
async function addFilesToSection(packageName, folderName) {
    try {
        const config = FILE_SECTIONS[folderName];
        if (!config) {
            console.error(`No configuration found for folder: ${folderName}`);
            return false;
        }

        // Check mode: create new file or import existing files
        if (config.mode === 'create') {
            // Create new file with name prompt (like nodes)
            const fileName = await window.electronAPI.promptFileName(
                packageName,
                `Create ${config.label} File`,
                `Enter ${config.label.toLowerCase()} file name (without extension)`
            );
            if (!fileName) return false;

            // Determine file extension based on folder type
            let extension = config.extensions[0]; // Default to first extension (.yaml for config)

            const fullFileName = fileName + extension;

            // Call backend to create the file
            const result = await window.electronAPI.createSectionFile(packageName, folderName, fullFileName);

            if (result.success) {
                console.log(`Created ${fullFileName} in ${folderName}`);
                await refreshPackageList();
                return true;
            } else {
                alert('Error: ' + result.message);
                return false;
            }
        } else {
            // Import existing files (for meshes)
            const result = await window.electronAPI.selectFolderFiles(
                packageName,
                folderName,
                config.extensions,
                config.fileTypeLabel
            );

            // Check if duplicates were found
            if (result.message === 'duplicates_found') {
                const confirmed = await window.electronAPI.confirm({
                    message: `The selected files already exist. Do you want to overwrite them?`,
                    confirmButtonText: 'Overwrite',
                    cancelButtonText: 'Cancel'
                });

                if (confirmed) {
                    const copyResult = await window.electronAPI.copyFolderFilesForce(
                        packageName,
                        folderName,
                        result.selectedFiles
                    );
                    if (copyResult.success) {
                        console.log(`${copyResult.filesCopied} ${folderName} file(s) copied`);
                        await refreshPackageList();
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
                await refreshPackageList();
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
 * Load mesh files (legacy, uses generic function)
 */
async function loadMeshFiles(meshesList, packageName) {
    await loadSectionFiles(meshesList, packageName, 'meshes');
}

/**
 * Export refresh function
 */
export { refreshPackageList };
