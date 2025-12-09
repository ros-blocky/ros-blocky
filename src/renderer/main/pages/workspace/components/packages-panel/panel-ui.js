/**
 * Packages Panel - UI
 * UI rendering, refreshPackageList, and collapse logic
 */

import { FILE_SECTIONS } from './config.js';
import * as api from './api.js';
import * as fileOps from './file-operations.js';

// ============================================
// Panel Resize
// ============================================

/**
 * Setup panel resize functionality
 */
export function setupPanelResize() {
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

// ============================================
// Create Package Button
// ============================================

/**
 * Setup create package button
 */
export function setupCreatePackageButton() {
    const createPackageBtn = document.getElementById('create-package-btn');
    const headerAddBtn = document.getElementById('header-add-package-btn');

    if (createPackageBtn) {
        createPackageBtn.classList.remove('hidden');
        createPackageBtn.addEventListener('click', fileOps.createNewPackage);
    }
    if (headerAddBtn) {
        headerAddBtn.addEventListener('click', fileOps.createNewPackage);
    }
}

// ============================================
// Package List Rendering
// ============================================

/**
 * Refresh the package list
 */
export async function refreshPackageList() {
    console.log('refreshPackageList called');
    try {
        const packages = await api.listPackages();
        console.log('Packages received:', packages);

        const emptyStateContainer = document.getElementById('empty-state-container');
        const packagesList = document.getElementById('packages-list');

        if (packages && packages.length > 0) {
            console.log('Showing packages list with', packages.length, 'packages');
            emptyStateContainer.classList.add('hidden');
            packagesList.classList.remove('hidden');
            packagesList.innerHTML = '';

            for (const packageName of packages) {
                console.log('Creating element for package:', packageName);
                await createPackageElement(packageName, packagesList);
            }
        } else {
            console.log('No packages found, showing empty state');
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

    // Get all items for this package
    const [nodes, urdfs, configs, launches, meshes] = await Promise.all([
        api.listPackageNodes(packageName),
        api.listPackageUrdfs(packageName),
        api.listPackageConfigs(packageName),
        api.listPackageLaunches(packageName),
        api.listPackageMeshes(packageName)
    ]);

    // Check if package has any content
    const hasContent = nodes.length > 0 || urdfs.length > 0 || configs.length > 0 || launches.length > 0 || meshes.length > 0;

    // Package HTML template
    packageItem.innerHTML = `
        <div class="package-header">
            <img class="collapse-arrow ${hasContent ? '' : 'hidden'}" src="assets/icons/chevron-right.svg" alt="">
            <img class="package-icon" src="assets/icons/package.svg" alt="package">
            <span class="package-name">${packageName}</span>
        </div>
        <div class="package-content hidden">
            <!-- NODES Section -->
            <div class="nodes-section" data-section="nodes">
                <div class="nodes-header">
                    <img class="section-collapse-arrow" src="assets/icons/chevron-right.svg" alt="">
                    <img class="section-icon" src="assets/icons/nodes.svg" alt="nodes">
                    <span class="section-name">NODES</span>
                    <button class="section-add-btn" data-type="node" title="Add Node">+</button>
                </div>
                <div class="nodes-list hidden"></div>
            </div>
            <!-- MESHES Section -->
            <div class="file-section" data-section="meshes" data-folder="meshes">
                <div class="file-header">
                    <img class="section-collapse-arrow" src="assets/icons/chevron-right.svg" alt="">
                    <img class="section-icon" src="assets/icons/meshes.svg" alt="meshes">
                    <span class="section-name">MESHES</span>
                    <button class="section-add-btn" data-type="meshes" title="Add Meshes">+</button>
                </div>
                <div class="file-list hidden"></div>
            </div>
            <!-- URDF Section -->
            <div class="file-section" data-section="urdf" data-folder="urdf">
                <div class="file-header">
                    <img class="section-collapse-arrow" src="assets/icons/chevron-right.svg" alt="">
                    <img class="section-icon" src="assets/icons/urdf.png" alt="urdf">
                    <span class="section-name">URDF</span>
                    <button class="section-add-btn" data-type="urdf" title="Add URDF">+</button>
                </div>
                <div class="file-list hidden"></div>
            </div>
            <!-- CONFIG Section -->
            <div class="file-section" data-section="config" data-folder="config">
                <div class="file-header">
                    <img class="section-collapse-arrow" src="assets/icons/chevron-right.svg" alt="">
                    <img class="section-icon" src="assets/icons/config.svg" alt="config">
                    <span class="section-name">CONFIG</span>
                    <button class="section-add-btn" data-type="config" title="Add Config">+</button>
                </div>
                <div class="file-list hidden"></div>
            </div>
            <!-- LAUNCH Section -->
            <div class="file-section" data-section="launch" data-folder="launch">
                <div class="file-header">
                    <img class="section-collapse-arrow" src="assets/icons/chevron-right.svg" alt="">
                    <img class="section-icon" src="assets/icons/launch.svg" alt="launch">
                    <span class="section-name">LAUNCH</span>
                    <button class="section-add-btn" data-type="launch" title="Add Launch">+</button>
                </div>
                <div class="file-list hidden"></div>
            </div>
        </div>
    `;

    // Setup package collapse behavior
    setupPackageCollapseBehavior(packageItem);

    // Setup nodes section
    setupNodesSection(packageItem, packageName, nodes);

    // Setup file sections (meshes, urdf, config, launch)
    const fileSections = [
        { type: 'meshes', files: meshes },
        { type: 'urdf', files: urdfs },
        { type: 'config', files: configs },
        { type: 'launch', files: launches }
    ];

    fileSections.forEach(({ type, files }) => {
        setupFileSection(packageItem, packageName, type, files);
    });

    // Auto-expand package if it has content
    if (hasContent) {
        autoExpandPackage(packageItem, nodes, fileSections);
    }

    packagesList.appendChild(packageItem);
}

/**
 * Setup package collapse behavior
 */
function setupPackageCollapseBehavior(packageItem) {
    const collapseArrow = packageItem.querySelector('.collapse-arrow');
    const packageContent = packageItem.querySelector('.package-content');

    if (collapseArrow && !collapseArrow.classList.contains('hidden')) {
        collapseArrow.addEventListener('click', (e) => {
            e.stopPropagation();
            collapseArrow.classList.toggle('expanded');
            packageContent.classList.toggle('hidden');
        });
    }
}

/**
 * Setup nodes section
 */
function setupNodesSection(packageItem, packageName, nodes) {
    const nodesSection = packageItem.querySelector('[data-section="nodes"]');
    const nodesHeader = nodesSection.querySelector('.nodes-header');
    const nodesSectionArrow = nodesSection.querySelector('.section-collapse-arrow');
    const nodesList = nodesSection.querySelector('.nodes-list');

    // Section collapse
    nodesHeader.addEventListener('click', (e) => {
        if (e.target.classList.contains('section-add-btn')) return;
        e.stopPropagation();
        nodesSectionArrow.classList.toggle('expanded');
        nodesList.classList.toggle('hidden');
    });

    // Add node button
    const addNodeBtn = nodesSection.querySelector('.section-add-btn');
    addNodeBtn.addEventListener('click', async (e) => {
        e.stopPropagation();
        await fileOps.addNodeToPackage(packageName);
    });

    // Populate nodes
    if (nodes.length > 0) {
        nodes.forEach(nodeName => {
            const nodeItem = document.createElement('div');
            nodeItem.className = 'node-item';
            nodeItem.dataset.nodeName = nodeName;
            nodeItem.innerHTML = `
                <img class="node-icon" src="assets/icons/file-icon.svg" alt="file">
                <span class="node-name">${nodeName}</span>
            `;
            nodesList.appendChild(nodeItem);
        });
    } else {
        nodesSection.classList.add('hidden');
    }
}

/**
 * Setup file section (meshes, urdf, config, launch)
 */
function setupFileSection(packageItem, packageName, type, files) {
    const section = packageItem.querySelector(`[data-section="${type}"]`);
    const header = section.querySelector('.file-header');
    const sectionArrow = section.querySelector('.section-collapse-arrow');
    const fileList = section.querySelector('.file-list');
    const fileIconPath = 'assets/icons/file-icon.svg';

    // Section collapse
    header.addEventListener('click', (e) => {
        if (e.target.classList.contains('section-add-btn')) return;
        e.stopPropagation();
        sectionArrow.classList.toggle('expanded');
        fileList.classList.toggle('hidden');
    });

    // Add button
    const addBtn = section.querySelector('.section-add-btn');
    addBtn.addEventListener('click', async (e) => {
        e.stopPropagation();
        if (type === 'meshes') {
            await fileOps.importMeshes(packageName);
        } else {
            await fileOps.addFilesToSection(packageName, type);
        }
    });

    // Populate files
    if (files.length > 0) {
        files.forEach(fileName => {
            const fileItem = document.createElement('div');
            fileItem.className = 'file-item';
            fileItem.dataset.fileName = fileName;
            fileItem.dataset.fileType = type;
            fileItem.dataset.packageName = packageName;
            fileItem.innerHTML = `
                <img class="file-icon" src="${fileIconPath}" alt="file">
                <span class="file-name">${fileName}</span>
            `;

            // Click handler to select file
            fileItem.addEventListener('click', (e) => {
                e.stopPropagation();

                // Remove selection from all file items
                document.querySelectorAll('.file-item.selected').forEach(item => {
                    item.classList.remove('selected');
                });

                // Add selection to this item
                fileItem.classList.add('selected');

                // Dispatch window event for blocks component to listen to
                window.dispatchEvent(new CustomEvent('fileSelected', {
                    detail: { type: type, fileName: fileName, packageName: packageName }
                }));

                console.log('[PackagesPanel] File selected:', type, fileName);
            });

            fileList.appendChild(fileItem);
        });
    } else {
        section.classList.add('hidden');
    }
}

/**
 * Auto-expand package and sections with content
 */
function autoExpandPackage(packageItem, nodes, fileSections) {
    const collapseArrow = packageItem.querySelector('.collapse-arrow');
    const packageContent = packageItem.querySelector('.package-content');

    collapseArrow.classList.add('expanded');
    packageContent.classList.remove('hidden');

    // Auto-expand nodes section if it has items
    if (nodes.length > 0) {
        const nodesSection = packageItem.querySelector('[data-section="nodes"]');
        const nodesSectionArrow = nodesSection.querySelector('.section-collapse-arrow');
        const nodesList = nodesSection.querySelector('.nodes-list');
        nodesSectionArrow.classList.add('expanded');
        nodesList.classList.remove('hidden');
    }

    // Auto-expand file sections with items
    fileSections.forEach(({ type, files }) => {
        if (files.length > 0) {
            const section = packageItem.querySelector(`[data-section="${type}"]`);
            const arrow = section.querySelector('.section-collapse-arrow');
            const list = section.querySelector('.file-list');
            arrow.classList.add('expanded');
            list.classList.remove('hidden');
        }
    });
}

// ============================================
// Legacy Functions (kept for compatibility)
// ============================================

/**
 * Load nodes into the list
 */
export function loadNodes(nodesList, nodes) {
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
 * Load files in a section
 */
export async function loadSectionFiles(fileListElement, packageName, folderName) {
    try {
        const config = FILE_SECTIONS[folderName];
        if (!config) {
            console.error(`No configuration found for folder: ${folderName}`);
            return;
        }

        const files = await api.listFolderFiles(packageName, folderName, config.extensions);
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
