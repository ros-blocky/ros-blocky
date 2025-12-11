/**
 * Editor Component - VS Code Style File Tabs
 * Manages file tabs, breadcrumbs, and the editing workspace with Blockly
 */

import { initUrdfEditor, getUrdfCategories } from '../blocks/editors/urdf/urdf-editor.js';

let initialized = false;
let openFiles = []; // Array of {fileName, filePath, packageName, type}
let activeFile = null;
let mainWorkspace = null; // Main Blockly workspace

// File type icons (using Unicode for simplicity)
const FILE_ICONS = {
    urdf: '🤖',
    xacro: '🤖',
    py: '🐍',
    launch: '🚀',
    yaml: '⚙️',
    xml: '📄',
    default: '📄'
};

/**
 * Initialize the editor component
 */
export function initEditor() {
    if (initialized) {
        console.log('[Editor] Already initialized');
        return;
    }

    console.log('[Editor] Initializing...');

    // Listen for file selection events from packages panel
    window.addEventListener('fileSelected', handleFileOpen);

    initialized = true;
    console.log('[Editor] Component initialized');
}

/**
 * Handle file open event
 */
function handleFileOpen(event) {
    const { type, fileName, packageName } = event.detail;
    console.log('[Editor] Opening file:', fileName, 'type:', type);

    // Build file info
    const fileInfo = {
        fileName: fileName,
        filePath: `${packageName}/${fileName}`,
        packageName: packageName,
        type: type
    };

    // Check if file is already open
    const existingIndex = openFiles.findIndex(f =>
        f.fileName === fileName && f.packageName === packageName
    );

    if (existingIndex >= 0) {
        // File already open, just activate it
        setActiveFile(existingIndex);
    } else {
        // Add new file tab
        openFiles.push(fileInfo);
        setActiveFile(openFiles.length - 1);
    }

    renderTabs();
    renderBreadcrumb();
    showEditorForFile(fileInfo);
}

/**
 * Set active file by index
 */
function setActiveFile(index) {
    if (index >= 0 && index < openFiles.length) {
        activeFile = openFiles[index];
    }
}

/**
 * Render the file tabs
 */
function renderTabs() {
    const tabsContainer = document.getElementById('editor-tabs');
    if (!tabsContainer) return;

    if (openFiles.length === 0) {
        tabsContainer.innerHTML = '';
        return;
    }

    tabsContainer.innerHTML = openFiles.map((file, index) => {
        const isActive = activeFile &&
            file.fileName === activeFile.fileName &&
            file.packageName === activeFile.packageName;

        const icon = getFileIcon(file.fileName);

        return `
            <div class="editor-tab ${isActive ? 'active' : ''}" 
                 data-index="${index}">
                <span class="tab-icon">${icon}</span>
                <span class="tab-name">${file.fileName}</span>
                <span class="tab-close" data-index="${index}">×</span>
            </div>
        `;
    }).join('');

    // Attach click handlers
    tabsContainer.querySelectorAll('.editor-tab').forEach(tab => {
        tab.addEventListener('click', (e) => {
            // Don't activate if clicking close button
            if (e.target.classList.contains('tab-close')) return;

            const index = parseInt(tab.dataset.index);
            setActiveFile(index);
            renderTabs();
            renderBreadcrumb();
            showEditorForFile(openFiles[index]);
        });
    });

    // Attach close handlers
    tabsContainer.querySelectorAll('.tab-close').forEach(closeBtn => {
        closeBtn.addEventListener('click', (e) => {
            e.stopPropagation();
            const index = parseInt(closeBtn.dataset.index);
            closeFile(index);
        });
    });
}

/**
 * Close a file tab
 */
function closeFile(index) {
    const wasActive = activeFile &&
        openFiles[index].fileName === activeFile.fileName &&
        openFiles[index].packageName === activeFile.packageName;

    openFiles.splice(index, 1);

    if (wasActive) {
        if (openFiles.length > 0) {
            // Activate the previous tab or the first one
            const newIndex = Math.min(index, openFiles.length - 1);
            setActiveFile(newIndex);
            showEditorForFile(openFiles[newIndex]);
        } else {
            activeFile = null;
            showPlaceholder();
        }
    }

    renderTabs();
    renderBreadcrumb();
}

/**
 * Render the breadcrumb bar
 */
function renderBreadcrumb() {
    const breadcrumbContainer = document.getElementById('editor-breadcrumb');
    if (!breadcrumbContainer) return;

    if (!activeFile) {
        breadcrumbContainer.innerHTML = '';
        return;
    }

    // Build breadcrumb: package > folder > file
    const parts = [];

    // Add package name
    parts.push({
        icon: '📦',
        label: activeFile.packageName
    });

    // Add file type folder
    const folderNames = {
        urdf: 'urdf',
        node: 'scripts',
        config: 'config',
        launch: 'launch'
    };
    const folder = folderNames[activeFile.type] || activeFile.type;
    parts.push({
        icon: '📁',
        label: folder
    });

    // Add file name
    const icon = getFileIcon(activeFile.fileName);
    parts.push({
        icon: icon,
        label: activeFile.fileName
    });

    breadcrumbContainer.innerHTML = parts.map((part, index) => {
        const separator = index < parts.length - 1
            ? '<span class="breadcrumb-separator">›</span>'
            : '';
        return `
            <span class="breadcrumb-item">
                <span class="breadcrumb-icon">${part.icon}</span>
                ${part.label}
            </span>
            ${separator}
        `;
    }).join('');
}

/**
 * Get file icon based on filename
 */
function getFileIcon(fileName) {
    const ext = fileName.split('.').pop().toLowerCase();
    return FILE_ICONS[ext] || FILE_ICONS.default;
}

/**
 * Show editor for a specific file
 */
function showEditorForFile(file) {
    const placeholder = document.getElementById('editor-placeholder');
    const workspaceContainer = document.getElementById('editor-workspace');

    if (placeholder) {
        placeholder.style.display = 'none';
    }

    console.log('[Editor] Showing editor for:', file.fileName, 'type:', file.type);

    // For URDF files, create Blockly workspace
    if (file.type === 'urdf' || file.fileName.endsWith('.urdf') || file.fileName.endsWith('.xacro')) {
        createBlocklyWorkspace(workspaceContainer);
    }

    // Dispatch event for blocks component to update
    window.dispatchEvent(new CustomEvent('editorFileChanged', {
        detail: file
    }));
}

/**
 * Create Blockly workspace with toolbox
 * @param {HTMLElement} container - Container element for workspace
 */
function createBlocklyWorkspace(container) {
    // Dispose existing workspace if any
    if (mainWorkspace) {
        mainWorkspace.dispose();
        mainWorkspace = null;
    }

    // Check if Blockly is available
    if (typeof Blockly === 'undefined') {
        console.error('[Editor] Blockly not loaded');
        return;
    }

    // Initialize URDF editor (registers blocks)
    initUrdfEditor(Blockly);

    // Generate toolbox from URDF categories
    const toolboxXml = generateToolboxXml();

    // Create or get workspace div
    let workspaceDiv = container.querySelector('#blockly-main-workspace');
    if (!workspaceDiv) {
        workspaceDiv = document.createElement('div');
        workspaceDiv.id = 'blockly-main-workspace';
        workspaceDiv.style.width = '100%';
        workspaceDiv.style.height = '100%';
        workspaceDiv.style.position = 'absolute';
        workspaceDiv.style.top = '0';
        workspaceDiv.style.left = '0';
        container.appendChild(workspaceDiv);
    }

    // Inject Blockly workspace
    mainWorkspace = Blockly.inject(workspaceDiv, {
        toolbox: toolboxXml,
        renderer: 'zelos',
        grid: {
            spacing: 20,
            length: 3,
            colour: '#ccc',
            snap: true
        },
        zoom: {
            controls: true,
            wheel: true,
            startScale: 0.9,
            maxScale: 3,
            minScale: 0.3,
            scaleSpeed: 1.2
        },
        move: {
            scrollbars: true,
            drag: true,
            wheel: true
        },
        trashcan: true,
        sounds: false
    });

    // Handle window resize
    window.addEventListener('resize', () => {
        if (mainWorkspace) {
            Blockly.svgResize(mainWorkspace);
        }
    });

    // Initial resize
    Blockly.svgResize(mainWorkspace);

    console.log('[Editor] Blockly workspace created');
}

/**
 * Generate toolbox XML from URDF categories
 * @returns {string} Toolbox XML string
 */
function generateToolboxXml() {
    const categories = getUrdfCategories();

    const categoriesXml = categories.map(category => {
        const blocksXml = category.blocks.map(blockType =>
            `<block type="${blockType}"></block>`
        ).join('');

        return `<category name="${category.label}" colour="${category.color}">
            ${blocksXml}
        </category>`;
    }).join('');

    return `<xml id="toolbox" style="display: none">
        ${categoriesXml}
    </xml>`;
}

/**
 * Show placeholder when no file is open
 */
function showPlaceholder() {
    // Dispose Blockly workspace
    if (mainWorkspace) {
        mainWorkspace.dispose();
        mainWorkspace = null;
    }

    // Remove workspace div
    const workspaceDiv = document.getElementById('blockly-main-workspace');
    if (workspaceDiv) {
        workspaceDiv.remove();
    }

    const placeholder = document.getElementById('editor-placeholder');
    if (placeholder) {
        placeholder.style.display = 'flex';
    }
}

/**
 * Reset the editor component
 */
export function resetEditor() {
    // Dispose Blockly workspace
    if (mainWorkspace) {
        mainWorkspace.dispose();
        mainWorkspace = null;
    }

    openFiles = [];
    activeFile = null;
    initialized = false;
    window.removeEventListener('fileSelected', handleFileOpen);
    console.log('[Editor] Reset');
}
