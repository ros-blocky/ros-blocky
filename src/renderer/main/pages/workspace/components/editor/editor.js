/**
 * Editor Component - VS Code Style File Tabs
 * Manages file tabs, breadcrumbs, and the editing workspace
 */

let initialized = false;
let openFiles = []; // Array of {fileName, filePath, packageName, type}
let activeFile = null;

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
    const workspace = document.getElementById('editor-workspace');

    if (placeholder) {
        placeholder.style.display = 'none';
    }

    // For now, just log - Blockly integration comes later
    console.log('[Editor] Showing editor for:', file.fileName, 'type:', file.type);

    // Dispatch event for blocks component to update
    window.dispatchEvent(new CustomEvent('editorFileChanged', {
        detail: file
    }));
}

/**
 * Show placeholder when no file is open
 */
function showPlaceholder() {
    const placeholder = document.getElementById('editor-placeholder');
    if (placeholder) {
        placeholder.style.display = 'flex';
    }
}

/**
 * Reset the editor component
 */
export function resetEditor() {
    openFiles = [];
    activeFile = null;
    initialized = false;
    window.removeEventListener('fileSelected', handleFileOpen);
    console.log('[Editor] Reset');
}
