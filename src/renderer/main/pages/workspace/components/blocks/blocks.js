/**
 * Blocks Component
 * Main orchestrator for the block-based editor
 * Implements 2-state UI: hidden (no file) / visible (file open)
 */

// Import core modules
import { setActiveFile, clearActiveFile, onFileChange, hasActiveFile, syncActiveCategory } from './core/editor-state.js';
import { getEditorForFile, hasEditorForFile } from './core/editor-registry.js';
import { initIconSidebar, showSidebar, hideSidebar } from './core/icon-sidebar.js';
import { initBlockPalette, showPalette, hidePalette, resizeFlyout } from './core/block-palette.js';

// Import URDF editor (auto-registers with registry)
import { initUrdfEditor, getUrdfCategories } from './editors/urdf/urdf-editor.js';

// State
let initialized = false;
let mainWorkspace = null;
let openFiles = [];
let activeFile = null;

// File type icons for tabs
const FILE_ICONS = {
    urdf: '<img src="assets/icons/urdf.png" class="file-type-icon" alt="urdf">',
    xacro: '<img src="assets/icons/urdf.png" class="file-type-icon" alt="xacro">',
    py: '<img src="assets/icons/nodes.svg" class="file-type-icon" alt="python">',
    launch: '<img src="assets/icons/launch.svg" class="file-type-icon" alt="launch">',
    yaml: '<img src="assets/icons/config.svg" class="file-type-icon" alt="yaml">',
    xml: '<img src="assets/icons/meshes.svg" class="file-type-icon" alt="xml">',
    default: '📄'
};

/**
 * Initialize the blocks component
 */
export function initBlocks() {
    if (initialized) {
        console.log('[Blocks] Already initialized');
        return;
    }

    console.log('[Blocks] Initializing...');

    // Initialize URDF editor (registers blocks and categories)
    if (typeof Blockly !== 'undefined') {
        initUrdfEditor(Blockly);
    } else {
        console.warn('[Blocks] Blockly not loaded, editor features disabled');
    }

    // Initialize UI components (they will hide themselves initially)
    initIconSidebar('blocks-icon-sidebar');
    initBlockPalette('blocks-sidebar');

    // Setup sidebar resize
    setupSidebarResize();

    // Listen for file selection events from packages panel
    window.addEventListener('fileSelected', handleFileSelected);

    // Set initial state (no file open)
    updateUIState(false);

    initialized = true;
    console.log('[Blocks] Component initialized');
}

/**
 * Update UI state based on whether a file is open
 * @param {boolean} hasFile - Whether a file is currently open
 */
function updateUIState(hasFile) {
    const iconSidebar = document.querySelector('.blocks-icon-sidebar');
    const blocksSidebar = document.querySelector('.blocks-sidebar');
    const resizeHandle = document.querySelector('.blocks-sidebar-resize-handle');
    const placeholder = document.getElementById('editor-placeholder');
    const workspace = document.getElementById('blockly-workspace');

    if (hasFile) {
        // State 2: File is open - show everything
        if (iconSidebar) {
            iconSidebar.classList.remove('hidden');
            iconSidebar.style.display = '';
        }
        if (blocksSidebar) {
            blocksSidebar.classList.remove('hidden');
            blocksSidebar.style.display = '';
        }
        if (resizeHandle) {
            resizeHandle.classList.remove('hidden');
            resizeHandle.style.display = '';
        }
        if (placeholder) {
            placeholder.style.display = 'none';
        }
        if (workspace) {
            workspace.classList.add('active');
        }
    } else {
        // State 1: No file open - hide sidebar elements
        if (iconSidebar) {
            iconSidebar.classList.add('hidden');
            iconSidebar.style.display = 'none';
        }
        if (blocksSidebar) {
            blocksSidebar.classList.add('hidden');
            blocksSidebar.style.display = 'none';
        }
        if (resizeHandle) {
            resizeHandle.classList.add('hidden');
            resizeHandle.style.display = 'none';
        }
        if (placeholder) {
            placeholder.style.display = 'flex';
        }
        if (workspace) {
            workspace.classList.remove('active');
        }
    }
}

/**
 * Handle file selection event from packages panel
 * @param {CustomEvent} event - File selection event
 */
function handleFileSelected(event) {
    const { type, fileName, packageName } = event.detail;

    // Validate event data
    if (!fileName || typeof fileName !== 'string') {
        console.error('[Blocks] Invalid file selection event');
        return;
    }

    console.log('[Blocks] File selected:', type, fileName);

    // Create file info object
    const fileInfo = {
        fileName: sanitizeText(fileName),
        packageName: sanitizeText(packageName || ''),
        type: type || getFileType(fileName)
    };

    // Check if file already open
    const existingIndex = openFiles.findIndex(f =>
        f.fileName === fileInfo.fileName && f.packageName === fileInfo.packageName
    );

    if (existingIndex >= 0) {
        activeFile = openFiles[existingIndex];
    } else {
        openFiles.push(fileInfo);
        activeFile = fileInfo;
    }

    // Update editor state (triggers icon sidebar and block palette updates)
    setActiveFile({
        type: fileInfo.type,
        path: fileInfo.fileName,
        package: fileInfo.packageName
    });

    // Update UI
    updateUIState(true);
    renderTabs();
    renderBreadcrumb();

    // Create Blockly workspace for the file type
    createBlocklyWorkspace(fileInfo.type);
}

/**
 * Get file type from filename
 * @param {string} fileName - File name
 * @returns {string} File type
 */
function getFileType(fileName) {
    if (!fileName) return 'unknown';
    const lower = fileName.toLowerCase();
    if (lower.endsWith('.urdf') || lower.endsWith('.xacro')) return 'urdf';
    if (lower.endsWith('.launch.py')) return 'launch';
    if (lower.endsWith('.py')) return 'node';
    if (lower.endsWith('.yaml') || lower.endsWith('.yml')) return 'config';
    return 'unknown';
}

/**
 * Sanitize text for display (prevent XSS)
 * @param {string} text - Text to sanitize
 * @returns {string} Sanitized text
 */
function sanitizeText(text) {
    if (typeof text !== 'string') return '';
    return text
        .replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;')
        .replace(/'/g, '&#039;');
}

/**
 * Setup sidebar resize functionality
 */
function setupSidebarResize() {
    const resizeHandle = document.querySelector('.blocks-sidebar-resize-handle');
    const blocksSidebar = document.querySelector('.blocks-sidebar');

    if (!resizeHandle || !blocksSidebar) return;

    let isResizing = false;
    let startX = 0;
    let startWidth = 0;
    let lastExpandedWidth = 200;

    const COLLAPSE_THRESHOLD = 80;
    const MIN_WIDTH = 150;
    const MAX_WIDTH = 350;

    // Double-click to toggle collapse
    resizeHandle.addEventListener('dblclick', () => {
        if (blocksSidebar.classList.contains('collapsed')) {
            blocksSidebar.classList.remove('collapsed');
            blocksSidebar.style.width = `${lastExpandedWidth}px`;
        } else {
            lastExpandedWidth = blocksSidebar.offsetWidth;
            blocksSidebar.classList.add('collapsed');
            blocksSidebar.style.width = '0px';
        }
        resizeFlyout();
    });

    resizeHandle.addEventListener('mousedown', (e) => {
        isResizing = true;
        startX = e.clientX;
        startWidth = blocksSidebar.classList.contains('collapsed') ? 0 : blocksSidebar.offsetWidth;
        document.body.style.cursor = 'ew-resize';
        document.body.style.userSelect = 'none';
        e.preventDefault();
    });

    document.addEventListener('mousemove', (e) => {
        if (!isResizing) return;

        const diff = e.clientX - startX;
        const newWidth = startWidth + diff;

        if (newWidth < COLLAPSE_THRESHOLD) {
            blocksSidebar.classList.add('collapsed');
            blocksSidebar.style.width = '0px';
        } else {
            blocksSidebar.classList.remove('collapsed');
            const clampedWidth = Math.max(MIN_WIDTH, Math.min(MAX_WIDTH, newWidth));
            blocksSidebar.style.width = `${clampedWidth}px`;
            lastExpandedWidth = clampedWidth;
        }

        resizeFlyout();
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
 * Render file tabs
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

        const ext = file.fileName.split('.').pop().toLowerCase();
        const icon = FILE_ICONS[ext] || FILE_ICONS.default;

        return `
            <div class="editor-tab ${isActive ? 'active' : ''}" data-index="${index}">
                <span class="tab-icon">${icon}</span>
                <span class="tab-name">${file.fileName}</span>
                <span class="tab-close" data-index="${index}">×</span>
            </div>
        `;
    }).join('');

    // Attach click handlers
    tabsContainer.querySelectorAll('.editor-tab').forEach(tab => {
        tab.addEventListener('click', (e) => {
            if (e.target.classList.contains('tab-close')) return;

            const index = parseInt(tab.dataset.index);
            activeFile = openFiles[index];

            // Update editor state
            setActiveFile({
                type: activeFile.type,
                path: activeFile.fileName,
                package: activeFile.packageName
            });

            renderTabs();
            renderBreadcrumb();
        });
    });

    // Attach close handlers
    tabsContainer.querySelectorAll('.tab-close').forEach(closeBtn => {
        closeBtn.addEventListener('click', (e) => {
            e.stopPropagation();
            const index = parseInt(closeBtn.dataset.index);
            closeTab(index);
        });
    });
}

/**
 * Close a tab
 * @param {number} index - Tab index to close
 */
function closeTab(index) {
    const wasActive = activeFile &&
        openFiles[index].fileName === activeFile.fileName &&
        openFiles[index].packageName === activeFile.packageName;

    openFiles.splice(index, 1);

    if (wasActive) {
        if (openFiles.length > 0) {
            const newIndex = Math.min(index, openFiles.length - 1);
            activeFile = openFiles[newIndex];
            setActiveFile({
                type: activeFile.type,
                path: activeFile.fileName,
                package: activeFile.packageName
            });
        } else {
            activeFile = null;
            clearActiveFile();
            updateUIState(false);
        }
    }

    renderTabs();
    renderBreadcrumb();
}

/**
 * Render breadcrumb
 */
function renderBreadcrumb() {
    const breadcrumbContainer = document.getElementById('editor-breadcrumb');
    if (!breadcrumbContainer) return;

    if (!activeFile) {
        breadcrumbContainer.innerHTML = '';
        return;
    }

    const folderNames = {
        urdf: 'urdf',
        node: 'scripts',
        config: 'config',
        launch: 'launch'
    };
    const folder = folderNames[activeFile.type] || activeFile.type;

    const ext = activeFile.fileName.split('.').pop().toLowerCase();
    const icon = FILE_ICONS[ext] || FILE_ICONS.default;

    breadcrumbContainer.innerHTML = `
        <span class="breadcrumb-item">
            <span class="breadcrumb-icon">📦</span>
            ${activeFile.packageName}
        </span>
        <span class="breadcrumb-separator">›</span>
        <span class="breadcrumb-item">
            <span class="breadcrumb-icon">📁</span>
            ${folder}
        </span>
        <span class="breadcrumb-separator">›</span>
        <span class="breadcrumb-item">
            <span class="breadcrumb-icon">${icon}</span>
            ${activeFile.fileName}
        </span>
    `;
}

/**
 * Get currently active file
 * @returns {Object|null} Active file info or null
 */
export function getActiveFile() {
    return activeFile ? { ...activeFile } : null;
}

/**
 * Get all open files
 * @returns {Array} Array of open file info objects
 */
export function getOpenFiles() {
    return [...openFiles];
}

/**
 * Get the main Blockly workspace
 * @returns {Blockly.Workspace|null} The main workspace or null
 */
export function getMainWorkspace() {
    return mainWorkspace;
}

// Track if custom flyout has been registered
let customFlyoutRegistered = false;

/**
 * Register a custom flyout class that maintains a fixed scale
 * This prevents the flyout from zooming when the workspace zoom controls are used
 */
function registerFixedScaleFlyout() {
    if (customFlyoutRegistered || typeof Blockly === 'undefined') {
        return;
    }

    // Create a custom flyout class that returns a fixed scale
    class FixedScaleFlyout extends Blockly.VerticalFlyout {
        /**
         * Override getFlyoutScale to return a constant value
         * instead of the workspace's scale
         * @returns {number} Fixed scale value for the flyout
         */
        getFlyoutScale() {
            // Return a fixed scale of 0.9 regardless of workspace zoom
            return 0.9;
        }
    }

    // Register the custom flyout
    Blockly.registry.register(
        Blockly.registry.Type.FLYOUTS_VERTICAL_TOOLBOX,
        'fixedScaleFlyout',
        FixedScaleFlyout,
        true  // Allow overwriting if already registered
    );

    customFlyoutRegistered = true;
    console.log('[Blocks] Fixed-scale flyout registered');
}

/**
 * Create main Blockly workspace with toolbox
 * @param {string} fileType - Type of file being edited
 */
function createBlocklyWorkspace(fileType) {
    const container = document.getElementById('blockly-workspace');
    if (!container) {
        console.error('[Blocks] Workspace container not found');
        return;
    }

    // Dispose existing workspace if any
    if (mainWorkspace) {
        mainWorkspace.dispose();
        mainWorkspace = null;
    }

    // Check if Blockly is available
    if (typeof Blockly === 'undefined') {
        console.error('[Blocks] Blockly not loaded');
        return;
    }

    // Register our custom fixed-scale flyout
    registerFixedScaleFlyout();

    // Generate toolbox from URDF categories
    const toolboxXml = generateToolboxXml(fileType);

    // Inject Blockly workspace with always-open flyout for native drag-drop behavior
    // Using our custom fixed-scale flyout to prevent block palette from zooming
    mainWorkspace = Blockly.inject(container, {
        toolbox: toolboxXml,
        renderer: 'zelos',
        toolboxPosition: 'start',  // Put toolbox on the left
        horizontalLayout: false,   // Vertical toolbox
        media: './pages/workspace/components/blocks/blockly-media/',  // Local media for offline use
        plugins: {
            // Use our custom flyout that maintains fixed scale
            flyoutsVerticalToolbox: 'fixedScaleFlyout'
        },
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

    // Store on window immediately so other modules can access it
    window.blocksMainWorkspace = mainWorkspace;

    // Auto-select first category to show flyout immediately
    setTimeout(() => {
        const toolbox = mainWorkspace.getToolbox();
        if (toolbox) {
            const items = toolbox.getToolboxItems();
            if (items && items.length > 0) {
                // Select item twice - first clears any bad state, second activates
                toolbox.clearSelection();
                toolbox.setSelectedItem(items[0]);

                // Sync state so duplicate click check works
                const firstCategoryName = items[0].getName ? items[0].getName().toLowerCase() : 'structure';
                syncActiveCategory(firstCategoryName);

                // Keep flyout always open and interactive
                const flyout = mainWorkspace.getFlyout();
                if (flyout && flyout.setAutoClose) {
                    flyout.setAutoClose(false);
                }

                // Force resize
                Blockly.svgResize(mainWorkspace);

                console.log('[Blocks] Flyout activated for first category:', firstCategoryName);
            }
        }
    }, 100);

    // Handle window resize
    const resizeHandler = () => {
        if (mainWorkspace) {
            Blockly.svgResize(mainWorkspace);
        }
    };
    window.addEventListener('resize', resizeHandler);

    // Initial resize after a short delay
    setTimeout(() => {
        Blockly.svgResize(mainWorkspace);
    }, 50);

    console.log('[Blocks] Main Blockly workspace created');
}


/**
 * Generate toolbox XML from categories
 * @param {string} fileType - Type of file being edited
 * @returns {string} Toolbox XML string
 */
function generateToolboxXml(fileType) {
    let categories = [];

    // Get categories based on file type
    if (fileType === 'urdf' || fileType === 'xacro') {
        categories = getUrdfCategories();
    }

    if (categories.length === 0) {
        return '<xml id="toolbox"></xml>';
    }

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
 * Dispose the main workspace
 */
function disposeWorkspace() {
    if (mainWorkspace) {
        mainWorkspace.dispose();
        mainWorkspace = null;
    }
}
