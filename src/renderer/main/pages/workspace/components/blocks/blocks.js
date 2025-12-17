/**
 * Blocks Component
 * Main orchestrator for the block-based editor
 * Implements 2-state UI: hidden (no file) / visible (file open)
 */

// Import core modules
import { setActiveFile, clearActiveFile, onFileChange, hasActiveFile, syncActiveCategory, onCategoryChange, updateFlyoutMargins } from './core/editor-state.js';
import { getEditorForFile, hasEditorForFile } from './core/editor-registry.js';
import { initIconSidebar, showSidebar, hideSidebar } from './core/icon-sidebar.js';
import { initBlockPalette, showPalette, hidePalette, resizeFlyout } from './core/block-palette.js';

// Import URDF editor (auto-registers with registry)
import { initUrdfEditor, getUrdfCategories, generateUrdfCode, hasOrphanBlocks, getSkippedBlocks, validateUrdf, enableRealtimeValidation } from './editors/urdf/urdf-editor.js';
import { registerUrdfBlocks } from './editors/urdf/urdf-blocks.js';
import { getCategoryById } from './editors/urdf/urdf-categories.js';

// Import Node editor (auto-registers with registry)
import { initNodeEditor, getNodeCategories, generateNodeCode, validateNode, enableRealtimeValidation as enableNodeValidation } from './editors/nodes/node-editor.js';
import { getCategoryById as getNodeCategoryById } from './editors/nodes/node-categories.js';

// Import i18n for translations
import { t, onLanguageChange } from '../../../../../i18n/index.js';

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

// Global log store - keyed by node name (persists across file switches)
const globalNodeLogs = {};
// Track visible log panels keyed by node name
const visibleLogPanels = {};

// Export for use by node blocks
export function getNodeLogs(nodeName) {
    if (!globalNodeLogs[nodeName]) {
        globalNodeLogs[nodeName] = [];
    }
    return globalNodeLogs[nodeName];
}

export function addNodeLog(nodeName, message, type) {
    if (!globalNodeLogs[nodeName]) {
        globalNodeLogs[nodeName] = [];
    }
    globalNodeLogs[nodeName].push({ message, type, timestamp: Date.now() });
    // Keep only last 100 logs
    if (globalNodeLogs[nodeName].length > 100) {
        globalNodeLogs[nodeName].shift();
    }
    // Update any visible log panel for this node
    if (visibleLogPanels[nodeName]) {
        const panel = visibleLogPanels[nodeName];
        const content = panel.querySelector('.log-panel-content');
        if (content) {
            if (content.querySelector('.log-empty')) {
                content.innerHTML = '';
            }
            const logLine = document.createElement('div');
            logLine.className = 'log-line ' + (type || 'info');
            logLine.textContent = message;
            content.appendChild(logLine);
            content.scrollTop = content.scrollHeight;
        }
    }
}

export function registerLogPanel(nodeName, panel) {
    visibleLogPanels[nodeName] = panel;
}

export function unregisterLogPanel(nodeName) {
    delete visibleLogPanels[nodeName];
}

export function getLogPanel(nodeName) {
    return visibleLogPanels[nodeName] || null;
}

export function closeLogPanel(nodeName) {
    if (visibleLogPanels[nodeName]) {
        visibleLogPanels[nodeName].remove();
        delete visibleLogPanels[nodeName];
        return true;
    }
    return false;
}

/**
 * Initialize the blocks component
 */
export function initBlocks() {
    if (initialized) {
        console.log('[Blocks] Already initialized');
        return;
    }

    console.log('[Blocks] Initializing...');

    // Initialize editors (register blocks and categories)
    if (typeof Blockly !== 'undefined') {
        initUrdfEditor(Blockly);
        initNodeEditor(Blockly);
    } else {
        console.warn('[Blocks] Blockly not loaded, editor features disabled');
    }

    // Initialize UI components (they will hide themselves initially)
    initIconSidebar('blocks-icon-sidebar');
    initBlockPalette('blocks-sidebar');

    // Create the flyout category header
    createFlyoutHeader();

    // Listen for category changes to update the header
    onCategoryChange((categoryId) => {
        updateFlyoutHeader(categoryId);
    });

    // Listen for language changes to update blocks and header
    onLanguageChange(() => {
        console.log('[Blocks] Language changed, updating blocks...');

        // Update flyout header translation
        const header = document.querySelector('.flyout-category-header');
        if (header && !header.classList.contains('hidden')) {
            import('./core/editor-state.js').then(({ getActiveCategory }) => {
                const currentCategory = getActiveCategory();
                if (currentCategory) {
                    updateFlyoutHeader(currentCategory);
                }
            });
        }

        // If a file is open, recreate the workspace to update block translations
        if (activeFile && mainWorkspace) {
            console.log('[Blocks] Recreating workspace for language update...');

            // Save current state BEFORE disposing
            const currentXml = Blockly.Xml.workspaceToDom(mainWorkspace);
            const xmlText = Blockly.Xml.domToText(currentXml);
            const savedFileType = activeFile.type;

            // Re-register blocks with new translations
            registerUrdfBlocks(Blockly);
            console.log('[Blocks] Re-registered blocks with new language');

            // Recreate workspace (this disposes the old one)
            createBlocklyWorkspace(savedFileType);

            // Wait for workspace to be ready, then restore blocks
            setTimeout(() => {
                if (mainWorkspace) {
                    try {
                        // Clear any blocks that might have been loaded from file
                        mainWorkspace.clear();

                        // Restore the blocks we saved
                        const newDom = Blockly.utils.xml.textToDom(xmlText);
                        Blockly.Xml.domToWorkspace(newDom, mainWorkspace);

                        // Clear undo/redo history
                        mainWorkspace.clearUndo();

                        console.log('[Blocks] Restored blocks after language change');
                    } catch (e) {
                        console.warn('[Blocks] Could not restore blocks after language change:', e);
                    }
                }
            }, 300);
        }
    });

    // Setup sidebar resize
    setupSidebarResize();

    // Listen for file selection events from packages panel
    window.addEventListener('fileSelected', handleFileSelected);

    // Listen for save before run requests (from URDF run button)
    window.addEventListener('saveBeforeRun', async (event) => {
        const { packageName, fileName } = event.detail;
        console.log('[Blocks] Save before run requested for:', fileName);

        // Check if the file being run is the active file
        if (activeFile && activeFile.fileName === fileName && activeFile.packageName === packageName) {
            await saveCurrentFile();
        }

        // Dispatch completion event
        window.dispatchEvent(new CustomEvent('saveComplete', {
            detail: { success: true, fileName }
        }));
    });

    // Setup keyboard shortcuts (Ctrl+S to save)
    setupKeyboardShortcuts();

    // Setup toolbar buttons
    setupToolbar();

    // Listen for build completion to show toast
    if (window.electronAPI && window.electronAPI.onBuildResult) {
        window.electronAPI.onBuildResult((result) => {
            if (result.success) {
                if (result.target === 'all') {
                    showSuccessToast('All packages built successfully');
                } else {
                    showSuccessToast(`Package "${result.target}" built successfully`);
                }
            } else {
                if (result.target === 'all') {
                    showErrorToast('Build failed: ' + (result.error || 'Unknown error'));
                } else {
                    showErrorToast(`Build failed for "${result.target}": ` + (result.error || 'Unknown error'));
                }
            }
        });
    }

    // Listen for ROS output to route to node log panels
    if (window.electronAPI && window.electronAPI.onRosOutput) {
        window.electronAPI.onRosOutput((data) => {
            // Only process node logs
            if (!data.processKey || !data.processKey.startsWith('node:')) {
                return;
            }

            // Skip status messages - only show actual log output
            if (data.type === 'status') {
                return;
            }

            // Skip non-ROS log messages (status updates, etc)
            if (!data.message || !data.message.includes('[')) {
                return;
            }

            // Try to extract actual ROS node name from log message
            // ROS log format: [INFO] [timestamp] [node_name]: message
            let nodeName = null;
            if (data.message) {
                const rosLogMatch = data.message.match(/\]\s*\[([^\]]+)\]:\s*/);
                if (rosLogMatch) {
                    nodeName = rosLogMatch[1];
                }
            }

            // Fallback to file name from processKey if no node name in message
            if (!nodeName) {
                const match = data.processKey.match(/node:[^/]+\/(.+)/);
                if (match) {
                    nodeName = match[1].replace('.py', '');
                }
            }

            if (!nodeName) return;

            // Add to global log store (this also updates any visible log panel)
            const logType = data.type === 'error' ? 'error' :
                data.message && data.message.includes('[WARN]') ? 'warn' : 'info';
            addNodeLog(nodeName, data.message, logType);
        });
    }

    // Set initial state (no file open)
    updateUIState(false);

    initialized = true;
    console.log('[Blocks] Component initialized');
}

/**
 * Setup toolbar buttons
 */
function setupToolbar() {
    const saveBtn = document.getElementById('editor-save-btn');
    if (saveBtn) {
        saveBtn.addEventListener('click', async () => {
            await saveCurrentFile();
        });
    }

    const undoBtn = document.getElementById('editor-undo-btn');
    if (undoBtn) {
        undoBtn.addEventListener('click', () => {
            if (mainWorkspace) {
                mainWorkspace.undo(false); // false = undo
            }
        });
    }

    const redoBtn = document.getElementById('editor-redo-btn');
    if (redoBtn) {
        redoBtn.addEventListener('click', () => {
            if (mainWorkspace) {
                mainWorkspace.undo(true); // true = redo
            }
        });
    }

    // RViz button - launches RViz2 visualization tool
    const rvizBtn = document.getElementById('rviz-btn');
    if (rvizBtn) {
        rvizBtn.addEventListener('click', async () => {
            try {
                console.log('[Blocks] Launching RViz2...');
                const result = await window.electronAPI.runRviz();
                if (result.success) {
                    console.log('[Blocks] RViz2 launched successfully, PID:', result.pid);
                } else {
                    console.error('[Blocks] Failed to launch RViz2:', result.error);
                    if (result.isRunning) {
                        showWarningToast('RViz2 is already running');
                    } else {
                        showErrorToast('Failed to launch RViz2: ' + result.error);
                    }
                }
            } catch (error) {
                console.error('[Blocks] Error launching RViz2:', error);
                showErrorToast('Error launching RViz2');
            }
        });
    }

    // Joint State Publisher GUI button - launches joint_state_publisher_gui
    const jspGuiBtn = document.getElementById('jsp-gui-btn');
    if (jspGuiBtn) {
        jspGuiBtn.addEventListener('click', async () => {
            try {
                console.log('[Blocks] Launching Joint State Publisher GUI...');
                const result = await window.electronAPI.runJointStatePublisherGui();
                if (result.success) {
                    console.log('[Blocks] JSP GUI launched successfully, PID:', result.pid);
                } else {
                    console.error('[Blocks] Failed to launch JSP GUI:', result.error);
                    if (result.isRunning) {
                        showWarningToast('Joint State Publisher GUI is already running');
                    } else {
                        showErrorToast('Failed to launch JSP GUI: ' + result.error);
                    }
                }
            } catch (error) {
                console.error('[Blocks] Error launching JSP GUI:', error);
                showErrorToast('Error launching Joint State Publisher GUI');
            }
        });
    }

    // TurtleSim button - launches turtlesim_node
    const turtlesimBtn = document.getElementById('turtlesim-btn');
    if (turtlesimBtn) {
        turtlesimBtn.addEventListener('click', async () => {
            try {
                console.log('[Blocks] Launching TurtleSim...');
                const result = await window.electronAPI.runTurtlesim();
                if (result.success) {
                    console.log('[Blocks] TurtleSim launched successfully, PID:', result.pid);
                } else {
                    console.error('[Blocks] Failed to launch TurtleSim:', result.error);
                    if (result.isRunning) {
                        showWarningToast('TurtleSim is already running');
                    } else {
                        showErrorToast('Failed to launch TurtleSim: ' + result.error);
                    }
                }
            } catch (error) {
                console.error('[Blocks] Error launching TurtleSim:', error);
                showErrorToast('Error launching TurtleSim');
            }
        });
    }
}

/**
 * Setup keyboard shortcuts for the editor
 */
function setupKeyboardShortcuts() {
    document.addEventListener('keydown', async (e) => {
        // Ctrl+S or Cmd+S to save
        if ((e.ctrlKey || e.metaKey) && e.key === 's') {
            e.preventDefault();
            await saveCurrentFile();
        }
    });
}

/**
 * Save the current file by generating code from workspace
 */
async function saveCurrentFile() {
    if (!activeFile || !mainWorkspace) {
        console.log('[Blocks] No active file to save');
        return;
    }

    try {
        console.log('[Blocks] Saving file:', activeFile.fileName, 'type:', activeFile.type);

        // Handle based on file type
        if (activeFile.type === 'node') {
            // SAVE NODE FILE - No validation for now
            const pythonCode = generateNodeCode(mainWorkspace);

            if (!pythonCode) {
                console.warn('[Blocks] No code generated - workspace may be empty');
                showWarningToast('No Node block found - nothing to save');
                return;
            }

            // Save Python file
            const nodeResult = await window.electronAPI.saveNodeFile(
                activeFile.packageName,
                activeFile.fileName,
                pythonCode
            );

            // Serialize block state to XML
            const blockXml = Blockly.Xml.domToText(
                Blockly.Xml.workspaceToDom(mainWorkspace)
            );

            // Save block state sidecar file
            const blockResult = await window.electronAPI.saveNodeBlockState(
                activeFile.packageName,
                activeFile.fileName,
                blockXml
            );

            if (nodeResult.success && blockResult.success) {
                console.log('[Blocks] Node file and block state saved successfully');
                showSaveIndicator();
            } else {
                if (!nodeResult.success) {
                    console.error('[Blocks] Failed to save Node:', nodeResult.message);
                    showErrorToast('Failed to save Node file');
                }
                if (!blockResult.success) {
                    console.error('[Blocks] Failed to save block state:', blockResult.message);
                }
            }
        } else {
            // SAVE URDF FILE
            // Validate Workspace First
            const validation = validateUrdf(mainWorkspace);

            if (validation.errors.length > 0) {
                console.error('[Blocks] Validation errors:', validation.errors);
                showErrorToast(`Error: ${validation.errors[0]}`);
                return; // STOP SAVE
            }

            // Generate URDF code from workspace with actual package name for mesh paths
            const urdfCode = generateUrdfCode(mainWorkspace, activeFile.packageName);

            // Check for orphan blocks (blocks outside Robot)
            if (hasOrphanBlocks()) {
                const skipped = getSkippedBlocks();
                showWarningToast(`Warning: ${skipped.length} block(s) outside Robot were not saved`);
            }

            if (!urdfCode) {
                console.warn('[Blocks] No code generated - workspace may be empty');
                showWarningToast('No Robot block found - nothing to save');
                return;
            }

            // Save URDF file
            const urdfResult = await window.electronAPI.saveUrdfFile(
                activeFile.packageName,
                activeFile.fileName,
                urdfCode
            );

            // Serialize block state to XML
            const blockXml = Blockly.Xml.domToText(
                Blockly.Xml.workspaceToDom(mainWorkspace)
            );

            // Save block state sidecar file
            const blockResult = await window.electronAPI.saveBlockState(
                activeFile.packageName,
                activeFile.fileName,
                blockXml
            );

            if (urdfResult.success && blockResult.success) {
                console.log('[Blocks] File and block state saved successfully');
                showSaveIndicator();
            } else {
                if (!urdfResult.success) {
                    console.error('[Blocks] Failed to save URDF:', urdfResult.message);
                }
                if (!blockResult.success) {
                    console.error('[Blocks] Failed to save block state:', blockResult.message);
                }
            }
        }
    } catch (error) {
        console.error('[Blocks] Error saving file:', error);
    }
}

/**
 * Show a warning toast message
 * @param {string} message - Warning message to display
 */
function showWarningToast(message) {
    // Remove existing toast if any
    const existingToast = document.querySelector('.warning-toast');
    if (existingToast) {
        existingToast.remove();
    }

    // Create toast element
    const toast = document.createElement('div');
    toast.className = 'warning-toast';
    toast.innerHTML = `
        <span class="warning-icon">⚠️</span>
        <span class="warning-message">${message}</span>
    `;
    document.body.appendChild(toast);

    // Auto-remove after 4 seconds
    setTimeout(() => {
        toast.classList.add('fade-out');
        setTimeout(() => toast.remove(), 300);
    }, 4000);
}

/**
 * Show an error toast message (red)
 * @param {string} message - Error message to display
 */
function showErrorToast(message) {
    // Remove existing toast if any
    const existingToast = document.querySelector('.warning-toast');
    if (existingToast) {
        existingToast.remove();
    }

    // Create toast element
    const toast = document.createElement('div');
    toast.className = 'warning-toast error-toast'; // Re-use class + modifier
    toast.innerHTML = `
        <span class="warning-icon">❌</span>
        <span class="warning-message">${message}</span>
    `;
    document.body.appendChild(toast);

    // Auto-remove after 5 seconds
    setTimeout(() => {
        toast.classList.add('fade-out');
        setTimeout(() => toast.remove(), 300);
    }, 5000);
}

/**
 * Show a success toast message (green)
 * @param {string} message - Success message to display
 */
function showSuccessToast(message) {
    // Remove existing toast if any
    const existingToast = document.querySelector('.warning-toast');
    if (existingToast) {
        existingToast.remove();
    }

    // Create toast element
    const toast = document.createElement('div');
    toast.className = 'warning-toast success-toast';
    toast.innerHTML = `
        <span class="warning-icon">✓</span>
        <span class="warning-message">${message}</span>
    `;
    document.body.appendChild(toast);

    // Auto-remove after 3 seconds
    setTimeout(() => {
        toast.classList.add('fade-out');
        setTimeout(() => toast.remove(), 300);
    }, 3000);
}

/**
 * Show a brief save indicator
 */
function showSaveIndicator() {
    // Show success toast
    showSuccessToast('File saved successfully');
}

/**
 * Create the flyout category header element
 */
function createFlyoutHeader() {
    // Check if header already exists
    if (document.querySelector('.flyout-category-header')) {
        return;
    }

    // Create header element
    const header = document.createElement('div');
    header.className = 'flyout-category-header hidden';
    header.innerHTML = `
        <img class="header-icon" src="" alt="">
        <span class="header-text">BLOCKS</span>
    `;

    // Add to the blocks container
    const blocksContainer = document.querySelector('.blocks-top-content');
    if (blocksContainer) {
        blocksContainer.appendChild(header);
    }
}

/**
 * Update the flyout header with the current category
 * @param {string} categoryId - The category ID
 */
function updateFlyoutHeader(categoryId) {
    const header = document.querySelector('.flyout-category-header');
    if (!header) return;

    if (!categoryId) {
        header.classList.add('hidden');
        return;
    }

    // Get category info based on active file type
    const fileType = activeFile?.type || 'urdf';
    let category = null;
    let editorType = 'urdf';

    if (fileType === 'node') {
        category = getNodeCategoryById(categoryId);
        editorType = 'node';
    } else {
        category = getCategoryById(categoryId);
        editorType = 'urdf';
    }

    if (!category) {
        header.classList.add('hidden');
        return;
    }

    // Update header content
    const icon = header.querySelector('.header-icon');
    const text = header.querySelector('.header-text');

    if (icon) {
        icon.src = category.icon;
        icon.alt = category.label;
    }
    if (text) {
        // Use translated category name from i18n (dynamic based on editor type)
        const translationKey = `blocks.${editorType}.${categoryId}`;
        text.textContent = t(translationKey) || category.label;
    }

    // Update header background color based on category
    header.style.background = `linear-gradient(135deg, ${category.color} 0%, ${adjustColor(category.color, -20)} 100%)`;

    // Update header width based on flyout width
    const flyout = window.blocksMainWorkspace?.getFlyout();
    if (flyout) {
        const flyoutWidth = flyout.getWidth ? flyout.getWidth() : 180;
        header.style.width = `${flyoutWidth}px`;
    }

    header.classList.remove('hidden');
}

/**
 * Adjust color brightness
 * @param {string} hex - Hex color
 * @param {number} amount - Amount to adjust (-100 to 100)
 * @returns {string} Adjusted hex color
 */
function adjustColor(hex, amount) {
    // Remove # if present
    hex = hex.replace('#', '');

    // Parse RGB
    let r = parseInt(hex.substring(0, 2), 16);
    let g = parseInt(hex.substring(2, 4), 16);
    let b = parseInt(hex.substring(4, 6), 16);

    // Adjust
    r = Math.max(0, Math.min(255, r + amount));
    g = Math.max(0, Math.min(255, g + amount));
    b = Math.max(0, Math.min(255, b + amount));

    // Convert back to hex
    return `#${r.toString(16).padStart(2, '0')}${g.toString(16).padStart(2, '0')}${b.toString(16).padStart(2, '0')}`;
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
    const saveBtn = document.getElementById('editor-save-btn');

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
        if (saveBtn) {
            saveBtn.classList.remove('hidden');
            saveBtn.disabled = false;
        }
        // Show undo/redo buttons
        const undoBtn = document.getElementById('editor-undo-btn');
        const redoBtn = document.getElementById('editor-redo-btn');
        if (undoBtn) undoBtn.classList.remove('hidden');
        if (redoBtn) redoBtn.classList.remove('hidden');
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
        if (saveBtn) {
            saveBtn.classList.add('hidden');
            saveBtn.disabled = true;
        }
        // Hide undo/redo buttons
        const undoBtn = document.getElementById('editor-undo-btn');
        const redoBtn = document.getElementById('editor-redo-btn');
        if (undoBtn) undoBtn.classList.add('hidden');
        if (redoBtn) redoBtn.classList.add('hidden');
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
            const newFile = openFiles[index];

            // Skip if already active
            if (activeFile &&
                newFile.fileName === activeFile.fileName &&
                newFile.packageName === activeFile.packageName) {
                return;
            }

            activeFile = newFile;

            // Update editor state
            setActiveFile({
                type: activeFile.type,
                path: activeFile.fileName,
                package: activeFile.packageName
            });

            renderTabs();
            renderBreadcrumb();

            // Reload the workspace with the new file's blocks
            createBlocklyWorkspace(activeFile.type);
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
            // Return a fixed scale of 0.7 regardless of workspace zoom
            return 0.7;
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

                // Update the flyout header to show the first category
                updateFlyoutHeader(firstCategoryName);

                // Keep flyout always open and interactive
                const flyout = mainWorkspace.getFlyout();
                if (flyout && flyout.setAutoClose) {
                    flyout.setAutoClose(false);
                }

                // Update flyout margins for tabs/breadcrumb
                updateFlyoutMargins(true);

                // Enable real-time visual validation based on file type
                if (fileType === 'urdf' || fileType === 'xacro') {
                    enableRealtimeValidation(mainWorkspace);
                }
                // Node validation DISABLED FOR TESTING
                // else if (fileType === 'node') {
                //     enableNodeValidation(mainWorkspace);
                // }

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

    // Load saved block state if it exists
    loadBlocksFromFile();

    console.log('[Blocks] Main Blockly workspace created');
}

/**
 * Load blocks from sidecar file if it exists
 */
async function loadBlocksFromFile() {
    if (!activeFile || !mainWorkspace) {
        return;
    }

    try {
        console.log('[Blocks] Checking for saved block state...', activeFile.type);

        let result;

        // Use appropriate loader based on file type
        if (activeFile.type === 'node') {
            result = await window.electronAPI.loadNodeBlockState(
                activeFile.packageName,
                activeFile.fileName
            );
        } else {
            result = await window.electronAPI.loadBlockState(
                activeFile.packageName,
                activeFile.fileName
            );
        }

        if (result.success && result.blockXml) {
            console.log('[Blocks] Loading saved block state...');

            // Clear current workspace
            mainWorkspace.clear();

            // Parse and load the XML
            const dom = Blockly.utils.xml.textToDom(result.blockXml);
            Blockly.Xml.domToWorkspace(dom, mainWorkspace);

            // Clear undo stack so loaded blocks can't be undone all at once
            mainWorkspace.clearUndo();

            console.log('[Blocks] Block state loaded successfully');
        } else {
            console.log('[Blocks] No saved block state found, starting fresh');
        }
    } catch (error) {
        console.error('[Blocks] Error loading block state:', error);
    }
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
    } else if (fileType === 'node') {
        categories = getNodeCategories();
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
