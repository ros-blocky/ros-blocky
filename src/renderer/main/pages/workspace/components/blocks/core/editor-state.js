/**
 * Editor State Management
 * Central state for tracking active file and category
 * Used by both blocks and editor components
 */

// Current state
let state = {
    activeFile: null,      // { type: 'urdf'|'node'|'config'|'launch', path: string, package: string }
    activeCategory: null,  // string - current icon category selected
    activeEditor: null     // 'urdf' | 'node' | 'config' | 'launch'
};

// Event listeners
const listeners = {
    fileChange: [],
    categoryChange: [],
    editorChange: []
};

/**
 * Set the active file (called when user clicks a file in packages panel)
 * @param {Object} file - { type, path, package }
 */
export function setActiveFile(file) {
    const previousType = state.activeFile?.type;
    state.activeFile = file;
    state.activeEditor = file?.type || null;

    // Reset category when file type changes
    if (previousType !== file?.type) {
        state.activeCategory = null;
    }

    // Notify listeners
    listeners.fileChange.forEach(cb => cb(file));

    if (previousType !== file?.type) {
        listeners.editorChange.forEach(cb => cb(file?.type));
    }

    console.log('[EditorState] Active file set:', file);
}

/**
 * Set the active category (called when user clicks an icon in sidebar)
 * Clicking the same category toggles flyout visibility
 * @param {string} category - Category ID
 */
export function setActiveCategory(category) {
    const flyout = window.blocksMainWorkspace?.getFlyout();
    const flyoutSvg = flyout?.svgGroup_;

    // Check if flyout is currently collapsed
    const isCollapsed = flyoutSvg?.style.display === 'none';

    // If clicking the same category
    if (state.activeCategory === category) {
        // Toggle flyout visibility
        if (isCollapsed) {
            // Show the flyout
            if (flyoutSvg) {
                flyoutSvg.style.display = '';
                if (flyoutSvg.parentElement) {
                    flyoutSvg.parentElement.classList.remove('collapsed');
                }
            }
            // Update toggle button if exists
            const toggleBtn = document.querySelector('.flyout-toggle-btn');
            if (toggleBtn) {
                toggleBtn.innerHTML = '◀';
                toggleBtn.title = 'Collapse palette';
                const flyoutWidth = flyout?.getWidth ? flyout.getWidth() : 180;
                toggleBtn.style.left = `${flyoutWidth}px`;
                toggleBtn.classList.remove('collapsed');
            }
            if (window.blocksMainWorkspace) {
                Blockly.svgResize(window.blocksMainWorkspace);
            }
            console.log('[EditorState] Flyout expanded for:', category);
        } else {
            // Hide the flyout
            if (flyoutSvg) {
                flyoutSvg.style.display = 'none';
                if (flyoutSvg.parentElement) {
                    flyoutSvg.parentElement.classList.add('collapsed');
                }
            }
            // Update toggle button if exists
            const toggleBtn = document.querySelector('.flyout-toggle-btn');
            if (toggleBtn) {
                toggleBtn.innerHTML = '▶';
                toggleBtn.title = 'Expand palette';
                toggleBtn.style.left = '0px';
                toggleBtn.classList.add('collapsed');
            }
            if (window.blocksMainWorkspace) {
                Blockly.svgResize(window.blocksMainWorkspace);
            }
            console.log('[EditorState] Flyout collapsed for:', category);
        }
        return;
    }

    // New category selected
    state.activeCategory = category;

    // Notify listeners
    listeners.categoryChange.forEach(cb => cb(category));

    // Trigger Blockly's native toolbox flyout if workspace is available
    if (window.blocksMainWorkspace) {
        const toolbox = window.blocksMainWorkspace.getToolbox();
        if (toolbox) {
            // Find the category by name/ID and select it
            const toolboxItems = toolbox.getToolboxItems();
            for (const item of toolboxItems) {
                // Match by lowercase name (Structure, Visual, etc.)
                const itemName = item.getName ? item.getName().toLowerCase() : '';
                if (itemName === category || itemName.includes(category)) {
                    toolbox.setSelectedItem(item);

                    // Ensure flyout is visible when switching categories
                    if (flyoutSvg && flyoutSvg.style.display === 'none') {
                        flyoutSvg.style.display = '';
                        if (flyoutSvg.parentElement) {
                            flyoutSvg.parentElement.classList.remove('collapsed');
                        }
                        const toggleBtn = document.querySelector('.flyout-toggle-btn');
                        if (toggleBtn) {
                            toggleBtn.innerHTML = '◀';
                            toggleBtn.title = 'Collapse palette';
                            const flyoutWidth = flyout?.getWidth ? flyout.getWidth() : 180;
                            toggleBtn.style.left = `${flyoutWidth}px`;
                            toggleBtn.classList.remove('collapsed');
                        }
                    }

                    // Keep flyout always open and interactive
                    if (flyout && flyout.setAutoClose) {
                        flyout.setAutoClose(false);
                    }
                    break;
                }
            }
        }
    }

    console.log('[EditorState] Active category set:', category);
}

/**
 * Sync the active category state without triggering toolbox
 * Used when blocks.js auto-selects to keep state in sync
 * @param {string} category - Category ID
 */
export function syncActiveCategory(category) {
    state.activeCategory = category;
}

/**
 * Get current state
 * @returns {Object} Current state
 */
export function getState() {
    return { ...state };
}

/**
 * Subscribe to file changes
 * @param {Function} callback 
 * @returns {Function} Unsubscribe function
 */
export function onFileChange(callback) {
    listeners.fileChange.push(callback);
    return () => {
        const index = listeners.fileChange.indexOf(callback);
        if (index > -1) listeners.fileChange.splice(index, 1);
    };
}

/**
 * Subscribe to category changes
 * @param {Function} callback 
 * @returns {Function} Unsubscribe function
 */
export function onCategoryChange(callback) {
    listeners.categoryChange.push(callback);
    return () => {
        const index = listeners.categoryChange.indexOf(callback);
        if (index > -1) listeners.categoryChange.splice(index, 1);
    };
}

/**
 * Subscribe to editor type changes
 * @param {Function} callback 
 * @returns {Function} Unsubscribe function
 */
export function onEditorChange(callback) {
    listeners.editorChange.push(callback);
    return () => {
        const index = listeners.editorChange.indexOf(callback);
        if (index > -1) listeners.editorChange.splice(index, 1);
    };
}

/**
 * Clear active file (reset state)
 */
export function clearActiveFile() {
    state.activeFile = null;
    state.activeCategory = null;
    state.activeEditor = null;

    listeners.fileChange.forEach(cb => cb(null));
    listeners.categoryChange.forEach(cb => cb(null));
    listeners.editorChange.forEach(cb => cb(null));

    console.log('[EditorState] State cleared');
}

/**
 * Check if there is an active file open
 * @returns {boolean} True if a file is currently open
 */
export function hasActiveFile() {
    return state.activeFile !== null;
}

/**
 * Get the current file type (editor type)
 * @returns {string|null} Current file type or null
 */
export function getActiveFileType() {
    return state.activeEditor;
}
