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
 * @param {string} category - Category ID
 */
export function setActiveCategory(category) {
    state.activeCategory = category;

    // Notify listeners
    listeners.categoryChange.forEach(cb => cb(category));

    console.log('[EditorState] Active category set:', category);
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
