/**
 * Editor Registry
 * Central registry for file-type specific editors (URDF, Node, Config, Launch, etc.)
 * Provides a plugin architecture for adding new editor types.
 * 
 * Security: All file paths and inputs are validated before processing.
 */

// Registered editors storage
const registeredEditors = new Map();

// Event listeners for registry changes
const registryListeners = [];

/**
 * Validate file extension format
 * @param {string} ext - File extension to validate
 * @returns {boolean} True if valid
 */
function isValidExtension(ext) {
    if (typeof ext !== 'string') return false;
    // Extensions must start with a dot and contain only alphanumeric chars
    return /^\.[a-zA-Z0-9]+$/.test(ext);
}

/**
 * Validate editor configuration object
 * @param {Object} config - Editor configuration
 * @returns {boolean} True if valid
 */
function isValidEditorConfig(config) {
    if (!config || typeof config !== 'object') return false;
    if (!config.id || typeof config.id !== 'string') return false;
    if (!config.name || typeof config.name !== 'string') return false;
    if (!Array.isArray(config.extensions) || config.extensions.length === 0) return false;
    if (!Array.isArray(config.categories)) return false;
    return true;
}

/**
 * Sanitize string input to prevent XSS
 * @param {string} input - String to sanitize
 * @returns {string} Sanitized string
 */
export function sanitizeInput(input) {
    if (typeof input !== 'string') return '';
    return input
        .replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;')
        .replace(/'/g, '&#039;');
}

/**
 * Register an editor type
 * @param {Object} editorConfig - Editor configuration object
 * @param {string} editorConfig.id - Unique editor identifier (e.g., 'urdf', 'node')
 * @param {string} editorConfig.name - Display name (e.g., 'URDF Editor')
 * @param {string[]} editorConfig.extensions - File extensions (e.g., ['.urdf', '.xacro'])
 * @param {Array} editorConfig.categories - Category configurations
 * @param {Function} editorConfig.registerBlocks - Function to register Blockly blocks
 * @param {Object} editorConfig.theme - Blockly theme configuration
 * @returns {boolean} True if registration successful
 */
export function registerEditor(editorConfig) {
    // Validate configuration
    if (!isValidEditorConfig(editorConfig)) {
        console.error('[EditorRegistry] Invalid editor configuration:', editorConfig);
        return false;
    }

    // Validate all extensions
    for (const ext of editorConfig.extensions) {
        if (!isValidExtension(ext)) {
            console.error('[EditorRegistry] Invalid extension format:', ext);
            return false;
        }
    }

    // Check for duplicate ID
    if (registeredEditors.has(editorConfig.id)) {
        console.warn('[EditorRegistry] Editor already registered, updating:', editorConfig.id);
    }

    // Store the editor config
    registeredEditors.set(editorConfig.id, {
        ...editorConfig,
        registeredAt: Date.now()
    });

    console.log(`[EditorRegistry] Registered editor: ${editorConfig.id} for extensions:`, editorConfig.extensions);

    // Notify listeners
    notifyRegistryChange('register', editorConfig.id);

    return true;
}

/**
 * Unregister an editor type
 * @param {string} editorId - Editor ID to unregister
 * @returns {boolean} True if unregistration successful
 */
export function unregisterEditor(editorId) {
    if (!registeredEditors.has(editorId)) {
        console.warn('[EditorRegistry] Editor not found:', editorId);
        return false;
    }

    registeredEditors.delete(editorId);
    console.log('[EditorRegistry] Unregistered editor:', editorId);

    notifyRegistryChange('unregister', editorId);
    return true;
}

/**
 * Get editor configuration for a file
 * @param {string} fileName - File name or path
 * @returns {Object|null} Editor configuration or null if not found
 */
export function getEditorForFile(fileName) {
    if (typeof fileName !== 'string' || !fileName) {
        return null;
    }

    // Extract extension(s) - handle compound extensions like .launch.py
    const lowerName = fileName.toLowerCase();

    // Try compound extensions first (e.g., .launch.py)
    for (const [id, config] of registeredEditors) {
        for (const ext of config.extensions) {
            if (lowerName.endsWith(ext.toLowerCase())) {
                return config;
            }
        }
    }

    return null;
}

/**
 * Get editor by ID
 * @param {string} editorId - Editor ID
 * @returns {Object|null} Editor configuration or null
 */
export function getEditorById(editorId) {
    return registeredEditors.get(editorId) || null;
}

/**
 * Get all registered editors
 * @returns {Array} Array of editor configurations
 */
export function getAllEditors() {
    return Array.from(registeredEditors.values());
}

/**
 * Check if an editor is registered for a file type
 * @param {string} fileName - File name to check
 * @returns {boolean} True if editor is available
 */
export function hasEditorForFile(fileName) {
    return getEditorForFile(fileName) !== null;
}

/**
 * Get editor type ID from file name
 * @param {string} fileName - File name
 * @returns {string|null} Editor type ID or null
 */
export function getEditorTypeForFile(fileName) {
    const editor = getEditorForFile(fileName);
    return editor ? editor.id : null;
}

/**
 * Add listener for registry changes
 * @param {Function} callback - Callback function(action, editorId)
 * @returns {Function} Unsubscribe function
 */
export function onRegistryChange(callback) {
    if (typeof callback !== 'function') {
        console.error('[EditorRegistry] Invalid callback');
        return () => { };
    }

    registryListeners.push(callback);

    // Return unsubscribe function
    return () => {
        const index = registryListeners.indexOf(callback);
        if (index > -1) {
            registryListeners.splice(index, 1);
        }
    };
}

/**
 * Notify all listeners of registry changes
 * @param {string} action - Action type ('register' | 'unregister')
 * @param {string} editorId - Editor ID
 */
function notifyRegistryChange(action, editorId) {
    for (const listener of registryListeners) {
        try {
            listener(action, editorId);
        } catch (error) {
            console.error('[EditorRegistry] Listener error:', error);
        }
    }
}

/**
 * Get supported file extensions for an editor type
 * @param {string} editorId - Editor ID
 * @returns {string[]} Array of extensions or empty array
 */
export function getExtensionsForEditor(editorId) {
    const editor = registeredEditors.get(editorId);
    return editor ? [...editor.extensions] : [];
}

// Export for debugging
export function _debugGetRegistry() {
    return new Map(registeredEditors);
}
