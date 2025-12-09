/**
 * URDF Editor Module
 * Main entry point for the URDF block-based editor
 * Registers itself with the editor registry
 */

import { registerEditor } from '../../core/editor-registry.js';
import { registerUrdfBlocks, getAllUrdfBlockTypes } from './urdf-blocks.js';
import { URDF_CATEGORIES, getCategoryById, getBlocksForCategory } from './urdf-categories.js';
import { createUrdfTheme, getUrdfThemeName } from './urdf-theme.js';

// Track initialization state
let initialized = false;
let urdfTheme = null;

/**
 * URDF Editor Configuration
 */
const URDF_EDITOR_CONFIG = {
    id: 'urdf',
    name: 'URDF Editor',
    extensions: ['.urdf', '.xacro'],
    categories: URDF_CATEGORIES,
    description: 'Visual block-based editor for URDF robot description files',

    /**
     * Register all URDF blocks with Blockly
     * @param {Object} Blockly - Blockly library instance
     */
    registerBlocks: (Blockly) => {
        registerUrdfBlocks(Blockly);
    },

    /**
     * Create and return the theme
     * @param {Object} Blockly - Blockly library instance
     * @returns {Object} Blockly theme
     */
    getTheme: (Blockly) => {
        if (!urdfTheme) {
            urdfTheme = createUrdfTheme(Blockly);
        }
        return urdfTheme;
    },

    /**
     * Get theme name for workspace configuration
     * @returns {string} Theme name
     */
    getThemeName: () => getUrdfThemeName()
};

/**
 * Initialize the URDF editor module
 * Registers with the editor registry and sets up blocks
 * @param {Object} Blockly - Blockly library instance (optional, will use global if not provided)
 * @returns {boolean} True if initialization successful
 */
export function initUrdfEditor(Blockly) {
    if (initialized) {
        console.log('[URDF Editor] Already initialized');
        return true;
    }

    // Use global Blockly if not provided
    const blocklyInstance = Blockly || window.Blockly;

    if (!blocklyInstance) {
        console.error('[URDF Editor] Blockly not available');
        return false;
    }

    console.log('[URDF Editor] Initializing...');

    // Register blocks
    URDF_EDITOR_CONFIG.registerBlocks(blocklyInstance);

    // Create theme
    URDF_EDITOR_CONFIG.getTheme(blocklyInstance);

    // Register with editor registry
    const success = registerEditor(URDF_EDITOR_CONFIG);

    if (success) {
        initialized = true;
        console.log('[URDF Editor] Initialized successfully');
    } else {
        console.error('[URDF Editor] Registration failed');
    }

    return success;
}

/**
 * Get URDF editor configuration
 * @returns {Object} Editor configuration
 */
export function getUrdfEditorConfig() {
    return { ...URDF_EDITOR_CONFIG };
}

/**
 * Get categories for URDF editor
 * @returns {Array} Category configurations
 */
export function getUrdfCategories() {
    return [...URDF_CATEGORIES];
}

/**
 * Get blocks for a specific category
 * @param {string} categoryId - Category ID
 * @returns {string[]} Block type names
 */
export function getUrdfBlocksForCategory(categoryId) {
    return getBlocksForCategory(categoryId);
}

/**
 * Check if URDF editor is initialized
 * @returns {boolean} Initialization state
 */
export function isUrdfEditorInitialized() {
    return initialized;
}

/**
 * Get the URDF theme
 * @returns {Object|null} Theme object or null if not initialized
 */
export function getUrdfTheme() {
    return urdfTheme;
}

// Export config for external use
export { URDF_CATEGORIES };
