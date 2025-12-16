/**
 * Node Editor Module
 * Main entry point for the ROS 2 Python Node block-based editor
 * Registers itself with the editor registry
 */

import { registerEditor } from '../../core/editor-registry.js';
import { registerNodeBlocks, getAllNodeBlockTypes } from './node-blocks.js';
import { NODE_CATEGORIES, getCategoryById, getBlocksForCategory } from './node-categories.js';
import { createNodeTheme, getNodeThemeName } from './node-theme.js';
import { initNodeGenerator, generateNodeCode, hasOrphanBlocks, getSkippedBlocks } from './node-generator.js';
import { validateNode } from './node-validator.js';

// Track initialization state
let initialized = false;
let nodeTheme = null;
let pythonGenerator = null;

/**
 * Node Editor Configuration
 */
const NODE_EDITOR_CONFIG = {
    id: 'node',
    name: 'Node Editor',
    extensions: ['.py'],
    categories: NODE_CATEGORIES,
    description: 'Visual block-based editor for ROS 2 Python nodes',

    /**
     * Register all Node blocks with Blockly
     * @param {Object} Blockly - Blockly library instance
     */
    registerBlocks: (Blockly) => {
        registerNodeBlocks(Blockly);
    },

    /**
     * Create and return the theme
     * @param {Object} Blockly - Blockly library instance
     * @returns {Object} Blockly theme
     */
    getTheme: (Blockly) => {
        if (!nodeTheme) {
            nodeTheme = createNodeTheme(Blockly);
        }
        return nodeTheme;
    },

    /**
     * Get theme name for workspace configuration
     * @returns {string} Theme name
     */
    getThemeName: () => getNodeThemeName()
};

/**
 * Initialize the Node editor module
 * Registers with the editor registry and sets up blocks
 * @param {Object} Blockly - Blockly library instance (optional, will use global if not provided)
 * @returns {boolean} True if initialization successful
 */
export function initNodeEditor(Blockly) {
    if (initialized) {
        console.log('[Node Editor] Already initialized');
        return true;
    }

    // Use global Blockly if not provided
    const blocklyInstance = Blockly || window.Blockly;

    if (!blocklyInstance) {
        console.error('[Node Editor] Blockly not available');
        return false;
    }

    console.log('[Node Editor] Initializing...');

    // Register blocks
    NODE_EDITOR_CONFIG.registerBlocks(blocklyInstance);

    // Initialize code generator
    pythonGenerator = initNodeGenerator(blocklyInstance);

    // Create theme
    NODE_EDITOR_CONFIG.getTheme(blocklyInstance);

    // Register with editor registry
    const success = registerEditor(NODE_EDITOR_CONFIG);

    if (success) {
        initialized = true;
        console.log('[Node Editor] Initialized successfully');
    } else {
        console.error('[Node Editor] Registration failed');
    }

    return success;
}

/**
 * Generate Python code from workspace
 * @param {Blockly.Workspace} workspace - The workspace
 * @returns {string} Generated Python code
 */
export function generateCode(workspace) {
    if (!pythonGenerator) {
        console.error('[Node Editor] Generator not initialized');
        return '';
    }
    return generateNodeCode(workspace, pythonGenerator);
}

// Re-export generateCode for external use
export { generateCode as generateNodeCode };

/**
 * Get Node editor configuration
 * @returns {Object} Editor configuration
 */
export function getNodeEditorConfig() {
    return { ...NODE_EDITOR_CONFIG };
}

/**
 * Get categories for Node editor
 * @returns {Array} Category configurations
 */
export function getNodeCategories() {
    return [...NODE_CATEGORIES];
}

/**
 * Get blocks for a specific category
 * @param {string} categoryId - Category ID
 * @returns {string[]} Block type names
 */
export function getNodeBlocksForCategory(categoryId) {
    return getBlocksForCategory(categoryId);
}

/**
 * Check if Node editor is initialized
 * @returns {boolean} Initialization state
 */
export function isNodeEditorInitialized() {
    return initialized;
}

/**
 * Enable real-time validation on a workspace
 * @param {Blockly.Workspace} workspace - The workspace to validate
 */
export function enableRealtimeValidation(workspace) {
    if (!workspace) return;

    // Debounce timer
    let validationTimer = null;

    // Change listener
    const onChange = (event) => {
        // Skip UI events (clicks, scrolling, etc.)
        if (event.type === Blockly.Events.UI) return;

        // Start/Restart timer
        if (validationTimer) clearTimeout(validationTimer);

        validationTimer = setTimeout(() => {
            // Run validation
            const result = validateNode(workspace);

            // Clear existing warnings first
            const allBlocks = workspace.getAllBlocks(false);
            for (const block of allBlocks) {
                // Only clear if it's a node block
                if (block.type.startsWith('node_')) {
                    block.setWarningText(null);
                }
            }

            // Apply new warnings/errors
            if (result.blockErrors) {
                for (const [blockId, msg] of result.blockErrors) {
                    const block = workspace.getBlockById(blockId);
                    if (block) {
                        block.setWarningText(msg);
                    }
                }
            }
        }, 800); // 800ms debounce
    };

    workspace.addChangeListener(onChange);

    // Run once immediately
    onChange({ type: 'create' }); // Dummy event to trigger
}

/**
 * Get the Node theme
 * @returns {Object|null} Theme object or null if not initialized
 */
export function getNodeTheme() {
    return nodeTheme;
}

/**
 * Get the Python generator instance
 * @returns {Object|null} Generator or null if not initialized
 */
export function getPythonGenerator() {
    return pythonGenerator;
}

// Export config for external use
export { NODE_CATEGORIES };

// Re-export orphan block detection and validation functions
export { hasOrphanBlocks, getSkippedBlocks, validateNode };
