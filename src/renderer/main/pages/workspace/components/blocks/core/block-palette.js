/**
 * Block Palette
 * Renders Blockly blocks based on active category from editor registry
 * Hides when no file is selected
 */

import { getState, onCategoryChange, onEditorChange, hasActiveFile } from './editor-state.js';
import { getEditorById } from './editor-registry.js';

let paletteElement = null;
let paletteContainer = null;
let flyoutWorkspace = null;
let currentEditorType = null;
let currentCategory = null;

/**
 * Initialize the block palette
 * @param {string} containerId - ID of the palette container
 */
export function initBlockPalette(containerId) {
    paletteElement = document.getElementById(containerId);
    if (!paletteElement) {
        // Try to find by class
        paletteElement = document.querySelector('.blocks-sidebar');
    }

    if (!paletteElement) {
        console.error('[BlockPalette] Container not found:', containerId);
        return;
    }

    paletteContainer = paletteElement;

    // Listen for editor type changes
    onEditorChange((editorType) => {
        currentEditorType = editorType;
        currentCategory = null;

        if (editorType) {
            showPalette();
        } else {
            hidePalette();
        }
    });

    // Listen for category changes
    onCategoryChange((category) => {
        currentCategory = category;
        if (currentEditorType && category) {
            renderBlocksForCategory(currentEditorType, category);
        }
    });

    // Initially hide if no file is open
    if (!hasActiveFile()) {
        hidePalette();
    }

    console.log('[BlockPalette] Initialized');
}

/**
 * Show the palette
 */
export function showPalette() {
    if (paletteContainer) {
        paletteContainer.classList.remove('hidden');
        paletteContainer.style.display = '';
    }
}

/**
 * Hide the palette
 */
export function hidePalette() {
    if (paletteContainer) {
        paletteContainer.classList.add('hidden');
        paletteContainer.style.display = 'none';
    }
    // Dispose flyout workspace if exists
    if (flyoutWorkspace) {
        flyoutWorkspace.dispose();
        flyoutWorkspace = null;
    }
}

/**
 * Render Blockly blocks for a category
 * @param {string} editorType - Editor type ID
 * @param {string} categoryId - Category ID
 */
function renderBlocksForCategory(editorType, categoryId) {
    if (!paletteElement) return;

    // Get editor config from registry
    const editorConfig = getEditorById(editorType);
    if (!editorConfig) {
        console.warn('[BlockPalette] Editor not found:', editorType);
        return;
    }

    // Find category
    const category = editorConfig.categories.find(c => c.id === categoryId);
    if (!category) {
        console.warn('[BlockPalette] Category not found:', categoryId);
        return;
    }

    // Get flyout container
    const flyoutContainer = paletteElement.querySelector('.blockly-flyout-container') ||
        paletteElement.querySelector('#blockly-flyout');

    if (!flyoutContainer) {
        console.error('[BlockPalette] Flyout container not found');
        return;
    }

    // Update header
    const header = paletteElement.querySelector('.blocks-sidebar-header');
    if (header) {
        header.textContent = category.label.toUpperCase();
    }

    // Dispose existing flyout workspace
    if (flyoutWorkspace) {
        flyoutWorkspace.dispose();
        flyoutWorkspace = null;
    }

    // Clear container
    flyoutContainer.innerHTML = '';

    // Check if Blockly is available
    if (typeof Blockly === 'undefined') {
        console.error('[BlockPalette] Blockly not loaded');
        flyoutContainer.innerHTML = '<p class="palette-error">Blockly not loaded</p>';
        return;
    }

    // Create flyout workspace with Zelos renderer
    try {
        flyoutWorkspace = Blockly.inject(flyoutContainer, {
            renderer: 'zelos',
            readOnly: true,
            scrollbars: false,
            zoom: {
                controls: false,
                wheel: false,
                startScale: 0.75
            },
            move: {
                scrollbars: false,
                drag: false,
                wheel: false
            },
            sounds: false,
            trashcan: false
        });

        // Build XML for blocks in this category
        const blocksXml = buildBlocksXml(category.blocks);

        if (blocksXml) {
            Blockly.Xml.domToWorkspace(
                Blockly.utils.xml.textToDom(blocksXml),
                flyoutWorkspace
            );
        }

        console.log('[BlockPalette] Rendered blocks for:', categoryId, `(${category.blocks.length} blocks)`);
    } catch (error) {
        console.error('[BlockPalette] Failed to create flyout:', error);
        flyoutContainer.innerHTML = '<p class="palette-error">Failed to load blocks</p>';
    }
}

/**
 * Build XML string for blocks
 * @param {string[]} blockTypes - Array of block type names
 * @returns {string} XML string
 */
function buildBlocksXml(blockTypes) {
    if (!blockTypes || blockTypes.length === 0) {
        return null;
    }

    let y = 10;
    const blockElements = blockTypes.map(blockType => {
        // Check if block type is registered
        if (!Blockly.Blocks[blockType]) {
            console.warn('[BlockPalette] Block type not registered:', blockType);
            return '';
        }
        const element = `<block type="${sanitizeXml(blockType)}" x="10" y="${y}"></block>`;
        y += 80; // Spacing between blocks
        return element;
    }).filter(Boolean);

    if (blockElements.length === 0) {
        return null;
    }

    return `<xml>${blockElements.join('')}</xml>`;
}

/**
 * Sanitize string for XML
 * @param {string} str - String to sanitize
 * @returns {string} Sanitized string
 */
function sanitizeXml(str) {
    if (typeof str !== 'string') return '';
    return str
        .replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;')
        .replace(/'/g, '&apos;');
}

/**
 * Get current flyout workspace
 * @returns {Object|null} Blockly workspace or null
 */
export function getFlyoutWorkspace() {
    return flyoutWorkspace;
}

/**
 * Check if palette is visible
 * @returns {boolean} True if visible
 */
export function isPaletteVisible() {
    return paletteContainer && !paletteContainer.classList.contains('hidden');
}

/**
 * Resize the flyout workspace (call after container resize)
 */
export function resizeFlyout() {
    if (flyoutWorkspace) {
        Blockly.svgResize(flyoutWorkspace);
    }
}
