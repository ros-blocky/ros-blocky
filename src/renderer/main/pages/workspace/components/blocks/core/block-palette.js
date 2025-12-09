/**
 * Block Palette
 * Renders block options based on active category
 */

import { getState, onCategoryChange, onEditorChange } from './editor-state.js';

// Block definitions per category (for URDF editor)
const URDF_BLOCKS = {
    links: [
        { id: 'link', label: 'Link', icon: '🔗', color: '#667eea' },
        { id: 'visual', label: 'Visual', icon: '👁️', color: '#764ba2' },
        { id: 'collision', label: 'Collision', icon: '💥', color: '#8b5cf6' },
        { id: 'inertial', label: 'Inertial', icon: '⚖️', color: '#a78bfa' }
    ],
    joints: [
        { id: 'joint_fixed', label: 'Fixed Joint', icon: '🔒', color: '#11998e' },
        { id: 'joint_revolute', label: 'Revolute', icon: '🔄', color: '#38ef7d' },
        { id: 'joint_continuous', label: 'Continuous', icon: '♾️', color: '#10b981' },
        { id: 'joint_prismatic', label: 'Prismatic', icon: '↔️', color: '#059669' },
        { id: 'axis', label: 'Axis', icon: '➡️', color: '#34d399' },
        { id: 'limit', label: 'Limit', icon: '🚧', color: '#6ee7b7' }
    ],
    geometry: [
        { id: 'box', label: 'Box', icon: '📦', color: '#ee0979' },
        { id: 'cylinder', label: 'Cylinder', icon: '🛢️', color: '#ff6a00' },
        { id: 'sphere', label: 'Sphere', icon: '🔴', color: '#f97316' },
        { id: 'mesh', label: 'Mesh', icon: '🦾', color: '#fb923c' }
    ],
    materials: [
        { id: 'material', label: 'Material', icon: '🎨', color: '#f093fb' },
        { id: 'color', label: 'Color RGBA', icon: '🌈', color: '#f5576c' }
    ],
    physics: [
        { id: 'mass', label: 'Mass', icon: '⚖️', color: '#4facfe' },
        { id: 'inertia', label: 'Inertia', icon: '🌀', color: '#00f2fe' },
        { id: 'origin', label: 'Origin XYZ RPY', icon: '📍', color: '#22d3ee' }
    ]
};

// Store all block configs for different editors
const BLOCK_CONFIGS = {
    urdf: URDF_BLOCKS,
    // node: NODE_BLOCKS,   // Future
    // config: CONFIG_BLOCKS, // Future
    // launch: LAUNCH_BLOCKS  // Future
};

let paletteElement = null;
let currentEditorType = null;

/**
 * Initialize the block palette
 * @param {string} containerId - ID of the palette container
 */
export function initBlockPalette(containerId) {
    paletteElement = document.getElementById(containerId);
    if (!paletteElement) {
        console.error('[BlockPalette] Container not found:', containerId);
        return;
    }

    // Listen for editor type changes
    onEditorChange((editorType) => {
        currentEditorType = editorType;
        // Clear palette when editor changes
        renderBlocks(null);
    });

    // Listen for category changes
    onCategoryChange((category) => {
        renderBlocks(category);
    });

    // Render empty state initially
    renderEmptyState();

    console.log('[BlockPalette] Initialized');
}

/**
 * Render blocks for the given category
 * @param {string|null} category 
 */
function renderBlocks(category) {
    if (!paletteElement) return;

    if (!currentEditorType || !category) {
        renderEmptyState();
        return;
    }

    const editorBlocks = BLOCK_CONFIGS[currentEditorType];
    if (!editorBlocks) {
        renderEmptyState();
        return;
    }

    const blocks = editorBlocks[category];
    if (!blocks || blocks.length === 0) {
        renderEmptyState();
        return;
    }

    // Find category label
    const categoryLabel = category.charAt(0).toUpperCase() + category.slice(1);

    const blocksHtml = blocks.map(block => `
        <div class="block-item" 
             data-block-id="${block.id}"
             draggable="true"
             style="background: linear-gradient(135deg, ${block.color} 0%, ${adjustColor(block.color, -20)} 100%)">
            <span class="block-icon">${block.icon}</span>
            <span class="block-label">${block.label}</span>
        </div>
    `).join('');

    paletteElement.innerHTML = `
        <div class="palette-header">${categoryLabel.toUpperCase()}</div>
        <div class="palette-blocks">
            ${blocksHtml}
        </div>
    `;

    // Attach drag handlers
    paletteElement.querySelectorAll('.block-item').forEach(item => {
        item.addEventListener('dragstart', handleDragStart);
        item.addEventListener('click', handleBlockClick);
    });

    console.log('[BlockPalette] Rendered blocks for:', category);
}

/**
 * Render empty state
 */
function renderEmptyState() {
    if (!paletteElement) return;

    paletteElement.innerHTML = `
        <div class="palette-header">BLOCKS</div>
        <div class="palette-empty">
            <p>Select a file and category</p>
        </div>
    `;
}

/**
 * Handle drag start for blocks
 * @param {DragEvent} e 
 */
function handleDragStart(e) {
    const blockId = e.target.dataset.blockId;
    e.dataTransfer.setData('text/plain', blockId);
    e.dataTransfer.effectAllowed = 'copy';

    console.log('[BlockPalette] Drag started:', blockId);
}

/**
 * Handle block click (for adding to workspace)
 * @param {MouseEvent} e 
 */
function handleBlockClick(e) {
    const blockItem = e.currentTarget;
    const blockId = blockItem.dataset.blockId;

    // Dispatch custom event for editor to handle
    const event = new CustomEvent('blockSelected', {
        detail: { blockId, editorType: currentEditorType }
    });
    document.dispatchEvent(event);

    console.log('[BlockPalette] Block clicked:', blockId);
}

/**
 * Adjust color brightness
 * @param {string} hex 
 * @param {number} amount 
 * @returns {string}
 */
function adjustColor(hex, amount) {
    const num = parseInt(hex.replace('#', ''), 16);
    const r = Math.max(0, Math.min(255, (num >> 16) + amount));
    const g = Math.max(0, Math.min(255, ((num >> 8) & 0x00FF) + amount));
    const b = Math.max(0, Math.min(255, (num & 0x0000FF) + amount));
    return `#${((r << 16) | (g << 8) | b).toString(16).padStart(6, '0')}`;
}

/**
 * Get blocks for a specific editor type and category
 * @param {string} editorType 
 * @param {string} category 
 * @returns {Array}
 */
export function getBlocksForCategory(editorType, category) {
    return BLOCK_CONFIGS[editorType]?.[category] || [];
}
