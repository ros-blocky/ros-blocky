/**
 * Block Palette
 * Renders Blockly blocks based on active category from editor registry
 * Blocks can be clicked to add to the main workspace
 */

import { getState, onCategoryChange, onEditorChange, hasActiveFile } from './editor-state.js';
import { getEditorById } from './editor-registry.js';
import { t } from '../../../../../../i18n/index.js';

// Note: mainWorkspace is accessed via window.blocksMainWorkspace to avoid circular imports

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
        // NOTE: We no longer render custom blocks here - using Blockly's native flyout instead
        // The icon-sidebar triggers toolbox.setSelectedItem() in editor-state.js
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

    // Update header with translated label
    const header = paletteElement.querySelector('.blocks-sidebar-header');
    if (header) {
        const translationKey = `blocks.urdf.${category.id}`;
        const translated = t(translationKey);
        const translatedLabel = (translated && translated !== translationKey) ? translated : category.label;
        header.textContent = translatedLabel.toUpperCase();
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
            readOnly: true,  // Disable Blockly drag - we use our own
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

        // Build XML for blocks - all at position 0,0 initially
        const blocksXml = buildBlocksXml(category.blocks);

        if (blocksXml) {
            Blockly.Xml.domToWorkspace(
                Blockly.utils.xml.textToDom(blocksXml),
                flyoutWorkspace
            );

            // Resize workspace first to ensure proper container sizing
            flyoutWorkspace.resize();

            // Use requestAnimationFrame to ensure layout is complete before repositioning
            requestAnimationFrame(() => {
                flyoutWorkspace.resize();
                repositionBlocksDynamically(flyoutWorkspace);

                // Create clickable overlay for drag functionality
                createBlockOverlay(flyoutContainer, flyoutWorkspace);
            });
        }

        console.log('[BlockPalette] Rendered blocks for:', categoryId, `(${category.blocks.length} blocks)`);
    } catch (error) {
        console.error('[BlockPalette] Failed to create flyout:', error);
        flyoutContainer.innerHTML = '<p class="palette-error">Failed to load blocks</p>';
    }
}

/**
 * Reposition blocks dynamically based on their actual heights
 * @param {Blockly.Workspace} workspace - The workspace containing blocks
 */
function repositionBlocksDynamically(workspace) {
    const blocks = workspace.getTopBlocks(false);
    const MARGIN = 15; // Space between blocks
    const START_X = 10;
    const START_Y = 10;

    let currentY = START_Y;

    blocks.forEach((block) => {
        // Move block to current position
        block.moveTo(new Blockly.utils.Coordinate(START_X, currentY));

        // Get the block's height and add margin for next block
        const blockHeight = block.getHeightWidth().height;
        currentY += blockHeight + MARGIN;
    });

    // Setup drag handlers for blocks
    setupBlockDragHandlers(workspace);
}

// Track dragging state
let draggedBlockType = null;
let dragGhost = null;
let isDragging = false;
let dropHighlight = null;  // Visual preview of where block will snap

/**
 * Create an overlay with clickable areas for each block
 * @param {HTMLElement} container - The flyout container
 * @param {Blockly.Workspace} workspace - The flyout workspace
 */
function createBlockOverlay(container, workspace) {
    // Remove existing overlay
    const existingOverlay = container.querySelector('.block-overlay');
    if (existingOverlay) {
        existingOverlay.remove();
    }

    // Get the SVG element
    const svg = container.querySelector('svg.blocklySvg');
    if (!svg) return;

    // Create overlay container
    const overlay = document.createElement('div');
    overlay.className = 'block-overlay';
    overlay.style.cssText = `
        position: absolute;
        top: 0;
        left: 0;
        width: 100%;
        height: 100%;
        pointer-events: none;
        z-index: 100;
    `;

    const svgRect = svg.getBoundingClientRect();
    const containerRect = container.getBoundingClientRect();
    const scale = workspace.scale || 0.75;

    // Create a clickable area for each block
    const blocks = workspace.getTopBlocks(false);

    blocks.forEach((block) => {
        const blockRect = block.getBoundingRectangle();

        // Convert workspace coordinates to pixel coordinates
        const left = blockRect.left * scale;
        const top = blockRect.top * scale;
        const width = (blockRect.right - blockRect.left) * scale;
        const height = (blockRect.bottom - blockRect.top) * scale;

        // Create clickable div for this block
        const clickArea = document.createElement('div');
        clickArea.className = 'block-click-area';
        clickArea.dataset.blockType = block.type;
        clickArea.style.cssText = `
            position: absolute;
            left: ${left}px;
            top: ${top}px;
            width: ${width}px;
            height: ${height}px;
            cursor: grab;
            pointer-events: all;
            background: transparent;
        `;

        // Add mousedown handler
        clickArea.addEventListener('mousedown', (e) => {
            console.log('[BlockPalette] Overlay clicked for block:', block.type);
            e.preventDefault();
            e.stopPropagation();
            startBlockDrag(e, block);
        });

        overlay.appendChild(clickArea);
    });

    // Make container relative for absolute positioning
    container.style.position = 'relative';
    container.appendChild(overlay);

    console.log('[BlockPalette] Created overlay with', blocks.length, 'click areas');
}

/**
 * Setup drag handlers on blocks to drag them to main workspace
 * @param {Blockly.Workspace} workspace - The flyout workspace
 */
function setupBlockDragHandlers(workspace) {
    const blocks = workspace.getTopBlocks(false);

    blocks.forEach((block) => {
        // Get the SVG element for the block
        const svg = block.getSvgRoot();
        if (svg) {
            svg.style.cursor = 'grab';

            // Mouse down - start potential drag
            svg.addEventListener('mousedown', (e) => {
                console.log('[BlockPalette] Block mousedown:', block.type);
                e.preventDefault();
                e.stopPropagation();
                startBlockDrag(e, block);
            });

            // Add hover effect
            svg.addEventListener('mouseenter', () => {
                svg.style.filter = 'brightness(1.1)';
            });
            svg.addEventListener('mouseleave', () => {
                svg.style.filter = '';
            });
        }
    });
}

/**
 * Start dragging a block
 */
function startBlockDrag(e, block) {
    console.log('[BlockPalette] Starting drag for:', block.type);
    draggedBlockType = block.type;

    // Clone the block's SVG for visual feedback
    const blockSvg = block.getSvgRoot();
    if (blockSvg) {
        // Create container for the ghost
        dragGhost = document.createElement('div');
        dragGhost.className = 'block-drag-ghost';

        // Clone the SVG
        const svgClone = blockSvg.cloneNode(true);

        // Create an SVG container
        const svgContainer = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
        svgContainer.setAttribute('width', '300');
        svgContainer.setAttribute('height', '100');
        svgContainer.style.overflow = 'visible';
        svgContainer.appendChild(svgClone);

        // Reset transform on the cloned block to position it at origin
        svgClone.setAttribute('transform', 'translate(5, 5)');

        dragGhost.appendChild(svgContainer);
        dragGhost.style.cssText = `
            position: fixed;
            z-index: 10000;
            pointer-events: none;
            opacity: 0.8;
            left: ${e.clientX - 20}px;
            top: ${e.clientY - 20}px;
            filter: drop-shadow(0 4px 8px rgba(0,0,0,0.3));
        `;
        document.body.appendChild(dragGhost);
    }

    // Add global mouse move and up listeners
    document.addEventListener('mousemove', handleDragMove);
    document.addEventListener('mouseup', handleDragEnd);

    // Change cursor
    document.body.style.cursor = 'grabbing';
}

/**
 * Handle mouse move during drag
 */
function handleDragMove(e) {
    if (dragGhost) {
        dragGhost.style.left = `${e.clientX - 20}px`;
        dragGhost.style.top = `${e.clientY - 20}px`;
    }

    // Check if hovering over workspace and show drop preview
    if (draggedBlockType) {
        updateDropPreview(e.clientX, e.clientY);
    }
}

/**
 * Update the drop preview highlight
 */
function updateDropPreview(clientX, clientY) {
    const mainWorkspaceContainer = document.getElementById('blockly-workspace');
    const mainWorkspace = window.blocksMainWorkspace;

    if (!mainWorkspaceContainer || !mainWorkspace) {
        hideDropPreview();
        return;
    }

    const rect = mainWorkspaceContainer.getBoundingClientRect();

    // Check if over workspace
    if (clientX < rect.left || clientX > rect.right ||
        clientY < rect.top || clientY > rect.bottom) {
        hideDropPreview();
        return;
    }

    // Convert to workspace coordinates
    const wsCoords = Blockly.utils.svgMath.screenToWsCoordinates(
        mainWorkspace,
        new Blockly.utils.Coordinate(clientX, clientY)
    );

    // Find nearest compatible connection
    const connection = findNearestConnection(mainWorkspace, wsCoords, draggedBlockType);

    if (connection) {
        showDropPreview(connection, mainWorkspace, rect);
    } else {
        hideDropPreview();
    }
}

/**
 * Find the nearest compatible connection point for a block type
 */
function findNearestConnection(workspace, dropCoords, blockType) {
    const SNAP_RADIUS = 150;
    const allBlocks = workspace.getAllBlocks(false);

    let bestConnection = null;
    let bestDistance = SNAP_RADIUS;

    for (const block of allBlocks) {
        const blockConnections = block.getConnections_(false);

        for (const conn of blockConnections) {
            if (conn.isConnected()) continue;

            // Only look for NEXT_STATEMENT connections (statement inputs) 
            if (conn.type === Blockly.NEXT_STATEMENT) {
                const blockPos = block.getRelativeToSurfaceXY();
                const dx = dropCoords.x - blockPos.x;
                const dy = dropCoords.y - blockPos.y;
                const distance = Math.sqrt(dx * dx + dy * dy);

                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestConnection = conn;
                }
            }
        }
    }

    return bestConnection;
}

/**
 * Show the drop preview highlight at a connection point
 */
function showDropPreview(connection, workspace, containerRect) {
    // Get the connection's position in workspace coords
    const connX = connection.x;
    const connY = connection.y;

    // Convert to screen coords
    const scale = workspace.scale;
    const matrix = workspace.getCanvas().getCTM();

    const screenX = connX * scale + containerRect.left + (matrix ? matrix.e : 0);
    const screenY = connY * scale + containerRect.top + (matrix ? matrix.f : 0);

    // Create or update highlight
    if (!dropHighlight) {
        dropHighlight = document.createElement('div');
        dropHighlight.className = 'drop-preview-highlight';
        dropHighlight.style.cssText = `
            position: fixed;
            width: 120px;
            height: 30px;
            background: rgba(76, 151, 255, 0.3);
            border: 2px dashed #4c97ff;
            border-radius: 8px;
            pointer-events: none;
            z-index: 9999;
            transition: all 0.1s ease;
        `;
        document.body.appendChild(dropHighlight);
    }

    dropHighlight.style.display = 'block';
    dropHighlight.style.left = `${screenX}px`;
    dropHighlight.style.top = `${screenY}px`;
}

/**
 * Hide the drop preview
 */
function hideDropPreview() {
    if (dropHighlight) {
        dropHighlight.style.display = 'none';
    }
}

/**
 * Handle drag end - drop the block
 */
function handleDragEnd(e) {
    document.removeEventListener('mousemove', handleDragMove);
    document.removeEventListener('mouseup', handleDragEnd);
    document.body.style.cursor = '';

    // Remove ghost
    if (dragGhost) {
        dragGhost.remove();
        dragGhost = null;
    }

    // Hide drop preview
    hideDropPreview();

    if (!draggedBlockType) return;

    // Check if dropped over the main workspace
    const mainWorkspaceContainer = document.getElementById('blockly-workspace');
    if (mainWorkspaceContainer) {
        const rect = mainWorkspaceContainer.getBoundingClientRect();
        if (e.clientX >= rect.left && e.clientX <= rect.right &&
            e.clientY >= rect.top && e.clientY <= rect.bottom) {
            // Dropped in workspace - create block at drop position
            createBlockAtPosition(draggedBlockType, e.clientX, e.clientY, rect);
        }
    }

    draggedBlockType = null;
}

/**
 * Create a block at the drop position in the main workspace
 */
function createBlockAtPosition(blockType, clientX, clientY, workspaceRect) {
    const mainWorkspace = window.blocksMainWorkspace;

    if (!mainWorkspace) {
        console.warn('[BlockPalette] Main workspace not available');
        return;
    }

    // Get the workspace SVG element
    const svg = mainWorkspace.getParentSvg();
    const svgRect = svg.getBoundingClientRect();

    // Calculate position relative to the SVG
    const relativeX = clientX - svgRect.left;
    const relativeY = clientY - svgRect.top;

    // Convert to workspace coordinates using Blockly's method
    const wsCoords = Blockly.utils.svgMath.screenToWsCoordinates(
        mainWorkspace,
        new Blockly.utils.Coordinate(clientX, clientY)
    );

    // Create the block in the main workspace
    const newBlock = mainWorkspace.newBlock(blockType);
    newBlock.initSvg();
    newBlock.render();

    // Move to the exact drop position
    newBlock.moveTo(wsCoords);

    // Try to connect to nearby blocks
    tryConnectBlock(newBlock, mainWorkspace, wsCoords);

    console.log('[BlockPalette] Dropped block at workspace coords:', wsCoords.x, wsCoords.y);
}

/**
 * Try to connect a newly dropped block to nearby connection points
 * @param {Blockly.Block} newBlock - The newly created block
 * @param {Blockly.Workspace} workspace - The main workspace
 * @param {Blockly.utils.Coordinate} dropCoords - Where the block was dropped
 */
function tryConnectBlock(newBlock, workspace, dropCoords) {
    const SNAP_RADIUS = 100; // Larger radius for easier snapping

    console.log('[BlockPalette] Trying to connect block:', newBlock.type, 'at', dropCoords.x, dropCoords.y);

    // Get all connections on the new block
    const newBlockConnections = newBlock.getConnections_(false);
    console.log('[BlockPalette] New block has', newBlockConnections.length, 'connections');

    // Get all blocks in the workspace (excluding the new one)
    const allBlocks = workspace.getAllBlocks(false).filter(b => b !== newBlock);
    console.log('[BlockPalette] Checking against', allBlocks.length, 'other blocks');

    let bestConnection = null;
    let bestDistance = SNAP_RADIUS;
    let newBlockConnection = null;

    // Check each connection on the new block
    for (const newConn of newBlockConnections) {
        // Skip if already connected
        if (newConn.isConnected()) continue;

        const connType = newConn.type; // 1=input, 2=output, 3=prev, 4=next
        console.log('[BlockPalette] Checking connection type:', connType, 'at', newConn.x, newConn.y);

        // Check against all other blocks
        for (const block of allBlocks) {
            const blockConnections = block.getConnections_(false);

            for (const targetConn of blockConnections) {
                // Skip if already connected
                if (targetConn.isConnected()) continue;

                // Check if connection types are compatible manually
                // PREVIOUS_STATEMENT connects to NEXT_STATEMENT (inside statement inputs)
                // OUTPUT_VALUE connects to INPUT_VALUE
                const compatible =
                    (newConn.type === Blockly.PREVIOUS_STATEMENT && targetConn.type === Blockly.NEXT_STATEMENT) ||
                    (newConn.type === Blockly.OUTPUT_VALUE && targetConn.type === Blockly.INPUT_VALUE);

                if (compatible) {
                    // Use drop coordinates for distance calculation (not connection coords)
                    const blockPos = block.getRelativeToSurfaceXY();
                    const dx = dropCoords.x - blockPos.x;
                    const dy = dropCoords.y - blockPos.y;
                    const distance = Math.sqrt(dx * dx + dy * dy);

                    console.log('[BlockPalette] Found compatible connection on', block.type,
                        'distance:', distance.toFixed(0));

                    if (distance < bestDistance) {
                        bestDistance = distance;
                        bestConnection = targetConn;
                        newBlockConnection = newConn;
                    }
                }
            }
        }
    }

    // If we found a nearby compatible connection, connect!
    if (bestConnection && newBlockConnection) {
        try {
            newBlockConnection.connect(bestConnection);
            console.log('[BlockPalette] ✓ Auto-connected to:', bestConnection.getSourceBlock().type);
        } catch (e) {
            console.error('[BlockPalette] Connection failed:', e.message);
        }
    } else {
        console.log('[BlockPalette] No compatible connection found within', SNAP_RADIUS, 'pixels');
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

    // All blocks start at 0,0 - repositionBlocksDynamically will arrange them
    const blockElements = blockTypes.map(blockType => {
        // Check if block type is registered
        if (!Blockly.Blocks[blockType]) {
            console.warn('[BlockPalette] Block type not registered:', blockType);
            return '';
        }
        return `<block type="${sanitizeXml(blockType)}" x="0" y="0"></block>`;
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
