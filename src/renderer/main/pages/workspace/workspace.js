/**
 * Workspace Module
 * Manages the main workspace layout with blocks and packages panels
 */

export async function initWorkspace() {
    console.log('Initializing workspace...');

    // Load blocks component
    const blocksContainer = document.getElementById('blocks-container');
    if (blocksContainer) {
        const blocksResponse = await fetch('pages/workspace/components/blocks/blocks.html');
        const blocksHtml = await blocksResponse.text();
        blocksContainer.innerHTML = blocksHtml;

        // Initialize blocks module
        const { initBlocks } = await import('./components/blocks/blocks.js');
        initBlocks();
    }

    // Load packages panel component
    const packagesPanelContainer = document.getElementById('packages-panel-container');
    if (packagesPanelContainer) {
        const packagesResponse = await fetch('pages/workspace/components/packages-panel/packages-panel.html');
        const packagesHtml = await packagesResponse.text();
        packagesPanelContainer.innerHTML = packagesHtml;

        // Initialize packages panel module
        const { initPackagesPanel } = await import('./components/packages-panel/index.js');
        initPackagesPanel();
    }

    // Setup resize handle
    setupResizeHandle();

    console.log('Workspace initialized');
}

/**
 * Setup resize handle for workspace panels
 */
function setupResizeHandle() {
    const resizeHandle = document.querySelector('.workspace-resize-handle');
    const blocksPanel = document.querySelector('.blocks-panel');
    const packagesPanel = document.querySelector('.packages-panel');

    if (!resizeHandle || !blocksPanel || !packagesPanel) return;

    let isResizing = false;
    let startX = 0;
    let startBlocksWidth = 0;
    let startPackagesWidth = 0;
    let lastExpandedWidth = 300; // Remember last width before collapse

    const COLLAPSE_THRESHOLD = 100; // Collapse when width goes below this
    const MIN_WIDTH = 200;
    const MAX_WIDTH = 500;

    // Double-click to toggle collapse
    resizeHandle.addEventListener('dblclick', () => {
        if (packagesPanel.classList.contains('collapsed')) {
            // Expand
            packagesPanel.classList.remove('collapsed');
            packagesPanel.style.width = `${lastExpandedWidth}px`;
            blocksPanel.style.flex = '';  // Reset to CSS default
            resizeHandle.style.display = '';
        } else {
            // Collapse
            lastExpandedWidth = packagesPanel.offsetWidth;
            packagesPanel.classList.add('collapsed');
            packagesPanel.style.width = '0px';
            blocksPanel.style.flex = '1';  // Expand blocks to fill space
        }
    });

    resizeHandle.addEventListener('mousedown', (e) => {
        isResizing = true;
        startX = e.clientX;
        startBlocksWidth = blocksPanel.offsetWidth;
        startPackagesWidth = packagesPanel.classList.contains('collapsed') ? 0 : packagesPanel.offsetWidth;
        document.body.style.cursor = 'ew-resize';
        document.body.style.userSelect = 'none';
        e.preventDefault();
    });

    document.addEventListener('mousemove', (e) => {
        if (!isResizing) return;

        const diff = e.clientX - startX;
        const newPackagesWidth = startPackagesWidth - diff;

        // Check if should collapse (dragged to the right past threshold)
        if (newPackagesWidth < COLLAPSE_THRESHOLD) {
            // Collapse the panel
            packagesPanel.classList.add('collapsed');
            packagesPanel.style.width = '0px';
            blocksPanel.style.flex = '1';
        } else {
            // Expand/resize normally
            packagesPanel.classList.remove('collapsed');
            blocksPanel.style.flex = '1';  // Always fill remaining space

            // Clamp packages width between MIN and MAX
            const clampedWidth = Math.max(MIN_WIDTH, Math.min(MAX_WIDTH, newPackagesWidth));
            packagesPanel.style.width = `${clampedWidth}px`;
            lastExpandedWidth = clampedWidth;
        }
    });

    document.addEventListener('mouseup', () => {
        if (isResizing) {
            isResizing = false;
            document.body.style.cursor = '';
            document.body.style.userSelect = '';
        }
    });
}
