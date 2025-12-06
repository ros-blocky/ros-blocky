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
        const { initPackagesPanel } = await import('./components/packages-panel/packages-panel.js');
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

    resizeHandle.addEventListener('mousedown', (e) => {
        isResizing = true;
        startX = e.clientX;
        startBlocksWidth = blocksPanel.offsetWidth;
        startPackagesWidth = packagesPanel.offsetWidth;
        document.body.style.cursor = 'ew-resize';
        document.body.style.userSelect = 'none';
        e.preventDefault();
    });

    document.addEventListener('mousemove', (e) => {
        if (!isResizing) return;

        const diff = e.clientX - startX;
        const newBlocksWidth = startBlocksWidth + diff;
        const newPackagesWidth = startPackagesWidth - diff;

        // Apply min/max constraints
        if (newBlocksWidth >= 300 && newPackagesWidth >= 200 && newPackagesWidth <= 500) {
            blocksPanel.style.flex = `0 0 ${newBlocksWidth}px`;
            packagesPanel.style.width = `${newPackagesWidth}px`;
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
