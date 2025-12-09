/**
 * Blocks Component
 * Handles the visual block-based programming interface
 */

let initialized = false;
let currentFileType = null;

// URDF icon configuration (Option A organization)
const URDF_ICONS = [
    { id: 'structure', icon: '🏗️', label: 'Structure' },
    { id: 'geometry', icon: '📦', label: 'Geometry' },
    { id: 'transform', icon: '📍', label: 'Transform' },
    { id: 'appearance', icon: '🎨', label: 'Appearance' },
    { id: 'physics', icon: '⚡', label: 'Physics' }
];

// Default icons
const DEFAULT_ICONS = [
    { id: 'grid', icon: '⊞', label: 'Grid View' },
    { id: 'search', icon: '🔍', label: 'Search' },
    { id: 'blocks', icon: '📊', label: 'Blocks', active: true },
    { id: 'run', icon: '▶️', label: 'Run' }
];

// URDF blocks per category (Option A - no duplicates)
const URDF_BLOCKS = {
    structure: [
        { id: 'robot', label: 'Robot', icon: '🤖', color: '#667eea' },
        { id: 'link', label: 'Link', icon: '🔗', color: '#764ba2' },
        { id: 'joint_fixed', label: 'Fixed Joint', icon: '🔒', color: '#8b5cf6' },
        { id: 'joint_revolute', label: 'Revolute Joint', icon: '🔄', color: '#a78bfa' },
        { id: 'joint_continuous', label: 'Continuous Joint', icon: '♾️', color: '#c4b5fd' },
        { id: 'joint_prismatic', label: 'Prismatic Joint', icon: '↔️', color: '#ddd6fe' }
    ],
    geometry: [
        { id: 'box', label: 'Box', icon: '📦', color: '#ee0979' },
        { id: 'cylinder', label: 'Cylinder', icon: '🛢️', color: '#ff6a00' },
        { id: 'sphere', label: 'Sphere', icon: '🔴', color: '#f97316' },
        { id: 'mesh', label: 'Mesh', icon: '🦾', color: '#fb923c' },
        { id: 'visual', label: 'Visual', icon: '👁️', color: '#fdba74' },
        { id: 'collision', label: 'Collision', icon: '💥', color: '#fed7aa' }
    ],
    transform: [
        { id: 'origin', label: 'Origin XYZ RPY', icon: '📍', color: '#11998e' },
        { id: 'axis', label: 'Axis', icon: '➡️', color: '#38ef7d' }
    ],
    appearance: [
        { id: 'material', label: 'Material', icon: '🎨', color: '#f093fb' },
        { id: 'color', label: 'Color RGBA', icon: '🌈', color: '#f5576c' },
        { id: 'texture', label: 'Texture', icon: '🖼️', color: '#ec4899' }
    ],
    physics: [
        { id: 'mass', label: 'Mass', icon: '⚖️', color: '#4facfe' },
        { id: 'inertia', label: 'Inertia', icon: '🌀', color: '#00f2fe' },
        { id: 'inertial', label: 'Inertial', icon: '⚙️', color: '#22d3ee' },
        { id: 'limit', label: 'Limit', icon: '🚧', color: '#67e8f9' },
        { id: 'dynamics', label: 'Dynamics', icon: '📊', color: '#a5f3fc' }
    ]
};

// Open files state for tabs
let openFiles = [];
let activeFile = null;

// File type icons
const FILE_ICONS = {
    urdf: '🤖',
    xacro: '🤖',
    py: '🐍',
    launch: '🚀',
    yaml: '⚙️',
    xml: '📄',
    default: '📄'
};

/**
 * Initialize the blocks component
 */
export function initBlocks() {
    if (initialized) {
        console.log('[Blocks] Already initialized');
        return;
    }

    console.log('[Blocks] Initializing...');

    // Setup sidebar resize
    setupSidebarResize();

    // Listen for file selection events from packages panel
    window.addEventListener('fileSelected', handleFileSelected);

    initialized = true;
    console.log('[Blocks] Component initialized');
}

/**
 * Setup sidebar resize functionality
 */
function setupSidebarResize() {
    const resizeHandle = document.querySelector('.blocks-sidebar-resize-handle');
    const blocksSidebar = document.querySelector('.blocks-sidebar');

    if (!resizeHandle || !blocksSidebar) return;

    let isResizing = false;
    let startX = 0;
    let startWidth = 0;
    let lastExpandedWidth = 200;

    const COLLAPSE_THRESHOLD = 80;
    const MIN_WIDTH = 150;
    const MAX_WIDTH = 350;

    // Double-click to toggle collapse
    resizeHandle.addEventListener('dblclick', () => {
        if (blocksSidebar.classList.contains('collapsed')) {
            blocksSidebar.classList.remove('collapsed');
            blocksSidebar.style.width = `${lastExpandedWidth}px`;
        } else {
            lastExpandedWidth = blocksSidebar.offsetWidth;
            blocksSidebar.classList.add('collapsed');
            blocksSidebar.style.width = '0px';
        }
    });

    resizeHandle.addEventListener('mousedown', (e) => {
        isResizing = true;
        startX = e.clientX;
        startWidth = blocksSidebar.classList.contains('collapsed') ? 0 : blocksSidebar.offsetWidth;
        document.body.style.cursor = 'ew-resize';
        document.body.style.userSelect = 'none';
        e.preventDefault();
    });

    document.addEventListener('mousemove', (e) => {
        if (!isResizing) return;

        const diff = e.clientX - startX;
        const newWidth = startWidth + diff;

        if (newWidth < COLLAPSE_THRESHOLD) {
            blocksSidebar.classList.add('collapsed');
            blocksSidebar.style.width = '0px';
        } else {
            blocksSidebar.classList.remove('collapsed');
            const clampedWidth = Math.max(MIN_WIDTH, Math.min(MAX_WIDTH, newWidth));
            blocksSidebar.style.width = `${clampedWidth}px`;
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

/**
 * Handle file selection event - opens file as tab
 */
function handleFileSelected(event) {
    const { type, fileName, packageName } = event.detail;
    console.log('[Blocks] File selected event received:', type, fileName);

    // Create file info object
    const fileInfo = {
        fileName: fileName,
        packageName: packageName,
        type: type
    };

    // Check if file already open
    const existingIndex = openFiles.findIndex(f =>
        f.fileName === fileName && f.packageName === packageName
    );

    if (existingIndex >= 0) {
        // File already open, activate it
        activeFile = openFiles[existingIndex];
    } else {
        // Add new file
        openFiles.push(fileInfo);
        activeFile = fileInfo;
    }

    // Render tabs and breadcrumb
    renderTabs();
    renderBreadcrumb();

    // Update sidebar based on file type
    if (type === 'urdf') {
        currentFileType = 'urdf';
        renderUrdfIcons();
    } else {
        currentFileType = type;
        renderDefaultIcons();
    }

    // Hide placeholder
    const placeholder = document.getElementById('editor-placeholder');
    if (placeholder) {
        placeholder.style.display = 'none';
    }
}

/**
 * Render file tabs
 */
function renderTabs() {
    const tabsContainer = document.getElementById('editor-tabs');
    if (!tabsContainer) return;

    if (openFiles.length === 0) {
        tabsContainer.innerHTML = '';
        return;
    }

    tabsContainer.innerHTML = openFiles.map((file, index) => {
        const isActive = activeFile &&
            file.fileName === activeFile.fileName &&
            file.packageName === activeFile.packageName;

        const ext = file.fileName.split('.').pop().toLowerCase();
        const icon = FILE_ICONS[ext] || FILE_ICONS.default;

        return `
            <div class="editor-tab ${isActive ? 'active' : ''}" data-index="${index}">
                <span class="tab-icon">${icon}</span>
                <span class="tab-name">${file.fileName}</span>
                <span class="tab-close" data-index="${index}">×</span>
            </div>
        `;
    }).join('');

    // Attach click handlers
    tabsContainer.querySelectorAll('.editor-tab').forEach(tab => {
        tab.addEventListener('click', (e) => {
            if (e.target.classList.contains('tab-close')) return;

            const index = parseInt(tab.dataset.index);
            activeFile = openFiles[index];
            renderTabs();
            renderBreadcrumb();

            // Update icons for file type
            if (activeFile.type === 'urdf') {
                renderUrdfIcons();
            } else {
                renderDefaultIcons();
            }
        });
    });

    // Attach close handlers
    tabsContainer.querySelectorAll('.tab-close').forEach(closeBtn => {
        closeBtn.addEventListener('click', (e) => {
            e.stopPropagation();
            const index = parseInt(closeBtn.dataset.index);
            closeTab(index);
        });
    });
}

/**
 * Close a tab
 */
function closeTab(index) {
    const wasActive = activeFile &&
        openFiles[index].fileName === activeFile.fileName &&
        openFiles[index].packageName === activeFile.packageName;

    openFiles.splice(index, 1);

    if (wasActive) {
        if (openFiles.length > 0) {
            const newIndex = Math.min(index, openFiles.length - 1);
            activeFile = openFiles[newIndex];
        } else {
            activeFile = null;
            // Show placeholder
            const placeholder = document.getElementById('editor-placeholder');
            if (placeholder) {
                placeholder.style.display = 'flex';
            }
            renderDefaultIcons();
        }
    }

    renderTabs();
    renderBreadcrumb();
}

/**
 * Render breadcrumb
 */
function renderBreadcrumb() {
    const breadcrumbContainer = document.getElementById('editor-breadcrumb');
    if (!breadcrumbContainer) return;

    if (!activeFile) {
        breadcrumbContainer.innerHTML = '';
        return;
    }

    const folderNames = {
        urdf: 'urdf',
        node: 'scripts',
        config: 'config',
        launch: 'launch'
    };
    const folder = folderNames[activeFile.type] || activeFile.type;

    const ext = activeFile.fileName.split('.').pop().toLowerCase();
    const icon = FILE_ICONS[ext] || FILE_ICONS.default;

    breadcrumbContainer.innerHTML = `
        <span class="breadcrumb-item">
            <span class="breadcrumb-icon">📦</span>
            ${activeFile.packageName}
        </span>
        <span class="breadcrumb-separator">›</span>
        <span class="breadcrumb-item">
            <span class="breadcrumb-icon">📁</span>
            ${folder}
        </span>
        <span class="breadcrumb-separator">›</span>
        <span class="breadcrumb-item">
            <span class="breadcrumb-icon">${icon}</span>
            ${activeFile.fileName}
        </span>
    `;
}

/**
 * Render URDF-specific icons in the sidebar
 */
function renderUrdfIcons() {
    const iconSidebar = document.querySelector('.blocks-icon-sidebar');
    if (!iconSidebar) return;

    const topIcons = URDF_ICONS.map((icon, index) => `
        <button class="sidebar-icon-btn ${index === 0 ? 'active' : ''}" 
                data-category="${icon.id}" 
                title="${icon.label}">
            <span class="icon-emoji">${icon.icon}</span>
        </button>
    `).join('');

    iconSidebar.innerHTML = `
        <div class="icon-sidebar-top">
            ${topIcons}
        </div>
        <div class="icon-sidebar-bottom">
            <button class="sidebar-icon-btn" title="Settings">
                <span class="icon-emoji">⭐</span>
            </button>
        </div>
    `;

    // Attach click handlers to icons
    iconSidebar.querySelectorAll('.sidebar-icon-btn[data-category]').forEach(btn => {
        btn.addEventListener('click', () => {
            // Remove active from all
            iconSidebar.querySelectorAll('.sidebar-icon-btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');

            const category = btn.dataset.category;
            renderBlocksForCategory(category);
        });
    });

    // Render blocks for first category
    renderBlocksForCategory('structure');
}

/**
 * Render default icons in the sidebar
 */
function renderDefaultIcons() {
    const iconSidebar = document.querySelector('.blocks-icon-sidebar');
    if (!iconSidebar) return;

    const topIcons = DEFAULT_ICONS.map(icon => `
        <button class="sidebar-icon-btn ${icon.active ? 'active' : ''}" title="${icon.label}">
            <span class="icon-emoji">${icon.icon}</span>
        </button>
    `).join('');

    iconSidebar.innerHTML = `
        <div class="icon-sidebar-top">
            ${topIcons}
        </div>
        <div class="icon-sidebar-bottom">
            <button class="sidebar-icon-btn" title="Favorites">
                <span class="icon-emoji">⭐</span>
            </button>
        </div>
    `;
}

/**
 * Render blocks for a category in the sidebar
 */
function renderBlocksForCategory(category) {
    const blocksSidebar = document.querySelector('.blocks-sidebar');
    if (!blocksSidebar) return;

    const blocks = URDF_BLOCKS[category] || [];
    const categoryLabel = category.charAt(0).toUpperCase() + category.slice(1);

    const blocksHtml = blocks.map(block => `
        <div class="blocks-category" 
             style="background: linear-gradient(135deg, ${block.color} 0%, ${adjustColor(block.color, -20)} 100%)"
             data-block-id="${block.id}">
            <span class="blocks-category-icon">${block.icon}</span>
            <span class="blocks-category-label">${block.label}</span>
        </div>
    `).join('');

    blocksSidebar.innerHTML = `
        <div class="blocks-sidebar-header">${categoryLabel.toUpperCase()}</div>
        ${blocksHtml}
    `;

    // Attach click handlers
    blocksSidebar.querySelectorAll('.blocks-category').forEach(block => {
        block.addEventListener('click', () => {
            const blockId = block.dataset.blockId;
            console.log('[Blocks] Block clicked:', blockId);
            // Future: Add block to Blockly workspace
        });
    });
}

/**
 * Update editor placeholder text
 */
function updateEditorPlaceholder(title, subtitle) {
    const placeholder = document.querySelector('.blocks-placeholder');
    if (!placeholder) return;

    const h2 = placeholder.querySelector('h2');
    const p = placeholder.querySelector('p');

    if (h2) h2.textContent = title;
    if (p) p.textContent = subtitle;
}

/**
 * Adjust color brightness
 */
function adjustColor(hex, amount) {
    const num = parseInt(hex.replace('#', ''), 16);
    const r = Math.max(0, Math.min(255, (num >> 16) + amount));
    const g = Math.max(0, Math.min(255, ((num >> 8) & 0x00FF) + amount));
    const b = Math.max(0, Math.min(255, (num & 0x0000FF) + amount));
    return `#${((r << 16) | (g << 8) | b).toString(16).padStart(6, '0')}`;
}

/**
 * Setup tabs
 */
function setupTabs() {
    const tabs = document.querySelectorAll('.blocks-tab');

    tabs.forEach(tab => {
        tab.addEventListener('click', () => {
            tabs.forEach(t => t.classList.remove('active'));
            tab.classList.add('active');
            console.log('Tab clicked:', tab.textContent);
        });
    });
}

/**
 * Reset the blocks component
 */
export function resetBlocks() {
    initialized = false;
    currentFileType = null;
    window.removeEventListener('fileSelected', handleFileSelected);
    console.log('[Blocks] Reset');
}
