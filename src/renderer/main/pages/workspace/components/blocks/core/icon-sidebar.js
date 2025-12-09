/**
 * Dynamic Icon Sidebar
 * Renders icons based on active file type
 */

import { getState, setActiveCategory, onEditorChange, onCategoryChange } from './editor-state.js';

// Icon configurations per editor type
const ICON_CONFIGS = {
    urdf: [
        { id: 'links', icon: '🔗', label: 'Links', color: '#667eea' },
        { id: 'joints', icon: '⚙️', label: 'Joints', color: '#11998e' },
        { id: 'geometry', icon: '📦', label: 'Geometry', color: '#ee0979' },
        { id: 'materials', icon: '🎨', label: 'Materials', color: '#f093fb' },
        { id: 'physics', icon: '⚡', label: 'Physics', color: '#4facfe' }
    ],
    node: [
        { id: 'publishers', icon: '📤', label: 'Publishers', color: '#667eea' },
        { id: 'subscribers', icon: '📥', label: 'Subscribers', color: '#11998e' },
        { id: 'services', icon: '🔄', label: 'Services', color: '#ee0979' },
        { id: 'timers', icon: '⏱️', label: 'Timers', color: '#f093fb' },
        { id: 'control', icon: '🎮', label: 'Control', color: '#4facfe' }
    ],
    config: [
        { id: 'parameters', icon: '⚙️', label: 'Parameters', color: '#667eea' },
        { id: 'lists', icon: '📋', label: 'Lists', color: '#11998e' },
        { id: 'objects', icon: '📦', label: 'Objects', color: '#ee0979' }
    ],
    launch: [
        { id: 'nodes', icon: '🚀', label: 'Nodes', color: '#667eea' },
        { id: 'arguments', icon: '📝', label: 'Arguments', color: '#11998e' },
        { id: 'conditions', icon: '❓', label: 'Conditions', color: '#ee0979' },
        { id: 'groups', icon: '📁', label: 'Groups', color: '#f093fb' }
    ]
};

// Default icons when no file selected
const DEFAULT_ICONS = [
    { id: 'grid', icon: '⊞', label: 'Grid View', color: null },
    { id: 'search', icon: '🔍', label: 'Search', color: null },
    { id: 'blocks', icon: '📊', label: 'Blocks', color: null, active: true },
    { id: 'run', icon: '▶️', label: 'Run', color: null }
];

let sidebarElement = null;
let currentActiveId = null;

/**
 * Initialize the icon sidebar
 * @param {string} containerId - ID of the sidebar container
 */
export function initIconSidebar(containerId) {
    sidebarElement = document.getElementById(containerId);
    if (!sidebarElement) {
        console.error('[IconSidebar] Container not found:', containerId);
        return;
    }

    // Listen for editor type changes
    onEditorChange((editorType) => {
        renderIcons(editorType);
    });

    // Listen for category changes to update active state
    onCategoryChange((category) => {
        updateActiveState(category);
    });

    // Render default icons initially
    renderIcons(null);

    console.log('[IconSidebar] Initialized');
}

/**
 * Render icons for the given editor type
 * @param {string|null} editorType 
 */
function renderIcons(editorType) {
    if (!sidebarElement) return;

    const icons = editorType && ICON_CONFIGS[editorType]
        ? ICON_CONFIGS[editorType]
        : DEFAULT_ICONS;

    // Build top icons
    const topIconsHtml = icons.map(icon => `
        <button class="sidebar-icon-btn ${icon.active ? 'active' : ''}" 
                data-category="${icon.id}"
                title="${icon.label}"
                ${icon.color ? `style="--icon-color: ${icon.color}"` : ''}>
            <span class="icon-emoji">${icon.icon}</span>
        </button>
    `).join('');

    // Settings icon at bottom (always present)
    const bottomIconHtml = `
        <button class="sidebar-icon-btn" data-category="settings" title="Settings">
            <span class="icon-emoji">⭐</span>
        </button>
    `;

    sidebarElement.innerHTML = `
        <div class="icon-sidebar-top">
            ${topIconsHtml}
        </div>
        <div class="icon-sidebar-bottom">
            ${bottomIconHtml}
        </div>
    `;

    // Attach click handlers
    sidebarElement.querySelectorAll('.sidebar-icon-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            const categoryId = btn.dataset.category;
            if (categoryId !== 'settings') {
                setActiveCategory(categoryId);
            }
        });
    });

    // Reset active state
    currentActiveId = null;

    console.log('[IconSidebar] Rendered icons for:', editorType || 'default');
}

/**
 * Update active state on icons
 * @param {string|null} categoryId 
 */
function updateActiveState(categoryId) {
    if (!sidebarElement) return;

    // Remove active from all
    sidebarElement.querySelectorAll('.sidebar-icon-btn').forEach(btn => {
        btn.classList.remove('active');
    });

    // Add active to selected
    if (categoryId) {
        const activeBtn = sidebarElement.querySelector(`[data-category="${categoryId}"]`);
        if (activeBtn) {
            activeBtn.classList.add('active');
        }
    }

    currentActiveId = categoryId;
}

/**
 * Get icons for a specific editor type
 * @param {string} editorType 
 * @returns {Array} Icon configurations
 */
export function getIconsForType(editorType) {
    return ICON_CONFIGS[editorType] || DEFAULT_ICONS;
}
