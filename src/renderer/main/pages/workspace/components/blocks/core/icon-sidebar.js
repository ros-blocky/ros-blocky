/**
 * Dynamic Icon Sidebar
 * Renders icons based on active file type from editor registry
 * Hides when no file is selected
 */

import { getState, setActiveCategory, onEditorChange, onCategoryChange, hasActiveFile } from './editor-state.js';
import { getEditorById } from './editor-registry.js';
import { t, onLanguageChange } from '../../../../../../i18n/index.js';

let sidebarElement = null;
let sidebarContainer = null;
let currentActiveId = null;
let currentEditorType = null;

/**
 * Initialize the icon sidebar
 * @param {string} containerId - ID of the sidebar container (the icon sidebar div)
 */
export function initIconSidebar(containerId) {
    sidebarElement = document.getElementById(containerId);
    if (!sidebarElement) {
        // Try to find by class
        sidebarElement = document.querySelector('.blocks-icon-sidebar');
    }

    if (!sidebarElement) {
        console.error('[IconSidebar] Container not found:', containerId);
        return;
    }

    // Store reference to parent for show/hide
    sidebarContainer = sidebarElement;

    // Listen for editor type changes
    onEditorChange((editorType) => {
        currentEditorType = editorType;
        if (editorType) {
            showSidebar();
            renderIconsFromRegistry(editorType);
        } else {
            hideSidebar();
        }
    });

    // Listen for category changes to update active state
    onCategoryChange((category) => {
        updateActiveState(category);
    });

    // Listen for language changes to re-render with translated labels
    onLanguageChange(() => {
        if (currentEditorType) {
            renderIconsFromRegistry(currentEditorType);
        }
    });

    // Initially hide if no file is open
    if (!hasActiveFile()) {
        hideSidebar();
    }

    console.log('[IconSidebar] Initialized');
}

/**
 * Show the sidebar
 */
export function showSidebar() {
    if (sidebarContainer) {
        sidebarContainer.classList.remove('hidden');
        sidebarContainer.style.display = '';
    }
}

/**
 * Hide the sidebar
 */
export function hideSidebar() {
    if (sidebarContainer) {
        sidebarContainer.classList.add('hidden');
        sidebarContainer.style.display = 'none';
    }
    // Clear content
    if (sidebarElement) {
        sidebarElement.innerHTML = '';
    }
}

/**
 * Render icons from editor registry for the given editor type
 * @param {string} editorType - Editor type ID (e.g., 'urdf')
 */
function renderIconsFromRegistry(editorType) {
    if (!sidebarElement) return;

    // Get editor config from registry
    const editorConfig = getEditorById(editorType);

    if (!editorConfig || !editorConfig.categories) {
        console.warn('[IconSidebar] No categories found for editor:', editorType);
        hideSidebar();
        return;
    }

    const categories = editorConfig.categories;


    // Build category icons - support both emoji and image types
    const iconsHtml = categories.map((cat, index) => {
        const iconContent = cat.iconType === 'image'
            ? `<img src="${sanitizeAttr(cat.icon)}" class="sidebar-icon-img" alt="${sanitizeAttr(cat.label)}">`
            : `<span class="icon-emoji">${cat.icon}</span>`;

        // Try to get translated label, fallback to original label
        const translationKey = `blocks.urdf.${cat.id}`;
        const translated = t(translationKey);
        const translatedLabel = (translated && translated !== translationKey) ? translated : cat.label;

        return `
            <button class="sidebar-icon-btn ${index === 0 ? 'active' : ''}" 
                    data-category="${sanitizeAttr(cat.id)}"
                    title="${sanitizeAttr(translatedLabel)}"
                    style="--icon-color: ${cat.color}">
                ${iconContent}
                <span class="sidebar-icon-label">${sanitizeAttr(translatedLabel)}</span>
            </button>
        `;
    }).join('');

    sidebarElement.innerHTML = `
        <div class="icon-sidebar-top">
            ${iconsHtml}
        </div>
        <div class="icon-sidebar-bottom">
            <button class="sidebar-icon-btn" data-category="settings" title="Settings">
                <span class="icon-emoji">⚙️</span>
            </button>
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

    // NOTE: Auto-select is handled by blocks.js createBlocklyWorkspace()
    // Don't call setActiveCategory here to avoid double-selection conflict

    console.log('[IconSidebar] Rendered icons for:', editorType, `(${categories.length} categories)`);
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
 * Sanitize string for use in HTML attributes
 * @param {string} str - String to sanitize
 * @returns {string} Sanitized string
 */
function sanitizeAttr(str) {
    if (typeof str !== 'string') return '';
    return str
        .replace(/&/g, '&amp;')
        .replace(/"/g, '&quot;')
        .replace(/'/g, '&#039;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;');
}

/**
 * Get current active category
 * @returns {string|null} Active category ID
 */
export function getActiveCategory() {
    return currentActiveId;
}

/**
 * Check if sidebar is visible
 * @returns {boolean} True if visible
 */
export function isSidebarVisible() {
    return sidebarContainer && !sidebarContainer.classList.contains('hidden');
}
