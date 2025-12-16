/**
 * Generic Loading Dialog Script
 * Handles loading states for package creation, building, etc.
 */

// Store config received from main process (might arrive before DOM is ready)
let pendingConfig = null;
let titleEl = null;
let statusEl = null;

// Set up IPC listener immediately (before DOM is ready)
if (window.dialogAPI && window.dialogAPI.onLoadingConfig) {
    window.dialogAPI.onLoadingConfig((config) => {
        console.log('[LoadingDialog] Received config:', config);
        pendingConfig = config;
        applyConfig(config);
    });
}

// Listen for status updates
if (window.dialogAPI && window.dialogAPI.onLoadingStatus) {
    window.dialogAPI.onLoadingStatus((status) => {
        console.log('[LoadingDialog] Status update:', status);
        if (statusEl) {
            statusEl.textContent = status;
        }
    });
}

/**
 * Apply configuration to dialog
 * @param {Object} config - { type: 'create'|'build', target: 'all'|'packageName' }
 */
function applyConfig(config) {
    if (!titleEl || !statusEl) return;
    if (!config) return;

    // Get translations via dialogI18n if available
    const t = window.dialogI18n ? window.dialogI18n.t : (key) => key;

    if (config.type === 'create') {
        // Creating a package
        titleEl.textContent = t('loading.creatingPackage') || 'Creating Package...';
        statusEl.textContent = `${t('loading.creatingPackageStatus') || 'Creating package'} "${config.target}"...`;
    } else if (config.type === 'build') {
        if (config.target === 'all') {
            // Building all packages
            titleEl.textContent = t('loading.buildingAll') || 'Building All Packages...';
            statusEl.textContent = t('loading.buildingAllStatus') || 'Running colcon build...';
        } else {
            // Building single package
            titleEl.textContent = t('loading.buildingPackage') || 'Building Package...';
            statusEl.textContent = `${t('loading.buildingPackageStatus') || 'Building'} "${config.target}"...`;
        }
    }
}

document.addEventListener('DOMContentLoaded', async () => {
    // Initialize i18n for dialog
    if (window.dialogI18n) {
        await window.dialogI18n.init();
    }

    // Get DOM elements
    titleEl = document.getElementById('dialog-title');
    statusEl = document.getElementById('status-text');

    // Apply pending config if it arrived before DOM was ready
    if (pendingConfig) {
        applyConfig(pendingConfig);
    }

    // Also listen for the old package-creation-status for backwards compatibility
    if (window.dialogAPI && window.dialogAPI.onPackageCreationStatus) {
        window.dialogAPI.onPackageCreationStatus((status) => {
            if (statusEl) {
                statusEl.textContent = status;
            }
        });
    }
});
