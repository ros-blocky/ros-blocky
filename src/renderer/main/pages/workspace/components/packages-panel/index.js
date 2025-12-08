/**
 * Packages Panel Component
 * Main entry point - initializes the panel and imports all modules
 */

import * as api from './api.js';
import { setupPanelResize, setupCreatePackageButton, refreshPackageList } from './panel-ui.js';
import { setupContextMenu } from './context-menu.js';
import { setRefreshFunction } from './file-operations.js';

/**
 * Initialize the packages panel
 */
export function initPackagesPanel() {
    console.log('Packages panel initialized');

    // Set up the refresh function for file operations
    setRefreshFunction(refreshPackageList);

    // Setup UI components
    setupPanelResize();
    setupCreatePackageButton();
    setupContextMenu();

    // Refresh package list immediately (project might already be loaded)
    refreshPackageList();

    // Also refresh when project-loaded event fires
    api.onProjectLoaded((projectPath) => {
        console.log('Project loaded in packages panel:', projectPath);
        refreshPackageList();
    });
}

// Export refresh function for external use
export { refreshPackageList };
