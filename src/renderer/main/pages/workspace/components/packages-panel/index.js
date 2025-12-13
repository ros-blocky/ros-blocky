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

    // Listen for ROS process output to update button states
    if (window.electronAPI && window.electronAPI.onRosOutput) {
        window.electronAPI.onRosOutput((data) => {
            console.log('[ROS Output Listener] Received:', data);

            // Find the run button for this process
            const runBtn = document.querySelector(`.run-btn[data-process-key="${data.processKey}"]`);
            console.log('[ROS Output Listener] Button found:', runBtn, 'for key:', data.processKey);

            if (!runBtn) {
                console.warn('[ROS Output Listener] Button not found for processKey:', data.processKey);
                return;
            }

            if (data.type === 'status') {
                console.log('[ROS Output Listener] Status update:', data.message);

                // Check if this is a URDF process
                const isUrdf = data.processKey && data.processKey.startsWith('urdf:');

                if (data.message === 'starting' || data.message === 'running') {
                    if (data.message === 'running') {
                        // Robot initialized - show stop button
                        runBtn.classList.remove('loading');
                        runBtn.classList.add('running');
                        runBtn.innerHTML = `<svg viewBox="0 0 24 24" fill="currentColor" class="stop-icon"><rect x="6" y="6" width="12" height="12"></rect></svg>`;
                    }

                    // Hide run buttons on all OTHER URDF files
                    if (isUrdf) {
                        document.querySelectorAll('.run-btn[data-process-key^="urdf:"]').forEach(btn => {
                            if (btn !== runBtn) {
                                btn.classList.add('hidden-while-running');
                            }
                        });
                    }
                } else if (data.message === 'stopped' || data.message === 'error') {
                    // Process stopped - show play button
                    runBtn.classList.remove('loading', 'running');
                    runBtn.innerHTML = `<svg viewBox="0 0 24 24" fill="currentColor" class="run-icon"><polygon points="5 3 19 12 5 21 5 3"></polygon></svg>`;

                    // Show run buttons on all URDF files again
                    if (isUrdf) {
                        document.querySelectorAll('.run-btn[data-process-key^="urdf:"]').forEach(btn => {
                            btn.classList.remove('hidden-while-running');
                        });
                    }
                }
            }
        });
    }
}

// Export refresh function for external use
export { refreshPackageList };
