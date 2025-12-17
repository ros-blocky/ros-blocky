/**
 * Main renderer process JavaScript
 */

// Import i18n module
import { initI18n, t, updatePageTranslations } from '../i18n/index.js';
import { createLanguageSwitcher, getLanguageSwitcherStyles } from '../i18n/language-switcher.js';

document.addEventListener('DOMContentLoaded', async () => {
    console.log('Main window loaded');

    // ========================================
    // Initialize i18n (must be first)
    // ========================================
    await initI18n();

    // Inject language switcher styles
    const styleEl = document.createElement('style');
    styleEl.textContent = getLanguageSwitcherStyles();
    document.head.appendChild(styleEl);

    // Import RTL styles
    const rtlLink = document.createElement('link');
    rtlLink.rel = 'stylesheet';
    rtlLink.href = '../i18n/rtl-styles.css';
    document.head.appendChild(rtlLink);

    // Create language switcher in top bar
    createLanguageSwitcher('language-switcher-container');

    // ========================================
    // UI Elements
    // ========================================
    const welcomeScreen = document.querySelector('.welcome-screen');
    const loadingScreen = document.querySelector('.loading-screen');
    const workspace = document.querySelector('.workspace');

    // Ensure welcome screen is shown on startup
    if (welcomeScreen) welcomeScreen.classList.remove('hidden');
    if (loadingScreen) loadingScreen.classList.add('hidden');
    if (workspace) workspace.classList.add('hidden');

    // State flag to track if project is loaded (prevents race condition with loading screen)
    let isProjectLoaded = false;

    // Helper function to show IDE
    function showIDE() {
        console.log('[showIDE] Showing workspace, hiding loading and welcome screens');

        // Set flag to prevent loading screen from showing after this
        isProjectLoaded = true;

        if (welcomeScreen) welcomeScreen.classList.add('hidden');
        if (loadingScreen) {
            loadingScreen.classList.add('hidden');
            // Double-check: also set display none directly as failsafe
            loadingScreen.style.display = 'none';
        }
        if (workspace) workspace.classList.remove('hidden');

        // Show RViz button in workspace
        const rvizBtn = document.getElementById('rviz-btn');
        if (rvizBtn) rvizBtn.classList.remove('hidden');

        // Show JSP GUI button in workspace
        const jspGuiBtn = document.getElementById('jsp-gui-btn');
        if (jspGuiBtn) jspGuiBtn.classList.remove('hidden');

        // Show TurtleSim button in workspace
        const turtlesimBtn = document.getElementById('turtlesim-btn');
        if (turtlesimBtn) turtlesimBtn.classList.remove('hidden');

        // Show Build dropdown in workspace
        const buildDropdown = document.getElementById('build-dropdown');
        if (buildDropdown) buildDropdown.classList.remove('hidden');

        // Reapply translations when showing workspace
        updatePageTranslations();

        console.log('[showIDE] Workspace visible, loading screen hidden');
    }

    // Helper function to show loading screen
    function showLoading(title, message) {
        // Don't show loading screen if project is already loaded (race condition fix)
        if (isProjectLoaded) {
            console.log('[showLoading] Skipping - project already loaded');
            return;
        }

        console.log('[showLoading] Showing loading screen:', title);

        const loadingTitle = document.getElementById('loading-title');
        const loadingMessage = document.getElementById('loading-message');

        if (loadingTitle) loadingTitle.textContent = title;
        if (loadingMessage) loadingMessage.textContent = message;

        if (welcomeScreen) welcomeScreen.classList.add('hidden');
        if (loadingScreen) {
            loadingScreen.classList.remove('hidden');
            // Reset inline style that may have been set by showIDE
            loadingScreen.style.display = '';
        }
        if (workspace) workspace.classList.add('hidden');
    }

    // Helper function to show welcome screen
    async function showWelcome() {
        // Reset the project loaded flag when returning to welcome
        isProjectLoaded = false;

        if (welcomeScreen) welcomeScreen.classList.remove('hidden');
        if (loadingScreen) loadingScreen.classList.add('hidden');
        if (workspace) workspace.classList.add('hidden');

        // Hide toolbar buttons on welcome screen
        const saveBtn = document.getElementById('editor-save-btn');
        const undoBtn = document.getElementById('editor-undo-btn');
        const redoBtn = document.getElementById('editor-redo-btn');
        const rvizBtn = document.getElementById('rviz-btn');
        const jspGuiBtn = document.getElementById('jsp-gui-btn');
        const turtlesimBtn = document.getElementById('turtlesim-btn');
        const buildDropdown = document.getElementById('build-dropdown');
        if (saveBtn) saveBtn.classList.add('hidden');
        if (undoBtn) undoBtn.classList.add('hidden');
        if (redoBtn) redoBtn.classList.add('hidden');
        if (rvizBtn) rvizBtn.classList.add('hidden');
        if (jspGuiBtn) jspGuiBtn.classList.add('hidden');
        if (turtlesimBtn) turtlesimBtn.classList.add('hidden');
        if (buildDropdown) buildDropdown.classList.add('hidden');

        // Re-enable and reset welcome screen buttons using the module's function
        try {
            const { resetWelcomeButtons } = await import('./pages/welcome/welcome.js');
            resetWelcomeButtons();
        } catch (error) {
            console.error('Error resetting welcome buttons:', error);
        }

        // Reapply translations when returning to welcome
        updatePageTranslations();
    }

    // Helper function to update project name in top bar
    function updateProjectName(projectPath) {
        const projectName = document.getElementById('project-name');
        if (projectName && projectPath) {
            const parts = projectPath.split(/[\\/]/);
            const name = parts[parts.length - 1];
            projectName.textContent = name;
        }
    }

    // Load welcome screen HTML
    const welcomeContainer = document.getElementById('welcome-container');
    if (welcomeContainer) {
        const response = await fetch('pages/welcome/welcome.html');
        const html = await response.text();
        welcomeContainer.innerHTML = html;

        // Initialize welcome screen after HTML is loaded, passing parent functions
        const { initWelcomeScreen } = await import('./pages/welcome/welcome.js');
        initWelcomeScreen({
            updateProjectName,
            showIDE,
            showWelcome
        });

        // Apply translations to dynamically loaded welcome HTML
        updatePageTranslations();
    }

    // Load loading screen HTML
    const loadingContainer = document.getElementById('loading-container');
    if (loadingContainer) {
        const response = await fetch('pages/loading/loading.html');
        const html = await response.text();
        loadingContainer.innerHTML = html;

        // Initialize loading screen
        const { initLoadingScreen } = await import('./pages/loading/loading.js');
        initLoadingScreen();
    }

    // Load workspace HTML
    const workspaceContainer = document.getElementById('workspace-container');
    if (workspaceContainer) {
        const response = await fetch('pages/workspace/workspace.html');
        const html = await response.text();
        workspaceContainer.innerHTML = html;

        // Initialize workspace
        const { initWorkspace } = await import('./pages/workspace/workspace.js');
        initWorkspace();

        // Apply translations to dynamically loaded workspace HTML
        updatePageTranslations();
    }

    // Listen for when dialogs are complete (name + folder selected)
    if (window.electronAPI && window.electronAPI.onProjectDialogsComplete) {
        window.electronAPI.onProjectDialogsComplete((type) => {
            // Show loading screen when dialogs are done
            if (type === 'Creating') {
                showLoading(t('welcome.creatingProject'), t('welcome.loadingWorkspace'));
            } else if (type === 'Opening') {
                showLoading(t('welcome.openingProject'), t('welcome.loadingWorkspace'));
            }
        });
    }

    // ========================================
    // Window Controls
    // ========================================
    const minimizeBtn = document.getElementById('minimize-btn');
    const maximizeBtn = document.getElementById('maximize-btn');
    const closeBtn = document.getElementById('close-btn');

    if (minimizeBtn) {
        minimizeBtn.addEventListener('click', () => {
            if (window.electronAPI && window.electronAPI.minimizeWindow) {
                window.electronAPI.minimizeWindow();
            }
        });
    }

    if (maximizeBtn) {
        maximizeBtn.addEventListener('click', () => {
            if (window.electronAPI && window.electronAPI.maximizeWindow) {
                window.electronAPI.maximizeWindow();
            }
        });
    }

    if (closeBtn) {
        closeBtn.addEventListener('click', () => {
            if (window.electronAPI && window.electronAPI.closeWindow) {
                window.electronAPI.closeWindow();
            }
        });
    }

    // ========================================
    // Logo Click - Return to Welcome Screen
    // ========================================
    const logo = document.querySelector('.top-bar-logo');
    if (logo) {
        logo.addEventListener('click', () => {
            console.log('Logo clicked - returning to welcome screen');
            showWelcome();
        });
    }

    // ========================================
    // Project Name Click - Return to Workspace
    // ========================================
    const projectName = document.getElementById('project-name');
    if (projectName) {
        console.log('Project name element found:', projectName);

        projectName.addEventListener('click', (e) => {
            console.log('Project name clicked');

            // Only allow clicking if a project is open (not default text)
            if (projectName.textContent !== 'ROS Block Code') {
                console.log('Returning to workspace');
                showIDE();
            } else {
                console.log('Cannot return to workspace - no project open');
            }
        });
    }

    // ========================================
    // Build Dropdown
    // ========================================
    const buildDropdown = document.getElementById('build-dropdown');
    const buildBtn = document.getElementById('build-btn');
    const buildMenu = document.getElementById('build-menu');
    const buildAll = document.getElementById('build-all');
    const buildPackagesList = document.getElementById('build-packages-list');

    if (buildBtn && buildMenu) {
        // Toggle dropdown on button click
        buildBtn.addEventListener('click', async (e) => {
            e.stopPropagation();
            const isVisible = !buildMenu.classList.contains('hidden');

            if (isVisible) {
                buildMenu.classList.add('hidden');
            } else {
                // Populate package list before showing
                await populatePackageList();
                buildMenu.classList.remove('hidden');
            }
        });

        // Close dropdown when clicking outside
        document.addEventListener('click', (e) => {
            if (!buildDropdown.contains(e.target)) {
                buildMenu.classList.add('hidden');
            }
        });

        // Build All Packages
        if (buildAll) {
            buildAll.addEventListener('click', async () => {
                buildMenu.classList.add('hidden');
                console.log('[Build] Building all packages...');

                try {
                    const result = await window.electronAPI.buildAllPackages();
                    if (result.success) {
                        console.log('[Build] All packages built successfully');
                    } else {
                        console.error('[Build] Failed to build:', result.error);
                        alert(`Build failed: ${result.error}`);
                    }
                } catch (error) {
                    console.error('[Build] Error:', error);
                    alert(`Build error: ${error.message}`);
                }
            });
        }
    }

    // Populate package list in dropdown
    async function populatePackageList() {
        if (!buildPackagesList) return;

        try {
            const packages = await window.electronAPI.listPackages();
            if (packages && packages.length > 0) {
                buildPackagesList.innerHTML = packages.map(pkg => `
                    <div class="build-menu-item build-menu-package" data-package="${pkg}">
                        ${pkg}
                    </div>
                `).join('');

                // Add click handlers to each package
                buildPackagesList.querySelectorAll('.build-menu-package').forEach(item => {
                    item.addEventListener('click', async () => {
                        const packageName = item.dataset.package;
                        buildMenu.classList.add('hidden');
                        console.log(`[Build] Building package: ${packageName}`);

                        try {
                            const result = await window.electronAPI.buildPackage(packageName);
                            if (result.success) {
                                console.log(`[Build] Package ${packageName} built successfully`);
                            } else {
                                console.error(`[Build] Failed to build ${packageName}:`, result.error);
                                alert(`Build failed: ${result.error}`);
                            }
                        } catch (error) {
                            console.error('[Build] Error:', error);
                            alert(`Build error: ${error.message}`);
                        }
                    });
                });
            } else {
                buildPackagesList.innerHTML = '<div class="build-menu-item" style="color: #999; font-style: italic;">No packages found</div>';
            }
        } catch (error) {
            console.error('[Build] Error loading packages:', error);
            buildPackagesList.innerHTML = '<div class="build-menu-item" style="color: #999;">Error loading packages</div>';
        }
    }
});
