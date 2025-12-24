/**
 * Main renderer process JavaScript
 */

// Import i18n module
import { initI18n, t, updatePageTranslations } from '../i18n/index.js';
import { createLanguageSwitcher, getLanguageSwitcherStyles } from '../i18n/language-switcher.js';
import { escapeHtml, escapeAttribute } from './utils/html-utils.js';

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
            // Double-check: also set display none and pointer-events as failsafe
            loadingScreen.style.display = 'none';
            loadingScreen.style.pointerEvents = 'none';
            console.log('[showIDE] Loading screen hidden, display:', loadingScreen.style.display);
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

        // Show Topics button in workspace
        const topicsBtn = document.getElementById('topics-btn');
        if (topicsBtn) topicsBtn.classList.remove('hidden');

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
            // Reset inline styles that may have been set by showIDE
            loadingScreen.style.display = '';
            loadingScreen.style.pointerEvents = '';
        }
        if (workspace) workspace.classList.add('hidden');
    }

    // Helper function to show welcome screen
    async function showWelcome() {
        console.log('[showWelcome] Showing welcome screen, hiding loading and workspace');

        // Reset the project loaded flag when returning to welcome
        isProjectLoaded = false;

        if (welcomeScreen) welcomeScreen.classList.remove('hidden');
        if (loadingScreen) {
            loadingScreen.classList.add('hidden');
            // Also reset inline styles as failsafe
            loadingScreen.style.display = 'none';
            loadingScreen.style.pointerEvents = 'none';
        }
        if (workspace) workspace.classList.add('hidden');

        // Hide toolbar buttons on welcome screen
        const saveBtn = document.getElementById('editor-save-btn');
        const undoBtn = document.getElementById('editor-undo-btn');
        const redoBtn = document.getElementById('editor-redo-btn');
        const rvizBtn = document.getElementById('rviz-btn');
        const jspGuiBtn = document.getElementById('jsp-gui-btn');
        const turtlesimBtn = document.getElementById('turtlesim-btn');
        const topicsBtn = document.getElementById('topics-btn');
        const buildDropdown = document.getElementById('build-dropdown');
        if (saveBtn) saveBtn.classList.add('hidden');
        if (undoBtn) undoBtn.classList.add('hidden');
        if (redoBtn) redoBtn.classList.add('hidden');
        if (rvizBtn) rvizBtn.classList.add('hidden');
        if (jspGuiBtn) jspGuiBtn.classList.add('hidden');
        if (turtlesimBtn) turtlesimBtn.classList.add('hidden');
        if (topicsBtn) topicsBtn.classList.add('hidden');
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
                    <div class="build-menu-item build-menu-package" data-package="${escapeAttribute(pkg)}">
                        ${escapeHtml(pkg)}
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

    // ========================================
    // Topics Button
    // ========================================
    const topicsBtn = document.getElementById('topics-btn');
    let topicsPanel = null;

    if (topicsBtn) {
        topicsBtn.addEventListener('click', async () => {
            // If panel already exists, just refresh it
            if (topicsPanel && document.body.contains(topicsPanel)) {
                await refreshTopics();
                return;
            }

            // Create topics panel
            topicsPanel = document.createElement('div');
            topicsPanel.className = 'topics-panel';
            topicsPanel.innerHTML = `
                <div class="topics-panel-header">
                    <span class="topics-title">📡 ROS 2 Topics</span>
                    <div class="topics-header-actions">
                        <button class="topics-refresh-btn" title="Refresh">↻</button>
                        <button class="topics-close-btn" title="Close">×</button>
                    </div>
                </div>
                <div class="topics-panel-content">
                    <div class="topics-loading">Loading topics...</div>
                </div>
            `;

            document.body.appendChild(topicsPanel);

            // Position panel
            topicsPanel.style.position = 'fixed';
            topicsPanel.style.top = '100px';
            topicsPanel.style.right = '20px';
            topicsPanel.style.zIndex = '10000';

            // Make panel draggable
            const header = topicsPanel.querySelector('.topics-panel-header');
            let isDragging = false;
            let dragOffsetX = 0;
            let dragOffsetY = 0;

            header.addEventListener('mousedown', (e) => {
                if (e.target.tagName === 'BUTTON') return;
                isDragging = true;
                dragOffsetX = e.clientX - topicsPanel.offsetLeft;
                dragOffsetY = e.clientY - topicsPanel.offsetTop;
                header.style.cursor = 'grabbing';
            });

            document.addEventListener('mousemove', (e) => {
                if (!isDragging) return;
                topicsPanel.style.left = (e.clientX - dragOffsetX) + 'px';
                topicsPanel.style.top = (e.clientY - dragOffsetY) + 'px';
                topicsPanel.style.right = 'auto';
            });

            document.addEventListener('mouseup', () => {
                isDragging = false;
                header.style.cursor = 'grab';
            });

            // Close button
            topicsPanel.querySelector('.topics-close-btn').addEventListener('click', () => {
                topicsPanel.remove();
                topicsPanel = null;
            });

            // Refresh button
            topicsPanel.querySelector('.topics-refresh-btn').addEventListener('click', refreshTopics);

            // Initial load
            await refreshTopics();
        });
    }

    async function refreshTopics() {
        if (!topicsPanel) return;
        const content = topicsPanel.querySelector('.topics-panel-content');
        content.innerHTML = '<div class="topics-loading">Loading topics...</div>';

        try {
            const result = await window.electronAPI.getTopicList();
            if (result.success && result.topics.length > 0) {
                content.innerHTML = result.topics.map(topic => `
                    <div class="topic-wrapper">
                        <div class="topic-item" data-topic="${escapeAttribute(topic)}">
                            <span class="topic-expand">▶</span>
                            <span class="topic-name">${escapeHtml(topic)}</span>
                        </div>
                        <div class="topic-details hidden"></div>
                    </div>
                `).join('');

                // Add click handlers for each topic
                content.querySelectorAll('.topic-item').forEach(item => {
                    item.addEventListener('click', async () => {
                        const wrapper = item.closest('.topic-wrapper');
                        const details = wrapper.querySelector('.topic-details');
                        const expandIcon = item.querySelector('.topic-expand');
                        const topicName = item.dataset.topic;

                        // Toggle visibility
                        if (!details.classList.contains('hidden')) {
                            details.classList.add('hidden');
                            expandIcon.textContent = '▶';
                            return;
                        }

                        // Show loading
                        details.classList.remove('hidden');
                        expandIcon.textContent = '▼';
                        details.innerHTML = '<div class="topic-loading">Loading...</div>';

                        try {
                            const infoResult = await window.electronAPI.getTopicInfo(topicName);
                            if (infoResult.success) {
                                const info = infoResult.info;
                                details.innerHTML = `
                                    <div class="topic-info-row">
                                        <span class="info-label">Type:</span>
                                        <span class="info-value type">${info.type || 'Unknown'}</span>
                                    </div>
                                    <div class="topic-info-row">
                                        <span class="info-label">Publishers (${info.publisherCount}):</span>
                                        <span class="info-value">${info.publishers.length > 0 ? info.publishers.join(', ') : 'None'}</span>
                                    </div>
                                    <div class="topic-info-row">
                                        <span class="info-label">Subscribers (${info.subscriberCount}):</span>
                                        <span class="info-value">${info.subscribers.length > 0 ? info.subscribers.join(', ') : 'None'}</span>
                                    </div>
                                    <button class="echo-btn" data-topic="${topicName}">▶ Echo</button>
                                `;

                                // Add echo button handler
                                const echoBtn = details.querySelector('.echo-btn');
                                echoBtn.addEventListener('click', async (e) => {
                                    e.stopPropagation();
                                    await startTopicEchoPanel(topicName);
                                });
                            } else {
                                details.innerHTML = `<div class="topic-error">Error: ${infoResult.error}</div>`;
                            }
                        } catch (error) {
                            details.innerHTML = `<div class="topic-error">Error: ${error.message}</div>`;
                        }
                    });
                });
            } else if (result.success && result.topics.length === 0) {
                content.innerHTML = '<div class="topics-empty">No topics found. Start a ROS 2 node first.</div>';
            } else {
                content.innerHTML = `<div class="topics-error">Error: ${result.error}</div>`;
            }
        } catch (error) {
            content.innerHTML = `<div class="topics-error">Error: ${error.message}</div>`;
        }
    }

    // Echo panel for topic streaming
    let echoPanel = null;
    let currentEchoKey = null;

    async function startTopicEchoPanel(topicName) {
        // Close existing echo panel if open
        if (echoPanel) {
            echoPanel.remove();
            if (currentEchoKey) {
                await window.electronAPI.stopRosProcess(currentEchoKey);
            }
        }

        // Create echo panel
        echoPanel = document.createElement('div');
        echoPanel.className = 'echo-panel';
        echoPanel.innerHTML = `
            <div class="echo-panel-header">
                <span class="echo-title">📡 ${escapeHtml(topicName)}</span>
                <div class="echo-header-actions">
                    <button class="echo-stop-btn" title="Stop">■</button>
                    <button class="echo-close-btn" title="Close">×</button>
                </div>
            </div>
            <div class="echo-panel-content">
                <div class="echo-waiting">Waiting for messages...</div>
            </div>
        `;

        document.body.appendChild(echoPanel);

        // Position panel
        echoPanel.style.position = 'fixed';
        echoPanel.style.top = '150px';
        echoPanel.style.right = '400px';
        echoPanel.style.zIndex = '10001';

        // Make draggable
        const header = echoPanel.querySelector('.echo-panel-header');
        let isDragging = false;
        let dragOffsetX = 0;
        let dragOffsetY = 0;

        header.addEventListener('mousedown', (e) => {
            if (e.target.tagName === 'BUTTON') return;
            isDragging = true;
            dragOffsetX = e.clientX - echoPanel.offsetLeft;
            dragOffsetY = e.clientY - echoPanel.offsetTop;
        });

        document.addEventListener('mousemove', (e) => {
            if (!isDragging) return;
            echoPanel.style.left = (e.clientX - dragOffsetX) + 'px';
            echoPanel.style.top = (e.clientY - dragOffsetY) + 'px';
            echoPanel.style.right = 'auto';
        });

        document.addEventListener('mouseup', () => {
            isDragging = false;
        });

        const content = echoPanel.querySelector('.echo-panel-content');

        // Start echo
        const result = await window.electronAPI.startTopicEcho(topicName);
        if (result.success) {
            currentEchoKey = result.processKey;
            content.innerHTML = '';

            // Listen for echo output
            window.electronAPI.onEchoOutput((data, key) => {
                if (key === currentEchoKey) {
                    const line = document.createElement('div');
                    line.className = 'echo-line';
                    line.textContent = data;
                    content.appendChild(line);
                    content.scrollTop = content.scrollHeight;

                    // Keep only last 100 lines
                    while (content.children.length > 100) {
                        content.removeChild(content.firstChild);
                    }
                }
            });

            window.electronAPI.onEchoStopped((data, key) => {
                if (key === currentEchoKey) {
                    const line = document.createElement('div');
                    line.className = 'echo-line echo-stopped';
                    line.textContent = '--- Echo stopped ---';
                    content.appendChild(line);
                    currentEchoKey = null;
                }
            });
        } else {
            content.innerHTML = `<div class="echo-error">Error: ${result.error}</div>`;
        }

        // Close button
        echoPanel.querySelector('.echo-close-btn').addEventListener('click', async () => {
            if (currentEchoKey) {
                await window.electronAPI.stopRosProcess(currentEchoKey);
            }
            window.electronAPI.removeEchoListeners();
            echoPanel.remove();
            echoPanel = null;
            currentEchoKey = null;
        });

        // Stop button
        echoPanel.querySelector('.echo-stop-btn').addEventListener('click', async () => {
            if (currentEchoKey) {
                await window.electronAPI.stopRosProcess(currentEchoKey);
            }
        });
    }
});
