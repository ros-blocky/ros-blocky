/**
 * Main renderer process JavaScript
 */

document.addEventListener('DOMContentLoaded', async () => {
    console.log('Main window loaded');

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

    // Helper function to show IDE
    function showIDE() {
        if (welcomeScreen) welcomeScreen.classList.add('hidden');
        if (loadingScreen) loadingScreen.classList.add('hidden');
        if (workspace) workspace.classList.remove('hidden');
    }

    // Helper function to show loading screen
    function showLoading(title, message) {
        const loadingTitle = document.getElementById('loading-title');
        const loadingMessage = document.getElementById('loading-message');

        if (loadingTitle) loadingTitle.textContent = title;
        if (loadingMessage) loadingMessage.textContent = message;

        if (welcomeScreen) welcomeScreen.classList.add('hidden');
        if (loadingScreen) loadingScreen.classList.remove('hidden');
        if (workspace) workspace.classList.add('hidden');
    }

    // Helper function to show welcome screen
    async function showWelcome() {
        if (welcomeScreen) welcomeScreen.classList.remove('hidden');
        if (loadingScreen) loadingScreen.classList.add('hidden');
        if (workspace) workspace.classList.add('hidden');

        // Re-enable and reset welcome screen buttons using the module's function
        try {
            const { resetWelcomeButtons } = await import('./pages/welcome/welcome.js');
            resetWelcomeButtons();
        } catch (error) {
            console.error('Error resetting welcome buttons:', error);
        }
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
    }

    // Listen for when dialogs are complete (name + folder selected)
    if (window.electronAPI && window.electronAPI.onProjectDialogsComplete) {
        window.electronAPI.onProjectDialogsComplete((type) => {
            // Show loading screen when dialogs are done
            if (type === 'Creating') {
                showLoading('Creating Project...', 'Building your ROS2 workspace with colcon...');
            } else if (type === 'Opening') {
                showLoading('Opening Project...', 'Loading your ROS2 workspace...');
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
});
