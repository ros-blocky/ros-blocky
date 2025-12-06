/**
 * Welcome Screen Logic
 */

// Store reference to parent functions
let updateProjectNameFn = null;
let showIDEFn = null;
let showWelcomeFn = null;

// Initialize welcome screen when DOM is loaded
export function initWelcomeScreen(parentFunctions) {
    // Store parent function references
    if (parentFunctions) {
        updateProjectNameFn = parentFunctions.updateProjectName;
        showIDEFn = parentFunctions.showIDE;
        showWelcomeFn = parentFunctions.showWelcome;
    }

    const createProjectBtn = document.getElementById('create-project-btn');
    const openProjectBtn = document.getElementById('open-project-btn');

    // Create New Project button handler
    if (createProjectBtn) {
        createProjectBtn.addEventListener('click', async () => {
            console.log('Create new project clicked');

            // Disable BOTH buttons during creation
            createProjectBtn.disabled = true;
            openProjectBtn.disabled = true;
            createProjectBtn.textContent = 'Creating Project...';

            try {
                // Call IPC - dialogs will show, then loading screen will appear via event
                const result = await window.electronAPI.createProject();

                if (result.success) {
                    console.log('Project created at:', result.projectPath);

                    // Update project name in top bar (from parent)
                    if (updateProjectNameFn) updateProjectNameFn(result.projectPath);

                    // Show the IDE (from parent)
                    if (showIDEFn) showIDEFn();
                } else {
                    console.error('Project creation failed:', result.message);

                    // Hide loading and show welcome again
                    if (showWelcomeFn) showWelcomeFn();

                    alert(`Failed to create project: ${result.message}`);
                }
            } catch (error) {
                console.error('Error creating project:', error);

                // Hide loading and show welcome again
                if (showWelcomeFn) showWelcomeFn();

                alert(`Error: ${error.message}`);
            }
        });
    }

    // Open Existing Project button handler
    if (openProjectBtn) {
        openProjectBtn.addEventListener('click', async () => {
            console.log('Open existing project clicked');

            // Disable BOTH buttons during opening
            createProjectBtn.disabled = true;
            openProjectBtn.disabled = true;
            openProjectBtn.textContent = 'Opening Project...';

            try {
                // Call IPC - dialog will show, then loading screen will appear via event
                const result = await window.electronAPI.openProject();

                if (result.success) {
                    console.log('Project opened at:', result.projectPath);

                    // Update project name in top bar (from parent)
                    if (updateProjectNameFn) updateProjectNameFn(result.projectPath);

                    // Show the IDE (from parent)
                    if (showIDEFn) showIDEFn();
                } else {
                    console.error('Project opening failed:', result.message);

                    // Hide loading and show welcome again
                    if (showWelcomeFn) showWelcomeFn();

                    alert(`Failed to open project: ${result.message}`);
                }
            } catch (error) {
                console.error('Error opening project:', error);

                // Hide loading and show welcome again
                if (showWelcomeFn) showWelcomeFn();

                alert(`Error: ${error.message}`);
            }
        });
    }
}

// Helper function to reset buttons to their default state
function resetButtons(createBtn, openBtn) {
    if (createBtn) {
        createBtn.disabled = false;
        createBtn.innerHTML = `
      <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
        <path d="M19 13h-6v6h-2v-6H5v-2h6V5h2v6h6v2z"/>
      </svg>
      Create New Project
    `;
    }

    if (openBtn) {
        openBtn.disabled = false;
        openBtn.innerHTML = `
      <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
        <path d="M20 6h-8l-2-2H4c-1.1 0-1.99.9-1.99 2L2 18c0 1.1.9 2 2 2h16c1.1 0 2-.9 2-2V8c0-1.1-.9-2-2-2zm0 12H4V8h16v10z"/>
      </svg>
      Open Existing Project
    `;
    }
}

// Export the reset function so parent can call it
export function resetWelcomeButtons() {
    const createBtn = document.getElementById('create-project-btn');
    const openBtn = document.getElementById('open-project-btn');
    resetButtons(createBtn, openBtn);
}
