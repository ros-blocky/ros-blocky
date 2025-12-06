// Auto-start installation when page loads
let isInstalling = false;

function showError(errorMessage) {
    const errorSection = document.getElementById('error-section');
    const errorMessageEl = document.getElementById('error-message');
    const retryButton = document.getElementById('retry-button');
    const cancelButton = document.getElementById('cancel-button');
    const progressSection = document.querySelector('.progress-container');
    const statusEl = document.getElementById('status');

    // Update status to show error
    statusEl.textContent = 'Installation Failed';

    // Show error message
    errorMessageEl.textContent = errorMessage;
    errorSection.classList.remove('hidden');

    // Hide the big retry button (we'll use Cancel as Retry)
    retryButton.classList.add('hidden');

    // Transform Cancel button to Retry
    cancelButton.textContent = 'Retry Installation';
    cancelButton.onclick = () => startInstallation();

    // Hide progress bar
    progressSection.classList.add('hidden');

    isInstalling = false;
}

function hideError() {
    const errorSection = document.getElementById('error-section');
    const retryButton = document.getElementById('retry-button');
    const cancelButton = document.getElementById('cancel-button');
    const progressSection = document.querySelector('.progress-container');
    const statusEl = document.getElementById('status');

    // Reset status text
    statusEl.textContent = 'Installing required components…';

    errorSection.classList.add('hidden');
    retryButton.classList.add('hidden');
    progressSection.classList.remove('hidden');

    // Reset Cancel button back to Cancel
    cancelButton.textContent = 'Cancel';
    cancelButton.onclick = () => {
        if (window.setupAPI) {
            window.setupAPI.cancelInstallation();
        }
    };
}

async function startInstallation() {
    if (isInstalling) {
        return;
    }

    isInstalling = true;
    hideError();

    // Check if setupAPI exists
    if (!window.setupAPI) {
        showError('Setup API not available. Please check the preload script.');
        return;
    }

    // Check internet connection
    try {
        const isOnline = await window.setupAPI.checkInternet();

        if (!isOnline) {
            showError('No internet connection. Please check your network and try again.');
            return;
        }
    } catch (error) {
        console.error('Error checking internet:', error);
        // Proceed anyway if check fails, might be a local issue
    }

    window.setupAPI.startInstallation();
}

// Listen for progress updates
window.setupAPI.onInstallationProgress((data) => {
    const { percentage } = data;

    // Update progress bar width
    const progressBarFill = document.getElementById('progress-bar-fill');
    if (progressBarFill) {
        progressBarFill.style.width = `${percentage}%`;
    }

    // Update bubble position and text
    const bubble = document.getElementById('progress-bubble');
    if (bubble) {
        bubble.textContent = `${percentage}%`;
        bubble.style.left = `${percentage}%`;
    }
});

// Listen for installation completion
window.setupAPI.onInstallComplete((success) => {
    isInstalling = false;

    if (success) {
        // Update status text
        const statusEl = document.getElementById('status');
        if (statusEl) {
            statusEl.textContent = 'Installation completed successfully!';
        }

        // Hide progress bar on success
        const progressContainer = document.querySelector('.progress-container');
        if (progressContainer) {
            progressContainer.classList.add('hidden');
        }
    }
});

// Listen for errors
window.setupAPI.onError((errorMessage) => {
    showError(errorMessage);
});

// Retry button handler
document.getElementById('retry-button').addEventListener('click', () => {
    startInstallation();
});

// Start installation automatically when page loads
window.addEventListener('DOMContentLoaded', () => {
    // Initialize cancel button onclick
    const cancelButton = document.getElementById('cancel-button');
    cancelButton.onclick = () => {
        if (window.setupAPI) {
            window.setupAPI.cancelInstallation();
        }
    };

    // Small delay to ensure everything is ready
    setTimeout(() => {
        startInstallation();
    }, 500);
});
