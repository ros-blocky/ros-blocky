/**
 * Loading Screen Logic
 */

// Update loading screen title and message
export function updateLoadingScreen(title, message) {
    const loadingTitle = document.getElementById('loading-title');
    const loadingMessage = document.getElementById('loading-message');

    if (loadingTitle) loadingTitle.textContent = title;
    if (loadingMessage) loadingMessage.textContent = message;
}

// Initialize loading screen (if any setup needed in the future)
export function initLoadingScreen() {
    console.log('Loading screen initialized');
}
