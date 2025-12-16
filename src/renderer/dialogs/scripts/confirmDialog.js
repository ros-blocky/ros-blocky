/**
 * Generic Confirm Dialog Script
 * Handles Yes/No confirmation for delete operations
 */

// Store the config received from main process (might arrive before DOM is ready)
let pendingConfig = null;
let messageEl = null;

// Set up IPC listener IMMEDIATELY (before DOM is ready)
// This ensures we don't miss the message from main process
if (window.dialogAPI && window.dialogAPI.onSetConfirmConfig) {
    window.dialogAPI.onSetConfirmConfig((config) => {
        console.log('[ConfirmDialog] Received config:', config);
        pendingConfig = config;

        // If DOM is already ready, apply immediately
        if (messageEl) {
            messageEl.innerHTML = config.message;
        }
    });
}

document.addEventListener('DOMContentLoaded', async () => {
    // Initialize i18n for dialog
    if (window.dialogI18n) {
        await window.dialogI18n.init();
    }

    // DOM elements
    messageEl = document.getElementById('message');
    const yesBtn = document.getElementById('yes-btn');
    const noBtn = document.getElementById('no-btn');
    const closeBtn = document.getElementById('close-btn');

    // Apply pending config if it arrived before DOM was ready
    if (pendingConfig && messageEl) {
        console.log('[ConfirmDialog] Applying pending config:', pendingConfig.message);
        messageEl.innerHTML = pendingConfig.message;
    }

    // Yes button - confirm
    yesBtn.addEventListener('click', () => {
        window.dialogAPI.sendConfirmResult(true);
    });

    // No button - cancel
    noBtn.addEventListener('click', () => {
        window.dialogAPI.sendConfirmResult(false);
    });

    // Close button - cancel
    closeBtn.addEventListener('click', () => {
        window.dialogAPI.sendConfirmResult(false);
    });

    // Escape key to cancel
    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape') {
            window.dialogAPI.sendConfirmResult(false);
        }
    });
});
