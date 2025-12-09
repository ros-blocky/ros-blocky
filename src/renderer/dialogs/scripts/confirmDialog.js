/**
 * Generic Confirm Dialog Script
 * Handles Yes/No confirmation for delete operations
 */

document.addEventListener('DOMContentLoaded', async () => {
    // Initialize i18n for dialog
    if (window.dialogI18n) {
        await window.dialogI18n.init();
    }

    let dialogConfig = {
        message: '',
        onConfirm: null
    };

    // DOM elements
    const messageEl = document.getElementById('message');
    const yesBtn = document.getElementById('yes-btn');
    const noBtn = document.getElementById('no-btn');
    const closeBtn = document.getElementById('close-btn');

    // Receive dialog config from main process
    window.dialogAPI.onSetConfirmConfig((config) => {
        dialogConfig = config;
        messageEl.innerHTML = config.message;
    });

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
