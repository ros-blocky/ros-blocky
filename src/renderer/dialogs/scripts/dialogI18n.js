/**
 * Dialog i18n Helper
 * Lightweight translation for dialog windows
 */

// Translation cache
let translations = null;
let currentLang = 'en';

/**
 * Load translations for dialogs
 */
async function loadDialogTranslations() {
    try {
        // Get current language from localStorage (same key as main window)
        currentLang = localStorage.getItem('ros-blocky-language') || 'en';

        // Load the translation file
        const response = await fetch(`../i18n/locales/${currentLang}.json`);
        translations = await response.json();

        console.log(`Dialog loaded translations for: ${currentLang}`);
    } catch (error) {
        console.error('Failed to load dialog translations:', error);
        // Fall back to English
        try {
            const response = await fetch('../i18n/locales/en.json');
            translations = await response.json();
        } catch (e) {
            translations = {};
        }
    }
}

/**
 * Get nested value from object using dot notation
 */
function getNestedValue(obj, path) {
    if (!obj || !path) return undefined;
    const keys = path.split('.');
    let value = obj;
    for (const key of keys) {
        if (value && typeof value === 'object' && key in value) {
            value = value[key];
        } else {
            return undefined;
        }
    }
    return typeof value === 'string' ? value : undefined;
}

/**
 * Translate a key
 */
function t(key) {
    if (!translations) return key;
    return getNestedValue(translations, key) || key;
}

/**
 * Update all elements with data-i18n attribute
 */
function updateDialogTranslations() {
    const elements = document.querySelectorAll('[data-i18n]');
    elements.forEach(el => {
        const key = el.getAttribute('data-i18n');
        if (key) {
            el.textContent = t(key);
        }
    });

    // Handle placeholders
    const placeholderElements = document.querySelectorAll('[data-i18n-placeholder]');
    placeholderElements.forEach(el => {
        const key = el.getAttribute('data-i18n-placeholder');
        if (key) {
            el.placeholder = t(key);
        }
    });
}

/**
 * Initialize dialog i18n
 */
async function initDialogI18n() {
    await loadDialogTranslations();
    updateDialogTranslations();

    // Set text direction based on language
    if (currentLang === 'ar') {
        document.documentElement.setAttribute('lang', 'ar');
    } else {
        document.documentElement.setAttribute('lang', currentLang);
    }
}

// Export for use in dialogs
window.dialogI18n = {
    init: initDialogI18n,
    t: t,
    update: updateDialogTranslations
};
