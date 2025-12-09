/**
 * i18n Main Entry Point (Pure JavaScript Implementation)
 * No external dependencies required
 */

import {
    DEFAULT_LANGUAGE,
    FALLBACK_LANGUAGE
} from './i18n-config.js';
import {
    getCurrentLanguage,
    setLanguage,
    initializeLanguage,
    onLanguageChange,
    getAvailableLanguages
} from './language-service.js';
import { applyTextDirection, isCurrentRTL } from './rtl-handler.js';

// i18n state
let isInitialized = false;
let translations = {};
let currentLang = DEFAULT_LANGUAGE;

/**
 * Load translation files
 * @returns {Promise<object>}
 */
async function loadTranslations() {
    try {
        const [enRes, frRes, arRes] = await Promise.all([
            fetch('../i18n/locales/en.json').then(r => r.json()),
            fetch('../i18n/locales/fr.json').then(r => r.json()),
            fetch('../i18n/locales/ar.json').then(r => r.json())
        ]);

        return {
            en: enRes,
            fr: frRes,
            ar: arRes
        };
    } catch (error) {
        console.error('Failed to load translations:', error);
        return { en: {}, fr: {}, ar: {} };
    }
}

/**
 * Get nested value from object using dot notation
 * @param {object} obj - Object to search
 * @param {string} path - Dot notation path (e.g., 'packages.title')
 * @returns {string|undefined}
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
 * Initialize the i18n system
 * @returns {Promise<void>}
 */
export async function initI18n() {
    if (isInitialized) {
        console.warn('i18n already initialized');
        return;
    }

    // Load translations
    translations = await loadTranslations();

    // Get initial language from storage or detection
    currentLang = initializeLanguage();

    // Listen for language changes
    onLanguageChange((newLang) => {
        currentLang = newLang;
        updatePageTranslations();
    });

    isInitialized = true;
    console.log(`i18n initialized with language: ${currentLang}`);

    // Apply initial translations to DOM
    updatePageTranslations();
}

/**
 * Translate a key
 * @param {string} key - Translation key (e.g., 'packages.title')
 * @param {object} options - Interpolation options (e.g., { name: 'John' })
 * @returns {string} Translated string
 */
export function t(key, options = {}) {
    if (!isInitialized) {
        console.warn('i18n not initialized, returning key:', key);
        return key;
    }

    // Try current language first
    let value = getNestedValue(translations[currentLang], key);

    // Fallback to default language
    if (value === undefined && currentLang !== FALLBACK_LANGUAGE) {
        value = getNestedValue(translations[FALLBACK_LANGUAGE], key);
    }

    // Return key if not found
    if (value === undefined) {
        console.warn(`Translation not found for key: ${key}`);
        return key;
    }

    // Handle interpolation (replace {{variable}} with options.variable)
    if (options && Object.keys(options).length > 0) {
        value = value.replace(/\{\{(\w+)\}\}/g, (match, varName) => {
            return options[varName] !== undefined ? options[varName] : match;
        });
    }

    return value;
}

/**
 * Update all DOM elements with data-i18n attribute
 */
export function updatePageTranslations() {
    // Find all elements with data-i18n attribute
    const elements = document.querySelectorAll('[data-i18n]');

    elements.forEach(element => {
        const key = element.getAttribute('data-i18n');

        if (key) {
            element.textContent = t(key);
        }

        // Check for attribute translations
        if (element.hasAttribute('data-i18n-placeholder')) {
            const placeholderKey = element.getAttribute('data-i18n-placeholder');
            element.placeholder = t(placeholderKey);
        }

        if (element.hasAttribute('data-i18n-title')) {
            const titleKey = element.getAttribute('data-i18n-title');
            element.title = t(titleKey);
        }
    });

    console.log(`Updated ${elements.length} translated elements`);
}

/**
 * Change the current language
 * @param {string} langCode - Language code
 */
export async function changeLanguage(langCode) {
    setLanguage(langCode);
    currentLang = langCode;
    updatePageTranslations();
}

/**
 * Check if i18n is initialized
 * @returns {boolean}
 */
export function isI18nInitialized() {
    return isInitialized;
}

// Re-export useful functions from other modules
export {
    getCurrentLanguage,
    getAvailableLanguages,
    isCurrentRTL,
    onLanguageChange
};
