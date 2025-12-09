/**
 * Language Service
 * Handles language switching, persistence, and detection
 */

import {
    LANGUAGES,
    DEFAULT_LANGUAGE,
    LANGUAGE_STORAGE_KEY,
    getSupportedLanguages
} from './i18n-config.js';
import { applyTextDirection } from './rtl-handler.js';

// Current language state
let currentLanguage = DEFAULT_LANGUAGE;

// Listeners for language change events
const languageChangeListeners = [];

/**
 * Get current language code
 * @returns {string}
 */
export function getCurrentLanguage() {
    return currentLanguage;
}

/**
 * Set the current language
 * @param {string} langCode - Language code (en, fr, ar)
 * @param {boolean} persist - Whether to save to localStorage
 * @returns {boolean} Success
 */
export function setLanguage(langCode, persist = true) {
    // Validate language code
    if (!LANGUAGES[langCode]) {
        console.warn(`Language '${langCode}' is not supported. Using default.`);
        langCode = DEFAULT_LANGUAGE;
    }

    const previousLanguage = currentLanguage;
    currentLanguage = langCode;

    // Apply text direction (RTL/LTR)
    applyTextDirection(langCode);

    // Persist to localStorage
    if (persist) {
        persistLanguage(langCode);
    }

    // Notify listeners
    if (previousLanguage !== langCode) {
        notifyLanguageChange(langCode, previousLanguage);
    }

    console.log(`Language changed to: ${langCode}`);
    return true;
}

/**
 * Save language preference to localStorage
 * @param {string} langCode 
 */
export function persistLanguage(langCode) {
    try {
        localStorage.setItem(LANGUAGE_STORAGE_KEY, langCode);
    } catch (error) {
        console.error('Failed to persist language preference:', error);
    }
}

/**
 * Load language preference from localStorage
 * @returns {string} Stored language code or default
 */
export function loadPersistedLanguage() {
    try {
        const stored = localStorage.getItem(LANGUAGE_STORAGE_KEY);
        if (stored && LANGUAGES[stored]) {
            return stored;
        }
    } catch (error) {
        console.error('Failed to load language preference:', error);
    }
    return DEFAULT_LANGUAGE;
}

/**
 * Detect user's preferred language from browser/system
 * @returns {string} Detected or default language code
 */
export function detectUserLanguage() {
    try {
        // Check browser language
        const browserLang = navigator.language || navigator.userLanguage;

        if (browserLang) {
            // Try exact match first (e.g., 'fr-FR')
            if (LANGUAGES[browserLang]) {
                return browserLang;
            }

            // Try base language (e.g., 'fr' from 'fr-FR')
            const baseLang = browserLang.split('-')[0];
            if (LANGUAGES[baseLang]) {
                return baseLang;
            }
        }
    } catch (error) {
        console.error('Failed to detect user language:', error);
    }

    return DEFAULT_LANGUAGE;
}

/**
 * Initialize language from stored preference or detection
 * @returns {string} Initialized language code
 */
export function initializeLanguage() {
    // Priority: localStorage > browser detection > default
    let langCode = loadPersistedLanguage();

    // If no stored preference, try to detect
    if (langCode === DEFAULT_LANGUAGE) {
        const detected = detectUserLanguage();
        if (detected !== DEFAULT_LANGUAGE) {
            langCode = detected;
        }
    }

    setLanguage(langCode, false); // Don't persist on init
    return langCode;
}

/**
 * Add listener for language change events
 * @param {function} callback - Function to call when language changes
 * @returns {function} Unsubscribe function
 */
export function onLanguageChange(callback) {
    languageChangeListeners.push(callback);

    // Return unsubscribe function
    return () => {
        const index = languageChangeListeners.indexOf(callback);
        if (index > -1) {
            languageChangeListeners.splice(index, 1);
        }
    };
}

/**
 * Notify all listeners of language change
 * @param {string} newLang 
 * @param {string} oldLang 
 */
function notifyLanguageChange(newLang, oldLang) {
    languageChangeListeners.forEach(callback => {
        try {
            callback(newLang, oldLang);
        } catch (error) {
            console.error('Error in language change listener:', error);
        }
    });
}

/**
 * Get all available languages with metadata
 * @returns {object[]}
 */
export function getAvailableLanguages() {
    return Object.values(LANGUAGES);
}
