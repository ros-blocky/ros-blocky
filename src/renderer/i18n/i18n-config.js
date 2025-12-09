/**
 * i18n Configuration
 * Language settings and metadata for internationalization
 */

// Supported languages configuration
export const LANGUAGES = {
    en: {
        code: 'en',
        name: 'English',
        nativeName: 'English',
        direction: 'ltr',
        flag: '🇺🇸'
    },
    fr: {
        code: 'fr',
        name: 'French',
        nativeName: 'Français',
        direction: 'ltr',
        flag: '🇫🇷'
    },
    ar: {
        code: 'ar',
        name: 'Arabic',
        nativeName: 'العربية',
        direction: 'rtl',
        flag: '🇸🇦'
    }
};

// Default and fallback language
export const DEFAULT_LANGUAGE = 'en';
export const FALLBACK_LANGUAGE = 'en';

// LocalStorage key for persisting language preference
export const LANGUAGE_STORAGE_KEY = 'ros-blocky-language';

// Translation namespaces (for lazy loading in the future)
export const NAMESPACES = ['common', 'packages', 'dialogs', 'settings', 'workspace'];

/**
 * Get language metadata
 * @param {string} langCode - Language code (en, fr, ar)
 * @returns {object} Language metadata
 */
export function getLanguageInfo(langCode) {
    return LANGUAGES[langCode] || LANGUAGES[DEFAULT_LANGUAGE];
}

/**
 * Check if language is RTL
 * @param {string} langCode - Language code
 * @returns {boolean}
 */
export function isRTL(langCode) {
    const lang = LANGUAGES[langCode];
    return lang ? lang.direction === 'rtl' : false;
}

/**
 * Get all supported language codes
 * @returns {string[]}
 */
export function getSupportedLanguages() {
    return Object.keys(LANGUAGES);
}
