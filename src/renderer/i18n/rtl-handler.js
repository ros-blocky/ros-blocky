/**
 * RTL Handler
 * Manages Right-to-Left layout for Arabic and other RTL languages
 */

import { isRTL, getLanguageInfo } from './i18n-config.js';

/**
 * Apply RTL or LTR direction to the document
 * Note: We only set the lang attribute, NOT the dir attribute
 * This allows Arabic text to display correctly without flipping the UI layout
 * @param {string} langCode - Language code
 */
export function applyTextDirection(langCode) {
    // Only set language attribute - don't flip the layout
    document.documentElement.setAttribute('lang', langCode);

    // Keep layout LTR always - don't set dir="rtl"
    // Arabic text will still render correctly due to Unicode bidirectional algorithm
    document.documentElement.setAttribute('dir', 'ltr');
    document.body.classList.remove('rtl');
    document.body.classList.add('ltr');

    console.log(`Applied language: ${langCode} (layout stays LTR)`);
}

/**
 * Get current text direction
 * @returns {'ltr' | 'rtl'}
 */
export function getCurrentDirection() {
    return document.documentElement.getAttribute('dir') || 'ltr';
}

/**
 * Check if current layout is RTL
 * @returns {boolean}
 */
export function isCurrentRTL() {
    return getCurrentDirection() === 'rtl';
}

/**
 * Mirror an icon for RTL (e.g., arrows, chevrons)
 * @param {HTMLElement} element - Element to mirror
 */
export function mirrorIcon(element) {
    if (isCurrentRTL()) {
        element.style.transform = 'scaleX(-1)';
    } else {
        element.style.transform = '';
    }
}

/**
 * Get CSS logical property value
 * Converts left/right to start/end for RTL compatibility
 * @param {'left' | 'right'} physicalValue 
 * @returns {'start' | 'end'}
 */
export function getLogicalValue(physicalValue) {
    const isRtl = isCurrentRTL();

    if (physicalValue === 'left') {
        return isRtl ? 'end' : 'start';
    } else if (physicalValue === 'right') {
        return isRtl ? 'start' : 'end';
    }

    return physicalValue;
}
