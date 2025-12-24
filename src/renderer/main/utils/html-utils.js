/**
 * HTML Utilities - Security helpers for XSS prevention
 * Used to sanitize dynamic content before inserting into innerHTML
 */

/**
 * Escapes HTML special characters to prevent XSS attacks
 * @param {string} str - The string to escape
 * @returns {string} - HTML-safe string
 */
export function escapeHtml(str) {
    if (str === null || str === undefined) {
        return '';
    }

    const string = String(str);

    return string
        .replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;')
        .replace(/'/g, '&#x27;');
}

/**
 * Escapes HTML attribute value
 * @param {string} str - The attribute value to escape
 * @returns {string} - Safe attribute value
 */
export function escapeAttribute(str) {
    if (str === null || str === undefined) {
        return '';
    }

    return String(str)
        .replace(/&/g, '&amp;')
        .replace(/"/g, '&quot;')
        .replace(/'/g, '&#x27;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;');
}
