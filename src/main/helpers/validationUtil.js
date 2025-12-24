/**
 * Validation Utility - Input sanitization for security
 * Prevents command injection attacks by validating user inputs
 */

/**
 * Valid characters for ROS package/node/file names
 * Allows: alphanumeric, underscores, hyphens, dots
 */
const VALID_NAME_REGEX = /^[a-zA-Z][a-zA-Z0-9_\-\.]*$/;

/**
 * Valid characters for ROS topic names
 * Must start with /, allows: alphanumeric, underscores, forward slashes
 */
const VALID_TOPIC_REGEX = /^\/[a-zA-Z0-9_\/]*$/;

/**
 * Validates a ROS package, node, or file name
 * @param {string} name - The name to validate
 * @returns {boolean} True if valid
 */
function isValidName(name) {
    if (!name || typeof name !== 'string') {
        return false;
    }

    // Check length limits
    if (name.length < 1 || name.length > 255) {
        return false;
    }

    // Check for valid characters
    return VALID_NAME_REGEX.test(name);
}

/**
 * Validates a ROS topic name
 * @param {string} topicName - The topic name to validate
 * @returns {boolean} True if valid
 */
function isValidTopicName(topicName) {
    if (!topicName || typeof topicName !== 'string') {
        return false;
    }

    // Check length limits
    if (topicName.length < 2 || topicName.length > 255) {
        return false;
    }

    // Check for valid topic format
    return VALID_TOPIC_REGEX.test(topicName);
}

/**
 * Validates a file path to prevent directory traversal
 * @param {string} filePath - The path to validate
 * @param {string} basePath - The expected base directory
 * @returns {boolean} True if path is within basePath
 */
function isValidPath(filePath, basePath) {
    if (!filePath || !basePath) {
        return false;
    }

    const path = require('path');

    // Normalize both paths
    const normalizedPath = path.normalize(path.resolve(filePath));
    const normalizedBase = path.normalize(path.resolve(basePath));

    // Ensure the path starts with the base path
    return normalizedPath.startsWith(normalizedBase);
}

/**
 * Sanitizes a name by removing invalid characters
 * Use this for display purposes, not for validation
 * @param {string} name - The name to sanitize
 * @returns {string} Sanitized name
 */
function sanitizeName(name) {
    if (!name || typeof name !== 'string') {
        return '';
    }

    // Replace invalid characters with underscores
    return name.replace(/[^a-zA-Z0-9_\-\.]/g, '_');
}

/**
 * Creates a validation error result object
 * @param {string} field - The field that failed validation
 * @param {string} value - The invalid value (truncated for security)
 * @returns {Object} Error result object
 */
function createValidationError(field, value) {
    const displayValue = value && typeof value === 'string'
        ? value.substring(0, 50)
        : 'invalid';

    return {
        success: false,
        error: `Invalid ${field}: "${displayValue}". Only alphanumeric characters, underscores, hyphens, and dots are allowed.`
    };
}

module.exports = {
    isValidName,
    isValidTopicName,
    isValidPath,
    sanitizeName,
    createValidationError
};
