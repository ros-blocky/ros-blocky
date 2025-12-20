/**
 * SetupPy Service - Handles setup.py file modifications
 * Extracted from packageService.js for better separation of concerns
 */

const fs = require('fs').promises;
const path = require('path');

// Reference to get project path (set by packageService)
let getProjectPathFn = null;

/**
 * Initialize the service with a project path getter
 * @param {Function} getProjectPath - Function to get current project path
 */
function init(getProjectPath) {
    getProjectPathFn = getProjectPath;
}

/**
 * Get the current project path
 * @returns {string|null}
 */
function getProjectPath() {
    return getProjectPathFn ? getProjectPathFn() : null;
}

/**
 * Add node entry point to setup.py
 * @param {string} packageName - Package name
 * @param {string} nodeName - Node name
 */
async function addNodeEntryPoint(packageName, nodeName) {
    const projectPath = getProjectPath();
    if (!projectPath) return;

    const setupPyPath = path.join(projectPath, 'src', packageName, 'setup.py');

    try {
        let content = await fs.readFile(setupPyPath, 'utf8');

        // Find the console_scripts section and add the entry point
        const entryPoint = `'${nodeName} = ${packageName}.${nodeName}:main'`;

        // Check if entry point already exists
        if (content.includes(entryPoint)) {
            return;
        }

        // Find console_scripts array and add entry
        const consoleScriptsMatch = content.match(/'console_scripts':\s*\[\s*\]/);
        if (consoleScriptsMatch) {
            // Empty array - add first entry
            content = content.replace(
                /'console_scripts':\s*\[\s*\]/,
                `'console_scripts': [\n            ${entryPoint},\n        ]`
            );
        } else {
            // Array has entries - add to existing
            const match = content.match(/'console_scripts':\s*\[([^\]]*)\]/);
            if (match) {
                let existingEntries = match[1].trim();
                if (existingEntries) {
                    // Ensure existing entries end with a comma
                    if (!existingEntries.endsWith(',')) {
                        existingEntries += ',';
                    }
                    content = content.replace(
                        /'console_scripts':\s*\[[^\]]*\]/,
                        `'console_scripts': [\n            ${existingEntries}\n            ${entryPoint},\n        ]`
                    );
                }
            }
        }

        await fs.writeFile(setupPyPath, content);
    } catch (error) {
        console.error('Error updating setup.py:', error);
    }
}

/**
 * Remove node entry point from setup.py
 * @param {string} packageName - Package name
 * @param {string} nodeName - Node name to remove
 */
async function removeNodeEntryPoint(packageName, nodeName) {
    const projectPath = getProjectPath();
    if (!projectPath) return;

    const setupPyPath = path.join(projectPath, 'src', packageName, 'setup.py');

    try {
        let content = await fs.readFile(setupPyPath, 'utf8');

        // Create regex pattern to match the entry point line (with various whitespace)
        const entryPointPattern = new RegExp(
            `\\s*'${nodeName}\\s*=\\s*${packageName}\\.${nodeName}:main'\\s*,?`,
            'g'
        );

        // Remove the entry point
        content = content.replace(entryPointPattern, '');

        // Clean up any trailing commas before closing bracket
        content = content.replace(/,(\s*)\]/g, '$1]');

        // Clean up empty console_scripts arrays (remove extra whitespace)
        content = content.replace(
            /'console_scripts':\s*\[\s*\n\s*\]/g,
            "'console_scripts': []"
        );

        await fs.writeFile(setupPyPath, content);
        console.log(`Removed entry point for ${nodeName} from setup.py`);
    } catch (error) {
        console.error('Error removing entry point from setup.py:', error);
    }
}

/**
 * Add a data_files entry for a folder (meshes, urdf, config, launch)
 * @param {string} packageName - Package name
 * @param {string} folderName - Folder name (meshes, urdf, config, launch)
 */
async function addDataFilesEntry(packageName, folderName) {
    const projectPath = getProjectPath();
    if (!projectPath) return;

    const setupPyPath = path.join(projectPath, 'src', packageName, 'setup.py');

    try {
        let content = await fs.readFile(setupPyPath, 'utf8');

        // Add necessary imports if not present
        let contentModified = false;

        if (!content.includes('import os')) {
            content = content.replace(
                /^(from setuptools import .+)$/m,
                'import os\n$1'
            );
            contentModified = true;
        }
        if (!content.includes('from glob import glob')) {
            content = content.replace(
                /^(from setuptools import .+)$/m,
                '$1\nfrom glob import glob'
            );
            contentModified = true;
        }

        // Check if entry already exists
        const entryPattern = new RegExp(`os\\.path\\.join\\('share',\\s*package_name,\\s*'${folderName}'\\)`);
        if (entryPattern.test(content)) {
            if (contentModified) {
                await fs.writeFile(setupPyPath, content);
                console.log(`Added missing imports to setup.py for ${folderName}`);
            }
            console.log(`Data files entry for ${folderName} already exists`);
            return;
        }

        // Determine the glob pattern based on folder type
        const globPattern = folderName === 'launch' ? `glob('${folderName}/*.py')` : `glob('${folderName}/*')`;

        // The entry to add
        const entryToAdd = `(os.path.join('share', package_name, '${folderName}'), ${globPattern})`;

        // Find data_files array and add entry
        const dataFilesMatch = content.match(/data_files=\[([\s\S]*?)\],/);
        if (dataFilesMatch) {
            const existingContent = dataFilesMatch[1];
            const newDataFiles = `data_files=[${existingContent}        ${entryToAdd},\n    ],`;
            content = content.replace(dataFilesMatch[0], newDataFiles);
        }

        await fs.writeFile(setupPyPath, content);
        console.log(`Added data_files entry for ${folderName} to setup.py`);
    } catch (error) {
        console.error('Error adding data_files entry to setup.py:', error);
    }
}

/**
 * Remove a data_files entry for a folder
 * @param {string} packageName - Package name
 * @param {string} folderName - Folder name (meshes, urdf, config, launch)
 */
async function removeDataFilesEntry(packageName, folderName) {
    const projectPath = getProjectPath();
    if (!projectPath) return;

    const setupPyPath = path.join(projectPath, 'src', packageName, 'setup.py');

    try {
        let content = await fs.readFile(setupPyPath, 'utf8');

        // Remove the data_files entry line for this folder
        const entryPattern = new RegExp(
            `\\s*\\(os\\.path\\.join\\('share',\\s*package_name,\\s*'${folderName}'\\),\\s*glob\\('[^']*'\\)\\),?`,
            'g'
        );
        content = content.replace(entryPattern, '');

        // Clean up any double newlines
        content = content.replace(/\n\n\n+/g, '\n\n');

        await fs.writeFile(setupPyPath, content);
        console.log(`Removed data_files entry for ${folderName} from setup.py`);
    } catch (error) {
        console.error('Error removing data_files entry from setup.py:', error);
    }
}

module.exports = {
    init,
    addNodeEntryPoint,
    removeNodeEntryPoint,
    addDataFilesEntry,
    removeDataFilesEntry
};
