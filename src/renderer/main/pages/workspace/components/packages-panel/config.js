/**
 * Packages Panel Configuration
 * Static configuration and shared state for the packages panel
 */

// File sections configuration (URDF, CONFIG, LAUNCH, MESHES)
export const FILE_SECTIONS = {
    meshes: {
        label: 'MESHES',
        icon: '🦾',
        extensions: ['.stl', '.dae', '.STL', '.DAE'],
        fileTypeLabel: 'Mesh files',
        mode: 'import' // Import existing files
    },
    urdf: {
        label: 'URDF',
        icon: '🤖',
        extensions: ['.urdf', '.xacro'],
        fileTypeLabel: 'URDF files',
        mode: 'create' // Create new file
    },
    config: {
        label: 'CONFIG',
        icon: '⚙️',
        extensions: ['.yaml', '.json', '.yml'],
        fileTypeLabel: 'Config files',
        mode: 'create' // Create new file
    },
    launch: {
        label: 'LAUNCH',
        icon: '🚀',
        extensions: ['.py', '.launch'],
        fileTypeLabel: 'Launch files',
        mode: 'create' // Create new file
    }
};

// Display names for delete confirmation messages
export const SECTION_DISPLAY_NAMES = {
    'nodes': 'nodes',
    'meshes': 'mesh files',
    'urdf': 'URDF files',
    'config': 'config files',
    'launch': 'launch files'
};

// Context state for context menu operations
export const contextState = {
    currentPackageContext: null,
    currentNodeContext: null,
    currentFileContext: { folder: null, fileName: null },
    currentFolderContext: null
};

/**
 * Reset all context state
 */
export function resetContextState() {
    contextState.currentPackageContext = null;
    contextState.currentNodeContext = null;
    contextState.currentFileContext = { folder: null, fileName: null };
    contextState.currentFolderContext = null;
}
