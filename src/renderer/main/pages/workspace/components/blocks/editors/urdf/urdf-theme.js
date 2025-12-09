/**
 * URDF Theme for Blockly
 * Custom theme matching app cyan color for URDF blocks
 */

/**
 * Create URDF theme for Blockly
 * @param {Object} Blockly - The Blockly library
 * @returns {Object} Theme object
 */
export function createUrdfTheme(Blockly) {
    const theme = Blockly.Theme.defineTheme('urdf_theme', {
        'base': Blockly.Themes.Classic,
        'blockStyles': {
            'structure_blocks': {
                'colourPrimary': '#667eea',
                'colourSecondary': '#5a72d4',
                'colourTertiary': '#4e66be'
            },
            'visual_blocks': {
                'colourPrimary': '#764ba2',
                'colourSecondary': '#6a438f',
                'colourTertiary': '#5e3b7c'
            },
            'properties_blocks': {
                'colourPrimary': '#11998e',
                'colourSecondary': '#0f8a7f',
                'colourTertiary': '#0d7b70'
            },
            'shapes_blocks': {
                'colourPrimary': '#ee0979',
                'colourSecondary': '#d5086c',
                'colourTertiary': '#bc075f'
            }
        },
        'categoryStyles': {
            'structure_category': { 'colour': '#667eea' },
            'visual_category': { 'colour': '#764ba2' },
            'properties_category': { 'colour': '#11998e' },
            'shapes_category': { 'colour': '#ee0979' }
        },
        'componentStyles': {
            'workspaceBackgroundColour': '#f7f7f7',
            'toolboxBackgroundColour': '#f7f7f7',
            'flyoutBackgroundColour': '#f7f7f7',
            'flyoutForegroundColour': '#333',
            'scrollbarColour': '#c0c0c0'
        }
    });

    console.log('[URDF Theme] Theme created');
    return theme;
}

/**
 * Get theme name
 * @returns {string} Theme name
 */
export function getUrdfThemeName() {
    return 'urdf_theme';
}
