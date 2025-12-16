/**
 * Node Theme for Blockly
 * Custom theme matching the 11 Node category colors
 */

/**
 * Create Node theme for Blockly
 * @param {Object} Blockly - The Blockly library
 * @returns {Object} Theme object
 */
export function createNodeTheme(Blockly) {
    const theme = Blockly.Theme.defineTheme('node_theme', {
        'base': Blockly.Themes.Classic,
        'blockStyles': {
            'structure_blocks': {
                'colourPrimary': '#4c97ff',
                'colourSecondary': '#4280d7',
                'colourTertiary': '#3373cc'
            },
            'variables_blocks': {
                'colourPrimary': '#16a085',
                'colourSecondary': '#138d75',
                'colourTertiary': '#117a65'
            },
            'publishers_blocks': {
                'colourPrimary': '#FFAB19',
                'colourSecondary': '#e69900',
                'colourTertiary': '#cc8800'
            },
            'subscribers_blocks': {
                'colourPrimary': '#e67e22',
                'colourSecondary': '#d35400',
                'colourTertiary': '#ba4a00'
            },
            'timers_blocks': {
                'colourPrimary': '#f1c40f',
                'colourSecondary': '#d4ac0d',
                'colourTertiary': '#b7950b'
            },
            'messages_blocks': {
                'colourPrimary': '#ee0979',
                'colourSecondary': '#d5086c',
                'colourTertiary': '#bc075f'
            },
            'parameters_blocks': {
                'colourPrimary': '#00bcd4',
                'colourSecondary': '#00a5bb',
                'colourTertiary': '#008ea1'
            },
            'logging_blocks': {
                'colourPrimary': '#7f8c8d',
                'colourSecondary': '#6c7a7b',
                'colourTertiary': '#5a6869'
            },
            'controlflow_blocks': {
                'colourPrimary': '#9966FF',
                'colourSecondary': '#8855ee',
                'colourTertiary': '#7744dd'
            },
            'math_blocks': {
                'colourPrimary': '#e74c3c',
                'colourSecondary': '#d43f30',
                'colourTertiary': '#c23324'
            },
            'turtlesim_blocks': {
                'colourPrimary': '#1abc9c',
                'colourSecondary': '#17a689',
                'colourTertiary': '#149076'
            }
        },
        'categoryStyles': {
            'structure_category': { 'colour': '#4c97ff' },
            'variables_category': { 'colour': '#16a085' },
            'publishers_category': { 'colour': '#FFAB19' },
            'subscribers_category': { 'colour': '#e67e22' },
            'timers_category': { 'colour': '#f1c40f' },
            'messages_category': { 'colour': '#ee0979' },
            'parameters_category': { 'colour': '#00bcd4' },
            'logging_category': { 'colour': '#7f8c8d' },
            'controlflow_category': { 'colour': '#9966FF' },
            'math_category': { 'colour': '#e74c3c' },
            'turtlesim_category': { 'colour': '#1abc9c' }
        },
        'componentStyles': {
            'workspaceBackgroundColour': '#f7f7f7',
            'toolboxBackgroundColour': '#f7f7f7',
            'flyoutBackgroundColour': '#f7f7f7',
            'flyoutForegroundColour': '#333',
            'scrollbarColour': '#c0c0c0'
        }
    });

    console.log('[Node Theme] Theme created');
    return theme;
}

/**
 * Get theme name
 * @returns {string} Theme name
 */
export function getNodeThemeName() {
    return 'node_theme';
}
