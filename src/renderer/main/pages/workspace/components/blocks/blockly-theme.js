/**
 * Blockly Theme Configuration
 * Custom theme matching app cyan color for URDF blocks
 */

// Wait for Blockly to be loaded
if (typeof Blockly !== 'undefined') {
    window.rosTheme = Blockly.Theme.defineTheme('ros_theme', {
        'base': Blockly.Themes.Classic,
        'blockStyles': {
            'urdf_blocks': {
                'colourPrimary': '#00bbfe',
                'colourSecondary': '#0099cc',
                'colourTertiary': '#007799'
            }
        },
        'categoryStyles': {
            'structure_category': { 'colour': '#00bbfe' }
        },
        'componentStyles': {
            'workspaceBackgroundColour': '#f7f7f7',
            'toolboxBackgroundColour': '#f7f7f7'
        }
    });
    console.log('[Blockly] ROS theme defined');
}
