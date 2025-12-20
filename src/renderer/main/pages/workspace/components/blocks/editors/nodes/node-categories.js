/**
 * Node Categories Configuration
 * Defines the 11 categories for ROS 2 Python Node block editing
 */

/**
 * Node Category Definitions
 * Each category contains an id, icon, label, color, and list of block types
 */
export const NODE_CATEGORIES = [
    {
        id: 'structure',
        icon: 'assets/icons/node-structure.svg',
        iconType: 'image',
        label: 'Structure',
        color: '#4c97ff',
        description: 'Node structure, publishers, and subscribers',
        blocks: [
            'node_structure_node',
            'node_publishers_create',
            'node_publishers_publish',
            'node_subscribers_create'
        ]
    },
    {
        id: 'variables',
        icon: 'assets/icons/node-variables.svg',
        iconType: 'image',
        label: 'Variables',
        color: '#16a085',
        description: 'Variables, lists, and dictionaries',
        blocks: [
            'node_variables_set',
            'node_variables_get',
            'node_variables_number',
            'node_variables_string',
            'node_variables_boolean',
            'node_variables_property',
            'node_variables_scope',
            'node_variables_name'
        ]
    },
    {
        id: 'timers',
        icon: 'assets/icons/node-timers.svg',
        iconType: 'image',
        label: 'Timers',
        color: '#f1c40f',
        description: 'Create and manage timers',
        blocks: [
            'node_timers_create',
            'node_timers_cancel'
        ]
    },
    {
        id: 'messages',
        icon: 'assets/icons/node-messages.svg',
        iconType: 'image',
        label: 'Messages',
        color: '#ee0979',
        description: 'ROS 2 message types',
        blocks: [
            'node_messages_type_string',
            'node_messages_type_int32',
            'node_messages_type_float64',
            'node_messages_type_bool',
            'node_messages_type_twist',
            'node_messages_type_pose',
            'node_messages_string',
            'node_messages_int32',
            'node_messages_float64',
            'node_messages_bool',
            'node_messages_twist',
            'node_messages_pose',
            'node_messages_point'
        ]
    },
    {
        id: 'procedures',
        icon: 'assets/icons/node-parameters.svg',
        iconType: 'image',
        label: 'Procedures',
        color: '#9b59b6',
        description: 'Define and call functions',
        blocks: [
            'node_procedures_define',
            'node_procedures_call'
        ]
    },
    {
        id: 'logging',
        icon: 'assets/icons/node-logging.svg',
        iconType: 'image',
        label: 'Logging',
        color: '#7f8c8d',
        description: 'Node logging functions',
        blocks: [
            'node_logging_info',
            'node_logging_warn',
            'node_logging_error',
            'node_logging_debug'
        ]
    },
    {
        id: 'controlflow',
        icon: 'assets/icons/node-controlflow.svg',
        iconType: 'image',
        label: 'Control Flow',
        color: '#E9A23B',
        description: 'Conditionals and loops',
        blocks: [
            'node_controlflow_if',
            'node_controlflow_ifelse',
            'node_controlflow_for',
            'node_controlflow_while',
            'node_controlflow_break',
            'node_controlflow_continue'
        ]
    },
    {
        id: 'math',
        icon: 'assets/icons/node-math.svg',
        iconType: 'image',
        label: 'Math & Logic',
        color: '#e74c3c',
        description: 'Arithmetic, comparison, and logic operations',
        blocks: [
            'node_controlflow_compare',
            'node_controlflow_logic',
            'node_controlflow_not',
            'node_math_arithmetic',
            'node_math_function'
        ]
    }
];

/**
 * Get category by ID
 * @param {string} categoryId - Category ID
 * @returns {Object|null} Category configuration or null
 */
export function getCategoryById(categoryId) {
    return NODE_CATEGORIES.find(cat => cat.id === categoryId) || null;
}

/**
 * Get blocks for a category
 * @param {string} categoryId - Category ID
 * @returns {string[]} Array of block type names
 */
export function getBlocksForCategory(categoryId) {
    const category = getCategoryById(categoryId);
    return category ? [...category.blocks] : [];
}

/**
 * Get category for a block type
 * @param {string} blockType - Block type name
 * @returns {Object|null} Category containing the block or null
 */
export function getCategoryForBlock(blockType) {
    return NODE_CATEGORIES.find(cat => cat.blocks.includes(blockType)) || null;
}

/**
 * Get all category IDs
 * @returns {string[]} Array of category IDs
 */
export function getCategoryIds() {
    return NODE_CATEGORIES.map(cat => cat.id);
}

/**
 * Get default (first) category
 * @returns {Object} First category configuration
 */
export function getDefaultCategory() {
    return NODE_CATEGORIES[0];
}

/**
 * Get all block types for the node editor
 * @returns {string[]} Array of all block type names
 */
export function getAllNodeBlockTypes() {
    return NODE_CATEGORIES.flatMap(cat => cat.blocks);
}
