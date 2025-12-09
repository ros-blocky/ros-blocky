/**
 * URDF Categories Configuration
 * Defines the 4 main categories for URDF block editing
 */

/**
 * URDF Category Definitions
 * Each category contains an id, icon, label, color, and list of block types
 */
export const URDF_CATEGORIES = [
    {
        id: 'structure',
        icon: 'assets/icons/structure.svg',
        iconType: 'image',
        label: 'Structure',
        color: '#667eea',
        description: 'Robot structure elements - links and joints',
        blocks: [
            'urdf_robot',
            'urdf_link',
            'urdf_joint_fixed',
            'urdf_joint_revolute',
            'urdf_joint_continuous'
        ]
    },
    {
        id: 'visual',
        icon: 'assets/icons/visual.svg',
        iconType: 'image',
        label: 'Visual',
        color: '#764ba2',
        description: 'Visual and collision elements',
        blocks: [
            'urdf_visual',
            'urdf_collision',
            'urdf_material',
            'urdf_color'
        ]
    },
    {
        id: 'properties',
        icon: 'assets/icons/properties.svg',
        iconType: 'image',
        label: 'Properties',
        color: '#11998e',
        description: 'Physical properties and transforms',
        blocks: [
            'urdf_inertial',
            'urdf_mass',
            'urdf_inertia',
            'urdf_origin',
            'urdf_axis',
            'urdf_limit'
        ]
    },
    {
        id: 'shapes',
        icon: 'assets/icons/shapes.svg',
        iconType: 'image',
        label: 'Shapes',
        color: '#ee0979',
        description: 'Geometry primitives and meshes',
        blocks: [
            'urdf_box',
            'urdf_cylinder',
            'urdf_sphere',
            'urdf_mesh'
        ]
    }
];

/**
 * Get category by ID
 * @param {string} categoryId - Category ID
 * @returns {Object|null} Category configuration or null
 */
export function getCategoryById(categoryId) {
    return URDF_CATEGORIES.find(cat => cat.id === categoryId) || null;
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
    return URDF_CATEGORIES.find(cat => cat.blocks.includes(blockType)) || null;
}

/**
 * Get all category IDs
 * @returns {string[]} Array of category IDs
 */
export function getCategoryIds() {
    return URDF_CATEGORIES.map(cat => cat.id);
}

/**
 * Get default (first) category
 * @returns {Object} First category configuration
 */
export function getDefaultCategory() {
    return URDF_CATEGORIES[0];
}
