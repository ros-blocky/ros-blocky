/**
 * URDF Code Generator for Blockly
 * Converts Blockly workspace blocks into valid URDF/XACRO XML
 */

/**
 * Initialize the URDF code generator
 * @param {Object} Blockly - The Blockly library
 */
export function initUrdfGenerator(Blockly) {
    console.log('[URDF Generator] Initializing code generators...');

    // Create a new generator instance for URDF
    Blockly.URDF = new Blockly.Generator('URDF');

    // Default order of operations (not really needed for XML but required)
    Blockly.URDF.ORDER_ATOMIC = 0;

    // Indentation for nested XML
    Blockly.URDF.INDENT = '  ';

    /**
     * Generate code for the entire workspace
     * @param {Blockly.Workspace} workspace - The workspace to generate from
     * @returns {string} Generated URDF XML code
     */
    Blockly.URDF.workspaceToCode = function (workspace) {
        const topBlocks = workspace.getTopBlocks(true);
        let code = '';

        for (const block of topBlocks) {
            const blockCode = this.blockToCode(block);
            if (Array.isArray(blockCode)) {
                code += blockCode[0];
            } else {
                code += blockCode;
            }
        }

        return code;
    };

    /**
     * Generate code for statements (nested blocks)
     * @param {Blockly.Block} block - The parent block
     * @param {string} name - Name of the statement input
     * @returns {string} Generated code for all nested blocks
     */
    Blockly.URDF.statementToCode = function (block, name) {
        const targetBlock = block.getInputTargetBlock(name);
        let code = '';
        let currentBlock = targetBlock;

        while (currentBlock) {
            const blockCode = this.blockToCode(currentBlock);
            if (Array.isArray(blockCode)) {
                code += blockCode[0];
            } else if (blockCode) {
                code += blockCode;
            }
            currentBlock = currentBlock.getNextBlock();
        }

        // Add indentation to nested content
        if (code) {
            code = this.prefixLines(code, this.INDENT);
        }

        return code;
    };

    /**
     * Add prefix to each line of code
     */
    Blockly.URDF.prefixLines = function (text, prefix) {
        return text.split('\n').map(line => line ? prefix + line : line).join('\n');
    };

    // ========================================
    // STRUCTURE BLOCK GENERATORS
    // ========================================

    Blockly.URDF['urdf_robot'] = function (block) {
        const name = block.getFieldValue('NAME') || 'my_robot';
        const content = Blockly.URDF.statementToCode(block, 'CONTENT');

        return `<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="${escapeXml(name)}">
${content}</robot>
`;
    };

    Blockly.URDF['urdf_link'] = function (block) {
        const name = block.getFieldValue('NAME') || 'link_name';
        const content = Blockly.URDF.statementToCode(block, 'CONTENT');

        if (content.trim()) {
            return `<link name="${escapeXml(name)}">
${content}</link>
`;
        } else {
            return `<link name="${escapeXml(name)}"/>
`;
        }
    };

    Blockly.URDF['urdf_link_ref'] = function (block) {
        const name = block.getFieldValue('NAME') || 'link_name';
        return [`"${escapeXml(name)}"`, Blockly.URDF.ORDER_ATOMIC];
    };

    Blockly.URDF['urdf_joint'] = function (block) {
        const name = block.getFieldValue('NAME') || 'joint_name';
        const type = block.getFieldValue('TYPE') || 'fixed';
        const content = Blockly.URDF.statementToCode(block, 'CONTENT');

        return `<joint name="${escapeXml(name)}" type="${escapeXml(type)}">
${content}</joint>
`;
    };

    Blockly.URDF['urdf_parent'] = function (block) {
        const name = block.getFieldValue('NAME') || 'link_name';
        return `<parent link="${escapeXml(name)}"/>
`;
    };

    Blockly.URDF['urdf_child'] = function (block) {
        const name = block.getFieldValue('NAME') || 'link_name';
        return `<child link="${escapeXml(name)}"/>
`;
    };

    // ========================================
    // VISUAL BLOCK GENERATORS
    // ========================================

    Blockly.URDF['urdf_visual'] = function (block) {
        const content = Blockly.URDF.statementToCode(block, 'CONTENT');
        return `<visual>
${content}</visual>
`;
    };

    Blockly.URDF['urdf_collision'] = function (block) {
        const content = Blockly.URDF.statementToCode(block, 'CONTENT');
        return `<collision>
${content}</collision>
`;
    };

    Blockly.URDF['urdf_geometry'] = function (block) {
        const content = Blockly.URDF.statementToCode(block, 'CONTENT');
        return `<geometry>
${content}</geometry>
`;
    };

    Blockly.URDF['urdf_material'] = function (block) {
        const name = block.getFieldValue('NAME') || 'material_name';
        const colorContent = Blockly.URDF.statementToCode(block, 'COLOR');

        if (colorContent.trim()) {
            return `<material name="${escapeXml(name)}">
${colorContent}</material>
`;
        } else {
            return `<material name="${escapeXml(name)}"/>
`;
        }
    };

    Blockly.URDF['urdf_color'] = function (block) {
        const r = block.getFieldValue('R') || 1;
        const g = block.getFieldValue('G') || 0;
        const b = block.getFieldValue('B') || 0;
        const a = block.getFieldValue('A') || 1;
        return `<color rgba="${r} ${g} ${b} ${a}"/>
`;
    };

    // ========================================
    // PROPERTIES BLOCK GENERATORS
    // ========================================

    Blockly.URDF['urdf_inertial'] = function (block) {
        const content = Blockly.URDF.statementToCode(block, 'CONTENT');
        return `<inertial>
${content}</inertial>
`;
    };

    Blockly.URDF['urdf_mass'] = function (block) {
        const value = block.getFieldValue('VALUE') || 1;
        return `<mass value="${value}"/>
`;
    };

    Blockly.URDF['urdf_inertia'] = function (block) {
        const ixx = block.getFieldValue('IXX') || 1;
        const ixy = block.getFieldValue('IXY') || 0;
        const ixz = block.getFieldValue('IXZ') || 0;
        const iyy = block.getFieldValue('IYY') || 1;
        const iyz = block.getFieldValue('IYZ') || 0;
        const izz = block.getFieldValue('IZZ') || 1;
        return `<inertia ixx="${ixx}" ixy="${ixy}" ixz="${ixz}" iyy="${iyy}" iyz="${iyz}" izz="${izz}"/>
`;
    };

    Blockly.URDF['urdf_origin'] = function (block) {
        const x = block.getFieldValue('X') || 0;
        const y = block.getFieldValue('Y') || 0;
        const z = block.getFieldValue('Z') || 0;
        const roll = block.getFieldValue('ROLL') || 0;
        const pitch = block.getFieldValue('PITCH') || 0;
        const yaw = block.getFieldValue('YAW') || 0;
        return `<origin xyz="${x} ${y} ${z}" rpy="${roll} ${pitch} ${yaw}"/>
`;
    };

    Blockly.URDF['urdf_axis'] = function (block) {
        const x = block.getFieldValue('X') || 0;
        const y = block.getFieldValue('Y') || 0;
        const z = block.getFieldValue('Z') || 1;
        return `<axis xyz="${x} ${y} ${z}"/>
`;
    };

    Blockly.URDF['urdf_limit'] = function (block) {
        const lower = block.getFieldValue('LOWER') || -3.14;
        const upper = block.getFieldValue('UPPER') || 3.14;
        const effort = block.getFieldValue('EFFORT') || 100;
        const velocity = block.getFieldValue('VELOCITY') || 1;
        return `<limit lower="${lower}" upper="${upper}" effort="${effort}" velocity="${velocity}"/>
`;
    };

    // ========================================
    // SHAPES BLOCK GENERATORS
    // ========================================

    Blockly.URDF['urdf_box'] = function (block) {
        const x = block.getFieldValue('X') || 1;
        const y = block.getFieldValue('Y') || 1;
        const z = block.getFieldValue('Z') || 1;
        return `<box size="${x} ${y} ${z}"/>
`;
    };

    Blockly.URDF['urdf_cylinder'] = function (block) {
        const radius = block.getFieldValue('RADIUS') || 0.5;
        const length = block.getFieldValue('LENGTH') || 1;
        return `<cylinder radius="${radius}" length="${length}"/>
`;
    };

    Blockly.URDF['urdf_sphere'] = function (block) {
        const radius = block.getFieldValue('RADIUS') || 0.5;
        return `<sphere radius="${radius}"/>
`;
    };

    Blockly.URDF['urdf_mesh'] = function (block) {
        const filename = block.getFieldValue('FILENAME') || 'mesh.stl';
        const scaleX = block.getFieldValue('SCALE_X') || 1;
        const scaleY = block.getFieldValue('SCALE_Y') || 1;
        const scaleZ = block.getFieldValue('SCALE_Z') || 1;

        // Use package:// format for ROS compatibility
        const meshPath = `package://\${package_name}/meshes/${escapeXml(filename)}`;

        if (scaleX === 1 && scaleY === 1 && scaleZ === 1) {
            return `<mesh filename="${meshPath}"/>
`;
        } else {
            return `<mesh filename="${meshPath}" scale="${scaleX} ${scaleY} ${scaleZ}"/>
`;
        }
    };

    // ========================================
    // PROCEDURES BLOCK GENERATORS
    // ========================================

    // Store for procedure definitions (populated during code generation)
    const procedureDefinitions = new Map();

    // Define Procedure - stores the content but doesn't output directly
    Blockly.URDF['urdf_define_procedure'] = function (block) {
        const name = block.getFieldValue('NAME') || 'my_component';
        const content = Blockly.URDF.statementToCode(block, 'CONTENT');

        // Store in our procedure map for lookup by Call blocks
        procedureDefinitions.set(name, content);

        // Return empty - definitions don't output code directly
        // The code is inserted where Call blocks are used
        return '';
    };

    // Call Procedure - looks up and inserts the procedure content
    Blockly.URDF['urdf_call_procedure'] = function (block) {
        const name = block.getFieldValue('NAME') || 'my_component';

        // Look up the procedure definition
        const content = procedureDefinitions.get(name);

        if (content) {
            // Return the stored content (already indented from definition)
            return content.replace(/^  /gm, ''); // Remove one level of indentation
        } else {
            // Procedure not found - return a comment
            return `<!-- Procedure "${escapeXml(name)}" not defined -->
`;
        }
    };

    // Override workspaceToCode to process definitions first
    const originalWorkspaceToCode = Blockly.URDF.workspaceToCode;
    Blockly.URDF.workspaceToCode = function (workspace) {
        // Clear procedure definitions
        procedureDefinitions.clear();

        // Valid top-level block types
        const validTopLevelBlocks = ['urdf_robot', 'urdf_define_procedure'];

        // First pass: find and process all procedure definitions
        const topBlocks = workspace.getTopBlocks(true);
        for (const block of topBlocks) {
            if (block.type === 'urdf_define_procedure') {
                this.blockToCode(block);
            }
        }

        // Second pass: generate code ONLY from Robot blocks
        // Other blocks at top level are ignored (they must be inside Robot)
        let code = '';
        let skippedBlocks = [];

        for (const block of topBlocks) {
            if (block.type === 'urdf_robot') {
                // Valid: Robot block at top level
                const blockCode = this.blockToCode(block);
                if (Array.isArray(blockCode)) {
                    code += blockCode[0];
                } else {
                    code += blockCode;
                }
            } else if (block.type !== 'urdf_define_procedure') {
                // Invalid: Non-robot, non-procedure block at top level
                skippedBlocks.push(block.type);
            }
        }

        // Store skipped blocks for external access
        lastSkippedBlocks = skippedBlocks;

        // Log warning if blocks were skipped
        if (skippedBlocks.length > 0) {
            console.warn('[URDF Generator] Skipped orphan blocks (not inside Robot):', skippedBlocks);
        }

        return code;
    };

    console.log('[URDF Generator] Code generators registered');
}

// Store last skipped blocks for warning display
let lastSkippedBlocks = [];

/**
 * Get blocks that were skipped in the last code generation
 * @returns {string[]} Array of skipped block type names
 */
export function getSkippedBlocks() {
    return [...lastSkippedBlocks];
}

/**
 * Check if last generation had orphan blocks
 * @returns {boolean} True if blocks were skipped
 */
export function hasOrphanBlocks() {
    return lastSkippedBlocks.length > 0;
}

/**
 * Generate URDF code from a workspace
 * @param {Blockly.Workspace} workspace - The Blockly workspace
 * @returns {string} Generated URDF XML
 */
export function generateUrdfCode(workspace) {
    if (!workspace || typeof Blockly === 'undefined' || !Blockly.URDF) {
        console.error('[URDF Generator] Cannot generate code - workspace or generator not ready');
        return '';
    }

    try {
        const code = Blockly.URDF.workspaceToCode(workspace);
        console.log('[URDF Generator] Generated URDF code:', code.substring(0, 200) + '...');
        return code;
    } catch (error) {
        console.error('[URDF Generator] Error generating code:', error);
        return '';
    }
}

// ========================================
// UTILITY FUNCTIONS
// ========================================

/**
 * Escape special XML characters
 * @param {string} text - Text to escape
 * @returns {string} Escaped text
 */
function escapeXml(text) {
    if (typeof text !== 'string') {
        return String(text);
    }
    return text
        .replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;')
        .replace(/'/g, '&apos;');
}
