/**
 * URDF Block Definitions for Blockly
 * Defines all blocks for URDF editing with security sanitization
 * 
 * Categories:
 * - Structure: Robot, Link, Joints
 * - Visual: Visual, Collision, Material, Color
 * - Properties: Inertial, Mass, Inertia, Origin, Axis, Limit
 * - Shapes: Box, Cylinder, Sphere, Mesh
 */

import { sanitizeInput } from '../../core/editor-registry.js';
import { t } from '../../../../../../../i18n/index.js';

// Helper to get block labels
const L = (key) => t(`blocks.urdf.labels.${key}`);
const T = (key) => t(`blocks.urdf.tooltips.${key}`);
const JT = (key) => t(`blocks.urdf.jointTypes.${key}`);

/**
 * Register all URDF blocks with Blockly
 * @param {Object} Blockly - The Blockly library
 */
export function registerUrdfBlocks(Blockly) {
    console.log('[URDF Blocks] Registering block definitions...');

    // ========================================
    // PROCEDURES CATEGORY
    // ========================================

    // Define Procedure block - container for reusable blocks
    Blockly.Blocks['urdf_define_procedure'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('defineProcedure'))
                .appendField(new Blockly.FieldTextInput('my_component', validateName), 'NAME');
            this.appendStatementInput('CONTENT')
                // Allow any top-level URDF commands inside procedures
                .setCheck(['LINK', 'JOINT', 'CALL']);
            this.setColour('#9966FF'); // Purple for procedures
            this.setTooltip(T('defineProcedure'));
            this.setDeletable(true);
            this.setMovable(true);
        }
    };

    // Call Procedure block - references a defined procedure
    Blockly.Blocks['urdf_call_procedure'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('callProcedure'))
                .appendField(new Blockly.FieldTextInput('my_component', validateName), 'NAME');
            this.setPreviousStatement(true, ['LINK', 'JOINT', 'CALL']);
            this.setNextStatement(true, ['LINK', 'JOINT', 'CALL']);
            this.setColour('#9966FF'); // Purple for procedures
            this.setTooltip(T('callProcedure'));
        }
    };

    // ========================================
    // STRUCTURE CATEGORY
    // ========================================

    // Robot block - the main container (NEW)
    Blockly.Blocks['urdf_robot'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('robot'))
                .appendField(L('robotName'))
                .appendField(new Blockly.FieldTextInput('my_robot', validateName), 'NAME');
            this.appendStatementInput('CONTENT')
                // Robot can contain Links, Joints, and Procedure Calls
                .setCheck(['LINK', 'JOINT', 'CALL']);
            this.setColour('#4c97ff');
            this.setTooltip(T('robot'));
        }
    };

    Blockly.Blocks['urdf_link'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('link'))
                .appendField(L('linkName'))
                .appendField(new Blockly.FieldTextInput('link_name', validateName), 'NAME');
            this.appendStatementInput('CONTENT')
                // Link can contain Visual, Collision, Inertial
                .setCheck(['VISUAL', 'COLLISION', 'INERTIAL']);
            this.setPreviousStatement(true, ['LINK', 'JOINT', 'CALL']);
            this.setNextStatement(true, ['LINK', 'JOINT', 'CALL']);
            this.setColour('#4c97ff');
            this.setTooltip(T('link'));
        }
    };

    // Link reference block (No changes needed, output is specific)
    Blockly.Blocks['urdf_link_ref'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('link'))
                .appendField(new Blockly.FieldTextInput('link_name', validateName), 'NAME');
            this.setOutput(true, 'Link');
            this.setColour('#4c97ff');
            this.setTooltip(T('link'));
        }
    };

    // Unified Joint block with type dropdown
    Blockly.Blocks['urdf_joint'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('joint'))
                .appendField(L('jointName'))
                .appendField(new Blockly.FieldTextInput('joint_name', validateName), 'NAME')
                .appendField(L('jointType'))
                .appendField(new Blockly.FieldDropdown([
                    [JT('fixed'), 'fixed'],
                    [JT('revolute'), 'revolute'],
                    [JT('continuous'), 'continuous'],
                    [JT('prismatic'), 'prismatic'],
                    [JT('floating'), 'floating'],
                    [JT('planar'), 'planar']
                ]), 'TYPE');
            this.appendStatementInput('CONTENT')
                // Joint can contain Parent, Child, Origin, Axis, Limit
                .setCheck(['PARENT', 'CHILD', 'ORIGIN', 'AXIS', 'LIMIT']);
            this.setPreviousStatement(true, ['LINK', 'JOINT', 'CALL']);
            this.setNextStatement(true, ['LINK', 'JOINT', 'CALL']);
            this.setColour('#4c97ff');
            this.setTooltip(T('joint'));
        }
    };

    // Parent link reference block
    Blockly.Blocks['urdf_parent'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('parent'))
                .appendField(L('link'))
                .appendField(new Blockly.FieldTextInput('link_name', validateName), 'NAME');
            this.setPreviousStatement(true, ['PARENT', 'CHILD', 'ORIGIN', 'AXIS', 'LIMIT']);
            this.setNextStatement(true, ['PARENT', 'CHILD', 'ORIGIN', 'AXIS', 'LIMIT']);
            this.setColour('#4c97ff');
            this.setTooltip(T('parent'));
        }
    };

    // Child link reference block
    Blockly.Blocks['urdf_child'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('child'))
                .appendField(L('link'))
                .appendField(new Blockly.FieldTextInput('link_name', validateName), 'NAME');
            this.setPreviousStatement(true, ['PARENT', 'CHILD', 'ORIGIN', 'AXIS', 'LIMIT']);
            this.setNextStatement(true, ['PARENT', 'CHILD', 'ORIGIN', 'AXIS', 'LIMIT']);
            this.setColour('#4c97ff');
            this.setTooltip(T('child'));
        }
    };

    // ========================================
    // VISUAL CATEGORY
    // ========================================

    Blockly.Blocks['urdf_visual'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('visual'));
            this.appendStatementInput('CONTENT')
                // Visual can contain Geometry, Material, Origin
                .setCheck(['GEOMETRY', 'MATERIAL', 'ORIGIN']);
            this.setPreviousStatement(true, ['VISUAL', 'COLLISION', 'INERTIAL']);
            this.setNextStatement(true, ['VISUAL', 'COLLISION', 'INERTIAL']);
            this.setColour('#FFAB19');
            this.setTooltip(T('visual'));
        }
    };

    Blockly.Blocks['urdf_collision'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('collision'));
            this.appendStatementInput('CONTENT')
                // Collision can contain Geometry, Origin
                .setCheck(['GEOMETRY', 'ORIGIN']);
            this.setPreviousStatement(true, ['VISUAL', 'COLLISION', 'INERTIAL']);
            this.setNextStatement(true, ['VISUAL', 'COLLISION', 'INERTIAL']);
            this.setColour('#FFAB19');
            this.setTooltip(T('collision'));
        }
    };

    Blockly.Blocks['urdf_material'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('material'))
                .appendField(new Blockly.FieldTextInput('material_name', validateName), 'NAME');
            this.appendStatementInput('COLOR')
                .appendField(L('color'))
                .setCheck('COLOR');
            this.setPreviousStatement(true, ['GEOMETRY', 'MATERIAL', 'ORIGIN']);
            this.setNextStatement(true, ['GEOMETRY', 'MATERIAL', 'ORIGIN']);
            this.setColour('#FFAB19');
            this.setTooltip(T('material'));
        }
    };

    Blockly.Blocks['urdf_color'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('color'))
                .appendField('R').appendField(new Blockly.FieldNumber(1, 0, 1, 0.1), 'R')
                .appendField('G').appendField(new Blockly.FieldNumber(0, 0, 1, 0.1), 'G')
                .appendField('B').appendField(new Blockly.FieldNumber(0, 0, 1, 0.1), 'B')
                .appendField('A').appendField(new Blockly.FieldNumber(1, 0, 1, 0.1), 'A');
            this.setPreviousStatement(true, 'COLOR');
            this.setNextStatement(true, 'COLOR');
            this.setColour('#FFAB19');
            this.setTooltip(T('color'));
        }
    };

    Blockly.Blocks['urdf_geometry'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('geometry'));
            this.appendStatementInput('CONTENT')
                // Geometry contains SHAPES
                .setCheck(['SHAPE']);
            this.setPreviousStatement(true, ['GEOMETRY', 'MATERIAL', 'ORIGIN']);
            this.setNextStatement(true, ['GEOMETRY', 'MATERIAL', 'ORIGIN']);
            this.setColour('#FFAB19');
            this.setTooltip(T('geometry'));
        }
    };

    // ========================================
    // PROPERTIES CATEGORY
    // ========================================

    Blockly.Blocks['urdf_inertial'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('inertial'));
            this.appendStatementInput('CONTENT')
                .setCheck(['MASS', 'INERTIA', 'ORIGIN']);
            this.setPreviousStatement(true, ['VISUAL', 'COLLISION', 'INERTIAL']);
            this.setNextStatement(true, ['VISUAL', 'COLLISION', 'INERTIAL']);
            this.setColour('#FFAB19');
            this.setTooltip(T('inertial'));
        }
    };

    Blockly.Blocks['urdf_mass'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('mass'))
                .appendField(new Blockly.FieldNumber(1, 0), 'VALUE')
                .appendField(L('massUnit'));
            this.setPreviousStatement(true, ['MASS', 'INERTIA', 'ORIGIN']);
            this.setNextStatement(true, ['MASS', 'INERTIA', 'ORIGIN']);
            this.setColour('#40BF4A');
            this.setTooltip(T('mass'));
        }
    };

    Blockly.Blocks['urdf_inertia'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('inertia'))
                .appendField('ixx').appendField(new Blockly.FieldNumber(1), 'IXX')
                .appendField('ixy').appendField(new Blockly.FieldNumber(0), 'IXY')
                .appendField('ixz').appendField(new Blockly.FieldNumber(0), 'IXZ')
                .appendField('iyy').appendField(new Blockly.FieldNumber(1), 'IYY')
                .appendField('iyz').appendField(new Blockly.FieldNumber(0), 'IYZ')
                .appendField('izz').appendField(new Blockly.FieldNumber(1), 'IZZ');
            this.setPreviousStatement(true, ['MASS', 'INERTIA', 'ORIGIN']);
            this.setNextStatement(true, ['MASS', 'INERTIA', 'ORIGIN']);
            this.setColour('#40BF4A');
            this.setTooltip(T('inertia'));
        }
    };

    Blockly.Blocks['urdf_origin'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('origin'))
                .appendField(L('originXyz'))
                .appendField(new Blockly.FieldNumber(0), 'X')
                .appendField(new Blockly.FieldNumber(0), 'Y')
                .appendField(new Blockly.FieldNumber(0), 'Z')
                .appendField(L('originRpy'))
                .appendField(new Blockly.FieldNumber(0), 'ROLL')
                .appendField(new Blockly.FieldNumber(0), 'PITCH')
                .appendField(new Blockly.FieldNumber(0), 'YAW');
            // Origins appear in many places: Joints, Visuals, Collisions, Inertials
            this.setPreviousStatement(true, ['GEOMETRY', 'MATERIAL', 'ORIGIN', 'MASS', 'INERTIA', 'PARENT', 'CHILD', 'AXIS', 'LIMIT']);
            this.setNextStatement(true, ['GEOMETRY', 'MATERIAL', 'ORIGIN', 'MASS', 'INERTIA', 'PARENT', 'CHILD', 'AXIS', 'LIMIT']);
            this.setColour('#40BF4A');
            this.setTooltip(T('origin'));
        }
    };

    Blockly.Blocks['urdf_axis'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('axis'))
                .appendField('x').appendField(new Blockly.FieldNumber(0, -1, 1), 'X')
                .appendField('y').appendField(new Blockly.FieldNumber(0, -1, 1), 'Y')
                .appendField('z').appendField(new Blockly.FieldNumber(1, -1, 1), 'Z');
            this.setPreviousStatement(true, ['PARENT', 'CHILD', 'ORIGIN', 'AXIS', 'LIMIT']);
            this.setNextStatement(true, ['PARENT', 'CHILD', 'ORIGIN', 'AXIS', 'LIMIT']);
            this.setColour('#40BF4A');
            this.setTooltip(T('axis'));
        }
    };

    Blockly.Blocks['urdf_limit'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('limit'))
                .appendField(L('limitLower')).appendField(new Blockly.FieldNumber(-3.14), 'LOWER')
                .appendField(L('limitUpper')).appendField(new Blockly.FieldNumber(3.14), 'UPPER')
                .appendField(L('limitEffort')).appendField(new Blockly.FieldNumber(100), 'EFFORT')
                .appendField(L('limitVelocity')).appendField(new Blockly.FieldNumber(1), 'VELOCITY');
            this.setPreviousStatement(true, ['PARENT', 'CHILD', 'ORIGIN', 'AXIS', 'LIMIT']);
            this.setNextStatement(true, ['PARENT', 'CHILD', 'ORIGIN', 'AXIS', 'LIMIT']);
            this.setColour('#40BF4A');
            this.setTooltip(T('limit'));
        }
    };

    // ========================================
    // SHAPES CATEGORY
    // ========================================

    Blockly.Blocks['urdf_box'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('box'))
                .appendField('x').appendField(new Blockly.FieldNumber(1, 0), 'X')
                .appendField('y').appendField(new Blockly.FieldNumber(1, 0), 'Y')
                .appendField('z').appendField(new Blockly.FieldNumber(1, 0), 'Z');
            this.setPreviousStatement(true, ['SHAPE']);
            this.setNextStatement(true, ['SHAPE']);
            this.setColour('#ee0979');
            this.setTooltip(T('box'));
        }
    };

    Blockly.Blocks['urdf_cylinder'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('cylinder'))
                .appendField(L('cylinderRadius')).appendField(new Blockly.FieldNumber(0.5, 0), 'RADIUS')
                .appendField(L('cylinderLength')).appendField(new Blockly.FieldNumber(1, 0), 'LENGTH');
            this.setPreviousStatement(true, ['SHAPE']);
            this.setNextStatement(true, ['SHAPE']);
            this.setColour('#ee0979');
            this.setTooltip(T('cylinder'));
        }
    };

    Blockly.Blocks['urdf_sphere'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('sphere'))
                .appendField(L('sphereRadius')).appendField(new Blockly.FieldNumber(0.5, 0), 'RADIUS');
            this.setPreviousStatement(true, ['SHAPE']);
            this.setNextStatement(true, ['SHAPE']);
            this.setColour('#ee0979');
            this.setTooltip(T('sphere'));
        }
    };

    Blockly.Blocks['urdf_mesh'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('mesh'))
                .appendField(L('meshFilename'))
                .appendField(new Blockly.FieldTextInput('mesh.stl'), 'FILENAME')
                .appendField(L('meshScale'))
                .appendField(new Blockly.FieldNumber(1, 0), 'SCALE_X')
                .appendField(new Blockly.FieldNumber(1, 0), 'SCALE_Y')
                .appendField(new Blockly.FieldNumber(1, 0), 'SCALE_Z');
            this.setPreviousStatement(true, ['SHAPE']);
            this.setNextStatement(true, ['SHAPE']);
            this.setColour('#ee0979');
            this.setTooltip(T('mesh'));
        }
    };

    console.log('[URDF Blocks] Block definitions registered');
}

// ========================================
// VALIDATION FUNCTIONS (Security)
// ========================================

/**
 * Validate URDF name field (alphanumeric + underscore)
 * @param {string} newValue - New value to validate
 * @returns {string|null} Validated value or null if invalid
 */
function validateName(newValue) {
    if (!newValue || typeof newValue !== 'string') {
        return 'unnamed';
    }
    // Allow only alphanumeric and underscores for URDF names
    const sanitized = newValue.replace(/[^a-zA-Z0-9_]/g, '_');
    return sanitized || 'unnamed';
}

/**
 * Validate mesh file path
 * @param {string} newValue - New value to validate
 * @returns {string} Validated path
 */
function validateMeshPath(newValue) {
    if (!newValue || typeof newValue !== 'string') {
        return 'package://';
    }
    // Basic sanitization - remove potentially dangerous characters
    const sanitized = newValue.replace(/[<>"|?*]/g, '');
    return sanitized;
}

/**
 * Get all block type names
 * @returns {string[]} Array of all URDF block type names
 */
export function getAllUrdfBlockTypes() {
    return [
        // Procedures
        'urdf_define_procedure', 'urdf_call_procedure',
        // Structure
        'urdf_robot', 'urdf_link', 'urdf_link_ref', 'urdf_joint', 'urdf_parent', 'urdf_child',
        // Visual
        'urdf_visual', 'urdf_collision', 'urdf_geometry', 'urdf_material', 'urdf_color',
        // Properties
        'urdf_inertial', 'urdf_mass', 'urdf_inertia', 'urdf_origin', 'urdf_axis', 'urdf_limit',
        // Shapes
        'urdf_box', 'urdf_cylinder', 'urdf_sphere', 'urdf_mesh'
    ];
}
