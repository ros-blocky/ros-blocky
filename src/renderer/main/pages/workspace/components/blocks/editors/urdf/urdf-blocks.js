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

/**
 * Register all URDF blocks with Blockly
 * @param {Object} Blockly - The Blockly library
 */
export function registerUrdfBlocks(Blockly) {
    console.log('[URDF Blocks] Registering block definitions...');

    // ========================================
    // STRUCTURE CATEGORY
    // ========================================

    // Robot block - the main container (NEW)
    Blockly.Blocks['urdf_robot'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Robot')
                .appendField('name')
                .appendField(new Blockly.FieldTextInput('my_robot', validateName), 'NAME');
            this.appendStatementInput('CONTENT')
                .setCheck(null);
            this.setColour('#4c97ff');
            this.setTooltip('URDF Robot definition - the root element');
            this.setHelpUrl('http://wiki.ros.org/urdf/XML/robot');
        }
    };

    Blockly.Blocks['urdf_link'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Link')
                .appendField('name')
                .appendField(new Blockly.FieldTextInput('link_name', validateName), 'NAME');
            this.appendStatementInput('CONTENT')
                .setCheck(null);
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#4c97ff');
            this.setTooltip('Define a URDF link element');
            this.setHelpUrl('http://wiki.ros.org/urdf/XML/link');
        }
    };

    // Link reference block for connecting to Joint parent/child inputs
    Blockly.Blocks['urdf_link_ref'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Link')
                .appendField(new Blockly.FieldTextInput('link_name', validateName), 'NAME');
            this.setOutput(true, 'Link');
            this.setColour('#4c97ff');
            this.setTooltip('Reference to a link');
        }
    };

    // Unified Joint block with type dropdown
    Blockly.Blocks['urdf_joint'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Joint')
                .appendField('name')
                .appendField(new Blockly.FieldTextInput('joint_name', validateName), 'NAME')
                .appendField('type')
                .appendField(new Blockly.FieldDropdown([
                    ['Fixed', 'fixed'],
                    ['Revolute', 'revolute'],
                    ['Continuous', 'continuous'],
                    ['Prismatic', 'prismatic'],
                    ['Floating', 'floating'],
                    ['Planar', 'planar']
                ]), 'TYPE');
            this.appendStatementInput('CONTENT')
                .setCheck(null);
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#4c97ff');
            this.setTooltip('URDF Joint - connects two links');
            this.setHelpUrl('http://wiki.ros.org/urdf/XML/joint');
        }
    };

    // Parent link reference block
    Blockly.Blocks['urdf_parent'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('parent')
                .appendField('link')
                .appendField(new Blockly.FieldTextInput('link_name', validateName), 'NAME');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#4c97ff');
            this.setTooltip('Parent link reference');
        }
    };

    // Child link reference block
    Blockly.Blocks['urdf_child'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('child')
                .appendField('link')
                .appendField(new Blockly.FieldTextInput('link_name', validateName), 'NAME');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#4c97ff');
            this.setTooltip('Child link reference');
        }
    };

    // ========================================
    // VISUAL CATEGORY
    // ========================================

    Blockly.Blocks['urdf_visual'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Visual');
            this.appendStatementInput('CONTENT')
                .setCheck(null);
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#FFAB19');
            this.setTooltip('Visual representation of a link');
            this.setHelpUrl('http://wiki.ros.org/urdf/XML/link');
        }
    };

    Blockly.Blocks['urdf_collision'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Collision');
            this.appendStatementInput('CONTENT')
                .setCheck(null);
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#FFAB19');
            this.setTooltip('Collision geometry of a link');
            this.setHelpUrl('http://wiki.ros.org/urdf/XML/link');
        }
    };

    Blockly.Blocks['urdf_material'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Material')
                .appendField(new Blockly.FieldTextInput('material_name', validateName), 'NAME');
            this.appendStatementInput('COLOR')
                .appendField('color');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#FFAB19');
            this.setTooltip('Material definition');
            this.setHelpUrl('http://wiki.ros.org/urdf/XML/link');
        }
    };

    Blockly.Blocks['urdf_color'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Color RGBA')
                .appendField('R').appendField(new Blockly.FieldNumber(1, 0, 1, 0.1), 'R')
                .appendField('G').appendField(new Blockly.FieldNumber(0, 0, 1, 0.1), 'G')
                .appendField('B').appendField(new Blockly.FieldNumber(0, 0, 1, 0.1), 'B')
                .appendField('A').appendField(new Blockly.FieldNumber(1, 0, 1, 0.1), 'A');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#FFAB19');
            this.setTooltip('RGBA color values (0-1)');
        }
    };

    Blockly.Blocks['urdf_geometry'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Geometry');
            this.appendStatementInput('CONTENT')
                .setCheck(null);
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#FFAB19');
            this.setTooltip('Geometry container - add a shape inside');
        }
    };

    // ========================================
    // PROPERTIES CATEGORY
    // ========================================

    Blockly.Blocks['urdf_inertial'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Inertial');
            this.appendStatementInput('CONTENT')
                .setCheck(null);
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#FFAB19');
            this.setTooltip('Inertial properties of a link');
            this.setHelpUrl('http://wiki.ros.org/urdf/XML/link');
        }
    };

    Blockly.Blocks['urdf_mass'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Mass')
                .appendField(new Blockly.FieldNumber(1, 0), 'VALUE')
                .appendField('kg');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#40BF4A');
            this.setTooltip('Mass in kilograms');
        }
    };

    Blockly.Blocks['urdf_inertia'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Inertia')
                .appendField('ixx').appendField(new Blockly.FieldNumber(1), 'IXX')
                .appendField('ixy').appendField(new Blockly.FieldNumber(0), 'IXY')
                .appendField('ixz').appendField(new Blockly.FieldNumber(0), 'IXZ')
                .appendField('iyy').appendField(new Blockly.FieldNumber(1), 'IYY')
                .appendField('iyz').appendField(new Blockly.FieldNumber(0), 'IYZ')
                .appendField('izz').appendField(new Blockly.FieldNumber(1), 'IZZ');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#40BF4A');
            this.setTooltip('Inertia tensor');
        }
    };

    Blockly.Blocks['urdf_origin'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Origin')
                .appendField('xyz')
                .appendField(new Blockly.FieldNumber(0), 'X')
                .appendField(new Blockly.FieldNumber(0), 'Y')
                .appendField(new Blockly.FieldNumber(0), 'Z')
                .appendField('rpy')
                .appendField(new Blockly.FieldNumber(0), 'ROLL')
                .appendField(new Blockly.FieldNumber(0), 'PITCH')
                .appendField(new Blockly.FieldNumber(0), 'YAW');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#40BF4A');
            this.setTooltip('Origin position (xyz) and orientation (roll, pitch, yaw)');
        }
    };

    Blockly.Blocks['urdf_axis'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Axis')
                .appendField('x').appendField(new Blockly.FieldNumber(0, -1, 1), 'X')
                .appendField('y').appendField(new Blockly.FieldNumber(0, -1, 1), 'Y')
                .appendField('z').appendField(new Blockly.FieldNumber(1, -1, 1), 'Z');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#40BF4A');
            this.setTooltip('Joint rotation axis');
        }
    };

    Blockly.Blocks['urdf_limit'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Limit')
                .appendField('lower').appendField(new Blockly.FieldNumber(-3.14), 'LOWER')
                .appendField('upper').appendField(new Blockly.FieldNumber(3.14), 'UPPER')
                .appendField('effort').appendField(new Blockly.FieldNumber(100), 'EFFORT')
                .appendField('velocity').appendField(new Blockly.FieldNumber(1), 'VELOCITY');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#40BF4A');
            this.setTooltip('Joint limits');
        }
    };

    // ========================================
    // SHAPES CATEGORY
    // ========================================

    Blockly.Blocks['urdf_box'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Box')
                .appendField('x').appendField(new Blockly.FieldNumber(1, 0), 'X')
                .appendField('y').appendField(new Blockly.FieldNumber(1, 0), 'Y')
                .appendField('z').appendField(new Blockly.FieldNumber(1, 0), 'Z');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#ee0979');
            this.setTooltip('Box geometry with size x, y, z');
        }
    };

    Blockly.Blocks['urdf_cylinder'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Cylinder')
                .appendField('radius').appendField(new Blockly.FieldNumber(0.5, 0), 'RADIUS')
                .appendField('length').appendField(new Blockly.FieldNumber(1, 0), 'LENGTH');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#ee0979');
            this.setTooltip('Cylinder geometry');
        }
    };

    Blockly.Blocks['urdf_sphere'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Sphere')
                .appendField('radius').appendField(new Blockly.FieldNumber(0.5, 0), 'RADIUS');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#ee0979');
            this.setTooltip('Sphere geometry');
        }
    };

    Blockly.Blocks['urdf_mesh'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Mesh')
                .appendField('filename')
                .appendField(new Blockly.FieldTextInput('mesh.stl'), 'FILENAME')
                .appendField('scale')
                .appendField(new Blockly.FieldNumber(1, 0), 'SCALE_X')
                .appendField(new Blockly.FieldNumber(1, 0), 'SCALE_Y')
                .appendField(new Blockly.FieldNumber(1, 0), 'SCALE_Z');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#ee0979');
            this.setTooltip('Mesh geometry - enter filename from meshes folder');
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
