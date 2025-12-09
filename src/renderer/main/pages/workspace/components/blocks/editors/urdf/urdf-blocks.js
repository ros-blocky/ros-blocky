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
                .appendField('🤖 Robot')
                .appendField(new Blockly.FieldTextInput('my_robot', validateName), 'NAME');
            this.appendStatementInput('CONTENT')
                .setCheck(null)
                .appendField('contains');
            this.setColour('#00bbfe');
            this.setTooltip('URDF Robot definition - the root element');
            this.setHelpUrl('http://wiki.ros.org/urdf/XML/robot');
        }
    };

    Blockly.Blocks['urdf_link'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('🔗 Link')
                .appendField(new Blockly.FieldTextInput('link_name', validateName), 'NAME');
            this.appendStatementInput('CONTENT')
                .setCheck(null)
                .appendField('contains');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#667eea');
            this.setTooltip('Define a URDF link element');
            this.setHelpUrl('http://wiki.ros.org/urdf/XML/link');
        }
    };

    Blockly.Blocks['urdf_joint_fixed'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('🔒 Fixed Joint')
                .appendField(new Blockly.FieldTextInput('joint_name', validateName), 'NAME');
            this.appendDummyInput()
                .appendField('parent')
                .appendField(new Blockly.FieldTextInput('parent_link', validateName), 'PARENT');
            this.appendDummyInput()
                .appendField('child')
                .appendField(new Blockly.FieldTextInput('child_link', validateName), 'CHILD');
            this.appendStatementInput('ORIGIN')
                .appendField('origin');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#11998e');
            this.setTooltip('Fixed joint - no movement allowed');
            this.setHelpUrl('http://wiki.ros.org/urdf/XML/joint');
        }
    };

    Blockly.Blocks['urdf_joint_revolute'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('🔄 Revolute Joint')
                .appendField(new Blockly.FieldTextInput('joint_name', validateName), 'NAME');
            this.appendDummyInput()
                .appendField('parent')
                .appendField(new Blockly.FieldTextInput('parent_link', validateName), 'PARENT');
            this.appendDummyInput()
                .appendField('child')
                .appendField(new Blockly.FieldTextInput('child_link', validateName), 'CHILD');
            this.appendStatementInput('CONTENT')
                .appendField('properties');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#38ef7d');
            this.setTooltip('Revolute joint - rotates around an axis with limits');
            this.setHelpUrl('http://wiki.ros.org/urdf/XML/joint');
        }
    };

    Blockly.Blocks['urdf_joint_continuous'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('♾️ Continuous Joint')
                .appendField(new Blockly.FieldTextInput('joint_name', validateName), 'NAME');
            this.appendDummyInput()
                .appendField('parent')
                .appendField(new Blockly.FieldTextInput('parent_link', validateName), 'PARENT');
            this.appendDummyInput()
                .appendField('child')
                .appendField(new Blockly.FieldTextInput('child_link', validateName), 'CHILD');
            this.appendStatementInput('CONTENT')
                .appendField('properties');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#10b981');
            this.setTooltip('Continuous joint - rotates continuously without limits');
            this.setHelpUrl('http://wiki.ros.org/urdf/XML/joint');
        }
    };

    // ========================================
    // VISUAL CATEGORY
    // ========================================

    Blockly.Blocks['urdf_visual'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('👁️ Visual');
            this.appendStatementInput('CONTENT')
                .setCheck(null)
                .appendField('contains');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#764ba2');
            this.setTooltip('Visual representation of a link');
            this.setHelpUrl('http://wiki.ros.org/urdf/XML/link');
        }
    };

    Blockly.Blocks['urdf_collision'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('💥 Collision');
            this.appendStatementInput('CONTENT')
                .setCheck(null)
                .appendField('contains');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#8b5cf6');
            this.setTooltip('Collision geometry of a link');
            this.setHelpUrl('http://wiki.ros.org/urdf/XML/link');
        }
    };

    Blockly.Blocks['urdf_material'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('🎨 Material')
                .appendField(new Blockly.FieldTextInput('material_name', validateName), 'NAME');
            this.appendStatementInput('COLOR')
                .appendField('color');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#f093fb');
            this.setTooltip('Material definition');
            this.setHelpUrl('http://wiki.ros.org/urdf/XML/link');
        }
    };

    Blockly.Blocks['urdf_color'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('🌈 Color RGBA')
                .appendField('R').appendField(new Blockly.FieldNumber(1, 0, 1, 0.1), 'R')
                .appendField('G').appendField(new Blockly.FieldNumber(0, 0, 1, 0.1), 'G')
                .appendField('B').appendField(new Blockly.FieldNumber(0, 0, 1, 0.1), 'B')
                .appendField('A').appendField(new Blockly.FieldNumber(1, 0, 1, 0.1), 'A');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#f5576c');
            this.setTooltip('RGBA color values (0-1)');
        }
    };

    // ========================================
    // PROPERTIES CATEGORY
    // ========================================

    Blockly.Blocks['urdf_inertial'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('⚖️ Inertial');
            this.appendStatementInput('CONTENT')
                .setCheck(null);
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#a78bfa');
            this.setTooltip('Inertial properties of a link');
            this.setHelpUrl('http://wiki.ros.org/urdf/XML/link');
        }
    };

    Blockly.Blocks['urdf_mass'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('⚖️ Mass')
                .appendField(new Blockly.FieldNumber(1, 0), 'VALUE')
                .appendField('kg');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#4facfe');
            this.setTooltip('Mass in kilograms');
        }
    };

    Blockly.Blocks['urdf_inertia'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('🌀 Inertia Matrix');
            this.appendDummyInput()
                .appendField('ixx').appendField(new Blockly.FieldNumber(1), 'IXX')
                .appendField('ixy').appendField(new Blockly.FieldNumber(0), 'IXY')
                .appendField('ixz').appendField(new Blockly.FieldNumber(0), 'IXZ');
            this.appendDummyInput()
                .appendField('iyy').appendField(new Blockly.FieldNumber(1), 'IYY')
                .appendField('iyz').appendField(new Blockly.FieldNumber(0), 'IYZ')
                .appendField('izz').appendField(new Blockly.FieldNumber(1), 'IZZ');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#00f2fe');
            this.setTooltip('Inertia tensor');
        }
    };

    Blockly.Blocks['urdf_origin'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('📍 Origin');
            this.appendDummyInput()
                .appendField('xyz')
                .appendField(new Blockly.FieldNumber(0), 'X')
                .appendField(new Blockly.FieldNumber(0), 'Y')
                .appendField(new Blockly.FieldNumber(0), 'Z');
            this.appendDummyInput()
                .appendField('rpy')
                .appendField(new Blockly.FieldNumber(0), 'ROLL')
                .appendField(new Blockly.FieldNumber(0), 'PITCH')
                .appendField(new Blockly.FieldNumber(0), 'YAW');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#22d3ee');
            this.setTooltip('Origin position (xyz) and orientation (roll, pitch, yaw)');
        }
    };

    Blockly.Blocks['urdf_axis'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('➡️ Axis')
                .appendField('x').appendField(new Blockly.FieldNumber(0, -1, 1), 'X')
                .appendField('y').appendField(new Blockly.FieldNumber(0, -1, 1), 'Y')
                .appendField('z').appendField(new Blockly.FieldNumber(1, -1, 1), 'Z');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#34d399');
            this.setTooltip('Joint rotation axis');
        }
    };

    Blockly.Blocks['urdf_limit'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('🚧 Limit');
            this.appendDummyInput()
                .appendField('lower').appendField(new Blockly.FieldNumber(-3.14), 'LOWER')
                .appendField('upper').appendField(new Blockly.FieldNumber(3.14), 'UPPER');
            this.appendDummyInput()
                .appendField('effort').appendField(new Blockly.FieldNumber(100), 'EFFORT')
                .appendField('velocity').appendField(new Blockly.FieldNumber(1), 'VELOCITY');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#6ee7b7');
            this.setTooltip('Joint limits');
        }
    };

    // ========================================
    // SHAPES CATEGORY
    // ========================================

    Blockly.Blocks['urdf_box'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('📦 Box')
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
                .appendField('🛢️ Cylinder')
                .appendField('radius').appendField(new Blockly.FieldNumber(0.5, 0), 'RADIUS')
                .appendField('length').appendField(new Blockly.FieldNumber(1, 0), 'LENGTH');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#ff6a00');
            this.setTooltip('Cylinder geometry');
        }
    };

    Blockly.Blocks['urdf_sphere'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('🔴 Sphere')
                .appendField('radius').appendField(new Blockly.FieldNumber(0.5, 0), 'RADIUS');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#f97316');
            this.setTooltip('Sphere geometry');
        }
    };

    Blockly.Blocks['urdf_mesh'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('🦾 Mesh')
                .appendField(new Blockly.FieldTextInput('package://...', validateMeshPath), 'FILENAME');
            this.appendDummyInput()
                .appendField('scale')
                .appendField(new Blockly.FieldNumber(1, 0), 'SCALE_X')
                .appendField(new Blockly.FieldNumber(1, 0), 'SCALE_Y')
                .appendField(new Blockly.FieldNumber(1, 0), 'SCALE_Z');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour('#fb923c');
            this.setTooltip('Mesh geometry from file');
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
        'urdf_robot', 'urdf_link', 'urdf_joint_fixed', 'urdf_joint_revolute', 'urdf_joint_continuous',
        // Visual
        'urdf_visual', 'urdf_collision', 'urdf_material', 'urdf_color',
        // Properties
        'urdf_inertial', 'urdf_mass', 'urdf_inertia', 'urdf_origin', 'urdf_axis', 'urdf_limit',
        // Shapes
        'urdf_box', 'urdf_cylinder', 'urdf_sphere', 'urdf_mesh'
    ];
}
