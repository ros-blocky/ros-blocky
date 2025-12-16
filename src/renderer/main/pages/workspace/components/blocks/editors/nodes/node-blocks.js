/**
 * Node Block Definitions for Blockly
 * ROS 2 Python Node block definitions with security sanitization
 * 
 * Categories:
 * - Structure: Node, Init, Main, Spin, Shutdown
 * - Variables: Create, Set, Get, Lists, Dicts
 * - Publishers: Create, Publish, Message
 * - Subscribers: Create, Callback, GetData
 * - Timers: Create, Callback, Cancel
 * - Messages: String, Int, Float, Bool, Twist, Pose, etc.
 * - Parameters: Declare, Get, Set
 * - Logging: Info, Warn, Error, Debug
 * - Control Flow: If, Loops, Try/Except
 * - Math & Logic: Arithmetic, Compare, Logic
 * - TurtleSim: Teleport, Pen, Clear, Spawn, Kill
 */

import { t } from '../../../../../../../i18n/index.js';

// Translation helpers
const L = (key) => t(`blocks.node.labels.${key}`) || key;
const T = (key) => t(`blocks.node.tooltips.${key}`) || key;

// ========================================
// SHARED VALIDATORS
// ========================================

/**
 * Validate Python identifier (variable/function names)
 * @param {string} newValue - Value to validate
 * @returns {string} Validated name
 */
function validatePythonName(newValue) {
    if (!newValue || typeof newValue !== 'string') {
        return 'my_var';
    }
    // Python identifiers: start with letter/underscore, then alphanumeric/underscore
    let sanitized = newValue.replace(/[^a-zA-Z0-9_]/g, '_');
    // Cannot start with number
    if (/^[0-9]/.test(sanitized)) {
        sanitized = '_' + sanitized;
    }
    return sanitized || 'my_var';
}

/**
 * Validate topic name
 * @param {string} newValue - Value to validate
 * @returns {string} Validated topic
 */
function validateTopicName(newValue) {
    if (!newValue || typeof newValue !== 'string') {
        return '/topic';
    }
    // ROS topic names: alphanumeric, underscores, slashes
    let sanitized = newValue.replace(/[^a-zA-Z0-9_/]/g, '_');
    // Ensure starts with /
    if (!sanitized.startsWith('/')) {
        sanitized = '/' + sanitized;
    }
    return sanitized;
}

/**
 * Validate positive number
 * @param {number} value - Value to validate
 * @returns {number} Validated number
 */
function validatePositiveNumber(value) {
    const num = parseFloat(value);
    return isNaN(num) || num < 0 ? 0 : num;
}

// ========================================
// BLOCK REGISTRATION
// ========================================

/**
 * Register all Node blocks with Blockly
 * @param {Object} Blockly - The Blockly library
 */
export function registerNodeBlocks(Blockly) {
    console.log('[Node Blocks] Registering block definitions...');

    // ========================================
    // STRUCTURE CATEGORY
    // ========================================

    // Node block - main container (hat-shaped with rounded top)
    // First, register the hat extension if not already done
    if (!Blockly.Extensions.isRegistered('hat_block')) {
        Blockly.Extensions.register('hat_block', function () {
            this.hat = 'cap';
        });
    }

    Blockly.Blocks['node_structure_node'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Node')
                .appendField('name')
                .appendField(new Blockly.FieldTextInput('my_node', validatePythonName), 'NAME');
            // Blocks stack UNDER this block - they go into __init__
            // Accept any block type (null = no type restriction)
            this.setNextStatement(true, null);
            this.setColour('#4c97ff');
            this.setTooltip(T('node'));
            this.setDeletable(true);
            this.setMovable(true);
            // Hat block properties
            this.hat = 'cap';
        }
    };


    // NOTE: init, main, spin, spin_once, shutdown blocks removed
    // The Node block now generates all this boilerplate automatically

    // ========================================
    // VARIABLES CATEGORY
    // ========================================

    // Create variable
    Blockly.Blocks['node_variables_create'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('createVar'))
                .appendField(new Blockly.FieldTextInput('my_var', validatePythonName), 'NAME')
                .appendField('=')
                .appendField(new Blockly.FieldTextInput('0'), 'VALUE');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#16a085');
            this.setTooltip(T('createVar'));
        }
    };

    // Set variable
    Blockly.Blocks['node_variables_set'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('setVar'))
                .appendField(new Blockly.FieldTextInput('my_var', validatePythonName), 'NAME')
                .appendField('=')
                .appendField(new Blockly.FieldTextInput('0'), 'VALUE');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#16a085');
            this.setTooltip(T('setVar'));
        }
    };

    // Get variable
    Blockly.Blocks['node_variables_get'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('getVar'))
                .appendField(new Blockly.FieldTextInput('my_var', validatePythonName), 'NAME');
            this.setOutput(true, 'Variable');
            this.setColour('#16a085');
            this.setTooltip(T('getVar'));
        }
    };

    // Create list
    Blockly.Blocks['node_variables_create_list'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('createList'))
                .appendField(new Blockly.FieldTextInput('my_list', validatePythonName), 'NAME')
                .appendField('= []');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#16a085');
            this.setTooltip(T('createList'));
        }
    };

    // Append to list
    Blockly.Blocks['node_variables_append'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('append'))
                .appendField(new Blockly.FieldTextInput('my_list', validatePythonName), 'LIST')
                .appendField('.append(')
                .appendField(new Blockly.FieldTextInput('value'), 'VALUE')
                .appendField(')');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#16a085');
            this.setTooltip(T('append'));
        }
    };

    // Create dictionary
    Blockly.Blocks['node_variables_create_dict'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('createDict'))
                .appendField(new Blockly.FieldTextInput('my_dict', validatePythonName), 'NAME')
                .appendField('= {}');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#16a085');
            this.setTooltip(T('createDict'));
        }
    };

    // Set dict key
    Blockly.Blocks['node_variables_dict_set'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('dictSet'))
                .appendField(new Blockly.FieldTextInput('my_dict', validatePythonName), 'DICT')
                .appendField('[')
                .appendField(new Blockly.FieldTextInput('key'), 'KEY')
                .appendField('] =')
                .appendField(new Blockly.FieldTextInput('value'), 'VALUE');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#16a085');
            this.setTooltip(T('dictSet'));
        }
    };

    // Get dict key
    Blockly.Blocks['node_variables_dict_get'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('dictGet'))
                .appendField(new Blockly.FieldTextInput('my_dict', validatePythonName), 'DICT')
                .appendField('.get(')
                .appendField(new Blockly.FieldTextInput('key'), 'KEY')
                .appendField(')');
            this.setOutput(true, 'Value');
            this.setColour('#16a085');
            this.setTooltip(T('dictGet'));
        }
    };

    // ========================================
    // PUBLISHERS CATEGORY
    // ========================================

    // Create publisher
    Blockly.Blocks['node_publishers_create'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('createPublisher'))
                .appendField(new Blockly.FieldTextInput('publisher', validatePythonName), 'NAME');
            this.appendDummyInput()
                .appendField(L('msgType'))
                .appendField(new Blockly.FieldDropdown([
                    ['String', 'String'],
                    ['Int32', 'Int32'],
                    ['Float64', 'Float64'],
                    ['Bool', 'Bool'],
                    ['Twist', 'Twist'],
                    ['Pose', 'Pose']
                ]), 'MSG_TYPE');
            this.appendDummyInput()
                .appendField(L('topic'))
                .appendField(new Blockly.FieldTextInput('/topic', validateTopicName), 'TOPIC');
            this.appendDummyInput()
                .appendField(L('queueSize'))
                .appendField(new Blockly.FieldNumber(10, 1, 100), 'QOS');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#FFAB19');
            this.setTooltip(T('createPublisher'));
        }
    };

    // Publish message
    Blockly.Blocks['node_publishers_publish'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('publish'))
                .appendField(new Blockly.FieldTextInput('publisher', validatePythonName), 'PUBLISHER')
                .appendField(L('message'))
                .appendField(new Blockly.FieldTextInput('msg', validatePythonName), 'MSG');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#FFAB19');
            this.setTooltip(T('publish'));
        }
    };

    // Create message
    Blockly.Blocks['node_publishers_create_message'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('createMessage'))
                .appendField(new Blockly.FieldTextInput('msg', validatePythonName), 'NAME')
                .appendField('=')
                .appendField(new Blockly.FieldDropdown([
                    ['String()', 'String()'],
                    ['Int32()', 'Int32()'],
                    ['Float64()', 'Float64()'],
                    ['Bool()', 'Bool()'],
                    ['Twist()', 'Twist()'],
                    ['Pose()', 'Pose()']
                ]), 'MSG_TYPE');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#FFAB19');
            this.setTooltip(T('createMessage'));
        }
    };

    // ========================================
    // SUBSCRIBERS CATEGORY
    // ========================================

    // Create subscriber
    Blockly.Blocks['node_subscribers_create'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('createSubscriber'))
                .appendField(new Blockly.FieldTextInput('subscription', validatePythonName), 'NAME');
            this.appendDummyInput()
                .appendField(L('msgType'))
                .appendField(new Blockly.FieldDropdown([
                    ['String', 'String'],
                    ['Int32', 'Int32'],
                    ['Float64', 'Float64'],
                    ['Bool', 'Bool'],
                    ['Twist', 'Twist'],
                    ['Pose', 'Pose'],
                    ['LaserScan', 'LaserScan']
                ]), 'MSG_TYPE');
            this.appendDummyInput()
                .appendField(L('topic'))
                .appendField(new Blockly.FieldTextInput('/topic', validateTopicName), 'TOPIC');
            this.appendDummyInput()
                .appendField(L('callback'))
                .appendField(new Blockly.FieldTextInput('listener_callback', validatePythonName), 'CALLBACK');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#e67e22');
            this.setTooltip(T('createSubscriber'));
        }
    };

    // Subscriber callback
    Blockly.Blocks['node_subscribers_callback'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('callback'))
                .appendField(new Blockly.FieldTextInput('listener_callback', validatePythonName), 'NAME')
                .appendField('(self, msg):');
            this.appendStatementInput('CONTENT')
                .setCheck(['LOG', 'VARIABLE', 'PUBLISH']);
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#e67e22');
            this.setTooltip(T('callbackDef'));
        }
    };

    // Get message data
    Blockly.Blocks['node_subscribers_get_data'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('getMsgData'))
                .appendField(new Blockly.FieldTextInput('msg', validatePythonName), 'MSG')
                .appendField('.')
                .appendField(new Blockly.FieldTextInput('data'), 'FIELD');
            this.setOutput(true, 'Value');
            this.setColour('#e67e22');
            this.setTooltip(T('getMsgData'));
        }
    };

    // ========================================
    // TIMERS CATEGORY
    // ========================================

    // Create timer
    Blockly.Blocks['node_timers_create'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('createTimer'))
                .appendField(new Blockly.FieldTextInput('timer', validatePythonName), 'NAME');
            this.appendDummyInput()
                .appendField(L('period'))
                .appendField(new Blockly.FieldNumber(1.0, 0.01, 1000, 0.01), 'PERIOD')
                .appendField(L('seconds'));
            this.appendDummyInput()
                .appendField(L('callback'))
                .appendField(new Blockly.FieldTextInput('timer_callback', validatePythonName), 'CALLBACK');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#f1c40f');
            this.setTooltip(T('createTimer'));
        }
    };

    // Timer callback
    Blockly.Blocks['node_timers_callback'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('callback'))
                .appendField(new Blockly.FieldTextInput('timer_callback', validatePythonName), 'NAME')
                .appendField('(self):');
            this.appendStatementInput('CONTENT')
                .setCheck(['LOG', 'VARIABLE', 'PUBLISH']);
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#f1c40f');
            this.setTooltip(T('timerCallbackDef'));
        }
    };

    // Cancel timer
    Blockly.Blocks['node_timers_cancel'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('cancelTimer'))
                .appendField(new Blockly.FieldTextInput('timer', validatePythonName), 'NAME');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#f1c40f');
            this.setTooltip(T('cancelTimer'));
        }
    };

    // ========================================
    // MESSAGES CATEGORY
    // ========================================

    // String message
    Blockly.Blocks['node_messages_string'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('stringMsg'))
                .appendField(L('data'))
                .appendField(new Blockly.FieldTextInput('Hello'), 'DATA');
            this.setOutput(true, 'Message');
            this.setColour('#ee0979');
            this.setTooltip(T('stringMsg'));
        }
    };

    // Int32 message
    Blockly.Blocks['node_messages_int32'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('int32Msg'))
                .appendField(L('data'))
                .appendField(new Blockly.FieldNumber(0), 'DATA');
            this.setOutput(true, 'Message');
            this.setColour('#ee0979');
            this.setTooltip(T('int32Msg'));
        }
    };

    // Float64 message
    Blockly.Blocks['node_messages_float64'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('float64Msg'))
                .appendField(L('data'))
                .appendField(new Blockly.FieldNumber(0.0, -Infinity, Infinity, 0.01), 'DATA');
            this.setOutput(true, 'Message');
            this.setColour('#ee0979');
            this.setTooltip(T('float64Msg'));
        }
    };

    // Bool message
    Blockly.Blocks['node_messages_bool'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('boolMsg'))
                .appendField(L('data'))
                .appendField(new Blockly.FieldDropdown([
                    ['True', 'True'],
                    ['False', 'False']
                ]), 'DATA');
            this.setOutput(true, 'Message');
            this.setColour('#ee0979');
            this.setTooltip(T('boolMsg'));
        }
    };

    // Twist message
    Blockly.Blocks['node_messages_twist'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('twistMsg'));
            this.appendDummyInput()
                .appendField(L('linear'))
                .appendField('x').appendField(new Blockly.FieldNumber(0, -10, 10, 0.1), 'LIN_X')
                .appendField('y').appendField(new Blockly.FieldNumber(0, -10, 10, 0.1), 'LIN_Y')
                .appendField('z').appendField(new Blockly.FieldNumber(0, -10, 10, 0.1), 'LIN_Z');
            this.appendDummyInput()
                .appendField(L('angular'))
                .appendField('x').appendField(new Blockly.FieldNumber(0, -10, 10, 0.1), 'ANG_X')
                .appendField('y').appendField(new Blockly.FieldNumber(0, -10, 10, 0.1), 'ANG_Y')
                .appendField('z').appendField(new Blockly.FieldNumber(0, -10, 10, 0.1), 'ANG_Z');
            this.setOutput(true, 'Message');
            this.setColour('#ee0979');
            this.setTooltip(T('twistMsg'));
        }
    };

    // Pose message
    Blockly.Blocks['node_messages_pose'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('poseMsg'));
            this.appendDummyInput()
                .appendField(L('position'))
                .appendField('x').appendField(new Blockly.FieldNumber(0), 'POS_X')
                .appendField('y').appendField(new Blockly.FieldNumber(0), 'POS_Y')
                .appendField('z').appendField(new Blockly.FieldNumber(0), 'POS_Z');
            this.appendDummyInput()
                .appendField(L('orientation'))
                .appendField('x').appendField(new Blockly.FieldNumber(0), 'ORI_X')
                .appendField('y').appendField(new Blockly.FieldNumber(0), 'ORI_Y')
                .appendField('z').appendField(new Blockly.FieldNumber(0), 'ORI_Z')
                .appendField('w').appendField(new Blockly.FieldNumber(1), 'ORI_W');
            this.setOutput(true, 'Message');
            this.setColour('#ee0979');
            this.setTooltip(T('poseMsg'));
        }
    };

    // Point message
    Blockly.Blocks['node_messages_point'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('pointMsg'))
                .appendField('x').appendField(new Blockly.FieldNumber(0), 'X')
                .appendField('y').appendField(new Blockly.FieldNumber(0), 'Y')
                .appendField('z').appendField(new Blockly.FieldNumber(0), 'Z');
            this.setOutput(true, 'Message');
            this.setColour('#ee0979');
            this.setTooltip(T('pointMsg'));
        }
    };

    // LaserScan (read-only info)
    Blockly.Blocks['node_messages_laserscan'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('laserScanMsg'))
                .appendField(L('field'))
                .appendField(new Blockly.FieldDropdown([
                    ['ranges', 'ranges'],
                    ['intensities', 'intensities'],
                    ['angle_min', 'angle_min'],
                    ['angle_max', 'angle_max'],
                    ['range_min', 'range_min'],
                    ['range_max', 'range_max']
                ]), 'FIELD');
            this.setOutput(true, 'Value');
            this.setColour('#ee0979');
            this.setTooltip(T('laserScanMsg'));
        }
    };

    // Set field
    Blockly.Blocks['node_messages_set_field'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('setField'))
                .appendField(new Blockly.FieldTextInput('msg', validatePythonName), 'MSG')
                .appendField('.')
                .appendField(new Blockly.FieldTextInput('data'), 'FIELD')
                .appendField('=')
                .appendField(new Blockly.FieldTextInput('value'), 'VALUE');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#ee0979');
            this.setTooltip(T('setField'));
        }
    };

    // Get field
    Blockly.Blocks['node_messages_get_field'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('getField'))
                .appendField(new Blockly.FieldTextInput('msg', validatePythonName), 'MSG')
                .appendField('.')
                .appendField(new Blockly.FieldTextInput('data'), 'FIELD');
            this.setOutput(true, 'Value');
            this.setColour('#ee0979');
            this.setTooltip(T('getField'));
        }
    };

    // ========================================
    // PARAMETERS CATEGORY
    // ========================================

    // Declare parameter
    Blockly.Blocks['node_parameters_declare'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('declareParam'))
                .appendField(new Blockly.FieldTextInput('my_param'), 'NAME')
                .appendField(L('default'))
                .appendField(new Blockly.FieldTextInput('0'), 'DEFAULT');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#00bcd4');
            this.setTooltip(T('declareParam'));
        }
    };

    // Get parameter
    Blockly.Blocks['node_parameters_get'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('getParam'))
                .appendField(new Blockly.FieldTextInput('my_param'), 'NAME');
            this.setOutput(true, 'Value');
            this.setColour('#00bcd4');
            this.setTooltip(T('getParam'));
        }
    };

    // Set parameter
    Blockly.Blocks['node_parameters_set'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('setParam'))
                .appendField(new Blockly.FieldTextInput('my_param'), 'NAME')
                .appendField('=')
                .appendField(new Blockly.FieldTextInput('value'), 'VALUE');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#00bcd4');
            this.setTooltip(T('setParam'));
        }
    };

    // ========================================
    // LOGGING CATEGORY
    // ========================================

    // Log info
    Blockly.Blocks['node_logging_info'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('logInfo'))
                .appendField(new Blockly.FieldTextInput('Message'), 'MESSAGE');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#7f8c8d');
            this.setTooltip(T('logInfo'));
        }
    };

    // Log warning
    Blockly.Blocks['node_logging_warn'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('logWarn'))
                .appendField(new Blockly.FieldTextInput('Warning'), 'MESSAGE');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#7f8c8d');
            this.setTooltip(T('logWarn'));
        }
    };

    // Log error
    Blockly.Blocks['node_logging_error'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('logError'))
                .appendField(new Blockly.FieldTextInput('Error'), 'MESSAGE');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#7f8c8d');
            this.setTooltip(T('logError'));
        }
    };

    // Log debug
    Blockly.Blocks['node_logging_debug'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('logDebug'))
                .appendField(new Blockly.FieldTextInput('Debug'), 'MESSAGE');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#7f8c8d');
            this.setTooltip(T('logDebug'));
        }
    };

    // ========================================
    // CONTROL FLOW CATEGORY
    // ========================================

    // If block
    Blockly.Blocks['node_controlflow_if'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('if'))
                .appendField(new Blockly.FieldTextInput('condition'), 'CONDITION')
                .appendField(':');
            this.appendStatementInput('CONTENT');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#9966FF');
            this.setTooltip(T('if'));
        }
    };

    // If-else block
    Blockly.Blocks['node_controlflow_ifelse'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('if'))
                .appendField(new Blockly.FieldTextInput('condition'), 'CONDITION')
                .appendField(':');
            this.appendStatementInput('IF_CONTENT');
            this.appendDummyInput()
                .appendField(L('else'));
            this.appendStatementInput('ELSE_CONTENT');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#9966FF');
            this.setTooltip(T('ifelse'));
        }
    };

    // For loop
    Blockly.Blocks['node_controlflow_for'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('for'))
                .appendField(new Blockly.FieldTextInput('i', validatePythonName), 'VAR')
                .appendField(L('inRange'))
                .appendField(new Blockly.FieldNumber(0), 'START')
                .appendField(L('to'))
                .appendField(new Blockly.FieldNumber(10), 'END')
                .appendField(':');
            this.appendStatementInput('CONTENT');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#9966FF');
            this.setTooltip(T('for'));
        }
    };

    // While loop
    Blockly.Blocks['node_controlflow_while'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('while'))
                .appendField(new Blockly.FieldTextInput('condition'), 'CONDITION')
                .appendField(':');
            this.appendStatementInput('CONTENT');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#9966FF');
            this.setTooltip(T('while'));
        }
    };

    // Break
    Blockly.Blocks['node_controlflow_break'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('break'));
            this.setPreviousStatement(true);
            this.setColour('#9966FF');
            this.setTooltip(T('break'));
        }
    };

    // Continue
    Blockly.Blocks['node_controlflow_continue'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('continue'));
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#9966FF');
            this.setTooltip(T('continue'));
        }
    };

    // Try-except
    Blockly.Blocks['node_controlflow_try'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('try'));
            this.appendStatementInput('TRY_CONTENT');
            this.appendDummyInput()
                .appendField(L('except'))
                .appendField(new Blockly.FieldTextInput('Exception'), 'EXCEPTION')
                .appendField(L('as'))
                .appendField(new Blockly.FieldTextInput('e', validatePythonName), 'VAR');
            this.appendStatementInput('EXCEPT_CONTENT');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#9966FF');
            this.setTooltip(T('try'));
        }
    };

    // ========================================
    // MATH & LOGIC CATEGORY
    // ========================================

    // Arithmetic
    Blockly.Blocks['node_math_arithmetic'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(new Blockly.FieldTextInput('a'), 'LEFT')
                .appendField(new Blockly.FieldDropdown([
                    ['+', '+'],
                    ['-', '-'],
                    ['*', '*'],
                    ['/', '/'],
                    ['%', '%'],
                    ['**', '**']
                ]), 'OP')
                .appendField(new Blockly.FieldTextInput('b'), 'RIGHT');
            this.setOutput(true, 'Number');
            this.setColour('#e74c3c');
            this.setTooltip(T('arithmetic'));
        }
    };

    // Compare
    Blockly.Blocks['node_math_compare'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(new Blockly.FieldTextInput('a'), 'LEFT')
                .appendField(new Blockly.FieldDropdown([
                    ['==', '=='],
                    ['!=', '!='],
                    ['<', '<'],
                    ['>', '>'],
                    ['<=', '<='],
                    ['>=', '>=']
                ]), 'OP')
                .appendField(new Blockly.FieldTextInput('b'), 'RIGHT');
            this.setOutput(true, 'Boolean');
            this.setColour('#e74c3c');
            this.setTooltip(T('compare'));
        }
    };

    // Logic
    Blockly.Blocks['node_math_logic'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(new Blockly.FieldTextInput('a'), 'LEFT')
                .appendField(new Blockly.FieldDropdown([
                    ['and', 'and'],
                    ['or', 'or']
                ]), 'OP')
                .appendField(new Blockly.FieldTextInput('b'), 'RIGHT');
            this.setOutput(true, 'Boolean');
            this.setColour('#e74c3c');
            this.setTooltip(T('logic'));
        }
    };

    // Math function
    Blockly.Blocks['node_math_function'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(new Blockly.FieldDropdown([
                    ['abs', 'abs'],
                    ['min', 'min'],
                    ['max', 'max'],
                    ['round', 'round'],
                    ['int', 'int'],
                    ['float', 'float'],
                    ['math.sqrt', 'math.sqrt'],
                    ['math.sin', 'math.sin'],
                    ['math.cos', 'math.cos'],
                    ['math.tan', 'math.tan'],
                    ['math.pi', 'math.pi']
                ]), 'FUNC')
                .appendField('(')
                .appendField(new Blockly.FieldTextInput('x'), 'ARG')
                .appendField(')');
            this.setOutput(true, 'Number');
            this.setColour('#e74c3c');
            this.setTooltip(T('mathFunc'));
        }
    };

    // Random
    Blockly.Blocks['node_math_random'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('random'))
                .appendField(new Blockly.FieldDropdown([
                    ['random()', 'random.random()'],
                    ['randint', 'random.randint']
                ]), 'TYPE');
            this.appendDummyInput()
                .appendField(L('min'))
                .appendField(new Blockly.FieldNumber(0), 'MIN')
                .appendField(L('max'))
                .appendField(new Blockly.FieldNumber(100), 'MAX');
            this.setOutput(true, 'Number');
            this.setColour('#e74c3c');
            this.setTooltip(T('random'));
        }
    };

    // Clamp
    Blockly.Blocks['node_math_clamp'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('clamp'))
                .appendField(new Blockly.FieldTextInput('value'), 'VALUE')
                .appendField(L('min'))
                .appendField(new Blockly.FieldNumber(0), 'MIN')
                .appendField(L('max'))
                .appendField(new Blockly.FieldNumber(100), 'MAX');
            this.setOutput(true, 'Number');
            this.setColour('#e74c3c');
            this.setTooltip(T('clamp'));
        }
    };

    // ========================================
    // TURTLESIM CATEGORY
    // ========================================

    // Teleport
    Blockly.Blocks['node_turtlesim_teleport'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('teleport'))
                .appendField(L('turtle'))
                .appendField(new Blockly.FieldTextInput('turtle1'), 'TURTLE');
            this.appendDummyInput()
                .appendField('x').appendField(new Blockly.FieldNumber(5.5, 0, 11, 0.1), 'X')
                .appendField('y').appendField(new Blockly.FieldNumber(5.5, 0, 11, 0.1), 'Y')
                .appendField('θ').appendField(new Blockly.FieldNumber(0, 0, 6.28, 0.1), 'THETA');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#1abc9c');
            this.setTooltip(T('teleport'));
        }
    };

    // Set pen
    Blockly.Blocks['node_turtlesim_set_pen'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('setPen'))
                .appendField(L('turtle'))
                .appendField(new Blockly.FieldTextInput('turtle1'), 'TURTLE');
            this.appendDummyInput()
                .appendField('R').appendField(new Blockly.FieldNumber(255, 0, 255), 'R')
                .appendField('G').appendField(new Blockly.FieldNumber(255, 0, 255), 'G')
                .appendField('B').appendField(new Blockly.FieldNumber(255, 0, 255), 'B')
                .appendField(L('width')).appendField(new Blockly.FieldNumber(3, 1, 20), 'WIDTH');
            this.appendDummyInput()
                .appendField(L('penOff'))
                .appendField(new Blockly.FieldDropdown([
                    ['False', '0'],
                    ['True', '1']
                ]), 'OFF');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#1abc9c');
            this.setTooltip(T('setPen'));
        }
    };

    // Clear
    Blockly.Blocks['node_turtlesim_clear'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('clear'));
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#1abc9c');
            this.setTooltip(T('clear'));
        }
    };

    // Spawn
    Blockly.Blocks['node_turtlesim_spawn'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('spawn'))
                .appendField(L('name'))
                .appendField(new Blockly.FieldTextInput('turtle2'), 'NAME');
            this.appendDummyInput()
                .appendField('x').appendField(new Blockly.FieldNumber(5.5, 0, 11, 0.1), 'X')
                .appendField('y').appendField(new Blockly.FieldNumber(5.5, 0, 11, 0.1), 'Y')
                .appendField('θ').appendField(new Blockly.FieldNumber(0, 0, 6.28, 0.1), 'THETA');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#1abc9c');
            this.setTooltip(T('spawn'));
        }
    };

    // Kill
    Blockly.Blocks['node_turtlesim_kill'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('kill'))
                .appendField(L('turtle'))
                .appendField(new Blockly.FieldTextInput('turtle1'), 'NAME');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#1abc9c');
            this.setTooltip(T('kill'));
        }
    };

    console.log('[Node Blocks] Block definitions registered');
}

/**
 * Get all block type names
 * @returns {string[]} Array of all Node block type names
 */
export function getAllNodeBlockTypes() {
    return [
        // Structure
        'node_structure_node', 'node_structure_init', 'node_structure_main',
        'node_structure_spin', 'node_structure_spin_once', 'node_structure_shutdown',
        // Variables
        'node_variables_create', 'node_variables_set', 'node_variables_get',
        'node_variables_create_list', 'node_variables_append',
        'node_variables_create_dict', 'node_variables_dict_set', 'node_variables_dict_get',
        // Publishers
        'node_publishers_create', 'node_publishers_publish', 'node_publishers_create_message',
        // Subscribers
        'node_subscribers_create', 'node_subscribers_callback', 'node_subscribers_get_data',
        // Timers
        'node_timers_create', 'node_timers_callback', 'node_timers_cancel',
        // Messages
        'node_messages_string', 'node_messages_int32', 'node_messages_float64',
        'node_messages_bool', 'node_messages_twist', 'node_messages_pose',
        'node_messages_point', 'node_messages_laserscan',
        'node_messages_set_field', 'node_messages_get_field',
        // Parameters
        'node_parameters_declare', 'node_parameters_get', 'node_parameters_set',
        // Logging
        'node_logging_info', 'node_logging_warn', 'node_logging_error', 'node_logging_debug',
        // Control Flow
        'node_controlflow_if', 'node_controlflow_ifelse', 'node_controlflow_for',
        'node_controlflow_while', 'node_controlflow_break', 'node_controlflow_continue',
        'node_controlflow_try',
        // Math & Logic
        'node_math_arithmetic', 'node_math_compare', 'node_math_logic',
        'node_math_function', 'node_math_random', 'node_math_clamp',
        // TurtleSim
        'node_turtlesim_teleport', 'node_turtlesim_set_pen', 'node_turtlesim_clear',
        'node_turtlesim_spawn', 'node_turtlesim_kill'
    ];
}
