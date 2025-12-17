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
import { getNodeLogs, registerLogPanel, closeLogPanel } from '../../blocks.js';

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
                .appendField(new Blockly.FieldTextInput('my_node', validatePythonName), 'NAME')
                .appendField('  ')
                .appendField(new Blockly.FieldImage(
                    'data:image/svg+xml,' + encodeURIComponent('<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="white"><path d="M19 3H5c-1.1 0-2 .9-2 2v14c0 1.1.9 2 2 2h14c1.1 0 2-.9 2-2V5c0-1.1-.9-2-2-2zm-7 14H6v-2h6v2zm4-4H6v-2h10v2zm0-4H6V7h10v2z"/></svg>'),
                    16, 16, '📋 Logs', this.toggleLogPanel.bind(this)
                ));
            this.setNextStatement(true, null);
            this.setColour('#4c97ff');
            this.setTooltip(T('node'));
            this.setDeletable(true);
            this.setMovable(true);
            this.hat = 'cap';
            // Log panel state
            this.logPanelVisible = false;
            this.logPanel = null;
        },

        toggleLogPanel: function () {
            const nodeName = this.getFieldValue('NAME');
            console.log('[Node Block] Toggle log panel for:', nodeName);

            // Safety check
            if (!nodeName) {
                console.warn('[Node Block] No node name found');
                return;
            }

            // Check if panel exists globally (may have been opened from different file view)
            if (closeLogPanel(nodeName)) {
                // Panel was closed globally
                this.logPanel = null;
                this.logPanelVisible = false;
            } else {
                // No panel exists, create one
                this.createLogPanel(nodeName);
                this.logPanelVisible = true;
            }
        },

        createLogPanel: function (nodeName) {
            if (this.logPanel) {
                this.logPanel.remove();
            }

            // Get initial position from block
            const blockSvg = this.getSvgRoot();
            const bbox = blockSvg.getBoundingClientRect();

            // Create log panel
            this.logPanel = document.createElement('div');
            this.logPanel.className = 'node-log-panel';
            this.logPanel.innerHTML = `
                <div class="log-panel-header">
                    <span>📋 ${nodeName} Logs</span>
                    <button class="log-panel-close">×</button>
                </div>
                <div class="log-panel-content"></div>
            `;

            // Position panel initially below the block
            this.logPanel.style.position = 'fixed';
            this.logPanel.style.left = bbox.left + 'px';
            this.logPanel.style.top = (bbox.bottom + 10) + 'px';
            this.logPanel.style.zIndex = '10000';

            document.body.appendChild(this.logPanel);

            // Register panel globally so logs route to it
            registerLogPanel(nodeName, this.logPanel);

            // Close button handler - use nodeName captured in closure so it works from any file
            const capturedNodeName = nodeName;
            this.logPanel.querySelector('.log-panel-close').addEventListener('click', () => {
                closeLogPanel(capturedNodeName);
            });

            // Make header draggable
            const header = this.logPanel.querySelector('.log-panel-header');
            const panel = this.logPanel;
            let isDragging = false;
            let offsetX = 0;
            let offsetY = 0;

            header.style.cursor = 'move';
            header.addEventListener('mousedown', (e) => {
                if (e.target.classList.contains('log-panel-close')) return;
                isDragging = true;
                offsetX = e.clientX - panel.offsetLeft;
                offsetY = e.clientY - panel.offsetTop;
                e.preventDefault();
            });

            document.addEventListener('mousemove', (e) => {
                if (!isDragging) return;
                panel.style.left = (e.clientX - offsetX) + 'px';
                panel.style.top = (e.clientY - offsetY) + 'px';
            });

            document.addEventListener('mouseup', () => {
                isDragging = false;
            });

            // Populate with existing logs from global store
            const content = this.logPanel.querySelector('.log-panel-content');
            const logs = getNodeLogs(nodeName);
            if (logs.length === 0) {
                content.innerHTML = '<div class="log-empty">No logs yet. Run the node to see output.</div>';
            } else {
                logs.forEach(log => {
                    const logLine = document.createElement('div');
                    logLine.className = 'log-line ' + (log.type || 'info');
                    logLine.textContent = log.message;
                    content.appendChild(logLine);
                });
                content.scrollTop = content.scrollHeight;
            }
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
            this.appendValueInput('MSG_TYPE')
                .setCheck('MsgType')
                .appendField(L('msgType'));
            this.appendDummyInput()
                .appendField(L('topic'))
                .appendField(new Blockly.FieldTextInput('/topic', validateTopicName), 'TOPIC')
                .appendField(L('queueSize'))
                .appendField(new Blockly.FieldNumber(10, 1, 100), 'QOS');
            this.setInputsInline(true);
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#4c97ff');
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
            this.setColour('#4c97ff');
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
            this.setColour('#4c97ff');
            this.setTooltip(T('createMessage'));
        }
    };

    // Set message field
    Blockly.Blocks['node_publishers_set_field'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('set')
                .appendField(new Blockly.FieldTextInput('msg', validatePythonName), 'MSG')
                .appendField('.')
                .appendField(new Blockly.FieldTextInput('data'), 'FIELD')
                .appendField('=');
            this.appendValueInput('VALUE');
            this.setInputsInline(true);
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#4c97ff');
            this.setTooltip('Set a field on a message (e.g., msg.data = value)');
        }
    };

    // ========================================
    // SUBSCRIBERS CATEGORY
    // ========================================

    // Create subscriber with inline callback
    Blockly.Blocks['node_subscribers_create'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('createSubscriber'))
                .appendField(new Blockly.FieldTextInput('subscription', validatePythonName), 'NAME');
            this.appendValueInput('MSG_TYPE')
                .setCheck('MsgType')
                .appendField(L('msgType'));
            this.appendDummyInput()
                .appendField(L('topic'))
                .appendField(new Blockly.FieldTextInput('/topic', validateTopicName), 'TOPIC');
            this.appendStatementInput('CALLBACK_CONTENT')
                .appendField('on message (msg):');
            this.setInputsInline(true);
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#4c97ff');
            this.setTooltip('Create a subscriber with inline callback');
        }
    };

    // NOTE: Separate callback block removed - now inline in subscriber

    // Get message data
    Blockly.Blocks['node_subscribers_get_data'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('getMsgData'))
                .appendField(new Blockly.FieldTextInput('msg', validatePythonName), 'MSG')
                .appendField('.')
                .appendField(new Blockly.FieldTextInput('data'), 'FIELD');
            this.setOutput(true, 'Value');
            this.setColour('#4c97ff');
            this.setTooltip(T('getMsgData'));
        }
    };

    // ========================================
    // TIMERS CATEGORY
    // ========================================

    // Create timer with inline callback
    Blockly.Blocks['node_timers_create'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('createTimer'))
                .appendField(new Blockly.FieldTextInput('timer', validatePythonName), 'NAME')
                .appendField(L('period'))
                .appendField(new Blockly.FieldNumber(1.0, 0.01, 1000, 0.01), 'PERIOD')
                .appendField(L('seconds'));
            this.appendStatementInput('CALLBACK_CONTENT')
                .appendField('on tick:');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#f1c40f');
            this.setTooltip('Create a timer with inline callback');
        }
    };

    // NOTE: Separate callback block removed - now inline in timer

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

    // === Message Type Blocks (for publisher/subscriber type input) ===

    // String type
    Blockly.Blocks['node_messages_type_string'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('String');
            this.setOutput(true, 'MsgType');
            this.setColour('#ee0979');
            this.setTooltip('String message type');
        }
    };

    // Int32 type
    Blockly.Blocks['node_messages_type_int32'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Int32');
            this.setOutput(true, 'MsgType');
            this.setColour('#ee0979');
            this.setTooltip('Int32 message type');
        }
    };

    // Float64 type
    Blockly.Blocks['node_messages_type_float64'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Float64');
            this.setOutput(true, 'MsgType');
            this.setColour('#ee0979');
            this.setTooltip('Float64 message type');
        }
    };

    // Bool type
    Blockly.Blocks['node_messages_type_bool'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Bool');
            this.setOutput(true, 'MsgType');
            this.setColour('#ee0979');
            this.setTooltip('Bool message type');
        }
    };

    // Twist type
    Blockly.Blocks['node_messages_type_twist'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Twist');
            this.setOutput(true, 'MsgType');
            this.setColour('#ee0979');
            this.setTooltip('Twist message type (geometry_msgs)');
        }
    };

    // Pose type
    Blockly.Blocks['node_messages_type_pose'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Pose');
            this.setOutput(true, 'MsgType');
            this.setColour('#ee0979');
            this.setTooltip('Pose message type (geometry_msgs)');
        }
    };

    // LaserScan type
    Blockly.Blocks['node_messages_type_laserscan'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('LaserScan');
            this.setOutput(true, 'MsgType');
            this.setColour('#ee0979');
            this.setTooltip('LaserScan message type (sensor_msgs)');
        }
    };

    // === Message Data Blocks ===
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

    // Text value block (for string literals)
    Blockly.Blocks['node_logging_text'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('"')
                .appendField(new Blockly.FieldTextInput('Hello'), 'TEXT')
                .appendField('"');
            this.setOutput(true, 'String');
            this.setColour('#7f8c8d');
            this.setTooltip('Text value - use for string literals');
        }
    };

    // Msg variable block (for subscriber callback message)
    Blockly.Blocks['node_logging_msg'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('msg');
            this.setOutput(true, null);
            this.setColour('#7f8c8d');
            this.setTooltip('Message variable from subscriber callback');
        }
    };

    // Msg data block (get msg.data)
    Blockly.Blocks['node_logging_msg_data'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('msg.')
                .appendField(new Blockly.FieldTextInput('data'), 'FIELD');
            this.setOutput(true, null);
            this.setColour('#7f8c8d');
            this.setTooltip('Get a field from the message (e.g. msg.data)');
        }
    };

    // Log info
    Blockly.Blocks['node_logging_info'] = {
        init: function () {
            this.appendValueInput('MESSAGE')
                .appendField(L('logInfo'));
            this.setInputsInline(true);
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#7f8c8d');
            this.setTooltip('Log info message - accepts text or variables');
        }
    };

    // Log warning
    Blockly.Blocks['node_logging_warn'] = {
        init: function () {
            this.appendValueInput('MESSAGE')
                .appendField(L('logWarn'));
            this.setInputsInline(true);
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#7f8c8d');
            this.setTooltip('Log warning message - accepts text or variables');
        }
    };

    // Log error
    Blockly.Blocks['node_logging_error'] = {
        init: function () {
            this.appendValueInput('MESSAGE')
                .appendField(L('logError'));
            this.setInputsInline(true);
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#7f8c8d');
            this.setTooltip('Log error message - accepts text or variables');
        }
    };

    // Log debug
    Blockly.Blocks['node_logging_debug'] = {
        init: function () {
            this.appendValueInput('MESSAGE')
                .appendField(L('logDebug'));
            this.setInputsInline(true);
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#7f8c8d');
            this.setTooltip('Log debug message - accepts text or variables');
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
