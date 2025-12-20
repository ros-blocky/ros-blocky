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
    // CUSTOM CONNECTION CHECKER
    // Prevents MsgType and Message Instance blocks from connecting to wrong inputs
    // ========================================

    // Type blocks - ONLY these can connect to Publisher/Subscriber type slot
    const MsgTypeBlocks = [
        'node_messages_type_string', 'node_messages_type_int32', 'node_messages_type_float64',
        'node_messages_type_bool', 'node_messages_type_twist', 'node_messages_type_pose',
        'node_messages_type_laserscan'
    ];

    // Message instance blocks - these can NOT connect anywhere
    const MsgInstanceBlocks = [
        'node_messages_string', 'node_messages_int32', 'node_messages_float64',
        'node_messages_bool', 'node_messages_twist', 'node_messages_pose',
        'node_messages_point'
    ];

    // Only register once
    if (!Blockly.ConnectionChecker || !Blockly.registry.hasItem('connectionChecker', 'msgtype_checker')) {
        class MsgTypeConnectionChecker extends Blockly.ConnectionChecker {
            doTypeChecks(a, b) {
                // Get the blocks involved
                const blockA = a.getSourceBlock();
                const blockB = b.getSourceBlock();

                // Check if either block is an INSTANCE block
                const aIsInstance = blockA && MsgInstanceBlocks.includes(blockA.type);
                const bIsInstance = blockB && MsgInstanceBlocks.includes(blockB.type);
                if (aIsInstance || bIsInstance) {
                    // Instance blocks can ONLY connect to set variable VALUE input
                    const targetInput = a.getParentInput() || b.getParentInput();
                    const otherBlock = aIsInstance ? blockB : blockA;

                    // Check if connecting to node_variables_set VALUE input
                    if (otherBlock && otherBlock.type === 'node_variables_set') {
                        const inputName = targetInput?.name;
                        if (inputName === 'VALUE') {
                            return true; // Allow instance blocks to connect to set variable value slot
                        }
                    }
                    return false; // Block instance blocks from connecting elsewhere
                }

                // Check if either block is a TYPE block
                const aIsMsgType = blockA && MsgTypeBlocks.includes(blockA.type);
                const bIsMsgType = blockB && MsgTypeBlocks.includes(blockB.type);

                if (aIsMsgType || bIsMsgType) {
                    // TYPE blocks can ONLY connect to MSG_TYPE inputs on publisher/subscriber
                    const targetInput = a.getParentInput() || b.getParentInput();
                    const otherBlock = aIsMsgType ? blockB : blockA;

                    // Check if connecting to publisher/subscriber MSG_TYPE input
                    const validParents = ['node_publishers_create', 'node_subscribers_create'];
                    if (otherBlock && validParents.includes(otherBlock.type)) {
                        const inputName = targetInput?.name;
                        if (inputName === 'MSG_TYPE') {
                            return true; // Allow TYPE blocks to connect to type slot
                        }
                    }
                    return false; // Block TYPE blocks from connecting elsewhere
                }

                // For other blocks, use default checking
                return super.doTypeChecks(a, b);
            }
        }

        Blockly.registry.register(
            Blockly.registry.Type.CONNECTION_CHECKER,
            'msgtype_checker',
            MsgTypeConnectionChecker
        );
    }

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
            // Get node name from active file (will be updated when workspace loads)
            const initialName = this.getNodeNameFromFile() || 'my_node';

            this.appendDummyInput()
                .appendField(L('node'))
                .appendField(L('nodeName'))
                .appendField(new Blockly.FieldLabel(initialName), 'NAME')
                .appendField('  ')
                .appendField(new Blockly.FieldImage(
                    'data:image/svg+xml,' + encodeURIComponent('<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="white"><path d="M19 3H5c-1.1 0-2 .9-2 2v14c0 1.1.9 2 2 2h14c1.1 0 2-.9 2-2V5c0-1.1-.9-2-2-2zm-7 14H6v-2h6v2zm4-4H6v-2h10v2zm0-4H6V7h10v2z"/></svg>'),
                    16, 16, L('logs'), this.toggleLogPanel.bind(this)
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

        /**
         * Get node name from the active file name
         * @returns {string} Node name derived from file name
         */
        getNodeNameFromFile: function () {
            // Try to get from window.blocksActiveFile (set by blocks.js)
            if (window.blocksActiveFile && window.blocksActiveFile.fileName) {
                // Remove .py extension and return
                return window.blocksActiveFile.fileName.replace(/\.py$/, '');
            }
            return null;
        },

        /**
         * Update the node name field based on current file
         */
        updateNodeName: function () {
            const nodeName = this.getNodeNameFromFile();
            if (nodeName) {
                const nameField = this.getField('NAME');
                if (nameField) {
                    nameField.setValue(nodeName);
                }
            }
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

    // Set variable (with value input for both target and value)
    Blockly.Blocks['node_variables_set'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('setVar'));
            this.appendValueInput('TARGET');
            this.appendDummyInput()
                .appendField('=');
            this.appendValueInput('VALUE');
            this.setInputsInline(true);
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#16a085');
            this.setTooltip(T('setVar'));
        }
    };

    // Get variable (value block with value input)
    Blockly.Blocks['node_variables_get'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('getVar'));
            this.appendValueInput('TARGET');
            this.setInputsInline(true);
            this.setOutput(true, 'Value');
            this.setColour('#16a085');
            this.setTooltip(T('getVar'));
        }
    };

    // Number value block
    Blockly.Blocks['node_variables_number'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(new Blockly.FieldNumber(0), 'VALUE');
            this.setOutput(true, 'Number');
            this.setColour('#16a085');
            this.setTooltip('A number value');
        }
    };

    // String value block
    Blockly.Blocks['node_variables_string'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('"')
                .appendField(new Blockly.FieldTextInput('hello'), 'VALUE')
                .appendField('"');
            this.setOutput(true, 'String');
            this.setColour('#16a085');
            this.setTooltip('A text string value');
        }
    };

    // Boolean value block
    Blockly.Blocks['node_variables_boolean'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(new Blockly.FieldDropdown([
                    ['True', 'True'],
                    ['False', 'False']
                ]), 'VALUE');
            this.setOutput(true, 'Value');  // Use 'Value' for round shape
            this.setColour('#16a085');
            this.setTooltip('A boolean value (True/False)');
        }
    };

    // Property accessor block (for chaining like msg.linear.x)
    Blockly.Blocks['node_variables_property'] = {
        init: function () {
            this.appendValueInput('OBJECT');
            this.appendDummyInput()
                .appendField('.');
            this.appendValueInput('PROPERTY');
            this.setInputsInline(true);
            this.setOutput(true, 'Value');
            this.setColour('#16a085');
            this.setTooltip('Access a property on an object (e.g., msg.linear.x)');
        }
    };

    // Name/identifier block (simple variable name)
    Blockly.Blocks['node_variables_name'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(new Blockly.FieldTextInput('my_var', validatePythonName), 'VALUE');
            this.setOutput(true, 'Name');
            this.setColour('#1abc9c');
            this.setTooltip('A variable name');
        }
    };

    // Scope wrapper block (adds local/self prefix)
    Blockly.Blocks['node_variables_scope'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(new Blockly.FieldDropdown([
                    ['local', 'local'],
                    ['self', 'self']
                ]), 'SCOPE');
            this.appendValueInput('NAME');
            this.setInputsInline(true);
            this.setOutput(true, 'Value');
            this.setColour('#1abc9c');
            this.setTooltip('Wrap a variable with scope (local or self)');
        }
    };

    // Declare variable block (with scope selector)
    Blockly.Blocks['node_variables_declare'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('declare')
                .appendField(new Blockly.FieldDropdown([
                    ['local', 'local'],
                    ['self', 'self']
                ]), 'SCOPE')
                .appendField(new Blockly.FieldTextInput('my_var', validatePythonName), 'NAME')
                .appendField('=');
            this.appendValueInput('VALUE');
            this.setInputsInline(true);
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#16a085');
            this.setTooltip('Declare a variable (local = function scope, self = instance variable)');
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
                .appendField(new Blockly.FieldTextInput('publisher', validatePythonName), 'PUBLISHER');
            this.appendValueInput('MSG')
                .setCheck(null)
                .appendField(L('message'));
            this.setInputsInline(true);
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
            this.setColour('#ee0979');
            this.setTooltip(T('createMessage'));
        }
    };

    // Set message field
    Blockly.Blocks['node_publishers_set_field'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('setField'))
                .appendField(new Blockly.FieldTextInput('msg', validatePythonName), 'MSG')
                .appendField('.')
                .appendField(new Blockly.FieldTextInput('data'), 'FIELD')
                .appendField('=');
            this.appendValueInput('VALUE');
            this.setInputsInline(true);
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#ee0979');
            this.setTooltip(T('setField'));
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
                .appendField(L('onMessage'));
            this.setInputsInline(true);
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#4c97ff');
            this.setTooltip(T('createSubscriber'));
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
            this.setColour('#ee0979');
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
                .appendField(L('onTick'));
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#f1c40f');
            this.setTooltip(T('createTimer'));
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

    // === Message Instance Blocks (for creating message instances) ===
    // String() instance - with optional data input
    Blockly.Blocks['node_messages_string'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('String(');
            this.appendValueInput('DATA');
            this.appendDummyInput()
                .appendField(')');
            this.setInputsInline(true);
            this.setOutput(true, 'Value');
            this.setColour('#ee0979');
            this.setTooltip('Create a String message instance');
        }
    };

    // Int32() instance
    Blockly.Blocks['node_messages_int32'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Int32(');
            this.appendValueInput('DATA');
            this.appendDummyInput()
                .appendField(')');
            this.setInputsInline(true);
            this.setOutput(true, 'Value');
            this.setColour('#ee0979');
            this.setTooltip('Create an Int32 message instance');
        }
    };

    // Float64() instance
    Blockly.Blocks['node_messages_float64'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Float64(');
            this.appendValueInput('DATA');
            this.appendDummyInput()
                .appendField(')');
            this.setInputsInline(true);
            this.setOutput(true, 'Value');
            this.setColour('#ee0979');
            this.setTooltip('Create a Float64 message instance');
        }
    };

    // Bool() instance
    Blockly.Blocks['node_messages_bool'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('Bool(');
            this.appendValueInput('DATA');
            this.appendDummyInput()
                .appendField(')');
            this.setInputsInline(true);
            this.setOutput(true, 'Value');
            this.setColour('#ee0979');
            this.setTooltip('Create a Bool message instance');
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

    // Smart Get Field - with message type dropdown that updates field options
    Blockly.Blocks['node_messages_get_field'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('get')
                .appendField(new Blockly.FieldDropdown([
                    ['String', 'String'],
                    ['Int32', 'Int32'],
                    ['Float64', 'Float64'],
                    ['Bool', 'Bool'],
                    ['Twist', 'Twist'],
                    ['Pose', 'Pose'],
                    ['Point', 'Point']
                ], this.updateFieldDropdown.bind(this)), 'MSG_TYPE')
                .appendField(new Blockly.FieldDropdown([
                    ['data', 'data']
                ]), 'FIELD')
                .appendField('from')
                .appendField(new Blockly.FieldTextInput('msg', validatePythonName), 'MSG');
            this.setOutput(true, 'Value');
            this.setColour('#ee0979');
            this.setTooltip('Get a field value from a message');
        },

        updateFieldDropdown: function (newValue) {
            const fieldOptions = this.getFieldOptionsForType(newValue);
            const fieldDropdown = this.getField('FIELD');
            if (fieldDropdown) {
                fieldDropdown.menuGenerator_ = fieldOptions;
                fieldDropdown.setValue(fieldOptions[0][1]);
            }
            return newValue;
        },

        getFieldOptionsForType: function (msgType) {
            const fieldMap = {
                'String': [['data', 'data']],
                'Int32': [['data', 'data']],
                'Float64': [['data', 'data']],
                'Bool': [['data', 'data']],
                'Point': [['x', 'x'], ['y', 'y'], ['z', 'z']],
                'Twist': [
                    ['linear.x', 'linear.x'], ['linear.y', 'linear.y'], ['linear.z', 'linear.z'],
                    ['angular.x', 'angular.x'], ['angular.y', 'angular.y'], ['angular.z', 'angular.z']
                ],
                'Pose': [
                    ['position.x', 'position.x'], ['position.y', 'position.y'], ['position.z', 'position.z'],
                    ['orientation.x', 'orientation.x'], ['orientation.y', 'orientation.y'],
                    ['orientation.z', 'orientation.z'], ['orientation.w', 'orientation.w']
                ]
            };
            return fieldMap[msgType] || [['data', 'data']];
        }
    };

    // Smart Set Field - with message type dropdown that updates field options
    Blockly.Blocks['node_messages_set_field'] = {
        init: function () {
            this.appendDummyInput()
                .appendField('set')
                .appendField(new Blockly.FieldDropdown([
                    ['String', 'String'],
                    ['Int32', 'Int32'],
                    ['Float64', 'Float64'],
                    ['Bool', 'Bool'],
                    ['Twist', 'Twist'],
                    ['Pose', 'Pose'],
                    ['Point', 'Point']
                ], this.updateFieldDropdown.bind(this)), 'MSG_TYPE')
                .appendField(new Blockly.FieldDropdown([
                    ['data', 'data']
                ]), 'FIELD')
                .appendField('in')
                .appendField(new Blockly.FieldTextInput('msg', validatePythonName), 'MSG')
                .appendField('to');
            this.appendValueInput('VALUE');
            this.setInputsInline(true);
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#ee0979');
            this.setTooltip('Set a field value on a message');
        },

        updateFieldDropdown: function (newValue) {
            const fieldOptions = this.getFieldOptionsForType(newValue);
            const fieldDropdown = this.getField('FIELD');
            if (fieldDropdown) {
                fieldDropdown.menuGenerator_ = fieldOptions;
                fieldDropdown.setValue(fieldOptions[0][1]);
            }
            return newValue;
        },

        getFieldOptionsForType: function (msgType) {
            const fieldMap = {
                'String': [['data', 'data']],
                'Int32': [['data', 'data']],
                'Float64': [['data', 'data']],
                'Bool': [['data', 'data']],
                'Point': [['x', 'x'], ['y', 'y'], ['z', 'z']],
                'Twist': [
                    ['linear.x', 'linear.x'], ['linear.y', 'linear.y'], ['linear.z', 'linear.z'],
                    ['angular.x', 'angular.x'], ['angular.y', 'angular.y'], ['angular.z', 'angular.z']
                ],
                'Pose': [
                    ['position.x', 'position.x'], ['position.y', 'position.y'], ['position.z', 'position.z'],
                    ['orientation.x', 'orientation.x'], ['orientation.y', 'orientation.y'],
                    ['orientation.z', 'orientation.z'], ['orientation.w', 'orientation.w']
                ]
            };
            return fieldMap[msgType] || [['data', 'data']];
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
            this.setTooltip(T('text'));
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
            this.setTooltip(T('logInfo'));
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
            this.setTooltip(T('logWarn'));
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
            this.setTooltip(T('logError'));
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
            this.appendValueInput('A');
            this.appendDummyInput()
                .appendField(new Blockly.FieldDropdown([
                    ['+', '+'],
                    ['-', '-'],
                    ['*', '*'],
                    ['/', '/'],
                    ['%', '%'],
                    ['**', '**']
                ]), 'OP');
            this.appendValueInput('B');
            this.setInputsInline(true);
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
                .appendField('(');
            this.appendValueInput('ARG');
            this.appendDummyInput()
                .appendField(')');
            this.setInputsInline(true);
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
    // ========================================
    // PROCEDURES CATEGORY
    // ========================================

    // Define Procedure
    Blockly.Blocks['node_procedures_define'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('defineProcedure'))
                .appendField(new Blockly.FieldTextInput('my_procedure', validatePythonName), 'NAME');
            this.appendStatementInput('BODY');
            this.setColour('#9b59b6');
            this.setTooltip(T('defineProcedure'));
        }
    };

    // Call Procedure
    Blockly.Blocks['node_procedures_call'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('callProcedure'))
                .appendField(new Blockly.FieldTextInput('my_procedure', validatePythonName), 'NAME');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#9b59b6');
            this.setTooltip(T('callProcedure'));
        }
    };

    // Return value (removed - not needed for simple procedures)

    // ========================================
    // CONTROL FLOW CATEGORY
    // ========================================

    // If block
    Blockly.Blocks['node_controlflow_if'] = {
        init: function () {
            this.appendValueInput('CONDITION')
                .setCheck('Boolean')
                .appendField(L('if'));
            this.appendStatementInput('DO');
            this.setInputsInline(true);
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#E9A23B');
            this.setTooltip(T('if'));
        }
    };

    // If-Else block
    Blockly.Blocks['node_controlflow_ifelse'] = {
        init: function () {
            this.appendValueInput('CONDITION')
                .setCheck('Boolean')
                .appendField(L('if'));
            this.appendStatementInput('DO');
            this.appendStatementInput('ELSE')
                .appendField(L('else'));
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#E9A23B');
            this.setTooltip(T('ifelse'));
        }
    };

    // Repeat N times block
    Blockly.Blocks['node_controlflow_for'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('repeat'))
                .appendField(new Blockly.FieldNumber(10, 1), 'TIMES')
                .appendField(L('times'));
            this.appendStatementInput('DO');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#E9A23B');
            this.setTooltip(T('for'));
        }
    };

    // Repeat While block
    Blockly.Blocks['node_controlflow_while'] = {
        init: function () {
            this.appendValueInput('CONDITION')
                .setCheck('Boolean')
                .appendField(L('repeatWhile'));
            this.appendStatementInput('DO');
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour('#E9A23B');
            this.setTooltip(T('while'));
        }
    };

    // Break block
    Blockly.Blocks['node_controlflow_break'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('break'));
            this.setPreviousStatement(true);
            this.setColour('#E9A23B');
            this.setTooltip(T('break'));
        }
    };

    // Continue block
    Blockly.Blocks['node_controlflow_continue'] = {
        init: function () {
            this.appendDummyInput()
                .appendField(L('continue'));
            this.setPreviousStatement(true);
            this.setColour('#E9A23B');
            this.setTooltip(T('continue'));
        }
    };

    // Compare block (for conditions)
    Blockly.Blocks['node_controlflow_compare'] = {
        init: function () {
            this.appendValueInput('A');
            this.appendDummyInput()
                .appendField(new Blockly.FieldDropdown([
                    ['=', '=='],
                    ['≠', '!='],
                    ['<', '<'],
                    ['>', '>'],
                    ['≤', '<='],
                    ['≥', '>=']
                ]), 'OP');
            this.appendValueInput('B');
            this.setInputsInline(true);
            this.setOutput(true, 'Boolean');
            this.setColour('#e74c3c');
            this.setTooltip(T('compare'));
        }
    };

    // Logic AND/OR block
    Blockly.Blocks['node_controlflow_logic'] = {
        init: function () {
            this.appendValueInput('A');
            this.appendDummyInput()
                .appendField(new Blockly.FieldDropdown([
                    ['and', 'and'],
                    ['or', 'or']
                ]), 'OP');
            this.appendValueInput('B');
            this.setInputsInline(true);
            this.setOutput(true, 'Boolean');
            this.setColour('#e74c3c');
            this.setTooltip(T('logic'));
        }
    };

    // Not block
    Blockly.Blocks['node_controlflow_not'] = {
        init: function () {
            this.appendValueInput('VALUE')
                .appendField(L('not'));
            this.setInputsInline(true);
            this.setOutput(true, 'Boolean');
            this.setColour('#e74c3c');
            this.setTooltip(T('not'));
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
        'node_math_function', 'node_math_random', 'node_math_clamp'
    ];
}
