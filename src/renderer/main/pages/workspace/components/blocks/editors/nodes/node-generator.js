/**
 * Node Code Generator for Blockly
 * Converts Blockly workspace blocks into valid ROS 2 Python node code
 */

// Track required imports
let requiredImports = new Set();

// Track last skipped blocks
let lastSkippedBlocks = [];

/**
 * Initialize the Node code generator
 * @param {Object} Blockly - The Blockly library
 */
export function initNodeGenerator(Blockly) {
    console.log('[Node Generator] Initializing...');

    // Create generator instance
    const pythonGenerator = new Blockly.Generator('Python');

    // Python uses 4-space indentation
    pythonGenerator.INDENT = '    ';

    // Define order constants (Blockly's basic Generator doesn't have these)
    pythonGenerator.ORDER_ATOMIC = 0;           // 0 "" ...
    pythonGenerator.ORDER_COLLECTION = 1;       // tuples, lists, dictionaries
    pythonGenerator.ORDER_STRING_CONVERSION = 1; // `expression...`
    pythonGenerator.ORDER_MEMBER = 2.1;         // . []
    pythonGenerator.ORDER_FUNCTION_CALL = 2.2;  // ()
    pythonGenerator.ORDER_EXPONENTIATION = 3;   // **
    pythonGenerator.ORDER_UNARY_SIGN = 4;       // + -
    pythonGenerator.ORDER_BITWISE_NOT = 4;      // ~
    pythonGenerator.ORDER_MULTIPLICATIVE = 5;   // * / // %
    pythonGenerator.ORDER_ADDITIVE = 6;         // + -
    pythonGenerator.ORDER_BITWISE_SHIFT = 7;    // << >>
    pythonGenerator.ORDER_BITWISE_AND = 8;      // &
    pythonGenerator.ORDER_BITWISE_XOR = 9;      // ^
    pythonGenerator.ORDER_BITWISE_OR = 10;      // |
    pythonGenerator.ORDER_RELATIONAL = 11;      // in, not in, is, is not, <, <=, >, >=, <>, !=, ==
    pythonGenerator.ORDER_LOGICAL_NOT = 12;     // not
    pythonGenerator.ORDER_LOGICAL_AND = 13;     // and
    pythonGenerator.ORDER_LOGICAL_OR = 14;      // or
    pythonGenerator.ORDER_CONDITIONAL = 15;     // if else
    pythonGenerator.ORDER_LAMBDA = 16;          // lambda
    pythonGenerator.ORDER_NONE = 99;            // (...)

    // ========================================
    // GENERATOR UTILITIES
    // ========================================

    /**
     * Add required import
     * @param {string} importLine - Import statement
     */
    function addImport(importLine) {
        requiredImports.add(importLine);
    }

    /**
     * Generate code for statements (nested blocks)
     * @param {Blockly.Block} block - The parent block
     * @param {string} name - Name of the statement input
     * @returns {string} Generated code for all nested blocks
     */
    pythonGenerator.statementToCode = function (block, name) {
        const input = block.getInput(name);
        if (!input || !input.connection || !input.connection.targetBlock()) {
            return this.INDENT + 'pass\n';
        }

        let code = '';
        let targetBlock = input.connection.targetBlock();

        while (targetBlock) {
            const blockCode = this.blockToCode(targetBlock);
            if (blockCode && typeof blockCode === 'string') {
                code += this.prefixLines(blockCode, this.INDENT);
            } else if (Array.isArray(blockCode)) {
                code += this.prefixLines(blockCode[0], this.INDENT);
            }
            targetBlock = targetBlock.getNextBlock();
        }

        return code || this.INDENT + 'pass\n';
    };

    /**
     * Add prefix to each line of code
     * @param {string} text - Text to prefix
     * @param {string} prefix - Prefix to add
     * @returns {string} Prefixed text
     */
    pythonGenerator.prefixLines = function (text, prefix) {
        return text.split('\n').map(line => line ? prefix + line : '').join('\n');
    };

    // ========================================
    // STRUCTURE BLOCKS
    // ========================================

    pythonGenerator.forBlock['node_structure_node'] = function (block) {
        const name = block.getFieldValue('NAME') || 'my_node';
        const className = toPascalCase(name) + 'Node';

        addImport('import rclpy');
        addImport('from rclpy.node import Node');

        // Get blocks stacked UNDER this hat block (in the init)
        let initContent = '';
        let callbackMethods = '';
        let nextBlock = block.getNextBlock();

        // Clear any previously stored callbacks
        block.workspace.subscriberCallbacks = [];
        block.workspace.timerCallbacks = [];

        console.log('[Node Generator] Processing node block:', name, 'nextBlock:', nextBlock ? nextBlock.type : 'none');

        while (nextBlock) {
            console.log('[Node Generator] Processing child block:', nextBlock.type);
            const blockCode = pythonGenerator.blockToCode(nextBlock);
            console.log('[Node Generator] Generated code for', nextBlock.type, ':', blockCode);
            if (blockCode) {
                const codeStr = typeof blockCode === 'string' ? blockCode : blockCode[0];
                // Other blocks go into __init__
                initContent += pythonGenerator.INDENT + pythonGenerator.INDENT + codeStr;
            }
            nextBlock = nextBlock.getNextBlock();
        }

        console.log('[Node Generator] Final initContent:', initContent);

        // Collect stored callbacks from subscriber blocks
        if (block.workspace.subscriberCallbacks && block.workspace.subscriberCallbacks.length > 0) {
            for (const cb of block.workspace.subscriberCallbacks) {
                callbackMethods += `\n${pythonGenerator.INDENT}def ${cb.name}(self, msg):\n`;
                // Add extra indentation to callback content since it's inside a class method
                const indentedContent = cb.content
                    ? cb.content.split('\n').map(line => line ? pythonGenerator.INDENT + line : '').join('\n')
                    : `${pythonGenerator.INDENT}${pythonGenerator.INDENT}pass\n`;
                callbackMethods += indentedContent;
            }
        }

        // Collect stored callbacks from timer blocks
        if (block.workspace.timerCallbacks && block.workspace.timerCallbacks.length > 0) {
            for (const cb of block.workspace.timerCallbacks) {
                callbackMethods += `\n${pythonGenerator.INDENT}def ${cb.name}(self):\n`;
                // Add extra indentation to callback content since it's inside a class method
                const indentedContent = cb.content
                    ? cb.content.split('\n').map(line => line ? pythonGenerator.INDENT + line : '').join('\n')
                    : `${pythonGenerator.INDENT}${pythonGenerator.INDENT}pass\n`;
                callbackMethods += indentedContent;
            }
        }

        // Build the class
        let code = `\nclass ${className}(Node):\n`;
        code += `${pythonGenerator.INDENT}def __init__(self):\n`;
        code += `${pythonGenerator.INDENT}${pythonGenerator.INDENT}super().__init__('${name}')\n`;
        code += initContent || `${pythonGenerator.INDENT}${pythonGenerator.INDENT}pass\n`;

        // Add callback methods
        if (callbackMethods) {
            code += callbackMethods;
        }

        // Build the main function
        code += `\n\ndef main(args=None):\n`;
        code += `${pythonGenerator.INDENT}rclpy.init(args=args)\n`;
        code += `${pythonGenerator.INDENT}node = ${className}()\n`;
        code += `${pythonGenerator.INDENT}try:\n`;
        code += `${pythonGenerator.INDENT}${pythonGenerator.INDENT}rclpy.spin(node)\n`;
        code += `${pythonGenerator.INDENT}except KeyboardInterrupt:\n`;
        code += `${pythonGenerator.INDENT}${pythonGenerator.INDENT}pass\n`;
        code += `${pythonGenerator.INDENT}finally:\n`;
        code += `${pythonGenerator.INDENT}${pythonGenerator.INDENT}node.destroy_node()\n`;
        code += `${pythonGenerator.INDENT}${pythonGenerator.INDENT}rclpy.shutdown()\n`;

        return code;
    };

    pythonGenerator.forBlock['node_structure_init'] = function (block) {
        const content = pythonGenerator.statementToCode(block, 'CONTENT');

        let code = 'def __init__(self):\n';
        code += pythonGenerator.INDENT + "super().__init__('my_node')\n";
        code += content;

        return code;
    };

    pythonGenerator.forBlock['node_structure_main'] = function (block) {
        const content = pythonGenerator.statementToCode(block, 'CONTENT');

        let code = '\ndef main(args=None):\n';
        code += pythonGenerator.INDENT + 'rclpy.init(args=args)\n';
        code += content;

        return code;
    };

    pythonGenerator.forBlock['node_structure_spin'] = function (block) {
        const nodeVar = block.getFieldValue('NODE_VAR') || 'node';
        return `rclpy.spin(${nodeVar})\n`;
    };

    pythonGenerator.forBlock['node_structure_spin_once'] = function (block) {
        const nodeVar = block.getFieldValue('NODE_VAR') || 'node';
        return `rclpy.spin_once(${nodeVar})\n`;
    };

    pythonGenerator.forBlock['node_structure_shutdown'] = function (block) {
        return 'rclpy.shutdown()\n';
    };

    // ========================================
    // VARIABLES BLOCKS
    // ========================================

    pythonGenerator.forBlock['node_variables_set'] = function (block) {
        const target = pythonGenerator.valueToCode(block, 'TARGET', pythonGenerator.ORDER_NONE) || 'my_var';
        const value = pythonGenerator.valueToCode(block, 'VALUE', pythonGenerator.ORDER_NONE) || '0';
        return `${target} = ${value}\n`;
    };

    pythonGenerator.forBlock['node_variables_get'] = function (block) {
        const target = pythonGenerator.valueToCode(block, 'TARGET', pythonGenerator.ORDER_NONE) || 'my_var';
        return [target, pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_variables_number'] = function (block) {
        const value = block.getFieldValue('VALUE') || 0;
        return [String(value), pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_variables_string'] = function (block) {
        const value = block.getFieldValue('VALUE') || '';
        // Escape quotes in the string
        const escaped = value.replace(/'/g, "\\'");
        return [`'${escaped}'`, pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_variables_boolean'] = function (block) {
        const value = block.getFieldValue('VALUE') || 'True';
        return [value, pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_variables_property'] = function (block) {
        const object = pythonGenerator.valueToCode(block, 'OBJECT', pythonGenerator.ORDER_NONE) || 'obj';
        const property = pythonGenerator.valueToCode(block, 'PROPERTY', pythonGenerator.ORDER_NONE) || 'prop';
        return [`${object}.${property}`, pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_variables_name'] = function (block) {
        const value = block.getFieldValue('VALUE') || 'my_var';
        return [value, pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_variables_scope'] = function (block) {
        const scope = block.getFieldValue('SCOPE') || 'local';
        const name = pythonGenerator.valueToCode(block, 'NAME', pythonGenerator.ORDER_NONE) || 'my_var';
        // Add self. prefix for instance variables
        return [scope === 'self' ? `self.${name}` : name, pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_variables_declare'] = function (block) {
        const scope = block.getFieldValue('SCOPE') || 'local';
        const name = block.getFieldValue('NAME') || 'my_var';
        const value = pythonGenerator.valueToCode(block, 'VALUE', pythonGenerator.ORDER_NONE) || '0';
        // Add self. prefix for instance variables
        const varName = scope === 'self' ? `self.${name}` : name;
        return `${varName} = ${value}\n`;
    };

    // Keep legacy generators for backward compatibility
    pythonGenerator.forBlock['node_variables_create'] = function (block) {
        const name = block.getFieldValue('NAME') || 'my_var';
        const value = block.getFieldValue('VALUE') || '0';
        return `self.${name} = ${value}\n`;
    };

    pythonGenerator.forBlock['node_variables_create_list'] = function (block) {
        const name = block.getFieldValue('NAME') || 'my_list';
        return `self.${name} = []\n`;
    };

    pythonGenerator.forBlock['node_variables_append'] = function (block) {
        const list = block.getFieldValue('LIST') || 'my_list';
        const value = block.getFieldValue('VALUE') || 'value';
        return `self.${list}.append(${value})\n`;
    };

    pythonGenerator.forBlock['node_variables_create_dict'] = function (block) {
        const name = block.getFieldValue('NAME') || 'my_dict';
        return `self.${name} = {}\n`;
    };

    pythonGenerator.forBlock['node_variables_dict_set'] = function (block) {
        const dict = block.getFieldValue('DICT') || 'my_dict';
        const key = block.getFieldValue('KEY') || 'key';
        const value = block.getFieldValue('VALUE') || 'value';
        return `self.${dict}['${key}'] = ${value}\n`;
    };

    pythonGenerator.forBlock['node_variables_dict_get'] = function (block) {
        const dict = block.getFieldValue('DICT') || 'my_dict';
        const key = block.getFieldValue('KEY') || 'key';
        return [`self.${dict}.get('${key}')`, pythonGenerator.ORDER_ATOMIC];
    };

    // ========================================
    // PUBLISHERS BLOCKS
    // ========================================

    pythonGenerator.forBlock['node_publishers_create'] = function (block) {
        const name = block.getFieldValue('NAME') || 'publisher';
        // Check if type block is connected, otherwise use default
        const typeInput = block.getInput('MSG_TYPE');
        let msgType = 'String';
        if (typeInput && typeInput.connection && typeInput.connection.targetBlock()) {
            const typeBlock = typeInput.connection.targetBlock();
            console.log('[Node Generator] Type block connected:', typeBlock.type);
            // Get the type name from the block type itself (e.g. node_messages_type_string -> String)
            const typeMatch = typeBlock.type.match(/node_messages_type_(\w+)/);
            if (typeMatch) {
                msgType = typeMatch[1].charAt(0).toUpperCase() + typeMatch[1].slice(1);
                // Handle special cases
                if (msgType === 'Laserscan') msgType = 'LaserScan';
                if (msgType === 'Int32') msgType = 'Int32';
                if (msgType === 'Float64') msgType = 'Float64';
            }
            console.log('[Node Generator] Extracted msgType:', msgType);
        }
        const topic = block.getFieldValue('TOPIC') || '/topic';
        const qos = block.getFieldValue('QOS') || 10;

        // Add appropriate import
        const msgImport = getMsgImport(msgType);
        addImport(msgImport);

        return `self.${name} = self.create_publisher(${msgType}, '${topic}', ${qos})\n`;
    };

    pythonGenerator.forBlock['node_publishers_publish'] = function (block) {
        const publisher = block.getFieldValue('PUBLISHER') || 'publisher';
        const msg = pythonGenerator.valueToCode(block, 'MSG', pythonGenerator.ORDER_NONE) || 'msg';
        return `self.${publisher}.publish(${msg})\n`;
    };

    pythonGenerator.forBlock['node_publishers_create_message'] = function (block) {
        const name = block.getFieldValue('NAME') || 'msg';
        const msgType = block.getFieldValue('MSG_TYPE') || 'String()';
        return `${name} = ${msgType}\n`;
    };

    pythonGenerator.forBlock['node_publishers_set_field'] = function (block) {
        const msg = block.getFieldValue('MSG') || 'msg';
        const field = block.getFieldValue('FIELD') || 'data';
        const value = pythonGenerator.valueToCode(block, 'VALUE', pythonGenerator.ORDER_NONE) || "''";
        return `${msg}.${field} = ${value}\n`;
    };

    // ========================================
    // SUBSCRIBERS BLOCKS
    // ========================================

    pythonGenerator.forBlock['node_subscribers_create'] = function (block) {
        const name = block.getFieldValue('NAME') || 'subscription';
        // Check if type block is connected, otherwise use default
        const typeInput = block.getInput('MSG_TYPE');
        let msgType = 'String';
        if (typeInput && typeInput.connection && typeInput.connection.targetBlock()) {
            const typeBlock = typeInput.connection.targetBlock();
            // Get the type name from the block type itself
            const typeMatch = typeBlock.type.match(/node_messages_type_(\w+)/);
            if (typeMatch) {
                msgType = typeMatch[1].charAt(0).toUpperCase() + typeMatch[1].slice(1);
                if (msgType === 'Laserscan') msgType = 'LaserScan';
                if (msgType === 'Int32') msgType = 'Int32';
                if (msgType === 'Float64') msgType = 'Float64';
            }
        }
        const topic = block.getFieldValue('TOPIC') || '/topic';
        const callbackName = name + '_callback';

        // Get inline callback content
        const callbackContent = pythonGenerator.statementToCode(block, 'CALLBACK_CONTENT');

        const msgImport = getMsgImport(msgType);
        addImport(msgImport);

        // Store callback method to be added to the class
        if (!block.workspace.subscriberCallbacks) {
            block.workspace.subscriberCallbacks = [];
        }
        block.workspace.subscriberCallbacks.push({
            name: callbackName,
            content: callbackContent || pythonGenerator.INDENT + 'pass\n'
        });

        return `self.${name} = self.create_subscription(${msgType}, '${topic}', self.${callbackName}, 10)\n`;
    };

    // Legacy callback block - kept for backwards compatibility but not shown in palette

    pythonGenerator.forBlock['node_subscribers_get_data'] = function (block) {
        const msg = block.getFieldValue('MSG') || 'msg';
        const field = block.getFieldValue('FIELD') || 'data';
        return [`${msg}.${field}`, pythonGenerator.ORDER_ATOMIC];
    };

    // ========================================
    // TIMERS BLOCKS
    // ========================================

    pythonGenerator.forBlock['node_timers_create'] = function (block) {
        const name = block.getFieldValue('NAME') || 'timer';
        const period = block.getFieldValue('PERIOD') || 1.0;
        const callbackName = name + '_callback';

        // Get inline callback content
        const callbackContent = pythonGenerator.statementToCode(block, 'CALLBACK_CONTENT');

        // Store callback method to be added to the class
        if (!block.workspace.timerCallbacks) {
            block.workspace.timerCallbacks = [];
        }
        block.workspace.timerCallbacks.push({
            name: callbackName,
            content: callbackContent || pythonGenerator.INDENT + 'pass\n'
        });

        return `self.${name} = self.create_timer(${period}, self.${callbackName})\n`;
    };

    // Legacy callback block - kept for backwards compatibility but not shown in palette

    pythonGenerator.forBlock['node_timers_cancel'] = function (block) {
        const name = block.getFieldValue('NAME') || 'timer';
        return `self.${name}.cancel()\n`;
    };

    // ========================================
    // MESSAGES BLOCKS
    // ========================================

    // === Message Type Generators ===
    // Note: Order 0 = ATOMIC (highest precedence, no parentheses needed)
    pythonGenerator.forBlock['node_messages_type_string'] = function (block) {
        addImport('from std_msgs.msg import String');
        return ['String', pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_messages_type_int32'] = function (block) {
        addImport('from std_msgs.msg import Int32');
        return ['Int32', pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_messages_type_float64'] = function (block) {
        addImport('from std_msgs.msg import Float64');
        return ['Float64', pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_messages_type_bool'] = function (block) {
        addImport('from std_msgs.msg import Bool');
        return ['Bool', pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_messages_type_twist'] = function (block) {
        addImport('from geometry_msgs.msg import Twist');
        return ['Twist', pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_messages_type_pose'] = function (block) {
        addImport('from geometry_msgs.msg import Pose');
        return ['Pose', pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_messages_type_laserscan'] = function (block) {
        addImport('from sensor_msgs.msg import LaserScan');
        return ['LaserScan', pythonGenerator.ORDER_ATOMIC];
    };

    // === Message Instance Generators ===
    pythonGenerator.forBlock['node_messages_string'] = function (block) {
        const data = pythonGenerator.valueToCode(block, 'DATA', pythonGenerator.ORDER_NONE);
        addImport('from std_msgs.msg import String');
        // If data provided, use it; otherwise empty constructor
        return [data ? `String(data=${data})` : 'String()', pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_messages_int32'] = function (block) {
        const data = pythonGenerator.valueToCode(block, 'DATA', pythonGenerator.ORDER_NONE);
        addImport('from std_msgs.msg import Int32');
        return [data ? `Int32(data=${data})` : 'Int32()', pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_messages_float64'] = function (block) {
        const data = pythonGenerator.valueToCode(block, 'DATA', pythonGenerator.ORDER_NONE);
        addImport('from std_msgs.msg import Float64');
        return [data ? `Float64(data=${data})` : 'Float64()', pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_messages_bool'] = function (block) {
        const data = pythonGenerator.valueToCode(block, 'DATA', pythonGenerator.ORDER_NONE);
        addImport('from std_msgs.msg import Bool');
        return [data ? `Bool(data=${data})` : 'Bool()', pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_messages_twist'] = function (block) {
        const linX = block.getFieldValue('LIN_X') || 0;
        const linY = block.getFieldValue('LIN_Y') || 0;
        const linZ = block.getFieldValue('LIN_Z') || 0;
        const angX = block.getFieldValue('ANG_X') || 0;
        const angY = block.getFieldValue('ANG_Y') || 0;
        const angZ = block.getFieldValue('ANG_Z') || 0;

        addImport('from geometry_msgs.msg import Twist');

        const code = `Twist(linear=Vector3(x=${linX}, y=${linY}, z=${linZ}), angular=Vector3(x=${angX}, y=${angY}, z=${angZ}))`;
        addImport('from geometry_msgs.msg import Vector3');
        return [code, pythonGenerator.ORDER_ATOMIC];  // 0 = ATOMIC
    };

    pythonGenerator.forBlock['node_messages_pose'] = function (block) {
        const posX = block.getFieldValue('POS_X') || 0;
        const posY = block.getFieldValue('POS_Y') || 0;
        const posZ = block.getFieldValue('POS_Z') || 0;
        const oriX = block.getFieldValue('ORI_X') || 0;
        const oriY = block.getFieldValue('ORI_Y') || 0;
        const oriZ = block.getFieldValue('ORI_Z') || 0;
        const oriW = block.getFieldValue('ORI_W') || 1;

        addImport('from geometry_msgs.msg import Pose, Point, Quaternion');

        const code = `Pose(position=Point(x=${posX}, y=${posY}, z=${posZ}), orientation=Quaternion(x=${oriX}, y=${oriY}, z=${oriZ}, w=${oriW}))`;
        return [code, pythonGenerator.ORDER_ATOMIC];  // 0 = ATOMIC
    };

    pythonGenerator.forBlock['node_messages_point'] = function (block) {
        const x = block.getFieldValue('X') || 0;
        const y = block.getFieldValue('Y') || 0;
        const z = block.getFieldValue('Z') || 0;

        addImport('from geometry_msgs.msg import Point');
        return [`Point(x=${x}, y=${y}, z=${z})`, pythonGenerator.ORDER_ATOMIC];  // 0 = ATOMIC
    };

    pythonGenerator.forBlock['node_messages_laserscan'] = function (block) {
        const field = block.getFieldValue('FIELD') || 'ranges';
        addImport('from sensor_msgs.msg import LaserScan');
        return [`msg.${field}`, pythonGenerator.ORDER_ATOMIC];  // 0 = ATOMIC
    };

    pythonGenerator.forBlock['node_messages_set_field'] = function (block) {
        const msg = block.getFieldValue('MSG') || 'msg';
        const field = block.getFieldValue('FIELD') || 'data';
        const value = pythonGenerator.valueToCode(block, 'VALUE', pythonGenerator.ORDER_NONE) || '0';
        return `${msg}.${field} = ${value}\n`;
    };

    pythonGenerator.forBlock['node_messages_get_field'] = function (block) {
        const msg = block.getFieldValue('MSG') || 'msg';
        const field = block.getFieldValue('FIELD') || 'data';
        return [`${msg}.${field}`, pythonGenerator.ORDER_ATOMIC];  // 0 = ATOMIC (highest precedence)
    };

    // ========================================
    // PARAMETERS BLOCKS
    // ========================================

    pythonGenerator.forBlock['node_parameters_declare'] = function (block) {
        const name = block.getFieldValue('NAME') || 'my_param';
        const defaultValue = block.getFieldValue('DEFAULT') || '0';
        return `self.declare_parameter('${name}', ${defaultValue})\n`;
    };

    pythonGenerator.forBlock['node_parameters_get'] = function (block) {
        const name = block.getFieldValue('NAME') || 'my_param';
        return [`self.get_parameter('${name}').value`, pythonGenerator.ORDER_ATOMIC];  // 0 = ATOMIC
    };

    pythonGenerator.forBlock['node_parameters_set'] = function (block) {
        const name = block.getFieldValue('NAME') || 'my_param';
        const value = block.getFieldValue('VALUE') || 'value';
        addImport('from rclpy.parameter import Parameter');
        return `self.set_parameters([Parameter('${name}', Parameter.Type.STRING, ${value})])\n`;
    };

    // ========================================
    // LOGGING BLOCKS
    // ========================================

    // Text value generator
    pythonGenerator.forBlock['node_logging_text'] = function (block) {
        const text = block.getFieldValue('TEXT') || '';
        return [`'${text.replace(/'/g, "\\'")}'`, pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_logging_info'] = function (block) {
        const message = pythonGenerator.valueToCode(block, 'MESSAGE', pythonGenerator.ORDER_NONE) || "'Message'";
        return `self.get_logger().info(str(${message}))\n`;
    };

    pythonGenerator.forBlock['node_logging_warn'] = function (block) {
        const message = pythonGenerator.valueToCode(block, 'MESSAGE', pythonGenerator.ORDER_NONE) || "'Warning'";
        return `self.get_logger().warning(str(${message}))\n`;
    };

    pythonGenerator.forBlock['node_logging_error'] = function (block) {
        const message = pythonGenerator.valueToCode(block, 'MESSAGE', pythonGenerator.ORDER_NONE) || "'Error'";
        return `self.get_logger().error(str(${message}))\n`;
    };

    pythonGenerator.forBlock['node_logging_debug'] = function (block) {
        const message = pythonGenerator.valueToCode(block, 'MESSAGE', pythonGenerator.ORDER_NONE) || "'Debug'";
        return `self.get_logger().debug(str(${message}))\n`;
    };

    // ========================================
    // CONTROL FLOW BLOCKS
    // ========================================

    pythonGenerator.forBlock['node_controlflow_if'] = function (block) {
        const condition = block.getFieldValue('CONDITION') || 'True';
        const content = pythonGenerator.statementToCode(block, 'CONTENT');

        let code = `if ${condition}:\n`;
        code += content;

        return code;
    };

    pythonGenerator.forBlock['node_controlflow_ifelse'] = function (block) {
        const condition = block.getFieldValue('CONDITION') || 'True';
        const ifContent = pythonGenerator.statementToCode(block, 'IF_CONTENT');
        const elseContent = pythonGenerator.statementToCode(block, 'ELSE_CONTENT');

        let code = `if ${condition}:\n`;
        code += ifContent;
        code += 'else:\n';
        code += elseContent;

        return code;
    };

    pythonGenerator.forBlock['node_controlflow_for'] = function (block) {
        const varName = block.getFieldValue('VAR') || 'i';
        const start = block.getFieldValue('START') || 0;
        const end = block.getFieldValue('END') || 10;
        const content = pythonGenerator.statementToCode(block, 'CONTENT');

        let code = `for ${varName} in range(${start}, ${end}):\n`;
        code += content;

        return code;
    };

    pythonGenerator.forBlock['node_controlflow_while'] = function (block) {
        const condition = block.getFieldValue('CONDITION') || 'True';
        const content = pythonGenerator.statementToCode(block, 'CONTENT');

        let code = `while ${condition}:\n`;
        code += content;

        return code;
    };

    pythonGenerator.forBlock['node_controlflow_break'] = function (block) {
        return 'break\n';
    };

    pythonGenerator.forBlock['node_controlflow_continue'] = function (block) {
        return 'continue\n';
    };

    pythonGenerator.forBlock['node_controlflow_try'] = function (block) {
        const exception = block.getFieldValue('EXCEPTION') || 'Exception';
        const varName = block.getFieldValue('VAR') || 'e';
        const tryContent = pythonGenerator.statementToCode(block, 'TRY_CONTENT');
        const exceptContent = pythonGenerator.statementToCode(block, 'EXCEPT_CONTENT');

        let code = 'try:\n';
        code += tryContent;
        code += `except ${exception} as ${varName}:\n`;
        code += exceptContent;

        return code;
    };

    // ========================================
    // MATH & LOGIC BLOCKS
    // ========================================

    pythonGenerator.forBlock['node_math_arithmetic'] = function (block) {
        const left = pythonGenerator.valueToCode(block, 'A', pythonGenerator.ORDER_NONE) || '0';
        const op = block.getFieldValue('OP') || '+';
        const right = pythonGenerator.valueToCode(block, 'B', pythonGenerator.ORDER_NONE) || '0';
        return [`${left} ${op} ${right}`, pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_math_compare'] = function (block) {
        const left = block.getFieldValue('LEFT') || 'a';
        const op = block.getFieldValue('OP') || '==';
        const right = block.getFieldValue('RIGHT') || 'b';
        return [`${left} ${op} ${right}`, pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_math_logic'] = function (block) {
        const left = block.getFieldValue('LEFT') || 'a';
        const op = block.getFieldValue('OP') || 'and';
        const right = block.getFieldValue('RIGHT') || 'b';
        return [`${left} ${op} ${right}`, pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_math_function'] = function (block) {
        const func = block.getFieldValue('FUNC') || 'abs';
        const arg = pythonGenerator.valueToCode(block, 'ARG', pythonGenerator.ORDER_NONE) || '0';

        if (func.startsWith('math.')) {
            addImport('import math');
        }

        if (func === 'math.pi') {
            return ['math.pi', pythonGenerator.ORDER_ATOMIC];
        }

        return [`${func}(${arg})`, pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_math_random'] = function (block) {
        const type = block.getFieldValue('TYPE') || 'random.random()';
        const min = block.getFieldValue('MIN') || 0;
        const max = block.getFieldValue('MAX') || 100;

        addImport('import random');

        if (type === 'random.random()') {
            return ['random.random()', pythonGenerator.ORDER_ATOMIC];
        } else {
            return [`random.randint(${min}, ${max})`, pythonGenerator.ORDER_ATOMIC];
        }
    };

    pythonGenerator.forBlock['node_math_clamp'] = function (block) {
        const value = block.getFieldValue('VALUE') || 'value';
        const min = block.getFieldValue('MIN') || 0;
        const max = block.getFieldValue('MAX') || 100;
        return [`max(${min}, min(${max}, ${value}))`, pythonGenerator.ORDER_ATOMIC];
    };

    // ========================================
    // ========================================
    // PROCEDURES BLOCKS
    // ========================================

    pythonGenerator.forBlock['node_procedures_define'] = function (block) {
        const name = block.getFieldValue('NAME') || 'my_function';
        let body = pythonGenerator.statementToCode(block, 'BODY') || '';
        if (!body.trim()) {
            body = '        pass\n';
        } else {
            // Add 8 spaces base indentation, preserving relative indentation
            body = body.split('\n').map(line => {
                if (!line.trim()) return '';
                // Add base 8 spaces, keep any existing leading spaces
                return '        ' + line;
            }).join('\n');
        }
        return `\n    def ${name}(self):\n${body}`;
    };

    pythonGenerator.forBlock['node_procedures_call'] = function (block) {
        const name = block.getFieldValue('NAME') || 'my_function';
        return `self.${name}()\n`;
    };

    pythonGenerator.forBlock['node_procedures_return'] = function (block) {
        const value = pythonGenerator.valueToCode(block, 'VALUE', pythonGenerator.ORDER_NONE);
        return value ? `return ${value}\n` : 'return\n';
    };

    // ========================================
    // CONTROL FLOW BLOCKS
    // ========================================

    // Helper to add indentation to statement code
    function indentStatementCode(code) {
        if (!code || code.trim() === '') return '    pass\n';
        // Split by newlines, add 4 spaces to each non-empty line
        const lines = code.split('\n');
        const indentedLines = lines.map(line => {
            // Trim the line first, then add 4 spaces if not empty
            const trimmed = line.trimStart();
            return trimmed ? '    ' + trimmed : '';
        });
        // Join and ensure ends with newline
        let result = indentedLines.join('\n');
        if (!result.endsWith('\n')) result += '\n';
        return result;
    }

    pythonGenerator.forBlock['node_controlflow_if'] = function (block) {
        const condition = pythonGenerator.valueToCode(block, 'CONDITION', pythonGenerator.ORDER_NONE) || 'True';
        let doCode = pythonGenerator.statementToCode(block, 'DO');
        doCode = indentStatementCode(doCode);
        return `if ${condition}:\n${doCode}`;
    };

    pythonGenerator.forBlock['node_controlflow_ifelse'] = function (block) {
        const condition = pythonGenerator.valueToCode(block, 'CONDITION', pythonGenerator.ORDER_NONE) || 'True';
        let doCode = pythonGenerator.statementToCode(block, 'DO');
        let elseCode = pythonGenerator.statementToCode(block, 'ELSE');
        doCode = indentStatementCode(doCode);
        elseCode = indentStatementCode(elseCode);
        return `if ${condition}:\n${doCode}else:\n${elseCode}`;
    };

    pythonGenerator.forBlock['node_controlflow_for'] = function (block) {
        const times = block.getFieldValue('TIMES') || 10;
        let doCode = pythonGenerator.statementToCode(block, 'DO');
        doCode = indentStatementCode(doCode);
        return `for _ in range(${times}):\n${doCode}`;
    };

    pythonGenerator.forBlock['node_controlflow_while'] = function (block) {
        const condition = pythonGenerator.valueToCode(block, 'CONDITION', pythonGenerator.ORDER_NONE) || 'True';
        let doCode = pythonGenerator.statementToCode(block, 'DO');
        doCode = indentStatementCode(doCode);
        return `while ${condition}:\n${doCode}`;
    };

    pythonGenerator.forBlock['node_controlflow_break'] = function (block) {
        return 'break\n';
    };

    pythonGenerator.forBlock['node_controlflow_continue'] = function (block) {
        return 'continue\n';
    };

    pythonGenerator.forBlock['node_controlflow_compare'] = function (block) {
        const a = pythonGenerator.valueToCode(block, 'A', pythonGenerator.ORDER_NONE) || '0';
        const op = block.getFieldValue('OP') || '==';
        const b = pythonGenerator.valueToCode(block, 'B', pythonGenerator.ORDER_NONE) || '0';
        return [`(${a} ${op} ${b})`, pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_controlflow_logic'] = function (block) {
        const a = pythonGenerator.valueToCode(block, 'A', pythonGenerator.ORDER_NONE) || 'True';
        const op = block.getFieldValue('OP') || 'and';
        const b = pythonGenerator.valueToCode(block, 'B', pythonGenerator.ORDER_NONE) || 'True';
        return [`(${a} ${op} ${b})`, pythonGenerator.ORDER_ATOMIC];
    };

    pythonGenerator.forBlock['node_controlflow_not'] = function (block) {
        const value = pythonGenerator.valueToCode(block, 'VALUE', pythonGenerator.ORDER_NONE) || 'True';
        return [`not ${value}`, pythonGenerator.ORDER_ATOMIC];
    };

    console.log('[Node Generator] Initialized');
    return pythonGenerator;
}

// ========================================
// HELPER FUNCTIONS
// ========================================

/**
 * Get import statement for message type
 * @param {string} msgType - Message type name
 * @returns {string} Import statement
 */
function getMsgImport(msgType) {
    const imports = {
        'String': 'from std_msgs.msg import String',
        'Int32': 'from std_msgs.msg import Int32',
        'Float64': 'from std_msgs.msg import Float64',
        'Bool': 'from std_msgs.msg import Bool',
        'Twist': 'from geometry_msgs.msg import Twist',
        'Pose': 'from geometry_msgs.msg import Pose',
        'Point': 'from geometry_msgs.msg import Point',
        'LaserScan': 'from sensor_msgs.msg import LaserScan'
    };
    return imports[msgType] || `# Unknown import for ${msgType}`;
}

/**
 * Convert snake_case to PascalCase
 * @param {string} str - String to convert
 * @returns {string} PascalCase string
 */
function toPascalCase(str) {
    return str
        .split('_')
        .map(word => word.charAt(0).toUpperCase() + word.slice(1).toLowerCase())
        .join('');
}

/**
 * Escape string for Python
 * @param {string} str - String to escape
 * @returns {string} Escaped string
 */
function escapeString(str) {
    return str.replace(/'/g, "\\'").replace(/\n/g, '\\n');
}

/**
 * Generate complete Python code from workspace
 * @param {Blockly.Workspace} workspace - The workspace
 * @param {Object} pythonGenerator - The generator instance
 * @returns {string} Complete Python code
 */
export function generateNodeCode(workspace, pythonGenerator) {
    // Reset imports
    requiredImports.clear();
    lastSkippedBlocks = [];

    if (!workspace || !pythonGenerator) {
        return '';
    }

    // Get all top-level blocks
    const topBlocks = workspace.getTopBlocks(true);
    let nodeCode = '';
    let procedureCode = '';

    // Process blocks - look for Node hat block and procedure definitions
    for (const block of topBlocks) {
        if (block.type === 'node_structure_node') {
            const code = pythonGenerator.blockToCode(block);
            if (code) {
                nodeCode += typeof code === 'string' ? code : code[0];
            }
        } else if (block.type === 'node_procedures_define') {
            // Process procedure definitions separately
            const code = pythonGenerator.blockToCode(block);
            if (code) {
                procedureCode += typeof code === 'string' ? code : code[0];
            }
        } else {
            // Track orphan blocks (not connected to Node block)
            lastSkippedBlocks.push(block.type);
        }
    }

    // Insert procedure code into the node class (before the closing of __init__)
    // Find where to insert procedures - after __init__ but inside class
    if (procedureCode && nodeCode) {
        // Replace the pattern to insert procedures after __init__
        const initEndPattern = /(class \w+\(Node\):[\s\S]*?def __init__\(self\):[\s\S]*?)(\n\ndef main)/;
        nodeCode = nodeCode.replace(initEndPattern, `$1${procedureCode}$2`);
    }

    // Build final code
    let finalCode = '#!/usr/bin/env python3\n';
    finalCode += '"""Generated by ROS Visual Editor"""\n\n';

    // Add imports
    const sortedImports = Array.from(requiredImports).sort();
    for (const imp of sortedImports) {
        finalCode += imp + '\n';
    }

    // Add node class and main function (generated by Node block)
    finalCode += nodeCode;

    // Add entry point
    finalCode += "\n\nif __name__ == '__main__':\n";
    finalCode += '    main()\n';

    return finalCode;
}

/**
 * Get skipped blocks from last generation
 * @returns {string[]} Array of skipped block types
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
