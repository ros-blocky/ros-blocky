/**
 * Node Validator
 * Checks for logical and structural errors in the Node blocks
 */

import { t } from '../../../../../../../i18n/index.js';

/**
 * Validate the Node workspace
 * @param {Blockly.Workspace} workspace - The workspace to validate
 * @returns {Object} Result object { errors: string[], warnings: string[], blockErrors: Map<string, string> }
 */
export function validateNode(workspace) {
    const errors = [];
    const warnings = [];
    // Map of block ID to error message
    const blockErrors = new Map();

    /**
     * Helper to add error
     * @param {string} msg - Error message
     * @param {string|null} blockId - Block ID to attach error to
     */
    const addError = (msg, blockId = null) => {
        errors.push(msg);
        if (blockId) {
            blockErrors.set(blockId, msg);
        }
    };

    /**
     * Helper to add warning
     * @param {string} msg - Warning message
     * @param {string|null} blockId - Block ID to attach warning to
     */
    const addWarning = (msg, blockId = null) => {
        warnings.push(msg);
        if (blockId) {
            // Warnings go to the same map for now (displayed as warnings)
            blockErrors.set(blockId, `⚠ ${msg}`);
        }
    };

    if (!workspace) return { errors, warnings, blockErrors };

    const topBlocks = workspace.getTopBlocks(true);
    let nodeBlock = null;
    let mainBlock = null;
    const definedCallbacks = new Set();
    const usedCallbacks = new Set();
    const definedVariables = new Set();

    // ========================================
    // PASS 1: Find Node and Main blocks
    // ========================================
    for (const block of topBlocks) {
        if (block.type === 'node_structure_node') {
            if (nodeBlock) {
                addError(t('validation.multipleNodeBlocks'), block.id);
            }
            nodeBlock = block;
        } else if (block.type === 'node_structure_main') {
            if (mainBlock) {
                addError("Multiple Main blocks found. Only one is allowed.", block.id);
            }
            mainBlock = block;
        }
    }

    // Check required blocks - but don't return early, still check for orphan blocks
    const hasNodeBlock = !!nodeBlock;
    if (!hasNodeBlock) {
        addError(t('validation.noNodeBlock'));
    }

    // Note: Main block is not required - Node generates entry point automatically

    // ========================================
    // PASS 2: Collect definitions from Node block
    // ========================================
    const nodeDescendants = hasNodeBlock ? nodeBlock.getDescendants(false) : [];

    for (const block of nodeDescendants) {
        const type = block.type;

        // Collect callback definitions
        if (type === 'node_subscribers_callback' || type === 'node_timers_callback') {
            const name = block.getFieldValue('NAME');
            if (name) {
                if (definedCallbacks.has(name)) {
                    addError(t('validation.duplicateCallback').replace('{{name}}', name), block.id);
                } else {
                    definedCallbacks.add(name);
                }
            }
        }

        // Collect variable definitions
        if (type === 'node_variables_create' ||
            type === 'node_variables_create_list' ||
            type === 'node_variables_create_dict') {
            const name = block.getFieldValue('NAME');
            if (name) {
                definedVariables.add(name);
            }
        }

        // Collect callback usages (from subscribers and timers)
        if (type === 'node_subscribers_create') {
            const callbackName = block.getFieldValue('CALLBACK');
            if (callbackName) usedCallbacks.add(callbackName);
        }
        if (type === 'node_timers_create') {
            const callbackName = block.getFieldValue('CALLBACK');
            if (callbackName) usedCallbacks.add(callbackName);
        }
    }

    // ========================================
    // PASS 3: Validate structure
    // ========================================

    // Note: Init section is not required - Node block handles initialization directly

    // Check all used callbacks are defined
    for (const callbackName of usedCallbacks) {
        if (!definedCallbacks.has(callbackName)) {
            addError(t('validation.callbackNotDefined').replace('{{name}}', callbackName));
        }
    }

    // Check for unused callbacks
    for (const callbackName of definedCallbacks) {
        if (!usedCallbacks.has(callbackName)) {
            addWarning(t('validation.unusedCallback').replace('{{name}}', callbackName));
        }
    }

    // ========================================
    // PASS 4: Check publishers, subscribers and message types
    // ========================================

    // Get all blocks for validation
    const allBlocks = workspace.getAllBlocks(false);

    // Message type block names (must match block definitions)
    const msgTypeBlocks = [
        'node_messages_type_string', 'node_messages_type_int32', 'node_messages_type_float64',
        'node_messages_type_bool', 'node_messages_type_twist', 'node_messages_type_pose',
        'node_messages_type_laserscan'
    ];

    for (const block of allBlocks) {
        const type = block.type;

        // Check publisher required fields
        if (type === 'node_publishers_create') {
            const name = block.getFieldValue('NAME');
            const topic = block.getFieldValue('TOPIC');
            const msgTypeInput = block.getInput('MSG_TYPE');
            const hasMsgType = msgTypeInput && msgTypeInput.connection && msgTypeInput.connection.targetBlock();

            // Errors - truly missing
            if (!name) {
                addError(t('validation.publisherMissingName'), block.id);
            }
            if (!topic) {
                addError(t('validation.publisherMissingTopic'), block.id);
            }
            if (!hasMsgType) {
                addError(t('validation.publisherMissingType'), block.id);
            }
        }

        // Check subscriber required fields
        if (type === 'node_subscribers_create') {
            const name = block.getFieldValue('NAME');
            const topic = block.getFieldValue('TOPIC');
            const msgTypeInput = block.getInput('MSG_TYPE');
            const hasMsgType = msgTypeInput && msgTypeInput.connection && msgTypeInput.connection.targetBlock();

            // Errors - truly missing
            if (!name) {
                addError(t('validation.subscriberMissingName'), block.id);
            }
            if (!topic) {
                addError(t('validation.subscriberMissingTopic'), block.id);
            }
            if (!hasMsgType) {
                addError(t('validation.subscriberMissingType'), block.id);
            }
        }

        // Check timer period is reasonable
        if (type === 'node_timers_create') {
            const period = block.getFieldValue('PERIOD');
            if (period && period < 0.01) {
                addWarning(t('validation.smallTimerPeriod').replace('{{period}}', period), block.id);
            }
        }

        // Check MsgType blocks are only in Publisher/Subscriber slots
        if (msgTypeBlocks.includes(type)) {
            const parent = block.getParent();
            if (parent) {
                const parentType = parent.type;
                if (parentType !== 'node_publishers_create' && parentType !== 'node_subscribers_create') {
                    addError(t('validation.msgTypeWrongSlot'), block.id);
                }
            }
        }
    }

    // Note: Main block structure validation removed - Node handles spin automatically

    // ========================================
    // PASS 5: Check for orphan blocks (not inside Node or Procedure)
    // ========================================
    const nodeDescendantIds = new Set(nodeDescendants.map(b => b.id));

    // Check Procedure blocks - they need a Node block to be useful
    for (const block of allBlocks) {
        if (block.type === 'node_procedures_define' && !hasNodeBlock) {
            addWarning(t('validation.procedureNeedsNode'), block.id);
        }
    }

    // Block types that must be inside Node or Procedure
    const requiresParent = [
        'node_publishers_create', 'node_publishers_publish',
        'node_subscribers_create',
        'node_timers_create',
        'node_variables_create', 'node_variables_create_list', 'node_variables_create_dict',
        'node_variables_get', 'node_variables_set',
        'node_logging_info', 'node_logging_warn', 'node_logging_error', 'node_logging_debug',
        'node_parameters_declare', 'node_parameters_get',
        'node_messages_string', 'node_messages_int', 'node_messages_float', 'node_messages_bool',
        'node_messages_twist', 'node_messages_pose', 'node_messages_point',
        'node_messages_set_field', 'node_messages_get_field'
    ];

    for (const block of allBlocks) {
        // Skip if block is Node itself or inside Node
        if (block.type === 'node_structure_node' || nodeDescendantIds.has(block.id)) {
            continue;
        }

        // Check if this block type requires a parent
        if (requiresParent.includes(block.type)) {
            // Check if inside a procedure (node_procedures_define)
            let isInProcedure = false;
            let parent = block.getSurroundParent();
            while (parent) {
                if (parent.type === 'node_procedures_define') {
                    isInProcedure = true;
                    break;
                }
                parent = parent.getSurroundParent();
            }

            if (!isInProcedure) {
                addWarning(t('validation.orphanBlock'), block.id);
            }
        }
    }

    return { errors, warnings, blockErrors };
}
