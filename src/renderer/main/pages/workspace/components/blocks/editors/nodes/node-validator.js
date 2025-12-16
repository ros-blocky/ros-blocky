/**
 * Node Validator
 * Checks for logical and structural errors in the Node blocks
 */

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
                addError("Multiple Node blocks found. Only one is allowed.", block.id);
            }
            nodeBlock = block;
        } else if (block.type === 'node_structure_main') {
            if (mainBlock) {
                addError("Multiple Main blocks found. Only one is allowed.", block.id);
            }
            mainBlock = block;
        }
    }

    // Check required blocks
    if (!nodeBlock) {
        addError("No Node block found. Please add a Node block.");
        return { errors, warnings, blockErrors };
    }

    if (!mainBlock) {
        addWarning("No Main block found. Your node won't have an entry point.");
    }

    // ========================================
    // PASS 2: Collect definitions from Node block
    // ========================================
    const nodeDescendants = nodeBlock.getDescendants(false);

    for (const block of nodeDescendants) {
        const type = block.type;

        // Collect callback definitions
        if (type === 'node_subscribers_callback' || type === 'node_timers_callback') {
            const name = block.getFieldValue('NAME');
            if (name) {
                if (definedCallbacks.has(name)) {
                    addError(`Duplicate callback name '${name}'.`, block.id);
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

    // Check Node has Init
    let hasInit = false;
    const nodeInput = nodeBlock.getInput('CONTENT');
    if (nodeInput && nodeInput.connection && nodeInput.connection.targetBlock()) {
        let child = nodeInput.connection.targetBlock();
        while (child) {
            if (child.type === 'node_structure_init') {
                hasInit = true;
                break;
            }
            child = child.getNextBlock();
        }
    }
    if (!hasInit) {
        addWarning("Node block has no Init section. Consider adding one.", nodeBlock.id);
    }

    // Check all used callbacks are defined
    for (const callbackName of usedCallbacks) {
        if (!definedCallbacks.has(callbackName)) {
            addError(`Callback '${callbackName}' is used but not defined.`);
        }
    }

    // Check for unused callbacks
    for (const callbackName of definedCallbacks) {
        if (!usedCallbacks.has(callbackName)) {
            addWarning(`Callback '${callbackName}' is defined but never used.`);
        }
    }

    // ========================================
    // PASS 4: Check publishers and subscribers
    // ========================================
    for (const block of nodeDescendants) {
        const type = block.type;

        // Check publisher has a topic
        if (type === 'node_publishers_create') {
            const topic = block.getFieldValue('TOPIC');
            if (!topic || topic === '/topic') {
                addWarning("Publisher has default topic name. Consider renaming.", block.id);
            }
        }

        // Check subscriber has a topic
        if (type === 'node_subscribers_create') {
            const topic = block.getFieldValue('TOPIC');
            if (!topic || topic === '/topic') {
                addWarning("Subscriber has default topic name. Consider renaming.", block.id);
            }
        }

        // Check timer period is reasonable
        if (type === 'node_timers_create') {
            const period = block.getFieldValue('PERIOD');
            if (period && period < 0.01) {
                addWarning(`Timer period ${period}s is very small. This may cause performance issues.`, block.id);
            }
        }
    }

    // ========================================
    // PASS 5: Check Main block structure
    // ========================================
    if (mainBlock) {
        let hasSpin = false;
        const mainDescendants = mainBlock.getDescendants(false);

        for (const block of mainDescendants) {
            if (block.type === 'node_structure_spin' || block.type === 'node_structure_spin_once') {
                hasSpin = true;
                break;
            }
        }

        if (!hasSpin) {
            addWarning("Main block has no spin. Your node will exit immediately.", mainBlock.id);
        }
    }

    return { errors, warnings, blockErrors };
}
