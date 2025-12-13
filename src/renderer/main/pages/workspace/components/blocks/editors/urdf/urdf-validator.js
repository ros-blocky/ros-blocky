/**
 * URDF Validator
 * Checks for logical and structural errors in the URDF blocks
 */

/**
 * Validate the URDF workspace
 * @param {Blockly.Workspace} workspace - The workspace to validate
 * @returns {Object} Result object { errors: string[], warnings: string[], blockErrors: Map<string, string> }
 */
export function validateUrdf(workspace) {
    const errors = [];
    const warnings = [];
    // Map of block ID to error message
    const blockErrors = new Map();

    // Helper to add error
    const addError = (msg, blockId = null) => {
        errors.push(msg);
        if (blockId) {
            blockErrors.set(blockId, msg);
        }
    };

    if (!workspace) return { errors, warnings, blockErrors };

    const topBlocks = workspace.getTopBlocks(true);
    let robotBlock = null;
    const definedLinks = new Set();
    const definedJoints = new Set();
    const definedProcedures = new Set();

    // 1. Identify Robot Block and Procedures
    for (const block of topBlocks) {
        if (block.type === 'urdf_robot') {
            if (robotBlock) {
                addError("Multiple Robot blocks found. Only one is allowed.", block.id);
            }
            robotBlock = block;
        } else if (block.type === 'urdf_define_procedure') {
            const name = block.getFieldValue('NAME');
            if (name) definedProcedures.add(name);
        }
    }

    if (!robotBlock) {
        // Global error, no specific block to attach to (unless we attach to all tops?)
        addError("No Robot block found. Please add a Robot block.");
        // Cannot perform further checks without a robot context (mostly)
        // But we could check procedures.
        return { errors, warnings, blockErrors };
    }

    // 2. Collect Valid Names (Links & Joints)
    // We need to traverse inside the robot block
    const allBlocks = robotBlock.getDescendants(false);

    // First Pass: Collect Definitions
    for (const block of allBlocks) {
        const type = block.type;
        const name = block.getFieldValue('NAME');

        if (type === 'urdf_link') {
            if (!name) {
                addError("Found a Link with no name.", block.id);
            } else if (definedLinks.has(name)) {
                addError(`Duplicate Link name '${name}'.`, block.id);
            } else {
                definedLinks.add(name);
            }
        } else if (type === 'urdf_joint') {
            if (!name) {
                addError("Found a Joint with no name.", block.id);
            } else if (definedJoints.has(name)) {
                addError(`Duplicate Joint name '${name}'.`, block.id);
            } else {
                definedJoints.add(name);
            }
        }
    }

    // 3. Structural & Reference Checks
    for (const block of allBlocks) {
        const type = block.type;
        const name = block.getFieldValue('NAME') || 'Unnamed';

        // Check Joint Structure
        if (type === 'urdf_joint') {
            let hasParent = false;
            let hasChild = false;
            let parentLink = null;
            let childLink = null;

            // Iterate children in the CONTENT input
            const input = block.getInput('CONTENT');
            if (input && input.connection && input.connection.targetBlock()) {
                let child = input.connection.targetBlock();
                while (child) {
                    if (child.type === 'urdf_parent') {
                        hasParent = true;
                        parentLink = child.getFieldValue('NAME');
                    }
                    if (child.type === 'urdf_child') {
                        hasChild = true;
                        childLink = child.getFieldValue('NAME');
                    }
                    child = child.getNextBlock();
                }
            }

            if (!hasParent) addError(`Joint '${name}' is missing a Parent link.`, block.id);
            else if (!hasChild) addError(`Joint '${name}' is missing a Child link.`, block.id);

            // Check References
            if (parentLink && !definedLinks.has(parentLink)) {
                addError(`Joint '${name}' references non-existent parent link '${parentLink}'.`, block.id);
            }
            if (childLink && !definedLinks.has(childLink)) {
                addError(`Joint '${name}' references non-existent child link '${childLink}'.`, block.id);
            }
            if (parentLink && childLink && parentLink === childLink) {
                addError(`Joint '${name}' connects link '${parentLink}' to itself.`, block.id);
            }
        }

        // Check Proc Calls
        if (type === 'urdf_call_procedure') {
            const procName = block.getFieldValue('NAME');
            if (procName && !definedProcedures.has(procName)) {
                addError(`Call to undefined procedure '${procName}'.`, block.id);
            }
        }
    }

    // Check root link (Warning)
    // A simplified check: there should be at least one link that is never a child of any joint
    // Not strictly required for partial URDFs or XACROs, but good practice.
    // Keeping it simple for now to avoid false positives in XACRO/Procedures.

    return { errors, warnings, blockErrors };
}
