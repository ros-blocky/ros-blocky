/**
 * Pixi Utility - Shared pixi shell execution
 * Centralized pixi command execution to avoid code duplication
 */

const { spawn } = require('child_process');
const path = require('path');
const fs = require('fs');
const os = require('os');

// Pixi workspace path where pixi.toml is located
const PIXI_WS_PATH = 'C:\\pixi_ws';

/**
 * Execute a command in pixi shell and wait for completion
 * @param {string[]} commands - Array of commands to run
 * @param {Object} options - Options { targetDir, onOutput, onError }
 * @returns {Promise<{success: boolean, output?: string, error?: string, code?: number}>}
 */
function executeInPixiShell(commands, options = {}) {
    return new Promise((resolve) => {
        const { targetDir, onOutput, onError } = options;

        // Build command list
        let fullCommands = [];
        if (targetDir) {
            fullCommands.push(`cd /d "${targetDir}"`);
        }
        fullCommands = fullCommands.concat(commands);

        // Create temp files for commands
        const tempDir = os.tmpdir();
        const timestamp = Date.now();
        const commandsFile = path.join(tempDir, `pixi_cmds_${timestamp}.txt`);
        const commandsContent = fullCommands.join('\n') + '\n';

        fs.writeFileSync(commandsFile, commandsContent, { encoding: 'utf8' });

        // Create batch file that pipes commands to pixi shell
        const tempBatch = path.join(tempDir, `pixi_run_${timestamp}.bat`);
        const batchContent = `@echo off\ncd /d ${PIXI_WS_PATH}\ntype "${commandsFile}" | pixi shell\n`;
        fs.writeFileSync(tempBatch, batchContent, { encoding: 'utf8' });

        console.log('[PixiUtil] Running commands:', fullCommands);

        const child = spawn('cmd', ['/c', tempBatch], {
            cwd: PIXI_WS_PATH,
            windowsHide: true
        });

        let stdout = '';
        let stderr = '';

        child.stdout.on('data', (data) => {
            const text = data.toString();
            stdout += text;
            console.log('[PixiUtil Output]', text);
            if (onOutput) onOutput(text);
        });

        child.stderr.on('data', (data) => {
            const text = data.toString();
            stderr += text;
            console.log('[PixiUtil Error]', text);
            if (onError) onError(text);
        });

        child.on('close', (code) => {
            console.log('[PixiUtil] Closed with code:', code);

            // Cleanup temp files
            try {
                fs.unlinkSync(commandsFile);
                fs.unlinkSync(tempBatch);
            } catch (e) { /* ignore */ }

            if (code === 0 || stdout.includes('going to create')) {
                resolve({ success: true, output: stdout, code });
            } else {
                resolve({ success: false, error: stderr || stdout, output: stdout, code });
            }
        });

        child.on('error', (error) => {
            console.error('[PixiUtil] Process error:', error);
            resolve({ success: false, error: error.message });
        });
    });
}

module.exports = {
    PIXI_WS_PATH,
    executeInPixiShell
};
