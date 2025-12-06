const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process');
const dns = require('dns');
const progressTracker = require('./progressTracker');

class SetupService {
    constructor() {
        this.FLAG_FILE_PATH = 'C:/pixi_ws/env_ready.flag';
        this.PIXI_WORKING_DIR = 'C:/pixi_ws';
        this.setupWindow = null;
        this.createWindowCallback = null;
        this.installationProcess = null;
    }

    setSetupWindow(window) {
        this.setupWindow = window;
    }

    setMainWindowCreator(callback) {
        this.createWindowCallback = callback;
    }

    cancelInstallation() {
        console.log('Cancelling installation...');
        if (this.installationProcess && this.installationProcess.pid) {
            try {
                // On Windows, use taskkill to kill the entire process tree
                if (process.platform === 'win32') {
                    spawn('taskkill', ['/pid', this.installationProcess.pid, '/f', '/t'], {
                        windowsHide: true
                    });
                    console.log(`Killed process tree for PID ${this.installationProcess.pid}`);
                } else {
                    // On Unix-like systems, use regular kill
                    this.installationProcess.kill();
                    console.log('Installation process killed');
                }
            } catch (error) {
                console.error('Error killing installation process:', error);
            }
            this.installationProcess = null;
        }

        progressTracker.stopProgressMonitoring();

        if (this.setupWindow && !this.setupWindow.isDestroyed()) {
            this.setupWindow.close();
        }
    }

    isEnvironmentReady() {
        return fs.existsSync(this.FLAG_FILE_PATH);
    }

    createFlagFile() {
        try {
            const dir = path.dirname(this.FLAG_FILE_PATH);
            if (!fs.existsSync(dir)) {
                fs.mkdirSync(dir, { recursive: true });
            }
            fs.writeFileSync(this.FLAG_FILE_PATH, `Setup completed at: ${new Date().toISOString()}`);
            console.log('Flag file created successfully');
            return true;
        } catch (error) {
            console.error('Error creating flag file:', error);
            return false;
        }
    }

    checkInternetConnection() {
        return new Promise((resolve) => {
            dns.lookup('google.com', (err) => {
                if (err && err.code === 'ENOTFOUND') {
                    resolve(false);
                } else {
                    resolve(true);
                }
            });
        });
    }

    runPixiInstall() {
        return new Promise((resolve, reject) => {
            if (!fs.existsSync(this.PIXI_WORKING_DIR)) {
                const error = `Working directory does not exist: ${this.PIXI_WORKING_DIR}`;
                console.error('ERROR:', error);
                reject(new Error(error));
                return;
            }

            console.log('========================================');
            console.log('Starting pixi install...');
            console.log('Working directory:', this.PIXI_WORKING_DIR);
            console.log('Command: pixi install -vv');
            console.log('========================================');

            this.installationProcess = spawn('pixi', ['install', '-vv'], {
                cwd: this.PIXI_WORKING_DIR,
                shell: true,
                windowsHide: true
            });

            this.installationProcess.stdout.on('data', (data) => {
                console.log(`[STDOUT]: ${data}`);
            });

            this.installationProcess.stderr.on('data', (data) => {
                console.error(`[STDERR]: ${data}`);
            });

            this.installationProcess.on('close', (code) => {
                console.log(`pixi install exited with code ${code}`);
                this.installationProcess = null;
                if (code === 0) {
                    resolve();
                } else {
                    // If killed (null), don't reject with error, just resolve or handle gracefully
                    // But here we might want to reject if it wasn't a manual cancel
                    reject(new Error(`pixi install failed with exit code ${code}`));
                }
            });

            this.installationProcess.on('error', (error) => {
                console.error('Failed to start pixi install:', error);
                reject(error);
            });
        });
    }

    async handleInstallation() {
        try {
            const pixiLockPath = path.join(this.PIXI_WORKING_DIR, 'pixi.lock');
            const condaMetaPath = path.join(this.PIXI_WORKING_DIR, '.pixi', 'envs', 'default', 'conda-meta');

            progressTracker.startProgressMonitoring(this.setupWindow, pixiLockPath, condaMetaPath);

            await this.runPixiInstall();

            progressTracker.stopProgressMonitoring();

            const flagCreated = this.createFlagFile();
            if (!flagCreated) {
                throw new Error('Failed to create flag file');
            }

            if (this.setupWindow && !this.setupWindow.isDestroyed()) {
                this.setupWindow.webContents.send('installation-complete', true);
            }

            setTimeout(() => {
                if (this.setupWindow && !this.setupWindow.isDestroyed()) {
                    this.setupWindow.close();
                    this.setupWindow = null;
                }
                if (this.createWindowCallback) {
                    this.createWindowCallback();
                }
            }, 2000);

        } catch (error) {
            console.error('Installation failed:', error);
            progressTracker.stopProgressMonitoring();

            if (this.setupWindow && !this.setupWindow.isDestroyed()) {
                this.setupWindow.webContents.send('installation-error', error.message);
                this.setupWindow.webContents.send('installation-complete', false);
            }
        }
    }
}

module.exports = new SetupService();
