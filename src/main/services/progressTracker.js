const fs = require('fs');
const path = require('path');

/**
 * Module to track pixi installation progress
 */
class ProgressTracker {
    constructor() {
        this.intervalId = null;
        this.totalPackages = 0;
        this.installedPackages = 0;
        this.isMonitoring = false;
    }

    /**
     * Parses pixi.lock to get the total number of packages to install
     * @param {string} pixiLockPath - Path to pixi.lock file
     * @returns {Promise<number>} Total number of packages
     */
    async getExpectedPackageCount(pixiLockPath) {
        try {
            if (!fs.existsSync(pixiLockPath)) {
                console.warn(`[ProgressTracker] pixi.lock not found at ${pixiLockPath} (yet)`);
                return 0;
            }

            const content = await fs.promises.readFile(pixiLockPath, 'utf8');

            // Try parsing as YAML (v6+ format)
            // Look for lines starting with "- conda:" or "- pypi:" at the root level
            let packageMatches = content.match(/^- (conda|pypi):/gm);

            if (!packageMatches || packageMatches.length === 0) {
                // Fallback to TOML format (older versions)
                // Count [[package]] occurrences
                packageMatches = content.match(/^\[\[package\]\]/gm);
            }

            this.totalPackages = packageMatches ? packageMatches.length : 0;
            console.log(`[ProgressTracker] Parsed pixi.lock: found ${this.totalPackages} packages`);

            return this.totalPackages;
        } catch (error) {
            console.error('[ProgressTracker] Error parsing pixi.lock:', error);
            return 0;
        }
    }

    /**
     * Counts the number of installed packages in the conda-meta directory
     * @param {string} condaMetaPath - Path to conda-meta directory
     * @returns {Promise<number>} Number of installed packages
     */
    async getInstalledPackageCount(condaMetaPath) {
        try {
            // Handle case where directory doesn't exist yet (installation just started)
            if (!fs.existsSync(condaMetaPath)) {
                return 0;
            }

            const files = await fs.promises.readdir(condaMetaPath);

            // Count .json files (each installed package has a json metadata file)
            const jsonFiles = files.filter(file => file.endsWith('.json'));

            this.installedPackages = jsonFiles.length;
            return this.installedPackages;
        } catch (error) {
            // Directory might be locked or inaccessible temporarily
            return this.installedPackages; // Return last known count
        }
    }

    /**
     * Starts monitoring the installation progress
     * @param {BrowserWindow} setupWindow - The setup window to send IPC events to
     * @param {string} pixiLockPath - Path to pixi.lock
     * @param {string} condaMetaPath - Path to conda-meta directory
     */
    async startProgressMonitoring(setupWindow, pixiLockPath, condaMetaPath) {
        if (this.isMonitoring) return;

        this.isMonitoring = true;

        // Get initial total count
        await this.getExpectedPackageCount(pixiLockPath);

        // Send initial update
        this.sendUpdate(setupWindow);

        // Start polling interval
        this.intervalId = setInterval(async () => {
            if (!this.isMonitoring) return;

            // If total packages is still 0, try to parse pixi.lock again
            // (It might have been created or updated during installation)
            if (this.totalPackages === 0) {
                await this.getExpectedPackageCount(pixiLockPath);
            }

            await this.getInstalledPackageCount(condaMetaPath);
            this.sendUpdate(setupWindow);

            // If we've reached the total, we could stop, but let's keep monitoring 
            // until explicitly stopped in case of post-install scripts etc.
        }, 1000); // Check every second
    }

    /**
     * Stops the progress monitoring
     */
    stopProgressMonitoring() {
        this.isMonitoring = false;
        if (this.intervalId) {
            clearInterval(this.intervalId);
            this.intervalId = null;
        }
    }

    /**
     * Sends progress update to the window
     * @param {BrowserWindow} setupWindow 
     */
    sendUpdate(setupWindow) {
        if (!setupWindow || setupWindow.isDestroyed()) return;

        const percentage = this.totalPackages > 0
            ? Math.min(Math.round((this.installedPackages / this.totalPackages) * 100), 100)
            : 0;

        const message = `Installing packages: ${this.installedPackages}/${this.totalPackages} (${percentage}%)`;

        setupWindow.webContents.send('installation-progress', {
            current: this.installedPackages,
            total: this.totalPackages,
            percentage: percentage,
            message: message
        });
    }
}

module.exports = new ProgressTracker();
