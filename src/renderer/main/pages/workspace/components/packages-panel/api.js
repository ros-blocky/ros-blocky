/**
 * Packages Panel API
 * Wrapper functions for window.electronAPI calls
 */

// ============================================
// Package Operations
// ============================================

export async function promptPackageName() {
    return window.electronAPI.promptPackageName();
}

export async function createPackage(packageName) {
    return window.electronAPI.createPackage(packageName);
}

export async function deletePackage(packageName) {
    return window.electronAPI.deletePackage(packageName);
}

export async function listPackages() {
    return window.electronAPI.listPackages();
}

// ============================================
// Node Operations
// ============================================

export async function promptNodeName(packageName) {
    return window.electronAPI.promptNodeName(packageName);
}

export async function createNode(packageName, nodeName) {
    return window.electronAPI.createNode(packageName, nodeName);
}

export async function listPackageNodes(packageName) {
    return window.electronAPI.listPackageNodes(packageName);
}

// ============================================
// File Section Operations
// ============================================

export async function listPackageUrdfs(packageName) {
    return window.electronAPI.listPackageUrdfs(packageName);
}

export async function listPackageConfigs(packageName) {
    return window.electronAPI.listPackageConfigs(packageName);
}

export async function listPackageLaunches(packageName) {
    return window.electronAPI.listPackageLaunches(packageName);
}

export async function listPackageMeshes(packageName) {
    return window.electronAPI.listPackageMeshes(packageName);
}

export async function promptFileName(packageName, title, prompt) {
    return window.electronAPI.promptFileName(packageName, title, prompt);
}

export async function createSectionFile(packageName, folderName, fullFileName) {
    return window.electronAPI.createSectionFile(packageName, folderName, fullFileName);
}

export async function listFolderFiles(packageName, folderName, extensions) {
    return window.electronAPI.listFolderFiles(packageName, folderName, extensions);
}

// ============================================
// Mesh Operations
// ============================================

export async function importMeshFiles(packageName) {
    return window.electronAPI.importMeshFiles(packageName);
}

export async function selectFolderFiles(packageName, folderName, extensions, fileTypeLabel) {
    return window.electronAPI.selectFolderFiles(packageName, folderName, extensions, fileTypeLabel);
}

export async function copyFolderFilesForce(packageName, folderName, selectedFiles) {
    return window.electronAPI.copyFolderFilesForce(packageName, folderName, selectedFiles);
}

// ============================================
// Delete Operations
// ============================================

export async function deleteSectionFile(packageName, sectionType, fileName) {
    return window.electronAPI.deleteSectionFile(packageName, sectionType, fileName);
}

export async function deleteSectionFiles(packageName, sectionType) {
    return window.electronAPI.deleteSectionFiles(packageName, sectionType);
}

export async function showConfirmDialog(message) {
    return window.electronAPI.showConfirmDialog(message);
}

// ============================================
// Event Listeners
// ============================================

export function onProjectLoaded(callback) {
    return window.electronAPI.onProjectLoaded(callback);
}
