// Package Name Prompt Dialog Script

function validatePackageName(name) {
    const validNamePattern = /^[a-z][a-z0-9_]*$/;
    if (!name || name.trim() === '') {
        return 'Package name cannot be empty';
    }
    if (!validNamePattern.test(name)) {
        return 'Must start with a letter and contain only lowercase letters, numbers, and underscores';
    }
    return null;
}

function create() {
    const name = document.getElementById('packageName').value.trim();
    const errorMsg = document.getElementById('error-msg');
    const error = validatePackageName(name);

    if (error) {
        errorMsg.textContent = error;
        errorMsg.style.display = 'block';
        document.getElementById('packageName').focus();
    } else {
        errorMsg.style.display = 'none';
        window.dialogAPI.sendPackageNameResult(name);
    }
}

function cancel() {
    window.dialogAPI.sendPackageNameResult(null);
}

document.addEventListener('DOMContentLoaded', async () => {
    // Initialize i18n for dialog
    if (window.dialogI18n) {
        await window.dialogI18n.init();
    }

    const packageNameInput = document.getElementById('packageName');
    const createBtn = document.getElementById('create-btn');
    const cancelBtn = document.getElementById('cancel-btn');

    createBtn.addEventListener('click', create);
    cancelBtn.addEventListener('click', cancel);

    packageNameInput.addEventListener('keypress', (e) => {
        if (e.key === 'Enter') create();
    });

    packageNameInput.addEventListener('input', () => {
        document.getElementById('error-msg').style.display = 'none';
    });
});
