// Delete Package Confirm Dialog Script

let packageName = '';

function confirmDelete() {
    const inputValue = document.getElementById('confirmInput').value.trim();
    if (inputValue === packageName) {
        window.dialogAPI.sendDeletePackageConfirmed(packageName);
    } else {
        document.getElementById('error-msg').style.display = 'block';
    }
}

function cancel() {
    window.dialogAPI.sendDeletePackageConfirmed(null);
}

document.addEventListener('DOMContentLoaded', async () => {
    // Initialize translations
    if (window.dialogI18n) {
        await window.dialogI18n.init();
    }

    const confirmInput = document.getElementById('confirmInput');
    const deleteBtn = document.getElementById('delete-btn');
    const cancelBtn = document.getElementById('cancel-btn');
    const errorMsg = document.getElementById('error-msg');

    // Get package name from main process
    window.dialogAPI.onSetPackageName((name) => {
        packageName = name;
        document.getElementById('package-name-display').textContent = name;
    });

    deleteBtn.addEventListener('click', confirmDelete);
    cancelBtn.addEventListener('click', cancel);

    confirmInput.addEventListener('input', () => {
        const inputValue = confirmInput.value.trim();
        if (inputValue === packageName) {
            deleteBtn.disabled = false;
            errorMsg.style.display = 'none';
        } else {
            deleteBtn.disabled = true;
        }
    });

    confirmInput.addEventListener('keypress', (e) => {
        if (e.key === 'Enter' && !deleteBtn.disabled) {
            confirmDelete();
        }
    });
});
