// Package Creation Loading Dialog Script

document.addEventListener('DOMContentLoaded', () => {
    window.dialogAPI.onPackageCreationStatus((status) => {
        document.getElementById('status-text').textContent = status;
    });
});
