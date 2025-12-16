// Project Name Prompt Dialog Script

function create() {
    const name = document.getElementById('projectName').value.trim();
    if (name) {
        window.dialogAPI.sendProjectNameResult(name);
    } else {
        document.getElementById('projectName').focus();
    }
}

function cancel() {
    window.dialogAPI.sendProjectNameResult(null);
}

document.addEventListener('DOMContentLoaded', async () => {
    // Initialize translations
    if (window.dialogI18n) {
        await window.dialogI18n.init();
    }

    const projectNameInput = document.getElementById('projectName');
    const createBtn = document.getElementById('create-btn');
    const cancelBtn = document.getElementById('cancel-btn');

    createBtn.addEventListener('click', create);
    cancelBtn.addEventListener('click', cancel);

    projectNameInput.addEventListener('keypress', (e) => {
        if (e.key === 'Enter') create();
    });
});
