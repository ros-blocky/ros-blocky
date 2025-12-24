// Project Name Prompt Dialog Script

function create() {
    const name = document.getElementById('projectName').value.trim();
    console.log('[Dialog] Create clicked, name:', name);
    if (name) {
        console.log('[Dialog] Sending project name via IPC...');
        if (window.dialogAPI && window.dialogAPI.sendProjectNameResult) {
            window.dialogAPI.sendProjectNameResult(name);
            console.log('[Dialog] IPC sent successfully');
        } else {
            console.error('[Dialog] dialogAPI not available!', window.dialogAPI);
        }
    } else {
        console.log('[Dialog] Name is empty, focusing input');
        document.getElementById('projectName').focus();
    }
}

function cancel() {
    console.log('[Dialog] Cancel clicked');
    if (window.dialogAPI && window.dialogAPI.sendProjectNameResult) {
        window.dialogAPI.sendProjectNameResult(null);
    }
}

document.addEventListener('DOMContentLoaded', async () => {
    console.log('[Dialog] DOM loaded, dialogAPI:', window.dialogAPI);

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
