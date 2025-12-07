// Node Name Prompt Dialog Script

let packageName = '';

function isValidNodeName(name) {
    return /^[a-z][a-z0-9_]*$/.test(name);
}

function create() {
    const nodeName = document.getElementById('nodeInput').value.trim();
    if (isValidNodeName(nodeName)) {
        window.dialogAPI.sendNodeNameResult(nodeName);
    }
}

function cancel() {
    window.dialogAPI.sendNodeNameResult(null);
}

document.addEventListener('DOMContentLoaded', () => {
    const nodeInput = document.getElementById('nodeInput');
    const createBtn = document.getElementById('create-btn');
    const cancelBtn = document.getElementById('cancel-btn');
    const errorMsg = document.getElementById('error-msg');

    // Get package name from main process
    window.dialogAPI.onSetPackageName((name) => {
        packageName = name;
        document.getElementById('package-name').textContent = name;
    });

    createBtn.addEventListener('click', create);
    cancelBtn.addEventListener('click', cancel);

    nodeInput.addEventListener('input', () => {
        const value = nodeInput.value.trim();
        if (value === '') {
            createBtn.disabled = true;
            errorMsg.style.display = 'none';
        } else if (isValidNodeName(value)) {
            createBtn.disabled = false;
            errorMsg.style.display = 'none';
        } else {
            createBtn.disabled = true;
            errorMsg.style.display = 'block';
        }
    });

    nodeInput.addEventListener('keypress', (e) => {
        if (e.key === 'Enter' && !createBtn.disabled) {
            create();
        }
    });
});
