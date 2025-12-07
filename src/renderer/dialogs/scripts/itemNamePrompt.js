// Generic Item Name Prompt Dialog Script

let itemConfig = {
    type: 'node',
    packageName: ''
};

function isValidName(name) {
    return /^[a-z][a-z0-9_]*$/.test(name);
}

function create() {
    const itemName = document.getElementById('itemInput').value.trim();
    if (isValidName(itemName)) {
        window.dialogAPI.sendItemNameResult({
            name: itemName,
            type: itemConfig.type
        });
    }
}

function cancel() {
    window.dialogAPI.sendItemNameResult(null);
}

// Configuration for different item types
const typeConfigs = {
    node: {
        title: 'Create New Node',
        label: 'Node Name:',
        placeholder: 'e.g., simple_publisher',
        hint: 'Lowercase letters, numbers, and underscores only. Must start with a letter.'
    },
    urdf: {
        title: 'Create New URDF',
        label: 'URDF Name:',
        placeholder: 'e.g., robot_arm',
        hint: 'Lowercase letters, numbers, and underscores only. Must start with a letter.'
    },
    config: {
        title: 'Create New Config',
        label: 'Config Name:',
        placeholder: 'e.g., params',
        hint: 'Lowercase letters, numbers, and underscores only. Must start with a letter.'
    },
    launch: {
        title: 'Create New Launch File',
        label: 'Launch Name:',
        placeholder: 'e.g., bringup',
        hint: 'Lowercase letters, numbers, and underscores only. Must start with a letter.'
    }
};

function applyConfig(config) {
    const typeConfig = typeConfigs[config.type] || typeConfigs.node;

    document.getElementById('dialog-title').textContent = typeConfig.title;
    document.getElementById('input-label').textContent = typeConfig.label;
    document.getElementById('itemInput').placeholder = typeConfig.placeholder;
    document.getElementById('hint-text').textContent = typeConfig.hint;
    document.getElementById('package-name').textContent = config.packageName;

    itemConfig = config;
}

document.addEventListener('DOMContentLoaded', () => {
    const itemInput = document.getElementById('itemInput');
    const createBtn = document.getElementById('create-btn');
    const cancelBtn = document.getElementById('cancel-btn');
    const errorMsg = document.getElementById('error-msg');

    // Get configuration from main process
    window.dialogAPI.onSetItemConfig((config) => {
        applyConfig(config);
    });

    createBtn.addEventListener('click', create);
    cancelBtn.addEventListener('click', cancel);

    itemInput.addEventListener('input', () => {
        const value = itemInput.value.trim();
        if (value === '') {
            createBtn.disabled = true;
            errorMsg.style.display = 'none';
        } else if (isValidName(value)) {
            createBtn.disabled = false;
            errorMsg.style.display = 'none';
        } else {
            createBtn.disabled = true;
            errorMsg.style.display = 'block';
        }
    });

    itemInput.addEventListener('keypress', (e) => {
        if (e.key === 'Enter' && !createBtn.disabled) {
            create();
        }
    });
});
