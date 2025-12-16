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

// Configuration for different item types - uses i18n keys
const typeConfigs = {
    node: {
        titleKey: 'dialogs.createNewNode',
        labelKey: 'dialogs.nodeName',
        placeholderKey: 'dialogs.nodeNamePlaceholder',
        hintKey: 'dialogs.itemNameHint'
    },
    urdf: {
        titleKey: 'dialogs.createNewUrdf',
        labelKey: 'dialogs.urdfName',
        placeholderKey: 'dialogs.urdfNamePlaceholder',
        hintKey: 'dialogs.itemNameHint'
    },
    config: {
        titleKey: 'dialogs.createNewConfig',
        labelKey: 'dialogs.configName',
        placeholderKey: 'dialogs.configNamePlaceholder',
        hintKey: 'dialogs.itemNameHint'
    },
    launch: {
        titleKey: 'dialogs.createNewLaunch',
        labelKey: 'dialogs.launchName',
        placeholderKey: 'dialogs.launchNamePlaceholder',
        hintKey: 'dialogs.itemNameHint'
    }
};

function applyConfig(config) {
    const typeConfig = typeConfigs[config.type] || typeConfigs.node;

    // Update data-i18n attributes and let dialogI18n handle the translations
    document.getElementById('dialog-title').setAttribute('data-i18n', typeConfig.titleKey);
    document.getElementById('input-label').setAttribute('data-i18n', typeConfig.labelKey);
    document.getElementById('itemInput').setAttribute('data-i18n-placeholder', typeConfig.placeholderKey);
    document.getElementById('hint-text').setAttribute('data-i18n', typeConfig.hintKey);
    document.getElementById('package-name').textContent = config.packageName;

    itemConfig = config;

    // Re-apply translations after updating attributes
    if (window.dialogI18n && window.dialogI18n.update) {
        window.dialogI18n.update();
    }
}

document.addEventListener('DOMContentLoaded', async () => {
    // Initialize translations
    if (window.dialogI18n) {
        await window.dialogI18n.init();
    }

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
