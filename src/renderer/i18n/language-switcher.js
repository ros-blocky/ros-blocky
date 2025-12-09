/**
 * Language Switcher Component
 * Dropdown UI for changing application language
 */

import {
    getCurrentLanguage,
    getAvailableLanguages,
    changeLanguage
} from './index.js';
import { getLanguageInfo } from './i18n-config.js';

/**
 * Create language switcher HTML element
 * @param {string} containerId - ID of container to append switcher to
 * @returns {HTMLElement}
 */
export function createLanguageSwitcher(containerId = null) {
    const currentLang = getCurrentLanguage();
    const languages = getAvailableLanguages();
    const currentInfo = getLanguageInfo(currentLang);

    // Create switcher container
    const switcher = document.createElement('div');
    switcher.className = 'language-switcher';
    switcher.id = 'language-switcher';

    // Create current language button
    const currentBtn = document.createElement('button');
    currentBtn.className = 'language-switcher-btn';
    currentBtn.innerHTML = `
        <span class="lang-flag">${currentInfo.flag}</span>
        <span class="lang-code">${currentLang.toUpperCase()}</span>
        <span class="lang-arrow">▾</span>
    `;

    // Create dropdown
    const dropdown = document.createElement('div');
    dropdown.className = 'language-dropdown hidden';

    // Add language options
    languages.forEach(lang => {
        const option = document.createElement('div');
        option.className = 'language-option';
        if (lang.code === currentLang) {
            option.classList.add('active');
        }
        option.dataset.lang = lang.code;
        option.innerHTML = `
            <span class="lang-flag">${lang.flag}</span>
            <span class="lang-name">${lang.nativeName}</span>
        `;

        option.addEventListener('click', async () => {
            await selectLanguage(lang.code);
            dropdown.classList.add('hidden');
        });

        dropdown.appendChild(option);
    });

    // Toggle dropdown on button click
    currentBtn.addEventListener('click', (e) => {
        e.stopPropagation();
        dropdown.classList.toggle('hidden');
    });

    // Close dropdown when clicking outside
    document.addEventListener('click', () => {
        dropdown.classList.add('hidden');
    });

    switcher.appendChild(currentBtn);
    switcher.appendChild(dropdown);

    // Append to container if provided
    if (containerId) {
        const container = document.getElementById(containerId);
        if (container) {
            container.appendChild(switcher);
        }
    }

    return switcher;
}

/**
 * Select a language and update the switcher UI
 * @param {string} langCode 
 */
async function selectLanguage(langCode) {
    await changeLanguage(langCode);
    updateSwitcherUI(langCode);
}

/**
 * Update the switcher button to show current language
 * @param {string} langCode 
 */
function updateSwitcherUI(langCode) {
    const switcher = document.getElementById('language-switcher');
    if (!switcher) return;

    const langInfo = getLanguageInfo(langCode);
    const btn = switcher.querySelector('.language-switcher-btn');

    if (btn) {
        btn.innerHTML = `
            <span class="lang-flag">${langInfo.flag}</span>
            <span class="lang-code">${langCode.toUpperCase()}</span>
            <span class="lang-arrow">▾</span>
        `;
    }

    // Update active state in dropdown
    const options = switcher.querySelectorAll('.language-option');
    options.forEach(option => {
        if (option.dataset.lang === langCode) {
            option.classList.add('active');
        } else {
            option.classList.remove('active');
        }
    });
}

/**
 * Get CSS styles for the language switcher
 * @returns {string}
 */
export function getLanguageSwitcherStyles() {
    return `
        .language-switcher {
            position: relative;
            display: inline-flex;
            align-items: center;
            margin-right: 8px;
        }

        .language-switcher-btn {
            display: flex;
            align-items: center;
            gap: 4px;
            padding: 4px 8px;
            background: transparent;
            border: none;
            border-radius: 4px;
            color: white;
            cursor: pointer;
            font-size: 12px;
            font-weight: 500;
            transition: background 0.2s ease;
            height: 24px;
        }

        .language-switcher-btn:hover {
            background: rgba(255, 255, 255, 0.15);
        }

        .lang-flag {
            font-size: 14px;
            line-height: 1;
        }

        .lang-code {
            font-weight: 600;
            font-size: 11px;
            letter-spacing: 0.5px;
        }

        .lang-arrow {
            font-size: 8px;
            opacity: 0.8;
            margin-left: 2px;
        }

        .language-dropdown {
            position: absolute;
            top: calc(100% + 4px);
            right: 0;
            min-width: 140px;
            background: #ffffff;
            border-radius: 8px;
            box-shadow: 0 4px 16px rgba(0, 0, 0, 0.15), 0 1px 3px rgba(0, 0, 0, 0.1);
            z-index: 10000;
            overflow: hidden;
            border: 1px solid rgba(0, 0, 0, 0.08);
        }

        [dir="rtl"] .language-dropdown {
            right: auto;
            left: 0;
        }

        .language-dropdown.hidden {
            display: none;
        }

        .language-option {
            display: flex;
            align-items: center;
            gap: 10px;
            padding: 10px 14px;
            cursor: pointer;
            transition: background 0.15s ease;
            color: #333333;
        }

        .language-option:hover {
            background: #f0f0f0;
        }

        .language-option.active {
            background: #00bbfe;
            color: white;
        }

        .language-option.active .lang-name {
            color: white;
        }

        .lang-name {
            font-size: 13px;
            font-weight: 500;
            color: #333333;
        }
    `;
}
