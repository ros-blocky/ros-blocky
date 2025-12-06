/**
 * Blocks Component
 * Handles the visual block-based programming interface
 */

export function initBlocks() {
    console.log('Blocks component initialized');

    // Setup tab switching
    setupTabs();

    // Setup category selection (placeholder for future implementation)
    setupCategories();
}

/**
 * Setup tab switching functionality
 */
function setupTabs() {
    const tabs = document.querySelectorAll('.blocks-tab');
    const placeholder = document.querySelector('.blocks-placeholder');

    tabs.forEach(tab => {
        tab.addEventListener('click', () => {
            // Remove active from all tabs
            tabs.forEach(t => t.classList.remove('active'));

            // Add active to clicked tab
            tab.classList.add('active');

            // Update placeholder text
            if (placeholder) {
                const tabName = tab.textContent;
                placeholder.querySelector('h2').textContent = tabName;

                if (tabName === 'URDF Editor') {
                    placeholder.querySelector('p').textContent = "Design your robot's structure here";
                } else if (tabName === 'Programming') {
                    placeholder.querySelector('p').textContent = "Create your robot's behavior here";
                } else if (tabName === 'Launch Files') {
                    placeholder.querySelector('p').textContent = "Configure your robot's launch settings here";
                }
            }
        });
    });
}

/**
 * Setup category selection (placeholder)
 */
function setupCategories() {
    const categories = document.querySelectorAll('.blocks-category');

    categories.forEach(category => {
        category.addEventListener('click', () => {
            console.log('Category clicked:', category.querySelector('.blocks-category-label').textContent);
            // TODO: Implement block category logic
        });
    });
}
