/**
 * UI theme: CSS custom properties, dark mode.
 */

export function injectTheme(): void {
  const style = document.createElement('style');
  style.textContent = `
    :root {
      --bg-primary: #f5f5f8;
      --bg-secondary: #ffffff;
      --bg-tertiary: #eaeaf0;
      --bg-hover: #dddde5;
      --border: #c8c8d4;
      --text-primary: #1a1a2e;
      --text-secondary: #555570;
      --text-muted: #8888a0;
      --accent: #2a6ecc;
      --accent-dim: #1a5eaa;
      --success: #22aa55;
      --warning: #cc8800;
      --error: #cc3344;
      --stub-color: #888899;
      --simplified-color: #cc8800;
      --experimental-color: #2a6ecc;
      --validated-color: #22aa55;
      --font-ui: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
      --font-mono: 'SF Mono', 'Cascadia Code', 'Fira Code', 'Consolas', monospace;
      --font-size-xs: 10px;
      --font-size-sm: 12px;
      --font-size-md: 14px;
      --radius: 4px;
      --panel-padding: 10px;
    }

    .panel {
      background: var(--bg-secondary);
      border: 1px solid var(--border);
      border-radius: var(--radius);
      padding: var(--panel-padding);
      overflow-y: auto;
    }

    .panel-title {
      font-size: var(--font-size-md);
      font-weight: 600;
      color: var(--text-primary);
      margin-bottom: 8px;
      text-transform: uppercase;
      letter-spacing: 0.5px;
    }

    .param-row {
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 3px 0;
      font-size: var(--font-size-sm);
    }

    .param-label {
      color: var(--text-secondary);
      flex-shrink: 0;
      min-width: 90px;
    }

    .param-value {
      color: var(--text-primary);
      font-variant-numeric: tabular-nums;
      text-align: right;
    }

    .param-input {
      background: var(--bg-tertiary);
      border: 1px solid var(--border);
      color: var(--text-primary);
      font-family: var(--font-mono);
      font-size: var(--font-size-sm);
      padding: 2px 6px;
      border-radius: var(--radius);
      width: 80px;
      text-align: right;
    }
    .param-input:focus {
      outline: none;
      border-color: var(--accent);
    }

    .btn {
      background: var(--bg-tertiary);
      border: 1px solid var(--border);
      color: var(--text-primary);
      font-family: var(--font-ui);
      font-size: var(--font-size-sm);
      padding: 4px 12px;
      border-radius: var(--radius);
      cursor: pointer;
      transition: background 0.15s;
    }
    .btn:hover { background: var(--bg-hover); }
    .btn:active { background: var(--accent-dim); }
    .btn.active { background: var(--accent-dim); border-color: var(--accent); }

    .status-badge {
      display: inline-block;
      padding: 1px 6px;
      border-radius: 3px;
      font-size: 10px;
      font-weight: 600;
      text-transform: uppercase;
    }
    .status-badge.stub { background: var(--stub-color); color: #fff; }
    .status-badge.simplified { background: var(--simplified-color); color: #000; }
    .status-badge.experimental { background: var(--experimental-color); color: #000; }
    .status-badge.validated { background: var(--validated-color); color: #000; }

    .slider-container {
      display: flex;
      align-items: center;
      gap: 6px;
    }

    input[type="range"] {
      -webkit-appearance: none;
      appearance: none;
      height: 4px;
      background: var(--border);
      border-radius: 2px;
      outline: none;
      flex: 1;
    }
    input[type="range"]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 12px;
      height: 12px;
      border-radius: 50%;
      background: var(--accent);
      cursor: pointer;
    }

    .warning-text { color: var(--warning); font-weight: 600; }
    .error-text { color: var(--error); font-weight: 600; }

    .collapsible-header {
      cursor: pointer;
      user-select: none;
      display: flex;
      align-items: center;
      gap: 4px;
    }
    .collapsible-header::before {
      content: '▸';
      font-size: 10px;
      transition: transform 0.15s;
    }
    .collapsible-header.open::before {
      transform: rotate(90deg);
    }

    .panel-title {
      cursor: pointer;
      user-select: none;
    }
    .panel-title::before {
      content: '▾ ';
      font-size: 10px;
    }
    .panel-title.collapsed::before {
      content: '▸ ';
    }
    .panel-body { transition: none; }
    .panel-body.collapsed { display: none; }
  `;
  document.head.appendChild(style);
}

/** Make a panel collapsible. Wraps all children after the title in a toggle body. */
export function makeCollapsible(container: HTMLDivElement, startOpen = true): void {
  const title = container.querySelector('.panel-title');
  if (!title) return;

  // Wrap everything after title in a body div
  const body = document.createElement('div');
  body.className = 'panel-body' + (startOpen ? '' : ' collapsed');
  if (!startOpen) title.classList.add('collapsed');

  const children = Array.from(container.childNodes).filter(n => n !== title);
  for (const child of children) body.appendChild(child);
  container.appendChild(body);

  title.addEventListener('click', () => {
    body.classList.toggle('collapsed');
    title.classList.toggle('collapsed');
  });
}
