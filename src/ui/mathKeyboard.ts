/**
 * Desmos Graphing Calculator on-screen keypad — fixed dock, 1:1 layout.
 * Reference: https://www.desmos.com/calculator (screenshots in /opt/cursor/artifacts/desmos_*)
 */

export type MathFieldLike = {
  latex: (s?: string) => string;
  focus: () => void;
  write?: (latex: string) => void;
  cmd?: (latex: string) => void;
  keystroke?: (keys: string) => void;
  typedText?: (text: string) => void;
};

type KeyAction =
  | { kind: 'text'; text: string; alt?: string }
  | { kind: 'cmd'; latex: string }
  | { kind: 'write'; latex: string }
  | { kind: 'key'; key: string }
  | { kind: 'tab'; tab: 'main' | 'abc' }
  | { kind: 'functions' }
  | { kind: 'shift' }
  | { kind: 'enter' }
  | { kind: 'noop' };

type KeySpec = {
  label: string;
  title?: string;
  action: KeyAction;
  tone?: 'white' | 'gray' | 'blue';
  /** column units in a 20-column grid (default 2 = one “standard” key) */
  units?: number;
  html?: string;
  italic?: boolean;
  icon?: 'backspace' | 'enter' | 'left' | 'right' | 'shift' | 'audio' | 'hide';
};

type FuncItem = { label: string; latex: string; mode: 'cmd' | 'write' };

const FUNC_GROUPS: { title: string; items: FuncItem[] }[] = [
  {
    title: 'TRIG',
    items: [
      { label: 'sin', latex: '\\sin', mode: 'cmd' },
      { label: 'cos', latex: '\\cos', mode: 'cmd' },
      { label: 'tan', latex: '\\tan', mode: 'cmd' },
      { label: 'csc', latex: '\\csc', mode: 'write' },
      { label: 'sec', latex: '\\sec', mode: 'write' },
      { label: 'cot', latex: '\\cot', mode: 'write' },
    ],
  },
  {
    title: 'INVERSE TRIG',
    items: [
      { label: 'arcsin', latex: '\\arcsin', mode: 'write' },
      { label: 'arccos', latex: '\\arccos', mode: 'write' },
      { label: 'arctan', latex: '\\arctan', mode: 'write' },
    ],
  },
  {
    title: 'HYPERBOLIC',
    items: [
      { label: 'sinh', latex: '\\sinh', mode: 'write' },
      { label: 'cosh', latex: '\\cosh', mode: 'write' },
      { label: 'tanh', latex: '\\tanh', mode: 'write' },
    ],
  },
  {
    title: 'CALCULUS',
    items: [
      { label: 'exp', latex: '\\exp', mode: 'write' },
      { label: 'ln', latex: '\\ln', mode: 'cmd' },
      { label: 'log', latex: '\\log', mode: 'cmd' },
      { label: '√', latex: '\\sqrt', mode: 'cmd' },
      { label: '∑', latex: '\\sum', mode: 'cmd' },
      { label: '∏', latex: '\\prod', mode: 'cmd' },
      { label: '∫', latex: '\\int', mode: 'cmd' },
      { label: '∂', latex: '\\partial', mode: 'cmd' },
    ],
  },
  {
    title: 'STATS',
    items: [
      { label: 'nCr', latex: '\\operatorname{nCr}', mode: 'write' },
      { label: 'nPr', latex: '\\operatorname{nPr}', mode: 'write' },
      { label: 'mean', latex: '\\operatorname{mean}', mode: 'write' },
      { label: 'median', latex: '\\operatorname{median}', mode: 'write' },
      { label: 'stdev', latex: '\\operatorname{stdev}', mode: 'write' },
      { label: 'var', latex: '\\operatorname{var}', mode: 'write' },
    ],
  },
  {
    title: 'MISC',
    items: [
      { label: 'floor', latex: '\\lfloor', mode: 'cmd' },
      { label: 'ceil', latex: '\\lceil', mode: 'cmd' },
      { label: 'round', latex: '\\operatorname{round}', mode: 'write' },
      { label: 'abs', latex: '|', mode: 'cmd' },
      { label: 'gcd', latex: '\\gcd', mode: 'write' },
      { label: 'lcm', latex: '\\operatorname{lcm}', mode: 'write' },
      { label: 'sign', latex: '\\operatorname{sign}', mode: 'write' },
      { label: 'mod', latex: '\\bmod', mode: 'write' },
    ],
  },
];

const ICONS: Record<NonNullable<KeySpec['icon']>, string> = {
  backspace:
    '<svg viewBox="0 0 24 24" width="18" height="18" aria-hidden="true"><path fill="currentColor" d="M22 4H8l-6 8 6 8h14V4zm-3.5 10.6L17 16l-2.5-2.5L12 16l-1.5-1.4L13 12l-2.5-2.6L12 8l2.5 2.5L17 8l1.5 1.4L16 12l2.5 2.6z"/></svg>',
  enter:
    '<svg viewBox="0 0 24 24" width="18" height="18" aria-hidden="true"><path fill="currentColor" d="M19 7v6H8.8l2.1-2.1L9.5 9.5 5 14l4.5 4.5 1.4-1.4L8.8 15H21V7z"/></svg>',
  left:
    '<svg viewBox="0 0 24 24" width="16" height="16" aria-hidden="true"><path fill="currentColor" d="M15.4 7.4 14 6l-6 6 6 6 1.4-1.4L10.8 12z"/></svg>',
  right:
    '<svg viewBox="0 0 24 24" width="16" height="16" aria-hidden="true"><path fill="currentColor" d="M10 6 8.6 7.4 13.2 12l-4.6 4.6L10 18l6-6z"/></svg>',
  shift:
    '<svg viewBox="0 0 24 24" width="16" height="16" aria-hidden="true"><path fill="currentColor" d="M12 5 4 13h5v6h6v-6h5L12 5z"/></svg>',
  audio:
    '<svg viewBox="0 0 24 24" width="16" height="16" aria-hidden="true"><path fill="currentColor" d="M4 10v4h3l4 3V7L7 10H4zm11.5 2a3.5 3.5 0 0 0-1.8-3.1v6.2A3.5 3.5 0 0 0 15.5 12zm-1.8-6.9v2.1a5.5 5.5 0 0 1 0 9.6v2.1a7.5 7.5 0 0 0 0-13.8z"/></svg>',
  hide:
    '<svg viewBox="0 0 24 24" width="14" height="14" aria-hidden="true"><path fill="currentColor" d="M4 6h16v2H4V6zm2 4h12v8H6v-8zm2 2v4h8v-4H8z"/></svg>',
};

/** Desmos main keypad — 4 rows × 10 standard keys (20 grid units). */
function mainKeys(): KeySpec[] {
  const u = (units: number) => units;
  return [
    // row 1
    { label: 'x', action: { kind: 'text', text: 'x' }, tone: 'white', italic: true },
    { label: 'y', action: { kind: 'text', text: 'y' }, tone: 'white', italic: true },
    { label: 'a²', html: '<i>a</i><sup>2</sup>', action: { kind: 'cmd', latex: '^2' }, tone: 'white', title: 'square' },
    { label: 'aᵇ', html: '<i>a</i><sup><i>b</i></sup>', action: { kind: 'cmd', latex: '^' }, tone: 'white', title: 'exponent' },
    { label: '7', action: { kind: 'text', text: '7' }, tone: 'white' },
    { label: '8', action: { kind: 'text', text: '8' }, tone: 'white' },
    { label: '9', action: { kind: 'text', text: '9' }, tone: 'white' },
    { label: '÷', action: { kind: 'cmd', latex: '\\div' }, tone: 'gray' },
    { label: 'functions', action: { kind: 'functions' }, tone: 'gray', units: u(4) },

    // row 2
    { label: '(', action: { kind: 'cmd', latex: '(' }, tone: 'white' },
    { label: ')', action: { kind: 'cmd', latex: ')' }, tone: 'white' },
    { label: '<', action: { kind: 'text', text: '<' }, tone: 'white' },
    { label: '>', action: { kind: 'text', text: '>' }, tone: 'white' },
    { label: '4', action: { kind: 'text', text: '4' }, tone: 'white' },
    { label: '5', action: { kind: 'text', text: '5' }, tone: 'white' },
    { label: '6', action: { kind: 'text', text: '6' }, tone: 'white' },
    { label: '×', action: { kind: 'cmd', latex: '\\times' }, tone: 'gray' },
    { label: '←', action: { kind: 'key', key: 'Left' }, tone: 'gray', icon: 'left', title: 'Left' },
    { label: '→', action: { kind: 'key', key: 'Right' }, tone: 'gray', icon: 'right', title: 'Right' },

    // row 3
    { label: '|a|', html: '|<i>a</i>|', action: { kind: 'cmd', latex: '|' }, tone: 'white', title: 'absolute value' },
    { label: ',', action: { kind: 'text', text: ',' }, tone: 'white' },
    { label: '≤', action: { kind: 'cmd', latex: '\\le' }, tone: 'white' },
    { label: '≥', action: { kind: 'cmd', latex: '\\ge' }, tone: 'white' },
    { label: '1', action: { kind: 'text', text: '1' }, tone: 'white' },
    { label: '2', action: { kind: 'text', text: '2' }, tone: 'white' },
    { label: '3', action: { kind: 'text', text: '3' }, tone: 'white' },
    { label: '−', action: { kind: 'text', text: '-' }, tone: 'gray' },
    { label: '⌫', action: { kind: 'key', key: 'Backspace' }, tone: 'gray', units: u(4), icon: 'backspace', title: 'Backspace' },

    // row 4
    { label: 'ABC', action: { kind: 'tab', tab: 'abc' }, tone: 'gray' },
    { label: 'audio', action: { kind: 'noop' }, tone: 'gray', icon: 'audio', title: 'Audio (Desmos)' },
    { label: '√', action: { kind: 'cmd', latex: '\\sqrt' }, tone: 'white', title: 'square root' },
    { label: 'π', action: { kind: 'cmd', latex: '\\pi' }, tone: 'white', italic: true },
    { label: '0', action: { kind: 'text', text: '0' }, tone: 'white' },
    { label: '.', action: { kind: 'text', text: '.' }, tone: 'white' },
    { label: '=', action: { kind: 'text', text: '=' }, tone: 'white' },
    { label: '+', action: { kind: 'text', text: '+' }, tone: 'gray' },
    { label: '⏎', action: { kind: 'enter' }, tone: 'blue', units: u(4), icon: 'enter', title: 'Enter' },
  ];
}

function abcKeys(shiftOn: boolean): KeySpec[] {
  const L = (ch: string): KeySpec => {
    const t = shiftOn ? ch.toUpperCase() : ch;
    return {
      label: t,
      action: { kind: 'text', text: t },
      tone: 'white',
      italic: true,
    };
  };
  const dual = (primary: string, secondary: string, displayHtml: string): KeySpec => ({
    label: primary,
    html: displayHtml,
    action: { kind: 'text', text: primary, alt: secondary },
    tone: 'white',
    title: `${primary} / ${secondary}`,
  });
  return [
    // row 1
    L('q'), L('w'), L('e'), L('r'), L('t'), L('y'), L('u'), L('i'), L('o'), L('p'),
    // row 2
    L('a'), L('s'), L('d'), L('f'), L('g'), L('h'), L('j'), L('k'), L('l'),
    { label: 'θ', action: { kind: 'cmd', latex: '\\theta' }, tone: 'white', italic: true },
    // row 3 — Shift ~1.5, 7 letters, Backspace ~1.5  → 3+14+3 = 20
    { label: '⇧', action: { kind: 'shift' }, tone: 'gray', units: 3, icon: 'shift', title: 'Shift' },
    L('z'), L('x'), L('c'), L('v'), L('b'), L('n'), L('m'),
    { label: '⌫', action: { kind: 'key', key: 'Backspace' }, tone: 'gray', units: 3, icon: 'backspace', title: 'Backspace' },
    // row 4 — 123 ~2, 6 symbols, Enter ~4?  Desmos: 123 wider, Enter ~2× → use 3 + 6×2 + 5 = 20
    { label: '123', html: '1 2 3', action: { kind: 'tab', tab: 'main' }, tone: 'gray', units: 3 },
    { label: 'aₙ', html: '<i>a</i><sub><i>b</i></sub>', action: { kind: 'cmd', latex: '_' }, tone: 'white', title: 'subscript' },
    dual('!', "'", "! <span class=\"dsk-sec\">'</span>"),
    dual('[', ']', "[ <span class=\"dsk-sec\">]</span>"),
    {
      label: '{',
      html: '{ <span class="dsk-sec">}</span>',
      action: { kind: 'cmd', latex: '\\{' },
      tone: 'white',
      title: '{ / }',
    },
    dual('~', '%', "~ <span class=\"dsk-sec\">%</span>"),
    dual(':', ';', ": <span class=\"dsk-sec\">;</span>"),
    { label: '⏎', action: { kind: 'enter' }, tone: 'blue', units: 5, icon: 'enter', title: 'Enter' },
  ];
}

let activeField: MathFieldLike | null = null;
let tab: 'main' | 'abc' = 'main';
let shiftOn = false;
let functionsOpen = false;
let dockEl: HTMLElement | null = null;
let showBtnEl: HTMLElement | null = null;
let visible = false;
let userHidden = false;

export function setActiveMathField(field: MathFieldLike | null, el?: HTMLElement | null) {
  activeField = field;
  document.querySelectorAll('.mq-editable.mq-kb-target').forEach((n) => n.classList.remove('mq-kb-target'));
  if (el) el.classList.add('mq-kb-target');
}

export function getActiveMathField() {
  return activeField;
}

function ensureActiveField(): MathFieldLike | null {
  if (activeField) return activeField;
  const scratch = document.querySelector('.an-scratch [data-mq="edit"]') as HTMLElement | null;
  if (scratch && (scratch as any).mq) setActiveMathField((scratch as any).mq, scratch);
  return activeField;
}

function applyAction(action: KeyAction) {
  if (action.kind === 'tab') {
    tab = action.tab;
    shiftOn = false;
    functionsOpen = false;
    renderDock();
    return;
  }
  if (action.kind === 'shift') {
    shiftOn = !shiftOn;
    renderDock();
    return;
  }
  if (action.kind === 'functions') {
    functionsOpen = !functionsOpen;
    renderDock();
    return;
  }
  if (action.kind === 'noop') return;

  const f = ensureActiveField();
  if (!f) return;
  try { f.focus?.(); } catch { /* */ }

  if (action.kind === 'enter') {
    if (f.keystroke) f.keystroke('Enter');
    else if (f.write) f.write('\\\\');
    return;
  }
  if (action.kind === 'key') {
    f.keystroke?.(action.key);
    return;
  }
  if (action.kind === 'text') {
    const useAlt = !!(action.alt && shiftOn);
    const text = useAlt ? action.alt! : action.text;
    if (f.typedText) f.typedText(text);
    else if (f.write) f.write(text);
    else f.latex((f.latex() || '') + text);
    if (shiftOn && tab === 'abc') {
      shiftOn = false;
      renderDock();
    }
    return;
  }
  if (action.kind === 'cmd') {
    if (f.cmd) f.cmd(action.latex);
    else f.write?.(action.latex);
    return;
  }
  if (action.kind === 'write') {
    if (f.write) f.write(action.latex);
    else f.cmd?.(action.latex);
  }
}

function applyFunc(item: FuncItem) {
  applyAction(item.mode === 'cmd' ? { kind: 'cmd', latex: item.latex } : { kind: 'write', latex: item.latex });
  functionsOpen = false;
  renderDock();
}

function makeKeyButton(spec: KeySpec) {
  const units = spec.units ?? 2;
  const b = document.createElement('button');
  b.type = 'button';
  b.className = `dsk-key dsk-${spec.tone || 'white'}${spec.italic ? ' dsk-italic' : ''}`;
  b.style.gridColumn = `span ${units}`;
  if (spec.icon) b.innerHTML = ICONS[spec.icon];
  else if (spec.html) b.innerHTML = spec.html;
  else b.textContent = spec.label;
  if (spec.title) b.title = spec.title;
  if (spec.action.kind === 'shift' && shiftOn) b.classList.add('dsk-active');
  if (spec.action.kind === 'functions' && functionsOpen) b.classList.add('dsk-active');
  b.addEventListener('mousedown', (e) => e.preventDefault());
  b.addEventListener('click', (e) => {
    e.preventDefault();
    e.stopPropagation();
    applyAction(spec.action);
  });
  return b;
}

function renderFunctionsPopup(host: HTMLElement) {
  const pop = document.createElement('div');
  pop.className = 'dsk-functions';
  for (const g of FUNC_GROUPS) {
    const h = document.createElement('div');
    h.className = 'dsk-func-title';
    h.textContent = g.title;
    pop.appendChild(h);
    const row = document.createElement('div');
    row.className = 'dsk-func-row';
    for (const item of g.items) {
      const b = document.createElement('button');
      b.type = 'button';
      b.className = 'dsk-func-item';
      b.textContent = item.label;
      b.addEventListener('mousedown', (e) => e.preventDefault());
      b.addEventListener('click', (e) => {
        e.preventDefault();
        e.stopPropagation();
        applyFunc(item);
      });
      row.appendChild(b);
    }
    pop.appendChild(row);
  }
  host.appendChild(pop);
}

function renderDock() {
  if (!dockEl) return;
  const body = dockEl.querySelector('.dsk-body') as HTMLElement;
  if (!body) return;
  body.innerHTML = '';

  if (functionsOpen) renderFunctionsPopup(body);

  const wrap = document.createElement('div');
  wrap.className = 'dsk-wrap';
  const grid = document.createElement('div');
  grid.className = 'dsk-grid';
  const keys = tab === 'main' ? mainKeys() : abcKeys(shiftOn);
  for (const k of keys) grid.appendChild(makeKeyButton(k));
  wrap.appendChild(grid);
  body.appendChild(wrap);
}

function keypadRoot(): HTMLElement | null {
  return document.getElementById('pageAnalysis') || document.getElementById('right');
}

function syncShowButton() {
  if (!showBtnEl) return;
  const host = dockEl?.parentElement;
  const analysis = !!host?.classList.contains('analysis-keypad');
  showBtnEl.hidden = !(analysis && !visible);
  showBtnEl.classList.toggle('open', analysis && !visible);
}

/** Ensure a single fixed Desmos keypad dock under #right. */
export function ensureDesmosKeypadDock(rightEl: HTMLElement = document.getElementById('right') as HTMLElement) {
  if (!rightEl) return null;

  let dock = rightEl.querySelector('#desmosKeypadDock') as HTMLElement | null;
  if (!dock) {
    dock = document.createElement('div');
    dock.id = 'desmosKeypadDock';
    dock.className = 'desmos-keypad-dock';
    dock.hidden = true;
    dock.innerHTML = `
      <button type="button" class="dsk-hide" title="隐藏键盘" aria-label="隐藏键盘">
        ${ICONS.hide}<span class="dsk-hide-arrow">▾</span>
      </button>
      <div class="dsk-body"></div>
    `;
    rightEl.appendChild(dock);
    dock.querySelector('.dsk-hide')?.addEventListener('click', (e) => {
      e.preventDefault();
      userHidden = true;
      setDesmosKeypadVisible(false);
    });
  }

  let showBtn = rightEl.querySelector('#desmosKeypadShow') as HTMLButtonElement | null;
  if (!showBtn) {
    showBtn = document.createElement('button');
    showBtn.id = 'desmosKeypadShow';
    showBtn.type = 'button';
    showBtn.className = 'dsk-show';
    showBtn.title = '显示键盘';
    showBtn.setAttribute('aria-label', '显示键盘');
    showBtn.innerHTML = `${ICONS.hide}<span class="dsk-hide-arrow">▴</span>`;
    showBtn.hidden = true;
    rightEl.appendChild(showBtn);
    showBtn.addEventListener('click', (e) => {
      e.preventDefault();
      userHidden = false;
      setDesmosKeypadVisible(true);
    });
  }

  dockEl = dock;
  showBtnEl = showBtn;
  renderDock();
  syncShowButton();
  return dock;
}

export function setDesmosKeypadVisible(show: boolean) {
  visible = show;
  if (!dockEl) ensureDesmosKeypadDock(keypadRoot() ?? undefined);
  if (!dockEl) return;
  dockEl.classList.toggle('open', show);
  dockEl.hidden = !show;
  dockEl.parentElement?.classList.toggle('with-keypad', show);
  if (show) {
    functionsOpen = false;
    renderDock();
  }
  syncShowButton();
}

/** Show fixed keypad only on Analysis tab; respect user hide. */
export function syncDesmosKeypadForAnalysis(onAnalysisTab: boolean) {
  const right = keypadRoot();
  if (!right) return;
  ensureDesmosKeypadDock(right);
  right.classList.toggle('analysis-keypad', onAnalysisTab);
  if (!onAnalysisTab) {
    setDesmosKeypadVisible(false);
    if (showBtnEl) showBtnEl.hidden = true;
    return;
  }
  if (userHidden) {
    setDesmosKeypadVisible(false);
  } else {
    setDesmosKeypadVisible(true);
  }
}

export function isDesmosKeypadVisible() {
  return visible;
}

/** @deprecated */
export function mountMathKeyboard(_container: HTMLElement) {
  syncDesmosKeypadForAnalysis(true);
}
