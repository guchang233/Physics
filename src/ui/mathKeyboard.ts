/**
 * Desmos-style math keyboard for MathQuill MathFields.
 * Tracks the last-focused editable field and injects cmds / latex / keystrokes.
 */

export type MathFieldLike = {
  latex: (s?: string) => string;
  focus: () => void;
  write?: (latex: string) => void;
  cmd?: (latex: string) => void;
  keystroke?: (keys: string) => void;
  typedText?: (text: string) => void;
};

type KeyDef =
  | { label: string; title?: string; kind: 'write'; latex: string; wide?: boolean }
  | { label: string; title?: string; kind: 'cmd'; latex: string; wide?: boolean }
  | { label: string; title?: string; kind: 'key'; key: string; wide?: boolean }
  | { label: string; title?: string; kind: 'text'; text: string; wide?: boolean }
  | { label: string; title?: string; kind: 'clear'; wide?: boolean };

type PageDef = { id: string; name: string; keys: KeyDef[] };

const PAGES: PageDef[] = [
  {
    id: 'main',
    name: '主键盘',
    keys: [
      { label: '7', kind: 'text', text: '7' },
      { label: '8', kind: 'text', text: '8' },
      { label: '9', kind: 'text', text: '9' },
      { label: '÷', kind: 'cmd', latex: '\\div', title: '除' },
      { label: '←', kind: 'key', key: 'Left', title: '左移' },
      { label: '→', kind: 'key', key: 'Right', title: '右移' },

      { label: '4', kind: 'text', text: '4' },
      { label: '5', kind: 'text', text: '5' },
      { label: '6', kind: 'text', text: '6' },
      { label: '×', kind: 'cmd', latex: '\\times', title: '乘' },
      { label: '(', kind: 'cmd', latex: '(' },
      { label: ')', kind: 'cmd', latex: ')' },

      { label: '1', kind: 'text', text: '1' },
      { label: '2', kind: 'text', text: '2' },
      { label: '3', kind: 'text', text: '3' },
      { label: '−', kind: 'text', text: '-' },
      { label: 'a/b', kind: 'cmd', latex: '\\frac', title: '分数' },
      { label: '√', kind: 'cmd', latex: '\\sqrt', title: '平方根' },

      { label: '0', kind: 'text', text: '0' },
      { label: '.', kind: 'text', text: '.' },
      { label: '=', kind: 'text', text: '=' },
      { label: '+', kind: 'text', text: '+' },
      { label: 'xⁿ', kind: 'cmd', latex: '^', title: '指数' },
      { label: 'xₙ', kind: 'cmd', latex: '_', title: '下标' },

      { label: 'x', kind: 'text', text: 'x' },
      { label: 'y', kind: 'text', text: 'y' },
      { label: 't', kind: 'text', text: 't' },
      { label: 'τ', kind: 'cmd', latex: '\\tau' },
      { label: 'π', kind: 'cmd', latex: '\\pi' },
      { label: '⌫', kind: 'key', key: 'Backspace', title: '退格' },
    ],
  },
  {
    id: 'func',
    name: '函数',
    keys: [
      { label: 'sin', kind: 'cmd', latex: '\\sin' },
      { label: 'cos', kind: 'cmd', latex: '\\cos' },
      { label: 'tan', kind: 'cmd', latex: '\\tan' },
      { label: 'ln', kind: 'cmd', latex: '\\ln' },
      { label: 'log', kind: 'cmd', latex: '\\log' },
      { label: '|x|', kind: 'cmd', latex: '|', title: '绝对值' },

      { label: 'arcsin', kind: 'write', latex: '\\arcsin' },
      { label: 'arccos', kind: 'write', latex: '\\arccos' },
      { label: 'arctan', kind: 'write', latex: '\\arctan' },
      { label: 'exp', kind: 'write', latex: '\\exp' },
      { label: 'e^{}', kind: 'write', latex: 'e^{}', title: '自然指数' },
      { label: '10^{}', kind: 'write', latex: '10^{}' },

      { label: 'sinh', kind: 'write', latex: '\\sinh' },
      { label: 'cosh', kind: 'write', latex: '\\cosh' },
      { label: 'tanh', kind: 'write', latex: '\\tanh' },
      { label: 'floor', kind: 'write', latex: '\\lfloor\\rfloor' },
      { label: 'ceil', kind: 'write', latex: '\\lceil\\rceil' },
      { label: '⌫', kind: 'key', key: 'Backspace' },
    ],
  },
  {
    id: 'greek',
    name: '希腊',
    keys: [
      { label: 'α', kind: 'cmd', latex: '\\alpha' },
      { label: 'β', kind: 'cmd', latex: '\\beta' },
      { label: 'γ', kind: 'cmd', latex: '\\gamma' },
      { label: 'δ', kind: 'cmd', latex: '\\delta' },
      { label: 'ε', kind: 'cmd', latex: '\\varepsilon' },
      { label: 'θ', kind: 'cmd', latex: '\\theta' },

      { label: 'λ', kind: 'cmd', latex: '\\lambda' },
      { label: 'μ', kind: 'cmd', latex: '\\mu' },
      { label: 'ν', kind: 'cmd', latex: '\\nu' },
      { label: 'π', kind: 'cmd', latex: '\\pi' },
      { label: 'ρ', kind: 'cmd', latex: '\\rho' },
      { label: 'σ', kind: 'cmd', latex: '\\sigma' },

      { label: 'τ', kind: 'cmd', latex: '\\tau' },
      { label: 'φ', kind: 'cmd', latex: '\\phi' },
      { label: 'ω', kind: 'cmd', latex: '\\omega' },
      { label: 'Δ', kind: 'cmd', latex: '\\Delta' },
      { label: 'Ω', kind: 'cmd', latex: '\\Omega' },
      { label: 'Σ', kind: 'cmd', latex: '\\Sigma' },
    ],
  },
  {
    id: 'sym',
    name: '符号',
    keys: [
      { label: '·', kind: 'cmd', latex: '\\cdot' },
      { label: '×', kind: 'cmd', latex: '\\times' },
      { label: '÷', kind: 'cmd', latex: '\\div' },
      { label: '±', kind: 'cmd', latex: '\\pm' },
      { label: '≠', kind: 'cmd', latex: '\\ne' },
      { label: '≈', kind: 'cmd', latex: '\\approx' },

      { label: '≤', kind: 'cmd', latex: '\\le' },
      { label: '≥', kind: 'cmd', latex: '\\ge' },
      { label: '<', kind: 'text', text: '<' },
      { label: '>', kind: 'text', text: '>' },
      { label: '∞', kind: 'cmd', latex: '\\infty' },
      { label: '°', kind: 'write', latex: '^{\\circ}', title: '度' },

      { label: '→', kind: 'cmd', latex: '\\to' },
      { label: '⇒', kind: 'cmd', latex: '\\Rightarrow' },
      { label: '∈', kind: 'cmd', latex: '\\in' },
      { label: '⃗', kind: 'cmd', latex: '\\vec', title: '向量' },
      { label: '˙', kind: 'cmd', latex: '\\dot', title: '对时间导数' },
      { label: '¨', kind: 'cmd', latex: '\\ddot', title: '二阶导数' },

      { label: '∫', kind: 'cmd', latex: '\\int' },
      { label: '∑', kind: 'cmd', latex: '\\sum' },
      { label: '∂', kind: 'cmd', latex: '\\partial' },
      { label: '∇', kind: 'cmd', latex: '\\nabla' },
      { label: '√', kind: 'cmd', latex: '\\sqrt' },
      { label: '清空', kind: 'clear', wide: true, title: '清空当前输入框' },
    ],
  },
];

let activeField: MathFieldLike | null = null;
let currentPage = 'main';

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
  if (scratch && (scratch as any).mq) {
    setActiveMathField((scratch as any).mq as MathFieldLike, scratch);
  }
  return activeField;
}

function applyKey(def: KeyDef) {
  const f = ensureActiveField();
  if (!f) return;

  try { f.focus?.(); } catch { /* */ }

  if (def.kind === 'clear') {
    f.latex('');
    return;
  }
  if (def.kind === 'key') {
    if (f.keystroke) f.keystroke(def.key);
    return;
  }
  if (def.kind === 'text') {
    if (f.typedText) f.typedText(def.text);
    else if (f.write) f.write(def.text);
    else f.latex((f.latex() || '') + def.text);
    return;
  }
  if (def.kind === 'cmd') {
    if (f.cmd) f.cmd(def.latex);
    else if (f.write) f.write(def.latex);
    return;
  }
  if (def.kind === 'write') {
    if (f.write) f.write(def.latex);
    else if (f.cmd) f.cmd(def.latex);
  }
}

function renderKeys(host: HTMLElement, pageId: string) {
  const page = PAGES.find((p) => p.id === pageId) || PAGES[0];
  currentPage = page.id;
  host.querySelectorAll('[data-kb-page]').forEach((btn) => {
    btn.classList.toggle('active', btn.getAttribute('data-kb-page') === page.id);
  });
  const grid = host.querySelector('.math-kb-grid') as HTMLElement;
  if (!grid) return;
  grid.innerHTML = '';
  for (const key of page.keys) {
    const b = document.createElement('button');
    b.type = 'button';
    b.className = 'math-kb-key' + (key.wide ? ' wide' : '');
    b.textContent = key.label;
    if (key.title) b.title = key.title;
    b.addEventListener('mousedown', (e) => {
      // Prevent stealing focus from MathQuill before inject
      e.preventDefault();
    });
    b.addEventListener('click', (e) => {
      e.preventDefault();
      e.stopPropagation();
      applyKey(key);
    });
    grid.appendChild(b);
  }
}

/** Build keyboard DOM into container; returns destroy/refresh helpers. */
export function mountMathKeyboard(container: HTMLElement) {
  container.innerHTML = '';
  container.classList.add('math-kb');

  const tabs = document.createElement('div');
  tabs.className = 'math-kb-tabs';
  for (const p of PAGES) {
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'math-kb-tab' + (p.id === currentPage ? ' active' : '');
    btn.textContent = p.name;
    btn.setAttribute('data-kb-page', p.id);
    btn.addEventListener('mousedown', (e) => e.preventDefault());
    btn.addEventListener('click', (e) => {
      e.preventDefault();
      renderKeys(container, p.id);
    });
    tabs.appendChild(btn);
  }

  const hint = document.createElement('div');
  hint.className = 'math-kb-hint';
  hint.textContent = '点击公式框后，用下方键盘输入（Desmos 风格）';

  const grid = document.createElement('div');
  grid.className = 'math-kb-grid';

  container.appendChild(hint);
  container.appendChild(tabs);
  container.appendChild(grid);
  renderKeys(container, currentPage);

  return {
    setPage: (id: string) => renderKeys(container, id),
  };
}
