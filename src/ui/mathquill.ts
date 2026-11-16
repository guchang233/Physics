/**
 * MathQuill bootstrap for the analysis panel.
 * StaticMath for display; MathField for editable / copyable LaTeX.
 */
import $ from 'jquery';
import MQFactory from '@pie-framework/mathquill';
import '@pie-framework/mathquill/build/mathquill.css';

// Ensure jQuery is global for MathQuill's UMD build paths.
(window as any).jQuery = (window as any).$ = $;

type MQInterface = {
  StaticMath: (el: HTMLElement) => { latex: (s?: string) => string };
  MathField: (
    el: HTMLElement,
    config?: Record<string, unknown>,
  ) => { latex: (s?: string) => string; focus: () => void };
};

const MQ: MQInterface =
  typeof (MQFactory as any)?.getInterface === 'function'
    ? (MQFactory as any).getInterface(2)
    : ((window as any).MathQuill?.getInterface?.(2) ?? MQFactory);

export function latexToStatic(el: HTMLElement, latex: string) {
  el.textContent = '';
  el.classList.add('mq-host', 'mq-static');
  const field = MQ.StaticMath(el);
  try {
    field.latex(latex);
  } catch {
    el.textContent = latex;
  }
  return field;
}

export function latexToEditable(
  el: HTMLElement,
  latex: string,
  opts: { readOnly?: boolean } = {},
) {
  el.textContent = '';
  el.classList.add('mq-host', 'mq-editable');
  const field = MQ.MathField(el, {
    spaceBehavesLikeTab: true,
    leftRightIntoCmdGoes: 'up',
    restrictMismatchedBrackets: true,
    handlers: {},
  });
  try {
    field.latex(latex);
  } catch {
    el.textContent = latex;
  }
  if (opts.readOnly) {
    el.setAttribute('contenteditable', 'false');
    el.classList.add('mq-readonly');
  }
  return field;
}

/** Mount all [data-mq] nodes under root. data-mq="static"|"edit" */
export function mountMathQuill(root: HTMLElement) {
  const nodes = root.querySelectorAll<HTMLElement>('[data-mq]');
  const fields: { el: HTMLElement; kind: string; latex: (s?: string) => string }[] = [];
  nodes.forEach((el) => {
    const kind = el.getAttribute('data-mq') || 'static';
    const latex = el.getAttribute('data-latex') || el.textContent || '';
    el.removeAttribute('data-latex');
    if (kind === 'edit') {
      const f = latexToEditable(el, latex);
      (el as any).mq = f;
      fields.push({ el, kind, latex: (s?: string) => f.latex(s as any) });
    } else {
      const f = latexToStatic(el, latex);
      (el as any).mq = f;
      fields.push({ el, kind, latex: (s?: string) => f.latex(s as any) });
    }
  });
  return fields;
}

