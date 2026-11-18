/**
 * 数值公式。
 * 本物体：t,x,y,vx,vy,v,m,q,g,pi
 * 其他物体：名称.量 或 #编号.量，例如 小球2.vx、小球2.F、#3.m
 */

export type ForceEvalCtx = {
  t: number;
  x: number;
  y: number;
  vx: number;
  vy: number;
  v: number;
  m: number;
  q: number;
  g: number;
};

export type ExprName = { id: number; name: string };

export type ExprEvalOpts = {
  names?: ExprName[];
  /** 全局变量名（Desmos 式）：表达式里直接写名字，求值时经 ref(-1, 名字) 取值 */
  vars?: string[];
  ref?: (id: number, field: string) => number;
  /** 额外的本地量（例如显示项里的 ax、ay、a、Ek），从 ctx 里取同名字段；优先于同名全局变量 */
  locals?: string[];
};

/** 显示项表达式里额外可用的本物体量 */
export const DISPLAY_LOCALS = ['ax', 'ay', 'a', 'omega', 'ang', 'Ek', 'Ep', 'px', 'py', 'p', 'Fx', 'Fy', 'F'] as const;

const FN_NAMES = [
  'sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2',
  'abs', 'sqrt', 'hypot', 'min', 'max', 'pow', 'exp', 'log', 'log10', 'floor', 'ceil', 'round', 'sign',
] as const;

const ALLOWED = new Set<string>([
  ...FN_NAMES,
  'pi', 'e', 't', 'x', 'y', 'vx', 'vy', 'v', 'm', 'q', 'g',
  'true', 'false', 'NaN', 'Infinity',
  '__ref',
]);

/** 可写在「物体.量」后面的量。长名字在前，避免 vx 吃掉 vx0。 */
export const EXPR_FIELDS = [
  'softRadius', 'snapStep', 'maxDt', 'FaimX', 'FaimY', 'v0ang', 'vang', 'omega0', 'omega',
  'vx0', 'vy0', 'x0', 'y0', 'x1', 'y1', 'x2', 'y2', 'ang0', 'Fang', 'Fx', 'Fy',
  'vx', 'vy', 'ax', 'ay', 'mu', 'ang', 'L0',
  'v0', 'vPx', 'FPx',
  'x', 'y', 'v', 'm', 'q', 'F', 'r', 'w', 'h', 'e', 'k', 'L', 'B', 'E', 'g', 't',
  'drag', 'kc',
] as const;

const FIELD_SET = new Set<string>(EXPR_FIELDS);

const FN_NAMES_SET = new Set<string>(FN_NAMES);

type Compiled = (ctx: ForceEvalCtx, ref: (id: number, field: string) => number) => number;

const cache = new Map<string, Compiled | null>();

function groupsFrom(names: ExprName[]) {
  const map = new Map<string, number[]>();
  for (const n of names) {
    const name = String(n.name || '');
    if (!name) continue;
    const arr = map.get(name) || [];
    arr.push(n.id);
    map.set(name, arr);
  }
  return [...map.entries()]
    .map(([name, ids]) => ({ name, ids }))
    .sort((a, b) => b.name.length - a.name.length);
}

export type ExprRewrite = {
  code: string;
  refs: { id: number; field: string }[];
  error?: string;
};

/* ---------------- 全局变量 ---------------- */
const ID_START = /[A-Za-z_\u0391-\u03A9\u03B1-\u03C9]/;
const ID_CHAR = /[A-Za-z0-9_\u0391-\u03A9\u03B1-\u03C9]/;
const JS_WORDS = ['break', 'case', 'catch', 'const', 'continue', 'debugger', 'default', 'delete', 'do', 'else', 'export', 'extends', 'finally', 'for', 'function', 'if', 'import', 'in', 'instanceof', 'let', 'new', 'return', 'super', 'switch', 'this', 'throw', 'try', 'typeof', 'var', 'void', 'while', 'with', 'yield', 'null', 'undefined', 'ctx'];
/** 变量名规则：字母（含希腊字母）开头，后接字母、数字、下划线；不能与内置量、函数、关键字重名 */
export function validateVarName(name: string, others: string[] = []): string | null {
  const n = String(name || '');
  if (!n) return '名称不能为空';
  if (!/^[A-Za-z\u0391-\u03A9\u03B1-\u03C9][A-Za-z0-9_\u0391-\u03A9\u03B1-\u03C9]*$/.test(n)) return '只能用字母（含希腊字母）、数字和下划线，且以字母开头';
  if (n.length > 24) return '名称过长';
  if (ALLOWED.has(n) || FN_NAMES_SET.has(n) || JS_WORDS.includes(n) || n.startsWith('__')) return `“${n}”是内置名称，不能用作变量`;
  if (others.includes(n)) return `变量“${n}”已存在`;
  return null;
}

/** 把「小球2.vx」「#3.m」换成 __ref(id,"field")，把全局变量名换成 __ref(-1,"名字")。 */
export function rewriteExpr(src: string, names: ExprName[] = [], vars: string[] = []): ExprRewrite {
  const s = String(src || '');
  const groups = groupsFrom(names);
  const varSet = new Set(vars);
  const refs: { id: number; field: string }[] = [];
  let out = '';
  let i = 0;
  while (i < s.length) {
    const hash = /^#(\d+)\.([A-Za-z_][A-Za-z0-9_]*)/.exec(s.slice(i));
    if (hash && FIELD_SET.has(hash[2])) {
      refs.push({ id: +hash[1], field: hash[2] });
      out += `__ref(${hash[1]},"${hash[2]}")`;
      i += hash[0].length;
      continue;
    }
    let matched = false;
    for (const g of groups) {
      if (!s.startsWith(g.name + '.', i)) continue;
      const rest = s.slice(i + g.name.length + 1);
      const fm = /^[A-Za-z_][A-Za-z0-9_]*/.exec(rest);
      if (!fm || !FIELD_SET.has(fm[0])) continue;
      if (g.ids.length !== 1) {
        return { code: s, refs: [], error: `名称不唯一：${g.name}` };
      }
      refs.push({ id: g.ids[0], field: fm[0] });
      out += `__ref(${g.ids[0]},"${fm[0]}")`;
      i += g.name.length + 1 + fm[0].length;
      matched = true;
      break;
    }
    if (matched) continue;
    // 标识符整体读取，避免变量 a 匹配到 atan 之类的一部分
    if (ID_START.test(s[i]) && !(i > 0 && (ID_CHAR.test(s[i - 1]) || s[i - 1] === '.'))) {
      let j = i + 1;
      while (j < s.length && ID_CHAR.test(s[j])) j++;
      const word = s.slice(i, j);
      if (varSet.has(word) && s[j] !== '.' && s[j] !== '(') {
        refs.push({ id: -1, field: word });
        out += `__ref(-1,"${word}")`;
      } else out += word;
      i = j;
      continue;
    }
    out += s[i];
    i++;
  }
  return { code: out, refs };
}

export function validateForceExpr(src: string, names: ExprName[] = [], vars: string[] = [], locals: string[] = []): string | null {
  const s = String(src || '').trim();
  if (!s) return '表达式为空';
  if (s.length > 240) return '表达式过长';
  if (/[{}`$]|=>|new\b|function\b|while\b|for\b|import\b|class\b|this\b|window\b|global|process|require|eval|Function|constructor|__proto__|prototype|__ref\b/i.test(s)) {
    return '含有不允许的语法';
  }
  if (/['"]/.test(s)) return '不允许字符串';
  const rewritten = rewriteExpr(s, names, vars.filter(n => !locals.includes(n)));
  if (rewritten.error) return rewritten.error;
  // 引用写成 __ref(id,"field")，字段名/变量名在字符串里，不参与符号检查
  const bare = rewritten.code.replace(/"(?:\\.|[^"\\])*"/g, '""');
  if (/[^\x00-\x7F]/.test(bare)) {
    const m = /[^\x00-\x7F]+/.exec(bare);
    return `未知的名称：${m ? m[0] : ''}`;
  }
  const idRe = /[A-Za-z_][A-Za-z0-9_]*/g;
  let m: RegExpExecArray | null;
  while ((m = idRe.exec(bare))) {
    if (!ALLOWED.has(m[0]) && !FN_NAMES_SET.has(m[0]) && !locals.includes(m[0])) return `未知符号：${m[0]}`;
  }
  return null;
}

function compile(code: string, locals: string[] = []): Compiled | null {
  const key = locals.length ? locals.join(',') + '|' + code : code;
  if (cache.has(key)) return cache.get(key)!;
  try {
    const fn = new Function(
      'ctx',
      '__ref',
      `"use strict";
      const {t,x,y,vx,vy,v,m,q,g${locals.length ? ',' + locals.join(',') : ''}}=ctx;
      const pi=Math.PI,e=Math.E;
      const sin=Math.sin,cos=Math.cos,tan=Math.tan;
      const asin=Math.asin,acos=Math.acos,atan=Math.atan,atan2=Math.atan2;
      const abs=Math.abs,sqrt=Math.sqrt,hypot=Math.hypot;
      const min=Math.min,max=Math.max,pow=Math.pow,exp=Math.exp;
      const log=Math.log,log10=Math.log10||((n)=>Math.log(n)/Math.LN10);
      const floor=Math.floor,ceil=Math.ceil,round=Math.round,sign=Math.sign;
      return (${code});`,
    ) as Compiled;
    cache.set(key, fn);
    return fn;
  } catch {
    cache.set(key, null);
    return null;
  }
}

/**
 * 表达式里尚未定义、但可以成为全局变量的名字（Desmos 式“创建变量”提示用）。
 * 只返回符合变量命名规则、且不是内置量 / 函数 / 物体引用的标识符。
 */
export function findUnknownNames(src: string, names: ExprName[] = [], vars: string[] = [], locals: string[] = []): string[] {
  const rewritten = rewriteExpr(String(src || ''), names, vars.filter(n => !locals.includes(n)));
  if (rewritten.error) return [];
  const bare = rewritten.code.replace(/"(?:\\.|[^"\\])*"/g, '""');
  const out: string[] = [];
  const re = /[A-Za-z_\u0391-\u03A9\u03B1-\u03C9][A-Za-z0-9_\u0391-\u03A9\u03B1-\u03C9]*/g;
  let m: RegExpExecArray | null;
  while ((m = re.exec(bare))) {
    const w = m[0];
    const prev = bare[m.index - 1];
    if (prev === '.' || ALLOWED.has(w) || FN_NAMES_SET.has(w) || locals.includes(w)) continue;
    if (validateVarName(w, []) === null && !out.includes(w)) out.push(w);
  }
  return out;
}

export function evalForceExpr(
  expr: string,
  ctx: ForceEvalCtx,
  opts: ExprEvalOpts = {},
): { ok: true; value: number } | { ok: false; error: string } {
  const s = String(expr || '').trim();
  if (!s) return { ok: false, error: '表达式为空' };
  const names = opts.names || [], locals = opts.locals || [];
  const vars = (opts.vars || []).filter(n => !locals.includes(n));
  const verr = validateForceExpr(s, names, vars, locals);
  if (verr) return { ok: false, error: verr };
  const rewritten = rewriteExpr(s, names, vars);
  if (rewritten.error) return { ok: false, error: rewritten.error };
  const fn = compile(rewritten.code, locals);
  if (!fn) return { ok: false, error: '编译失败' };
  const ref = opts.ref || ((id: number, field: string) => {
    throw new Error(`无法引用 #${id}.${field}`);
  });
  try {
    const value = fn(ctx, ref);
    if (typeof value !== 'number' || !Number.isFinite(value)) {
      return { ok: false, error: '结果非有限数' };
    }
    return { ok: true, value };
  } catch (e: any) {
    return { ok: false, error: e?.message || '求值失败' };
  }
}

export function orderExprNodes(nodes: { id: string; deps: string[] }[]): { order: string[]; cyclic: string[] } {
  const ids = new Set(nodes.map(n => n.id));
  const self = new Set(nodes.filter(n => n.deps.includes(n.id)).map(n => n.id));
  const deps = new Map(nodes.map(n => [n.id, n.deps.filter(d => ids.has(d) && d !== n.id && !self.has(n.id))]));
  const indeg = new Map<string, number>();
  const next = new Map<string, string[]>();
  for (const n of nodes) {
    indeg.set(n.id, 0);
    next.set(n.id, []);
  }
  for (const n of nodes) {
    if (self.has(n.id)) continue;
    for (const d of deps.get(n.id) || []) {
      if (self.has(d)) continue;
      next.get(d)!.push(n.id);
      indeg.set(n.id, (indeg.get(n.id) || 0) + 1);
    }
  }
  const q = nodes.filter(n => !self.has(n.id) && (indeg.get(n.id) || 0) === 0).map(n => n.id);
  const order: string[] = [];
  while (q.length) {
    const id = q.shift()!;
    order.push(id);
    for (const nxt of next.get(id) || []) {
      indeg.set(nxt, (indeg.get(nxt) || 1) - 1);
      if (indeg.get(nxt) === 0) q.push(nxt);
    }
  }
  const cyclic = nodes.map(n => n.id).filter(id => self.has(id) || !order.includes(id));
  return { order, cyclic };
}

export const FORCE_EXPR_HELP =
  '本物体：t x y vx vy v m q g pi。其他物体：小球2.vx、小球2.F、小球2.Fy，或 #编号.量。函数：sin cos tan atan2 abs sqrt hypot min max pow';
