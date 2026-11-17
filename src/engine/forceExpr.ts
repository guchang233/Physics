/**
 * Safe-ish math expression evaluator for applied forces.
 * Supports: t,x,y,vx,vy,v,m,q,g,pi + sin/cos/tan/abs/sqrt/atan2/hypot/min/max/pow
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

const FN_NAMES = [
  'sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2',
  'abs', 'sqrt', 'hypot', 'min', 'max', 'pow', 'exp', 'log', 'log10', 'floor', 'ceil', 'round', 'sign',
] as const;

const ALLOWED = new Set([
  ...FN_NAMES,
  'pi', 'e', 't', 'x', 'y', 'vx', 'vy', 'v', 'm', 'q', 'g',
  'true', 'false', 'NaN', 'Infinity',
]);

/** Reject identifiers that aren't in the allow-list. */
export function validateForceExpr(src: string): string | null {
  const s = String(src || '').trim();
  if (!s) return '表达式为空';
  if (s.length > 180) return '表达式过长';
  // ban obvious dangerous tokens
  if (/[{}`$]|=>|new\b|function\b|while\b|for\b|import\b|class\b|this\b|window\b|global|process|require|eval|Function|constructor|__proto__|prototype/i.test(s)) {
    return '含有不允许的语法';
  }
  // strip strings (none allowed)
  if (/['"]/.test(s)) return '不允许字符串';
  // find identifiers
  const idRe = /[A-Za-z_][A-Za-z0-9_]*/g;
  let m: RegExpExecArray | null;
  while ((m = idRe.exec(s))) {
    if (!ALLOWED.has(m[0])) return `未知符号：${m[0]}`;
  }
  return null;
}

type Compiled = (ctx: ForceEvalCtx) => number;

const cache = new Map<string, Compiled | null>();

function compile(expr: string): Compiled | null {
  const key = expr.trim();
  if (cache.has(key)) return cache.get(key)!;
  const err = validateForceExpr(key);
  if (err) {
    cache.set(key, null);
    return null;
  }
  try {
    // eslint-disable-next-line no-new-func
    const fn = new Function(
      'ctx',
      `"use strict";
      const {t,x,y,vx,vy,v,m,q,g}=ctx;
      const pi=Math.PI,e=Math.E;
      const sin=Math.sin,cos=Math.cos,tan=Math.tan;
      const asin=Math.asin,acos=Math.acos,atan=Math.atan,atan2=Math.atan2;
      const abs=Math.abs,sqrt=Math.sqrt,hypot=Math.hypot;
      const min=Math.min,max=Math.max,pow=Math.pow,exp=Math.exp;
      const log=Math.log,log10=Math.log10||((n)=>Math.log(n)/Math.LN10);
      const floor=Math.floor,ceil=Math.ceil,round=Math.round,sign=Math.sign;
      return (${key});`,
    ) as Compiled;
    cache.set(key, fn);
    return fn;
  } catch {
    cache.set(key, null);
    return null;
  }
}

export function evalForceExpr(expr: string, ctx: ForceEvalCtx): { ok: true; value: number } | { ok: false; error: string } {
  const s = String(expr || '').trim();
  if (!s) return { ok: false, error: '表达式为空' };
  const verr = validateForceExpr(s);
  if (verr) return { ok: false, error: verr };
  const fn = compile(s);
  if (!fn) return { ok: false, error: '编译失败' };
  try {
    const value = fn(ctx);
    if (typeof value !== 'number' || !Number.isFinite(value)) {
      return { ok: false, error: '结果非有限数' };
    }
    return { ok: true, value };
  } catch (e: any) {
    return { ok: false, error: e?.message || '求值失败' };
  }
}

export const FORCE_EXPR_HELP =
  '可用：t,x,y,vx,vy,v,m,q,g,pi · sin cos tan atan2 abs sqrt hypot min max pow';
