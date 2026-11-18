import { evalForceExpr, orderExprNodes, rewriteExpr, validateForceExpr, validateVarName, findUnknownNames } from './forceExpr.ts';

function assert(cond: unknown, msg: string) {
  if (!cond) throw new Error(msg);
}

const names = [
  { id: 1, name: '小球1' },
  { id: 2, name: '小球2' },
  { id: 3, name: '小球' },
];

const rewritten = rewriteExpr('小球2.vx + 小球.F + #1.m', names);
assert(rewritten.code === '__ref(2,"vx") + __ref(3,"F") + __ref(1,"m")', rewritten.code);
assert(rewritten.refs.length === 3, 'refs');

const amb = rewriteExpr('小球.v', [
  { id: 1, name: '小球' },
  { id: 2, name: '小球' },
]);
assert(amb.error && amb.error.includes('不唯一'), amb.error || 'expected ambiguous');

assert(validateForceExpr('小球2.vx*2', names) === null, 'valid cross ref');
assert(validateForceExpr('小球2.F + 小球2.Fy', names) === null, validateForceExpr('小球2.F + 小球2.Fy', names) || 'F field');
assert(validateForceExpr('alert(1)') !== null, 'reject unknown');

const ctx = { t: 0, x: 0, y: 0, vx: 1, vy: 0, v: 1, m: 1, q: 0, g: 9.8 };
const ev = evalForceExpr('小球2.vx + vx', ctx, {
  names,
  ref: (id, field) => (id === 2 && field === 'vx' ? 4 : 0),
});
assert(ev.ok && ev.value === 5, 'eval cross ref ' + JSON.stringify(ev));

const ordered = orderExprNodes([
  { id: '1:F', deps: ['2:vx'] },
  { id: '2:vx', deps: [] },
]);
assert(ordered.order[0] === '2:vx' && ordered.order[1] === '1:F', 'topo ' + ordered.order.join(','));
const cyc = orderExprNodes([
  { id: '1:F', deps: ['2:F'] },
  { id: '2:F', deps: ['1:F'] },
]);
assert(cyc.cyclic.length === 2 && cyc.order.length === 0, 'cycle');
const self = orderExprNodes([{ id: '1:F', deps: ['1:F'] }]);
assert(self.cyclic.length === 1 && self.order.length === 0, 'self cycle');

// 全局变量
{
  const vars = ['a', 'k1', 'ω'];
  const r = rewriteExpr('a*atan(a) + k1 + ω*t + 小球2.vx', names, vars);
  assert(r.code === '__ref(-1,"a")*atan(__ref(-1,"a")) + __ref(-1,"k1") + __ref(-1,"ω")*t + __ref(2,"vx")', r.code);
  assert(r.refs.filter(x => x.id === -1).map(x => x.field).join(',') === 'a,a,k1,ω', 'var refs');
  assert(rewriteExpr('atan(1)+aa', names, vars).code === 'atan(1)+aa', 'no partial match');
  assert(validateForceExpr('aa+1', names, vars)?.includes('未知符号'), 'unknown ascii');
  assert(validateForceExpr('β+1', names, vars)?.includes('β'), 'unknown greek');
  const ev = evalForceExpr('2*a + ω', ctx, { names, vars, ref: (id, f) => (id === -1 ? ({ a: 3, ω: 0.5 } as any)[f] : 0) });
  assert(ev.ok && Math.abs(ev.value - 6.5) < 1e-12, 'eval vars ' + JSON.stringify(ev));
  assert(validateVarName('a', []) === null && validateVarName('ω_1', []) === null, 'valid names');
  assert(validateVarName('t', []) !== null && validateVarName('sin', []) !== null && validateVarName('1a', []) !== null && validateVarName('a', ['a']) !== null && validateVarName('new', []) !== null, 'invalid names');
}

assert(findUnknownNames('v0*cos(θ) + a + 小球2.vx + sin(t)', names, ['a']).join(',') === 'v0,θ', findUnknownNames('v0*cos(θ) + a + 小球2.vx + sin(t)', names, ['a']).join(','));
assert(findUnknownNames('vx + g*t', names, []).length === 0, 'builtins not unknown');

// 显示项本地量：优先于同名全局变量
{
  const loc = ['ax', 'ay', 'a', 'Ek'];
  const c2 = Object.assign({}, ctx, { ax: 1, ay: -2, a: 7, Ek: 4 }) as any;
  const r1 = evalForceExpr('a + Ek + k1', c2, { names, vars: ['a', 'k1'], locals: loc, ref: (id, f) => (id === -1 && f === 'k1' ? 10 : NaN) });
  assert(r1.ok && r1.value === 21, 'locals ' + JSON.stringify(r1));
  assert(validateForceExpr('ax', names, []) !== null, 'ax not allowed without locals');
  assert(findUnknownNames('ax + zz', names, [], loc).join(',') === 'zz', 'locals not unknown');
}

console.log('forceExpr selftest OK');
