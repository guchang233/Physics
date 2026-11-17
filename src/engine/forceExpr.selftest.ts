import { evalForceExpr, orderExprNodes, rewriteExpr, validateForceExpr } from './forceExpr.ts';

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

console.log('forceExpr selftest OK');
