// 网页版自测：语法检查 + 提取 PURE 纯函数跑算法测试（与 exe 版自测对应）
import fs from 'fs';
import path from 'path';

const html = fs.readFileSync(path.join(import.meta.dirname, 'fourier-drawing.html'), 'utf8');

// 1) 整段脚本语法检查
const sm = html.match(/<script>([\s\S]*?)<\/script>/);
if (!sm) { console.error('FAIL: 未找到 <script>'); process.exit(1); }
try { new Function(sm[1]); } catch (e) { console.error('FAIL: 脚本语法错误:', e.message); process.exit(1); }
console.log('PASS: 脚本语法检查通过');

// 2) 提取 PURE 块
const pm = html.match(/\/\/ ==== PURE ====\n([\s\S]*?)\/\/ ==== END PURE ====/);
if (!pm) { console.error('FAIL: 未找到 PURE 块'); process.exit(1); }
const factory = new Function(pm[1] + '\nreturn {N_SAMP,N_EXT,MAX_PERIOD,MAX_CIRCLES,resample,dft,chainAt,approxError,makeCirclesFromFull,buildAnim,exportStrokesJSON,parseStroke,importStrokesJSON,palette,colLerp};');
const M = factory();

function assert(cond, msg) {
  if (!cond) { console.error('FAIL:', msg); process.exit(1); }
}
const { N_SAMP, N_EXT, MAX_PERIOD, resample, dft, chainAt, approxError, buildAnim, exportStrokesJSON, importStrokesJSON } = M;

// 3) 圆形 DFT
{
  const N = 256, R = 50, ctr = 100, pts = [];
  for (let n = 0; n < N; n++) { const a = 2 * Math.PI * n / N; pts.push([ctr + R * Math.cos(a), ctr + R * Math.sin(a)]); }
  const cs = dft(pts);
  const c0 = cs.find(c => c.k === 0), c1 = cs.find(c => c.k === 1), cn = cs.find(c => c.k === -1);
  assert(c0 && Math.abs(c0.re - 100) < 1e-6 && Math.abs(c0.im - 100) < 1e-6, '圆形 c0=圆心');
  assert(c1 && Math.abs(c1.re - 50) < 1e-6 && Math.abs(c1.im) < 1e-6, '圆形 c1=半径');
  assert(cn && cn.mag < 1e-6, '圆形 c_{-1}≈0');
  const cir = cs.slice(0, 2).map(c => ({ k: c.k, radius: c.mag, phase: Math.atan2(c.im, c.re) }));
  assert(approxError(pts, N, cir, 2, N) < 1e-6, '2 圆重建圆形误差≈0');
  console.log('PASS: 圆形 DFT');
}

// 4) 闭合：开放弧线轨迹无异常长段
{
  const arc = [];
  for (let i = 0; i <= 200; i++) { const t = i / 200, th = Math.PI + Math.PI * t; arc.push({ x: 300 + 200 * Math.cos(th), y: 300 + 200 * Math.sin(th) }); }
  const a = buildAnim(arc, 120);
  let maxSeg = 0;
  for (let f = 0; f < 300; f++) {
    a.n += a.N * 0.35 * 0.016;
    if (a.n >= a.N) { a.n -= a.N; a.trail.length = 0; }
    const ch = chainAt(a.circles, a.n, a.N);
    const pen = ch[ch.length - 1];
    if (a.n < a.N_stroke) {
      if (a.trail.length) maxSeg = Math.max(maxSeg, Math.hypot(pen.x - a.trail[a.trail.length - 1].x, pen.y - a.trail[a.trail.length - 1].y));
      a.trail.push(pen);
    }
  }
  console.log('闭合后最大轨迹段长:', maxSeg.toFixed(1), 'px');
  assert(maxSeg < 10, '闭合后无异常长轨迹段（吉布斯振荡消除）');
  assert(approxError(a.path, a.N_stroke, a.circles, a.M, a.N) < 5, '闭合后重建误差正常');
  console.log('PASS: 闭合消除吉布斯振荡');
}

// 5) 多笔画 导出→导入 往返（v2）
{
  const hp = [];
  for (let i = 0; i < 600; i++) { const t = 2 * Math.PI * i / 600; const s = Math.sin(t); hp.push({ x: 16 * s * s * s, y: 13 * Math.cos(t) - 5 * Math.cos(2 * t) - 2 * Math.cos(3 * t) - Math.cos(4 * t) }); }
  const sp = [];
  for (let i = 0; i < 1400; i++) { const t = 3 * 2 * Math.PI * i / 1399; const r = 6 + 170 * t / (3 * 2 * Math.PI); sp.push({ x: r * Math.cos(t), y: r * Math.sin(t) }); }
  const anims = [buildAnim(hp, 120), buildAnim(sp, 120)];
  const json = exportStrokesJSON(anims);
  const back = importStrokesJSON(json);
  assert(back.length === 2, 'v2 往返：2 个笔画');
  assert(back[0].N_stroke === N_SAMP && back[0].N === MAX_PERIOD, '往返：闭合周期正确');
  assert(back[0].M === 120 && back[1].M === 120, '往返：每笔 120 个圆');
  assert(Math.abs(back[0].circles[0].re - anims[0].circles[0].re) < 1e-4, '往返：系数一致');
  assert(approxError(back[0].path, back[0].N_stroke, back[0].circles, back[0].M, back[0].N) < 1, '往返：重建误差 < 1');
  console.log('PASS: v2 多笔画导出导入往返');
}

// 6) v1 旧格式导入（顶层 circles + 无 strokeCount → 自动闭合重算）
{
  const hp = [];
  for (let i = 0; i < 600; i++) { const t = 2 * Math.PI * i / 600; const s = Math.sin(t); hp.push({ x: 16 * s * s * s, y: 13 * Math.cos(t) - 5 * Math.cos(2 * t) - 2 * Math.cos(3 * t) - Math.cos(4 * t) }); }
  const a = buildAnim(hp, 120);
  const v1 = JSON.stringify({
    app: 'fourier-drawing', version: 1,
    description: '傅里叶画板导出的圆参数：k=频率，radius=半径，phase=初始相位，re/im=复系数。',
    sampleCount: N_SAMP,
    circles: a.circles.map(c => ({ k: c.k, radius: c.radius, phase: c.phase, re: c.re, im: c.im })),
    path: a.path
  });
  const back = importStrokesJSON(v1);
  assert(back.length === 1, 'v1 导入：1 个笔画');
  assert(back[0].N_stroke === N_SAMP && back[0].N === MAX_PERIOD, 'v1 导入：自动闭合');
  assert(approxError(back[0].path, back[0].N_stroke, back[0].circles, back[0].M, back[0].N) < 1, 'v1 导入：重建误差 < 1');
  console.log('PASS: v1 旧格式自动闭合导入');
}

// 7) v2 无 strokeCount（exe 旧版导出的 v2 文件）→ 自动闭合
{
  const hp = [];
  for (let i = 0; i < 600; i++) { const t = 2 * Math.PI * i / 600; const s = Math.sin(t); hp.push({ x: 16 * s * s * s, y: 13 * Math.cos(t) - 5 * Math.cos(2 * t) - 2 * Math.cos(3 * t) - Math.cos(4 * t) }); }
  const a = buildAnim(hp, 120);
  const v2old = JSON.stringify({
    app: 'fourier-drawing', version: 2,
    strokes: [{ sampleCount: N_SAMP, circles: a.circles.map(c => ({ k: c.k, radius: c.radius, phase: c.phase, re: c.re, im: c.im })), path: a.path }]
  });
  const back = importStrokesJSON(v2old);
  assert(back.length === 1 && back[0].N_stroke === N_SAMP && back[0].N === MAX_PERIOD, 'v2 无 strokeCount 自动闭合');
  console.log('PASS: v2 无 strokeCount 旧文件自动闭合');
}

console.log('ALL HTML TESTS PASSED');
