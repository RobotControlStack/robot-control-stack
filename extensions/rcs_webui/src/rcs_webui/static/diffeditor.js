"use strict";

// Line diff (Myers, O((N+M)D)) between two arrays of strings. Returns hunks:
// {aStart, aLines, bStart, bLines}: lines aStart..aStart+aLines.length of a are replaced by bLines at bStart in b.
function diffLines(a, b) {
  const n = a.length;
  const m = b.length;
  // strip the common prefix and suffix, the configs differ in few places
  let pre = 0;
  while (pre < n && pre < m && a[pre] === b[pre]) pre++;
  let suf = 0;
  while (suf < n - pre && suf < m - pre && a[n - 1 - suf] === b[m - 1 - suf]) suf++;
  const A = a.slice(pre, n - suf);
  const B = b.slice(pre, m - suf);
  const N = A.length;
  const M = B.length;
  const max = N + M;
  const off = max + 1;
  const v = new Int32Array(2 * max + 3);
  const trace = [];
  let found = N === 0 && M === 0;
  for (let d = 0; d <= max && !found; d++) {
    trace.push(v.slice());
    for (let k = -d; k <= d; k += 2) {
      let x = k === -d || (k !== d && v[off + k - 1] < v[off + k + 1]) ? v[off + k + 1] : v[off + k - 1] + 1;
      let y = x - k;
      while (x < N && y < M && A[x] === B[y]) {
        x++;
        y++;
      }
      v[off + k] = x;
      if (x >= N && y >= M) {
        found = true;
        break;
      }
    }
  }
  // backtrack to the edit script: list of [type, aIndex, bIndex]
  const ops = [];
  let x = N;
  let y = M;
  for (let d = trace.length - 1; d >= 0 && (x > 0 || y > 0); d--) {
    const vd = trace[d];
    const k = x - y;
    const prevK = k === -d || (k !== d && vd[off + k - 1] < vd[off + k + 1]) ? k + 1 : k - 1;
    const prevX = vd[off + prevK];
    const prevY = prevX - prevK;
    while (x > prevX && y > prevY) {
      x--;
      y--;
    }
    if (d > 0) {
      if (x === prevX) ops.push(["ins", x, prevY]);
      else ops.push(["del", prevX, y]);
    }
    x = prevX;
    y = prevY;
  }
  ops.reverse();
  // group consecutive edits into hunks
  const hunks = [];
  let cur = null;
  for (const [type, ai, bi] of ops) {
    const aIdx = type === "del" ? ai : ai; // position in A where the edit happens
    const bIdx = type === "ins" ? bi : bi;
    if (cur && aIdx === cur.aEnd && bIdx === cur.bEnd) {
      if (type === "del") {
        cur.aLines.push(A[ai]);
        cur.aEnd++;
      } else {
        cur.bLines.push(B[bi]);
        cur.bEnd++;
      }
      continue;
    }
    cur = { aStart: aIdx, bStart: bIdx, aEnd: aIdx, bEnd: bIdx, aLines: [], bLines: [] };
    if (type === "del") {
      cur.aLines.push(A[ai]);
      cur.aEnd++;
    } else {
      cur.bLines.push(B[bi]);
      cur.bEnd++;
    }
    hunks.push(cur);
  }
  return hunks.map((h) => ({
    aStart: h.aStart + pre,
    aLines: h.aLines,
    bStart: h.bStart + pre,
    bLines: h.bLines,
    kind: h.aLines.length === 0 ? "add" : h.bLines.length === 0 ? "del" : "mod",
  }));
}

// JSON text with arrays of plain values on one line (poses, joint vectors, ndarray rows), so that a changed
// number is one changed line and the config stays readable.
function formatJson(value, indent = "") {
  const inner = indent + "  ";
  const plain = (v) => v === null || typeof v !== "object";
  if (Array.isArray(value)) {
    if (value.length === 0) return "[]";
    if (value.every(plain)) return `[${value.map((v) => JSON.stringify(v)).join(", ")}]`;
    return `[\n${value.map((v) => inner + formatJson(v, inner)).join(",\n")}\n${indent}]`;
  }
  if (value && typeof value === "object") {
    const keys = Object.keys(value);
    if (keys.length === 0) return "{}";
    return `{\n${keys.map((k) => `${inner}${JSON.stringify(k)}: ${formatJson(value[k], inner)}`).join(",\n")}\n${indent}}`;
  }
  return JSON.stringify(value);
}

// Editor: a textarea with a gutter of line numbers and diff marks against a base text.
class DiffEditor {
  constructor(root, { onChange } = {}) {
    this.root = root;
    this.text = root.querySelector("textarea");
    this.gutter = root.querySelector(".gutter-inner");
    this.peek = root.querySelector(".peek");
    this.base = "";
    this.hunks = [];
    this.current = -1;
    this.onChange = onChange || (() => {});
    this._timer = null;
    this.text.addEventListener("input", () => {
      clearTimeout(this._timer);
      this._timer = setTimeout(() => this.refresh(), 120);
    });
    this.text.addEventListener("scroll", () => this._sync());
    this.text.addEventListener("keydown", (e) => {
      if (e.key === "Tab") {
        e.preventDefault();
        document.execCommand("insertText", false, "  ");
      }
      if (e.key === "Escape") this.closePeek();
    });
    this.gutter.addEventListener("click", (e) => {
      const mark = e.target.closest("[data-hunk]");
      if (mark) this.openPeek(+mark.dataset.hunk);
    });
    this.peek.addEventListener("click", (e) => {
      const act = e.target.closest("[data-act]");
      if (!act) return;
      if (act.dataset.act === "close") this.closePeek();
      if (act.dataset.act === "revert") this.revert(this.current);
      if (act.dataset.act === "prev") this.step(-1);
      if (act.dataset.act === "next") this.step(1);
    });
  }

  set(base, text = base) {
    this.base = base;
    this.text.value = text;
    this.text.scrollTop = 0;
    this.closePeek();
    this.refresh();
  }

  value() {
    return this.text.value;
  }

  dirty() {
    return this.text.value !== this.base;
  }

  refresh() {
    const a = this.base ? this.base.split("\n") : [];
    const b = this.text.value.split("\n");
    this.hunks = this.base ? diffLines(a, b) : [];
    const marks = new Array(b.length).fill(null);
    const delBefore = new Map();
    this.hunks.forEach((h, i) => {
      if (h.kind === "del") delBefore.set(Math.min(h.bStart, b.length), i);
      else for (let j = h.bStart; j < h.bStart + h.bLines.length; j++) marks[j] = [h.kind, i];
    });
    const rows = [];
    for (let i = 0; i < b.length; i++) {
      const m = marks[i];
      const del = delBefore.has(i) ? `<span class="del-mark" data-hunk="${delBefore.get(i)}" title="deleted lines, click to show"></span>` : "";
      const cls = m ? `mark ${m[0]}` : "mark";
      const attr = m ? ` data-hunk="${m[1]}" title="${m[0] === "add" ? "added" : "changed"}, click to show"` : "";
      rows.push(`<div class="ln">${del}<span class="${cls}"${attr}></span><span class="num">${i + 1}</span></div>`);
    }
    if (delBefore.has(b.length)) rows.push(`<div class="ln"><span class="del-mark" data-hunk="${delBefore.get(b.length)}"></span></div>`);
    this.gutter.innerHTML = rows.join("");
    if (this.current >= this.hunks.length) this.closePeek();
    else if (!this.peek.hidden) this.openPeek(this.current);
    this._sync();
    this.onChange(this.hunks);
  }

  _lineHeight() {
    return parseFloat(getComputedStyle(this.text).lineHeight) || 18;
  }

  _sync() {
    this.gutter.style.transform = `translateY(${-this.text.scrollTop}px)`;
    if (!this.peek.hidden) this._placePeek();
  }

  _placePeek() {
    const h = this.hunks[this.current];
    if (!h) return;
    const pad = parseFloat(getComputedStyle(this.text).paddingTop) || 0;
    const line = h.bStart + Math.max(h.bLines.length, 1);
    const top = pad + line * this._lineHeight() - this.text.scrollTop;
    this.peek.style.top = `${Math.max(0, Math.min(top, this.root.clientHeight - 40))}px`;
  }

  openPeek(i) {
    const h = this.hunks[i];
    if (!h) return;
    this.current = i;
    const esc = (s) => s.replace(/[&<>]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[c]));
    const removed = h.aLines.map((l) => `<div class="old">- ${esc(l)}</div>`).join("");
    const added = h.bLines.map((l) => `<div class="new">+ ${esc(l)}</div>`).join("");
    this.peek.innerHTML = `<div class="peek-head"><b>change ${i + 1} of ${this.hunks.length}</b>
      <span class="muted">picked config line ${h.aStart + 1}</span><span style="flex:1"></span>
      <button class="btn small" data-act="prev" title="previous change">&uarr;</button>
      <button class="btn small" data-act="next" title="next change">&darr;</button>
      <button class="btn small" data-act="revert">revert</button>
      <button class="btn small" data-act="close" title="close (Esc)">&times;</button></div>
      <div class="peek-body">${removed}${added}</div>`;
    this.peek.hidden = false;
    this._placePeek();
  }

  closePeek() {
    this.peek.hidden = true;
    this.current = -1;
  }

  step(dir) {
    if (!this.hunks.length) return;
    const next = this.current < 0 ? (dir > 0 ? 0 : this.hunks.length - 1) : (this.current + dir + this.hunks.length) % this.hunks.length;
    const h = this.hunks[next];
    this.text.scrollTop = Math.max(0, h.bStart * this._lineHeight() - this.text.clientHeight / 3);
    this.openPeek(next);
  }

  revert(i) {
    const h = this.hunks[i];
    if (!h) return;
    const lines = this.text.value.split("\n");
    lines.splice(h.bStart, h.bLines.length, ...h.aLines);
    const top = this.text.scrollTop;
    this.text.value = lines.join("\n");
    this.text.scrollTop = top;
    this.closePeek();
    this.refresh();
  }

  revertAll() {
    const top = this.text.scrollTop;
    this.text.value = this.base;
    this.text.scrollTop = top;
    this.closePeek();
    this.refresh();
  }
}

window.DiffEditor = DiffEditor;
window.formatJson = formatJson;
window.diffLines = diffLines;
