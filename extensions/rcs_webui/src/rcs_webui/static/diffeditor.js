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

// Index of the line that closes the bracket opened at the end of lines[i], -1 if there is none. Brackets inside
// JSON strings are ignored.
function matchClose(lines, i) {
  let depth = 0;
  for (let k = i; k < lines.length; k++) {
    let inStr = false;
    const line = lines[k];
    for (let c = 0; c < line.length; c++) {
      const ch = line[c];
      if (inStr) {
        if (ch === "\\") c++;
        else if (ch === '"') inStr = false;
      } else if (ch === '"') inStr = true;
      else if (ch === "{" || ch === "[") depth++;
      else if (ch === "}" || ch === "]") depth--;
    }
    if (k > i && depth <= 0) return depth === 0 ? k : -1;
  }
  return -1;
}

const opensBlock = (line) => /[{[]\s*$/.test(line);

// Editor: a textarea with a gutter of line numbers, diff marks against a base text and folding of JSON blocks.
// A folded block is one placeholder line in the textarea (read only, unfold it to edit), value() and the diff
// always use the complete text; the gutter shows the line numbers of the complete text.
class DiffEditor {
  constructor(root, { onChange } = {}) {
    this.root = root;
    this.text = root.querySelector("textarea");
    this.gutter = root.querySelector(".gutter-inner");
    this.peek = root.querySelector(".peek");
    this.base = "";
    this.hunks = [];
    this.folds = new Map(); // placeholder line -> the folded lines (header to closing line, may hold placeholders)
    this._starts = [0]; // first line of the complete text for every textarea line
    this._spans = [1]; // number of lines of the complete text for every textarea line
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
      const fold = e.target.closest("[data-fold]");
      if (fold) return this.toggleFold(+fold.dataset.fold);
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
    if (text === this.value() && this.text.value) {
      // same content (e.g. a refreshed catalog entry): keep the folds and the scroll position
      this.base = base;
      this.refresh();
      return;
    }
    this.base = base;
    this.folds.clear();
    this.text.value = text;
    this.text.scrollTop = 0;
    this.closePeek();
    this.refresh();
  }

  // complete text of the textarea lines, with the first line and number of lines of each textarea line
  _expand(display) {
    const lines = [];
    const starts = [];
    const spans = [];
    const flat = (line, depth) => {
      const hidden = depth < 64 ? this.folds.get(line) : undefined;
      if (!hidden) return [line];
      return hidden.flatMap((l) => flat(l, depth + 1));
    };
    for (const line of display) {
      const full = flat(line, 0);
      starts.push(lines.length);
      spans.push(full.length);
      lines.push(...full);
    }
    return { lines, starts, spans };
  }

  value() {
    return this.folds.size ? this._expand(this.text.value.split("\n")).lines.join("\n") : this.text.value;
  }

  dirty() {
    return this.value() !== this.base;
  }

  // textarea line that shows line `full` of the complete text
  _displayLine(full) {
    let lo = 0;
    let hi = this._starts.length - 1;
    while (lo < hi) {
      const mid = (lo + hi + 1) >> 1;
      if (this._starts[mid] <= full) lo = mid;
      else hi = mid - 1;
    }
    return lo;
  }

  _setDisplay(display) {
    const top = this.text.scrollTop;
    this.text.value = display.join("\n");
    this.text.scrollTop = top;
    this.refresh();
  }

  toggleFold(i) {
    const display = this.text.value.split("\n");
    const line = display[i];
    if (line === undefined) return;
    if (this.folds.has(line)) {
      const hidden = this.folds.get(line);
      this.folds.delete(line);
      display.splice(i, 1, ...hidden);
      return this._setDisplay(display);
    }
    if (!this._fold(display, i)) return;
    this._setDisplay(display);
  }

  _fold(display, i) {
    if (!opensBlock(display[i]) || this.folds.has(display[i])) return false;
    const j = matchClose(display, i);
    if (j < 0) return false;
    const hidden = display.slice(i, j + 1);
    const n = this._expand(hidden).lines.length - 2;
    let ph = `${display[i].replace(/\s+$/, "")} \u2026 ${n} line${n === 1 ? "" : "s"} \u2026 ${display[j].trim()}`;
    while (this.folds.has(ph)) ph += "\u200b"; // invisible, keeps equal looking placeholders apart
    this.folds.set(ph, hidden);
    display.splice(i, j - i + 1, ph);
    return true;
  }

  // fold every block below the first `keep` nesting levels, innermost first (unfolding shows the next level)
  foldAll(keep = 2) {
    const display = this.text.value.split("\n");
    const indentOf = (l) => l.length - l.trimStart().length;
    const levels = [...new Set(display.filter(opensBlock).map(indentOf))].sort((x, y) => y - x);
    const minIndent = 2 * keep;
    for (const lvl of levels) {
      if (lvl < minIndent) break;
      for (let i = display.length - 1; i >= 0; i--) if (indentOf(display[i]) === lvl) this._fold(display, i);
    }
    this._setDisplay(display);
  }

  unfoldAll() {
    if (!this.folds.size) return;
    const text = this.value();
    this.folds.clear();
    const top = this.text.scrollTop;
    this.text.value = text;
    this.text.scrollTop = top;
    this.refresh();
  }

  refresh() {
    const display = this.text.value.split("\n");
    // drop folds whose placeholder line was deleted or edited
    if (this.folds.size) {
      const present = new Set(display);
      const keep = new Set();
      const visit = (l) => {
        if (keep.has(l) || !this.folds.has(l)) return;
        keep.add(l);
        this.folds.get(l).forEach(visit);
      };
      present.forEach(visit);
      for (const k of [...this.folds.keys()]) if (!keep.has(k)) this.folds.delete(k);
    }
    const { lines: b, starts, spans } = this._expand(display);
    this._starts = starts;
    this._spans = spans;
    const a = this.base ? this.base.split("\n") : [];
    this.hunks = this.base ? diffLines(a, b) : [];
    const marks = new Array(b.length).fill(null);
    const delBefore = new Map();
    this.hunks.forEach((h, i) => {
      if (h.kind === "del") {
        const d = h.bStart >= b.length ? display.length : this._displayLine(h.bStart);
        if (!delBefore.has(d)) delBefore.set(d, i);
      } else for (let j = h.bStart; j < h.bStart + h.bLines.length; j++) marks[j] = [h.kind, i];
    });
    const rows = [];
    for (let i = 0; i < display.length; i++) {
      let m = null;
      for (let j = starts[i]; j < starts[i] + spans[i]; j++) {
        if (marks[j]) {
          m = spans[i] > 1 ? ["mod", marks[j][1]] : marks[j];
          break;
        }
      }
      const del = delBefore.has(i) ? `<span class="del-mark" data-hunk="${delBefore.get(i)}" title="deleted lines, click to show"></span>` : "";
      const cls = m ? `mark ${m[0]}` : "mark";
      const what = spans[i] > 1 ? "changes in the folded lines" : m && m[0] === "add" ? "added" : "changed";
      const attr = m ? ` data-hunk="${m[1]}" title="${what}, click to show"` : "";
      const folded = this.folds.has(display[i]);
      const fold = folded
        ? `<span class="fold folded" data-fold="${i}" title="unfold ${spans[i]} lines">&#9656;</span>`
        : opensBlock(display[i])
          ? `<span class="fold" data-fold="${i}" title="fold">&#9662;</span>`
          : "";
      rows.push(`<div class="ln">${del}<span class="${cls}"${attr}></span>${fold}<span class="num">${starts[i] + 1}</span></div>`);
    }
    if (delBefore.has(display.length)) rows.push(`<div class="ln"><span class="del-mark" data-hunk="${delBefore.get(display.length)}"></span></div>`);
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
    const line = this._displayLine(h.bStart + Math.max(h.bLines.length, 1) - 1) + 1;
    const top = pad + line * this._lineHeight() - this.text.scrollTop;
    this.peek.style.top = `${Math.max(0, Math.min(top, this.root.clientHeight - 40))}px`;
  }

  openPeek(i) {
    const h = this.hunks[i];
    if (!h) return;
    this._unfoldAt(h);
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
    this._unfoldAt(h);
    this.text.scrollTop = Math.max(0, this._displayLine(h.bStart) * this._lineHeight() - this.text.clientHeight / 3);
    this.openPeek(next);
  }

  // unfold the blocks that hide lines of a change, so that it can be shown and edited
  _unfoldAt(h) {
    for (let guard = 0; guard < 64; guard++) {
      const first = this._displayLine(h.bStart);
      const last = this._displayLine(h.bStart + Math.max(h.bLines.length, 1) - 1);
      const display = this.text.value.split("\n");
      let i = -1;
      for (let d = first; d <= last && d < display.length; d++) if (this.folds.has(display[d])) i = d;
      if (i < 0) return;
      const line = display[i];
      display.splice(i, 1, ...this.folds.get(line));
      this.folds.delete(line);
      this.text.value = display.join("\n");
      this.refresh();
    }
  }

  revert(i) {
    const h = this.hunks[i];
    if (!h) return;
    this._unfoldAt(h);
    const lines = this.value().split("\n");
    lines.splice(h.bStart, h.bLines.length, ...h.aLines);
    const top = this.text.scrollTop;
    this.folds.clear();
    this.text.value = lines.join("\n");
    this.text.scrollTop = top;
    this.closePeek();
    this.refresh();
  }

  revertAll() {
    const top = this.text.scrollTop;
    this.folds.clear();
    this.text.value = this.base;
    this.text.scrollTop = top;
    this.closePeek();
    this.refresh();
  }
}

window.DiffEditor = DiffEditor;
window.formatJson = formatJson;
window.diffLines = diffLines;
