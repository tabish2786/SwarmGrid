// Shared drawing code for docs/index.html (replay) and docs/control.html (interactive).
// A "plan" is the swarmgrid-plan-v1 JSON the C++ planner writes:
//   { map: {width, height, rows}, agents: [{id, start, goal, path: [[x, y, t], ...]}], constraints: [...] }
(function (global) {
  "use strict";

  const darkQuery = global.matchMedia("(prefers-color-scheme: dark)");

  function css(name) {
    return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  }

  function agentColor(i) {
    const hue = (i * 137.508 + 210) % 360;  // golden angle: neighbours get far-apart hues
    return darkQuery.matches ? `hsl(${hue} 75% 62%)` : `hsl(${hue} 70% 44%)`;
  }

  // Robot position at integer time t. After its path ends a robot stays on its last cell (its
  // goal), the same convention the planner uses.
  function posAt(agent, t) {
    const p = agent.path;
    const s = p[Math.min(Math.max(t, 0), p.length - 1)];
    return [s[0], s[1]];
  }

  function isPlan(p) {
    return !!p && p.format === "swarmgrid-plan-v1" && !!p.map && Array.isArray(p.map.rows) &&
      Number.isInteger(p.map.width) && Number.isInteger(p.map.height) &&
      p.map.rows.length === p.map.height &&
      p.map.rows.every((r) => typeof r === "string" && r.length === p.map.width) &&
      Array.isArray(p.agents) && p.agents.every((a) => Array.isArray(a.path) && a.path.length > 0 &&
        a.path.every((s) => Array.isArray(s) && s.length >= 2 && Number.isInteger(s[0]) && Number.isInteger(s[1])));
  }

  // Collisions between the drawn paths, found here in the browser:
  //   vertex: two robots on one cell at the same time
  //   target: same, but one of them has already finished and is parked on its goal
  //   edge:   two robots swap cells between t-1 and t
  function findCollisions(plan) {
    const out = [];
    const n = plan.agents.length;
    const W = plan.map.width;
    for (let t = 0; t <= plan.makespan; t++) {
      const byCell = new Map();
      for (let i = 0; i < n; i++) {
        const [x, y] = posAt(plan.agents[i], t);
        const key = y * W + x;
        if (!byCell.has(key)) byCell.set(key, []);
        byCell.get(key).push(i);
      }
      for (const [key, agents] of byCell) {
        if (agents.length < 2) continue;
        const parked = agents.some((i) => t >= plan.agents[i].path.length - 1);
        out.push({ type: parked ? "target" : "vertex", t, agents, cell: [key % W, Math.floor(key / W)] });
      }
      if (t === 0) continue;
      const moves = new Map();
      for (let i = 0; i < n; i++) {
        const a = posAt(plan.agents[i], t - 1);
        const b = posAt(plan.agents[i], t);
        if (a[0] === b[0] && a[1] === b[1]) continue;
        moves.set(`${a}>${b}`, i);
      }
      for (const [k, i] of moves) {
        const [from, to] = k.split(">");
        const j = moves.get(`${to}>${from}`);
        if (j !== undefined && i < j) {
          out.push({ type: "edge", t, agents: [i, j], cell: from.split(",").map(Number), to: to.split(",").map(Number) });
        }
      }
    }
    return out;
  }

  // Adds makespan, sumOfCosts and collisions to a plan (in place) and returns it.
  function prepare(plan) {
    plan.makespan = plan.agents.reduce((m, a) => Math.max(m, a.path.length - 1), 0);
    plan.sumOfCosts = plan.agents.reduce((s, a) => s + a.path.length - 1, 0);
    plan.constraints = Array.isArray(plan.constraints) ? plan.constraints : [];
    plan.collisions = findCollisions(plan);
    return plan;
  }

  function roundRect(g, x, y, w, h, r) {
    g.beginPath();
    g.moveTo(x + r, y);
    g.arcTo(x + w, y, x + w, y + h, r);
    g.arcTo(x + w, y + h, x, y + h, r);
    g.arcTo(x, y + h, x, y, r);
    g.arcTo(x, y, x + w, y, r);
    g.closePath();
  }

  class PlanView {
    constructor(canvas) {
      this.canvas = canvas;
      this.ctx = canvas.getContext("2d");
      this.plan = null;
      this.cell = 10;
      this.dpr = 1;
      this.mapLayer = null;
      if (darkQuery.addEventListener) darkQuery.addEventListener("change", () => this.rebuildMap());
    }

    setPlan(plan) {
      const sameMap = this.plan && this.plan.map.width === plan.map.width &&
        this.plan.map.height === plan.map.height && this.plan.map.rows.join("") === plan.map.rows.join("");
      this.plan = plan;
      if (!sameMap) this.mapLayer = null;
    }

    // Picks the largest whole-pixel cell size that fits maxW x maxH.
    resize(maxW, maxH) {
      const plan = this.plan;
      if (!plan) return;
      const cell = Math.max(4, Math.floor(Math.min(maxW / plan.map.width, maxH / plan.map.height)));
      const dpr = Math.min(global.devicePixelRatio || 1, 3);
      this.cell = cell;
      this.dpr = dpr;
      this.canvas.style.width = cell * plan.map.width + "px";
      this.canvas.style.height = cell * plan.map.height + "px";
      this.canvas.width = Math.round(cell * plan.map.width * dpr);
      this.canvas.height = Math.round(cell * plan.map.height * dpr);
      this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      this.rebuildMap();
    }

    // Floor and shelves never change during playback, so they are drawn once into a cache.
    rebuildMap() {
      const plan = this.plan;
      if (!plan) return;
      const c = this.cell;
      const layer = document.createElement("canvas");
      layer.width = this.canvas.width;
      layer.height = this.canvas.height;
      const g = layer.getContext("2d");
      g.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
      g.fillStyle = css("--free");
      g.fillRect(0, 0, c * plan.map.width, c * plan.map.height);
      if (c >= 8) {
        g.strokeStyle = css("--gridline");
        g.lineWidth = 1;
        g.beginPath();
        for (let x = 0; x <= plan.map.width; x++) { g.moveTo(x * c + 0.5, 0); g.lineTo(x * c + 0.5, plan.map.height * c); }
        for (let y = 0; y <= plan.map.height; y++) { g.moveTo(0, y * c + 0.5); g.lineTo(plan.map.width * c, y * c + 0.5); }
        g.stroke();
      }
      g.fillStyle = css("--shelf");
      const r = Math.max(1, c * 0.12);
      plan.map.rows.forEach((row, y) => {
        for (let x = 0; x < row.length; x++) {
          if (row[x] === ".") continue;
          roundRect(g, x * c + 1, y * c + 1, c - 2, c - 2, r);
          g.fill();
        }
      });
      this.mapLayer = layer;
    }

    // Grid cell under a mouse/touch point, or null outside the map.
    cellAt(clientX, clientY) {
      if (!this.plan) return null;
      const rect = this.canvas.getBoundingClientRect();
      const x = Math.floor((clientX - rect.left) / this.cell);
      const y = Math.floor((clientY - rect.top) / this.cell);
      if (x < 0 || y < 0 || x >= this.plan.map.width || y >= this.plan.map.height) return null;
      return [x, y];
    }

    center(p) {
      const c = this.cell;
      return [p[0] * c + c / 2, p[1] * c + c / 2];
    }

    robotXY(agent, time) {
      const t0 = Math.floor(time);
      const f = time - t0;
      const p0 = this.center(posAt(agent, t0));
      const p1 = this.center(posAt(agent, t0 + 1));
      return [p0[0] + (p1[0] - p0[0]) * f, p0[1] + (p1[1] - p0[1]) * f];
    }

    drawCross(cx, cy, size, color) {
      const ctx = this.ctx;
      ctx.strokeStyle = color;
      ctx.lineWidth = Math.max(2, this.cell * 0.12);
      ctx.lineCap = "round";
      ctx.beginPath();
      ctx.moveTo(cx - size, cy - size); ctx.lineTo(cx + size, cy + size);
      ctx.moveTo(cx + size, cy - size); ctx.lineTo(cx - size, cy + size);
      ctx.stroke();
    }

    // opts.trails: draw where robots have been. opts.selected: robot index to highlight (its whole
    // planned route is drawn dashed), or -1.
    draw(time, opts) {
      const plan = this.plan;
      if (!plan) return;
      if (!this.mapLayer) this.rebuildMap();
      const o = Object.assign({ trails: true, selected: -1 }, opts || {});
      const ctx = this.ctx;
      const c = this.cell;
      const tFloor = Math.floor(time);
      const tNear = Math.round(time);
      const danger = css("--danger");

      ctx.save();
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
      ctx.drawImage(this.mapLayer, 0, 0);
      ctx.restore();

      // Constraints: faint all the time, strong while they apply.
      for (const k of plan.constraints) {
        if (k.type === "edge" && k.to) {
          const active = time > k.t - 1 && time <= k.t;
          const [x1, y1] = this.center(k.cell);
          const [x2, y2] = this.center(k.to);
          ctx.globalAlpha = active ? 1 : 0.28;
          ctx.strokeStyle = danger;
          ctx.lineWidth = Math.max(2, c * 0.1);
          ctx.setLineDash([c * 0.18, c * 0.14]);
          ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
          ctx.setLineDash([]);
          this.drawCross((x1 + x2) / 2, (y1 + y2) / 2, c * 0.16, danger);
        } else {
          const active = k.type === "blocked_from" ? tNear >= k.t : tNear === k.t;
          const [x, y] = this.center(k.cell);
          ctx.globalAlpha = active ? 1 : 0.28;
          this.drawCross(x, y, c * 0.26, danger);
        }
        ctx.globalAlpha = 1;
      }

      // Goals.
      plan.agents.forEach((a, i) => {
        const goal = a.goal || a.path[a.path.length - 1];
        const inset = c * 0.14;
        ctx.strokeStyle = agentColor(i);
        ctx.lineWidth = Math.max(1.5, c * (i === o.selected ? 0.16 : 0.1));
        roundRect(ctx, goal[0] * c + inset, goal[1] * c + inset, c - 2 * inset, c - 2 * inset, c * 0.12);
        ctx.stroke();
      });

      // The selected robot's whole route, dashed.
      if (o.selected >= 0 && o.selected < plan.agents.length) {
        const a = plan.agents[o.selected];
        ctx.strokeStyle = agentColor(o.selected);
        ctx.lineWidth = Math.max(1.5, c * 0.12);
        ctx.setLineDash([c * 0.25, c * 0.2]);
        ctx.lineCap = "round";
        ctx.beginPath();
        a.path.forEach((s, t) => {
          const [x, y] = this.center(s);
          if (t === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        });
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // Trails: where each robot has been so far.
      if (o.trails) {
        ctx.lineCap = "round";
        ctx.lineJoin = "round";
        plan.agents.forEach((a, i) => {
          const last = Math.min(tFloor, a.path.length - 1);
          if (last <= 0 && time === tFloor) return;
          ctx.strokeStyle = agentColor(i);
          ctx.globalAlpha = 0.38;
          ctx.lineWidth = Math.max(1.5, c * 0.16);
          ctx.beginPath();
          const s = this.center(posAt(a, 0));
          ctx.moveTo(s[0], s[1]);
          for (let t = 1; t <= last; t++) { const p = this.center(posAt(a, t)); ctx.lineTo(p[0], p[1]); }
          const p = this.robotXY(a, time);
          ctx.lineTo(p[0], p[1]);
          ctx.stroke();
          ctx.globalAlpha = 1;
        });
      }

      // Swaps happening during this step.
      for (const h of plan.collisions) {
        if (h.type !== "edge" || !(time > h.t - 1 && time <= h.t)) continue;
        const [x1, y1] = this.center(h.cell);
        const [x2, y2] = this.center(h.to);
        ctx.strokeStyle = danger;
        ctx.lineWidth = Math.max(3, c * 0.2);
        ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
      }

      // Robots.
      const radius = c * 0.36;
      plan.agents.forEach((a, i) => {
        const [x, y] = this.robotXY(a, time);
        if (i === o.selected) {
          ctx.strokeStyle = css("--text");
          ctx.lineWidth = Math.max(2, c * 0.1);
          ctx.beginPath(); ctx.arc(x, y, radius + c * 0.12, 0, Math.PI * 2); ctx.stroke();
        }
        ctx.fillStyle = agentColor(i);
        ctx.beginPath(); ctx.arc(x, y, radius, 0, Math.PI * 2); ctx.fill();
        ctx.strokeStyle = "rgba(255,255,255,0.9)";
        ctx.lineWidth = Math.max(1, c * 0.06);
        ctx.stroke();
        if (c >= 15) {
          ctx.fillStyle = "#ffffff";
          ctx.font = `700 ${Math.round(c * (i >= 10 ? 0.34 : 0.42))}px system-ui, sans-serif`;
          ctx.textAlign = "center";
          ctx.textBaseline = "middle";
          ctx.fillText(String(a.id !== undefined ? a.id : i), x, y + 0.5);
        }
      });

      // Vertex / target collisions at the nearest whole timestep.
      if (Math.abs(time - tNear) < 0.35) {
        for (const h of plan.collisions) {
          if (h.t !== tNear || h.type === "edge") continue;
          const [x, y] = this.center(h.cell);
          ctx.strokeStyle = danger;
          ctx.lineWidth = Math.max(2.5, c * 0.13);
          ctx.beginPath(); ctx.arc(x, y, c * 0.52, 0, Math.PI * 2); ctx.stroke();
        }
      }

      // Constraint times last, on a small pill, so goals and robots never hide them.
      if (c >= 18) {
        ctx.font = `700 ${Math.round(c * 0.24)}px system-ui, sans-serif`;
        ctx.textAlign = "left";
        ctx.textBaseline = "top";
        for (const k of plan.constraints) {
          const label = (k.type === "blocked_from" ? "t≥" : "t=") + k.t;
          const x = k.cell[0] * c + c * 0.05;
          const y = k.cell[1] * c + c * 0.04;
          const w = ctx.measureText(label).width + c * 0.1;
          ctx.globalAlpha = 0.92;
          ctx.fillStyle = css("--card");
          roundRect(ctx, x, y, w, c * 0.3, c * 0.08);
          ctx.fill();
          ctx.globalAlpha = 1;
          ctx.fillStyle = danger;
          ctx.fillText(label, x + c * 0.05, y + c * 0.03);
        }
      }
    }
  }

  global.SwarmGridView = { agentColor, posAt, isPlan, findCollisions, prepare, PlanView };
})(window);
