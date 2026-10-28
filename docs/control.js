// Interactive robot control for docs/control.html.
// Planning and order assignment are the project's C++ code compiled to WebAssembly
// (docs/swarmgrid-planner.js, see web/wasm_api.cpp). Planning runs in a Web Worker
// (docs/planner-worker.js) so a long search never freezes the page; opened from file://, where
// browsers refuse workers, it runs on the page instead. This file handles taps, the order loop
// and drawing.
(function () {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const V = window.SwarmGridView;
  const WAITING_ORDERS = 6;                           // parcels kept on the floor while orders run
  const ORDER_LIMITS = { nodes: 3000, seconds: 0.4 }; // CBS/ECBS inside the order loop (then prioritized)
  // After a tap. In the worker the page stays responsive (and there is a Cancel button), so the
  // search may run longer than on the page itself.
  const tapLimits = () => planner.inWorker() ? { nodes: 200000, seconds: 15 } : { nodes: 20000, seconds: 3 };
  const view = new V.PlanView($("map"));
  const reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;

  // Small seeded random generator, so a run of orders can be repeated (Math.random can't be seeded).
  function mulberry32(seed) {
    let a = seed >>> 0;
    return function () {
      a = (a + 0x6D2B79F5) >>> 0;
      let t = a;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  // ---------- planner: Web Worker, or the page itself as a fallback ----------

  const planner = (function () {
    let worker = null;
    let pending = null;  // { id, resolve, request }
    let nextId = 1;

    // The same steps as planner-worker.js, on the page's own copy of the planner (whose map
    // already has the shelves and broken robots, so `blocked` is not needed).
    function runHere(q) {
      const M = state.M;
      M._sg_set_ecbs_weight(q.weight);
      M._sg_clear_agents();
      for (let i = 0; i < q.starts.length; i++) M._sg_add_agent(q.starts[i], q.goals[i]);
      const t0 = performance.now();
      const status = M._sg_plan_with_limits(q.mode, q.nodes, q.seconds);
      const out = {
        id: q.id, status, ms: performance.now() - t0, ctNodes: M._sg_ct_nodes(),
        limitReached: M._sg_limit_reached() === 1, attempts: M._sg_attempts(), failed: M._sg_failed_agent(),
        error: status < 0 ? M.UTF8ToString(M._sg_last_error()) : "", lowerBound: M._sg_lower_bound(), paths: null,
      };
      if (status === 1) {
        out.paths = [];
        for (let k = 0; k < q.starts.length; k++) {
          const p = [];
          for (let t = 0; t < M._sg_path_length(k); t++) p.push(M._sg_path_cell(k, t));
          out.paths.push(p);
        }
      }
      return out;
    }

    function start() {
      try {
        worker = new Worker("planner-worker.js");
      } catch (e) {
        worker = null;  // file:// pages: plan on the page
        return;
      }
      worker.onmessage = (e) => {
        if (!pending || e.data.id !== pending.id) return;
        const p = pending;
        pending = null;
        p.resolve(e.data);
      };
      worker.onerror = (e) => {  // the worker script could not load: plan on the page from now on
        if (e && e.preventDefault) e.preventDefault();
        worker.terminate();
        worker = null;
        if (pending) {
          const p = pending;
          pending = null;
          p.resolve(runHere(p.request));
        }
      };
    }

    return {
      start,
      inWorker: () => worker !== null,
      plan(q) {
        q.id = nextId++;
        if (!worker) return new Promise((resolve) => setTimeout(() => resolve(runHere(q)), 0));
        return new Promise((resolve) => {
          pending = { id: q.id, resolve, request: q };
          worker.postMessage(q);
        });
      },
      // Stops a search in progress: the worker is thrown away and a fresh one started.
      cancel() {
        if (!pending || !worker) return false;
        worker.terminate();
        const p = pending;
        pending = null;
        start();
        p.resolve({ id: p.id, status: -2, cancelled: true });
        return true;
      },
    };
  })();

  const state = {
    M: null,            // the WebAssembly planner (the page's copy: map, assignment, scenes)
    n: 10,              // robots
    busy: false,        // a plan is being computed: the clock stands still, taps wait
    W: 0,
    H: 0,
    rows: [],           // shelves only ('@'); broken robots are drawn as robots, not shelves
    starts: [],         // per robot: cell where the current plan starts
    goals: [],          // per robot: cell where it ends
    broken: new Map(),  // robot -> the cell it broke down on (blocked for the planner)
    removedShelves: new Set(),  // default shelves you deleted (they come back on "Reset map")
    plan: null,
    time: 0,
    playing: false,
    lastFrame: 0,
    selected: 0,
    tapMode: "goal",    // "goal" | "start" | "shelf"
    seed: 42,
    lastShownT: -1,
    orders: {
      running: false,
      list: [],         // { cell, robot }  robot = -1 while nobody has taken it
      done: 0,
      simTime: 0,       // timesteps simulated before the current plan started
      rng: mulberry32(7),
      lastTry: 0,       // wall-clock time of the last assignment attempt
      fallbacks: 0,     // replans where CBS/ECBS hit its limit and prioritized planning took over
      distance: 0,      // total robot -> parcel distance of every assignment so far
    },
  };

  const toXY = (c) => [c % state.W, Math.floor(c / state.W)];
  const toId = (x, y) => y * state.W + x;
  const fmtXY = (c) => { const [x, y] = toXY(c); return `(${x}, ${y})`; };
  // kind "info" for notices (muted), anything else for problems (red).
  const setMessage = (text, kind) => { $("msg").textContent = text; $("msg").classList.toggle("info", kind === "info"); };
  const plannerMode = () => Number($("planner").value);
  const ecbsWeight = () => Number($("weight").value);
  const plannerName = (mode) => mode === 4 ? "CBS + ML conflict order (optimal)"
    : mode === 3 ? `ECBS (w = ${ecbsWeight()})`
    : ["independent A*", "prioritized planning", "CBS (optimal)"][mode];
  const isWorking = (r) => !state.broken.has(r);
  const robotIds = () => [...Array(state.n).keys()];

  // ---------- map ----------

  function readRows() {
    const brokenCells = new Set(state.broken.values());
    state.rows = [];
    for (let y = 0; y < state.H; y++) {
      let row = "";
      for (let x = 0; x < state.W; x++) {
        const c = toId(x, y);
        row += state.M._sg_is_free(c) || brokenCells.has(c) ? "." : "@";
      }
      state.rows.push(row);
    }
  }

  const isShelf = (x, y) => x >= 0 && y >= 0 && x < state.W && y < state.H && state.rows[y][x] === "@";

  // ---------- planning ----------

  // Plans every working robot from starts[r] to goals[r]. Broken robots are blocked cells to the
  // planner and simply stay where they are.
  async function runPlanner(starts, goals, mode, limits) {
    const M = state.M;
    const working = robotIds().filter(isWorking);
    const res = {
      status: 1, mode, ms: 0, paths: null, failed: -1, error: "", ctNodes: 0, limitReached: false,
      attempts: 0, lowerBound: 0, cancelled: false,
    };
    if (working.length) {
      const blocked = new Uint8Array(state.W * state.H);
      for (let c = 0; c < blocked.length; c++) blocked[c] = M._sg_is_free(c) ? 0 : 1;
      const raw = await planner.plan({
        blocked, starts: working.map((r) => starts[r]), goals: working.map((r) => goals[r]),
        mode, nodes: limits.nodes, seconds: limits.seconds, weight: ecbsWeight(),
      });
      Object.assign(res, {
        status: raw.status, ms: raw.ms || 0, error: raw.error || "", ctNodes: raw.ctNodes || 0,
        limitReached: !!raw.limitReached, attempts: raw.attempts || 0, lowerBound: raw.lowerBound || 0,
        cancelled: !!raw.cancelled,
      });
      if (raw.status === 0 && mode === 1 && raw.failed >= 0) res.failed = working[raw.failed];
      if (raw.status === 1) {
        res.paths = new Array(state.n);
        working.forEach((r, k) => {
          res.paths[r] = raw.paths[k].map((cell, t) => { const [x, y] = toXY(cell); return [x, y, t]; });
        });
      }
    } else {
      res.paths = new Array(state.n);
    }
    if (res.status === 1) {
      for (const [r, cell] of state.broken) { const [x, y] = toXY(cell); res.paths[r] = [[x, y, 0]]; }
    }
    return res;
  }

  const costOf = (res) => res.status !== 1 ? null
    : res.paths.reduce((s, p, r) => s + (isWorking(r) ? p.length - 1 : 0), 0);

  function failureText(res) {
    if (res.cancelled) return "Planning cancelled. The robots keep their previous plan.";
    if (res.status < 0) return `Planner error: ${res.error}`;
    const name = res.mode === 3 ? "ECBS" : "CBS";
    if (res.mode >= 2 && res.limitReached) {
      return `${name} stopped after ${res.ctNodes.toLocaleString("en-US")} constraint-tree nodes without finishing. Try another cell, ${res.mode === 3 ? "a larger w, " : "ECBS, "}or the prioritized planner.`;
    }
    if (res.mode >= 2) return `${name} proved there is no collision-free plan for that.`;
    if (res.mode === 1 && res.failed >= 0) {
      return `No collision-free plan for that: robot ${res.failed} got stuck. Prioritized planning can miss solutions; try another cell or CBS.`;
    }
    return "Some robot cannot reach its goal any more.";
  }

  // While a plan is computed the clock stands still (the new plan starts where the robots are
  // now), taps are refused, and after a moment a Cancel button appears.
  let cancelTimer = 0;
  const BUSY_LOCKED = ["planner", "weight", "count", "shuffle", "reset-map", "break"];
  function setBusy(on) {
    state.busy = on;
    document.documentElement.dataset.busy = on ? "1" : "";
    for (const id of BUSY_LOCKED) $(id).disabled = on;  // Start/Stop orders stays: stopping cancels
    clearTimeout(cancelTimer);
    if (on) {
      cancelTimer = setTimeout(() => {
        if (!state.busy) return;
        $("status").textContent = `Planning with ${plannerName(plannerMode())}${planner.inWorker() ? " in a background worker" : ""}…`;
        $("cancel").hidden = !planner.inWorker();
      }, 250);
    } else {
      $("cancel").hidden = true;
    }
  }

  // Plans new starts/goals. On failure nothing changes, so the robots never jump.
  // opts.orderLoop: quick CBS/ECBS limit with a prioritized fallback. opts.quiet: don't touch the message.
  async function tryReplan(starts, goals, opts) {
    const o = opts || {};
    const mode = plannerMode();
    setBusy(true);
    try {
      let res = await runPlanner(starts, goals, mode, o.orderLoop ? ORDER_LIMITS : tapLimits());
      if (res.status === 0 && mode >= 2 && o.orderLoop) {
        const fallback = await runPlanner(starts, goals, 1, tapLimits());
        if (fallback.status === 1) { res = fallback; state.orders.fallbacks++; }
      }
      state.cancelled = res.cancelled;
      if (res.status !== 1) {
        if (!o.quiet || res.cancelled) setMessage(failureText(res));
        if (state.plan) showStatus(state.lastRes);
        return false;
      }
      if (!o.quiet) setMessage("");
      const prioritizedCost = mode >= 2 && res.mode >= 2 && !o.orderLoop
        ? costOf(await runPlanner(starts, goals, 1, tapLimits())) : null;

      state.starts = starts.slice();
      state.goals = goals.slice();
      state.plan = V.prepare({
        map: { width: state.W, height: state.H, rows: state.rows },
        agents: res.paths.map((path, i) => ({ id: i, start: toXY(starts[i]), goal: toXY(goals[i]), path })),
        constraints: [],
      });
      view.setPlan(state.plan);
      state.time = 0;
      state.lastShownT = -1;
      state.lastRes = res;
      showStats(res, prioritizedCost);
      layout();
      return true;
    } finally {
      setBusy(false);
    }
  }

  // Robot cells at integer time t of the current plan.
  function positionsAt(t) {
    return state.plan.agents.map((a) => { const [x, y] = V.posAt(a, t); return toId(x, y); });
  }

  // Where every robot is right now. The independent planner lets robots overlap; then there is
  // no valid "now", so we fall back to where the plan started.
  function currentPositions() {
    if (!state.plan) return state.starts.slice();
    const pos = positionsAt(Math.round(state.time));
    return new Set(pos).size === pos.length ? pos : state.starts.slice();
  }

  // ---------- orders (lifelong mode) ----------

  function orderAt(cell) { return state.orders.list.find((o) => o.cell === cell); }
  function orderOf(robot) { return state.orders.list.find((o) => o.robot === robot); }

  // New parcels on free aisle cells next to a shelf (where a picker would stop), away from robots,
  // goals and other parcels.
  function spawnOrders(positions) {
    const ord = state.orders;
    const taken = new Set([...positions, ...state.goals, ...ord.list.map((o) => o.cell)]);
    const candidates = [];
    for (let c = 0; c < state.W * state.H; c++) {
      if (!state.M._sg_is_free(c) || taken.has(c)) continue;
      const [x, y] = toXY(c);
      if (isShelf(x + 1, y) || isShelf(x - 1, y) || isShelf(x, y + 1) || isShelf(x, y - 1)) candidates.push(c);
    }
    while (ord.list.filter((o) => o.robot < 0).length < WAITING_ORDERS && candidates.length) {
      const i = Math.floor(ord.rng() * candidates.length);
      ord.list.push({ cell: candidates[i], robot: -1 });
      candidates.splice(i, 1);
    }
  }

  // Idle working robots take waiting parcels, computed in C++: greedy nearest pair, or the
  // optimal (Hungarian) assignment that minimises the robots' total distance to their parcels.
  // Returns how many parcels were handed out.
  function assignOrders(positions) {
    const M = state.M;
    const idle = robotIds().filter((r) => isWorking(r) && !orderOf(r));
    const waiting = state.orders.list.filter((o) => o.robot < 0);
    if (!idle.length || !waiting.length) return 0;
    M._sg_assign_clear();
    for (const r of idle) M._sg_assign_add_robot(positions[r]);
    for (const o of waiting) M._sg_assign_add_order(o.cell);
    const pairs = Math.max(0, M._sg_assign_run_policy($("assign").value === "hungarian" ? 1 : 0));
    for (let i = 0; i < pairs; i++) {
      waiting[M._sg_assign_order(i)].robot = idle[M._sg_assign_robot(i)];
      state.orders.distance += M._sg_assign_distance(i);
    }
    return pairs;
  }

  // The cell robot r's current plan ends on.
  function planGoal(r) {
    const path = state.plan.agents[r].path;
    const end = path[path.length - 1];
    return toId(end[0], end[1]);
  }

  // A robot with a parcel drives to it; everyone else stays where they are.
  function goalsFromOrders(positions) {
    const goals = positions.slice();
    for (const o of state.orders.list) if (o.robot >= 0) goals[o.robot] = o.cell;
    return goals;
  }

  // Called every frame while orders run: count pickups, top up parcels, hand them out, replan.
  // Pickups are handled at once; everything else (new parcels for idle robots, a replan that
  // failed last time) is retried at most every 0.7 s.
  async function orderTick(now) {
    const ord = state.orders;
    const p = state.plan;
    if (state.busy) return;
    const t = Math.min(Math.floor(state.time), p.makespan);
    const arrived = ord.list.filter((o) => o.robot >= 0 && planGoal(o.robot) === o.cell &&
      t >= p.agents[o.robot].path.length - 1);
    const stale = ord.list.some((o) => o.robot >= 0 && planGoal(o.robot) !== o.cell);
    const idleWithWork = ord.list.some((o) => o.robot < 0) &&
      robotIds().some((r) => isWorking(r) && !orderOf(r));
    const needsParcels = ord.list.filter((o) => o.robot < 0).length < WAITING_ORDERS;
    if (!arrived.length && !((idleWithWork || stale || needsParcels) && now - ord.lastTry > 700)) return;
    ord.lastTry = now;

    const positions = positionsAt(t);
    ord.list = ord.list.filter((o) => !arrived.includes(o));
    ord.done += arrived.length;
    spawnOrders(positions);
    const handedOut = assignOrders(positions);
    // Goals only change on a pickup, a new assignment, or an earlier replan that failed.
    if (arrived.length || handedOut || stale) {
      if (await tryReplan(positions, goalsFromOrders(positions), { orderLoop: true, quiet: true })) {
        ord.simTime += t;
        setMessage("");
      } else if (!ord.running) {
        return;  // stopped (or cancelled) while planning
      } else {
        setMessage("Couldn't plan the new orders this time; trying again in a moment.");
      }
    }
    showOrders();
    draw();
  }

  async function setOrdersRunning(on) {
    const ord = state.orders;
    if (state.busy) {
      if (on) return;
      planner.cancel();  // stopping wins over a replan in progress
    }
    if (on) {
      if (plannerMode() === 0) { $("planner").value = "1"; syncPlannerControls(); }  // overlapping robots can't continue
      ord.running = true;
      ord.list = [];
      ord.lastTry = 0;
      $("planner").querySelector('option[value="0"]').disabled = true;
      const positions = currentPositions();
      spawnOrders(positions);
      assignOrders(positions);
      if (!(await tryReplan(positions, goalsFromOrders(positions), { orderLoop: true }))) ord.list = [];
      setPlaying(true);
    } else {
      ord.running = false;
      ord.list = [];
      $("planner").querySelector('option[value="0"]').disabled = false;
    }
    $("orders-toggle").textContent = on ? "Stop orders" : "Start orders";
    $("orders-toggle").setAttribute("aria-pressed", String(on));
    showOrders();
    showHint();
    state.lastShownT = -1;
    draw();
  }

  function showOrders() {
    const ord = state.orders;
    const simulated = ord.simTime + (ord.running && state.plan ? Math.min(Math.floor(state.time), state.plan.makespan) : 0);
    $("or-done").textContent = ord.done;
    $("or-open").textContent = ord.list.filter((o) => o.robot < 0).length;
    $("or-time").textContent = simulated;
    $("or-rate").textContent = simulated > 0 ? (ord.done / simulated * 100).toFixed(1) : "–";
    const how = $("assign").value === "hungarian"
      ? "Parcels go to robots by the optimal (Hungarian) assignment"
      : "Each parcel goes to the nearest free robot";
    $("or-note").textContent = ord.running
      ? `${how}; the fleet replans at every pickup. Robots have driven ${ord.distance} cells to reach their parcels so far.` +
        (ord.fallbacks ? ` CBS/ECBS needed more than 0.4 s ${ord.fallbacks} time${ord.fallbacks === 1 ? "" : "s"}; prioritized planning took those replans.` : "")
      : `Press Start orders: parcels appear next to shelves. ${how}.`;
  }

  // ---------- taps and buttons ----------

  async function onMapTap(ev) {
    const cell = view.cellAt(ev.clientX, ev.clientY);
    if (!cell || !state.plan) return;
    if (state.busy) { setMessage("Still planning the last change. Wait a moment, or press Cancel."); return; }
    const id = toId(cell[0], cell[1]);
    const sel = state.selected;
    const now = currentPositions();
    const robotHere = now.indexOf(id);

    if (state.tapMode === "shelf") { toggleShelf(id, now, robotHere); return; }
    if (robotHere >= 0 && robotHere !== sel) { select(robotHere); setMessage(""); return; }
    if (state.orders.running) { setMessage("Orders are running, so robots pick their own goals. Stop orders to drive one by hand."); return; }
    if (!isWorking(sel)) { setMessage(`Robot ${sel} is broken. Repair it first.`); return; }
    if (!state.M._sg_is_free(id)) { setMessage("That's a shelf. Robots can only use the aisles."); return; }

    if (state.tapMode === "goal") {
      const owner = state.goals.indexOf(id);
      if (owner >= 0 && owner !== sel) { setMessage(`That cell is robot ${owner}'s goal. Every robot needs its own goal.`); return; }
      const goals = state.goals.slice();
      goals[sel] = id;
      if (await tryReplan(now, goals)) setPlaying(true);
    } else {
      if (robotHere === sel) return;
      const starts = now.slice();
      starts[sel] = id;
      if (await tryReplan(starts, state.goals.slice())) setPlaying(true);
    }
  }

  async function toggleShelf(id, now, robotHere) {
    if (robotHere >= 0) { setMessage(`Robot ${robotHere} is standing there.`); return; }
    const owner = state.goals.indexOf(id);
    if (owner >= 0) { setMessage(`That cell is robot ${owner}'s goal.`); return; }
    if (orderAt(id)) { setMessage("There is a parcel there."); return; }
    if ([...state.broken.values()].includes(id)) { setMessage("A broken robot is standing there."); return; }
    const wasFree = state.M._sg_is_free(id) === 1;
    state.M._sg_set_obstacle(id, wasFree ? 1 : 0);
    readRows();
    if (await tryReplan(now, state.goals.slice(), { orderLoop: state.orders.running })) {
      // Remember which of the warehouse's own shelves are gone, so "Reset map" can check them.
      if (wasFree) state.removedShelves.delete(id); else state.removedShelves.add(id);
      setPlaying(true);
    } else {
      state.M._sg_set_obstacle(id, wasFree ? 0 : 1);  // undo: that shelf would trap someone
      readRows();
      view.setPlan(state.plan);
      view.rebuildMap();
      draw();
    }
  }

  async function toggleBroken() {
    if (state.busy) return;
    const r = state.selected;
    const now = currentPositions();
    const goals = state.goals.slice();
    const M = state.M;
    if (state.broken.has(r)) {
      const cell = state.broken.get(r);
      state.broken.delete(r);
      M._sg_set_obstacle(cell, 0);
      goals[r] = now[r];  // back to work, idle for now
      readRows();
      if (!(await tryReplan(now, goals, { orderLoop: state.orders.running }))) {
        state.broken.set(r, cell);
        M._sg_set_obstacle(cell, 1);
        readRows();
      }
    } else {
      const cell = now[r];
      state.broken.set(r, cell);
      M._sg_set_obstacle(cell, 1);
      // Its parcel goes back on the list; anyone heading for this cell has to give it up.
      for (const o of state.orders.list) if (o.robot === r) o.robot = -1;
      state.orders.list = state.orders.list.filter((o) => o.cell !== cell);
      for (let k = 0; k < state.n; k++) {
        if (k !== r && goals[k] === cell) {
          goals[k] = now[k];
          for (const o of state.orders.list) if (o.robot === k) o.robot = -1;
        }
      }
      goals[r] = cell;
      readRows();
      if (!(await tryReplan(now, goals, { orderLoop: state.orders.running }))) {
        state.broken.delete(r);
        M._sg_set_obstacle(cell, 0);
        readRows();
      }
    }
    showOrders();
    state.lastShownT = -1;
    draw();
    setPlaying(true);
  }

  async function resetMap() {
    if (state.busy) return;
    const M = state.M;
    // Shelves you removed come back: nobody may be standing on, driving to, or waiting at them.
    const now = currentPositions();
    for (const cell of state.removedShelves) {
      const who = now.indexOf(cell);
      const owner = state.goals.indexOf(cell);
      if (who >= 0 || owner >= 0 || orderAt(cell)) {
        setMessage(`Can't reset yet: ${who >= 0 ? `robot ${who} is standing` : owner >= 0 ? `robot ${owner} is heading` : "a parcel is waiting"} at ${fmtXY(cell)}, where a shelf would come back.`);
        return;
      }
    }
    state.removedShelves.clear();
    M._sg_reset_map();
    for (const cell of state.broken.values()) M._sg_set_obstacle(cell, 1);
    readRows();
    if (await tryReplan(currentPositions(), state.goals.slice(), { orderLoop: state.orders.running })) setPlaying(true);
  }

  // A fresh random scene (repairs every robot, keeps your shelves). Prioritized planning can fail
  // on one, so try a few seeds.
  async function shuffle(keepSeed) {
    if (state.busy) return;
    if (state.orders.running) await setOrdersRunning(false);
    const M = state.M;
    for (const cell of state.broken.values()) M._sg_set_obstacle(cell, 0);
    state.broken.clear();
    readRows();
    for (let attempt = 0; attempt < 5; attempt++) {
      if (!keepSeed || attempt > 0) state.seed += 1;
      if (M._sg_random_agents(state.n, state.seed) !== state.n) continue;
      const starts = [];
      const goals = [];
      for (let i = 0; i < state.n; i++) { starts.push(M._sg_agent_start(i)); goals.push(M._sg_agent_goal(i)); }
      if (await tryReplan(starts, goals)) { setPlaying(true); return; }
      if (state.cancelled) return;  // Cancel stops the retries too
    }
  }

  // More robots: CBS is optimal but can't keep up with a crowded floor, so switch to ECBS.
  async function setRobotCount(n) {
    if (state.busy) { $("count").value = String(state.n); return; }
    const before = state.n;
    state.n = n;
    buildRobotButtons();
    select(0);
    const notes = [];
    if (n > 10 && (plannerMode() === 2 || plannerMode() === 4)) {
      $("planner").value = "3";
      notes.push("switched to ECBS: CBS is optimal but gets slow on a crowded floor");
    }
    if (n >= 50 && plannerMode() === 3 && ecbsWeight() < 1.5) {
      $("weight").value = "1.5";
      notes.push("w = 1.5: at 50 robots this small map is crowded (1-wide aisles) and a tighter bound often takes many seconds");
    }
    syncPlannerControls();
    await shuffle(true);
    if (!state.plan || state.plan.agents.length !== n) {  // no plan for the new fleet: keep the old one
      state.n = before;
      $("count").value = String(before);
      buildRobotButtons();
      select(0);
      return;
    }
    if (notes.length && !$("msg").textContent) setMessage(`${n} robots: ${notes.join("; ")}.`, "info");
    $("map").setAttribute("aria-label", `Warehouse map with ${n} robots`);
  }

  function syncPlannerControls() {
    $("weight-wrap").hidden = plannerMode() !== 3;
  }

  // ---------- display ----------

  function buildRobotButtons() {
    const box = $("robots");
    box.textContent = "";
    box.classList.toggle("compact", state.n > 10);
    for (let i = 0; i < state.n; i++) {
      const b = document.createElement("button");
      b.className = "robot-btn";
      b.id = `robot-${i}`;
      b.setAttribute("aria-pressed", "false");
      b.setAttribute("aria-label", `Robot ${i}`);
      b.innerHTML = `<span class="dot"></span><span class="st"></span>`;
      b.querySelector(".dot").textContent = String(i);
      b.addEventListener("click", () => select(i));
      box.appendChild(b);
    }
  }

  function select(i) {
    state.selected = i;
    for (let k = 0; k < state.n; k++) $(`robot-${k}`).setAttribute("aria-pressed", String(k === i));
    showHint();
    state.lastShownT = -1;
    draw();
  }

  function setTapMode(mode) {
    state.tapMode = mode;
    for (const [id, m] of [["mode-goal", "goal"], ["mode-start", "start"], ["mode-shelf", "shelf"]]) {
      $(id).setAttribute("aria-pressed", String(m === mode));
    }
    showHint();
  }

  function showHint() {
    const r = state.selected;
    $("hint").textContent =
      state.tapMode === "shelf" ? "Tap an aisle cell to add a shelf, or a shelf to remove it."
      : state.orders.running ? `Robot ${r} selected. Orders are running: robots choose their own goals.`
      : state.tapMode === "goal" ? `Robot ${r} selected. Tap a free cell to send it there.`
      : `Robot ${r} selected. Tap a free cell to place it there.`;
  }

  function showStats(res, prioritizedCost) {
    const p = state.plan;
    $("st-soc").textContent = p.sumOfCosts;
    $("st-mk").textContent = p.makespan;
    const count = (type) => p.collisions.filter((h) => h.type === type).length;
    $("st-hit").textContent = p.collisions.length;
    $("st-hit-s").textContent = `${count("vertex")} vertex · ${count("edge")} swap · ${count("target")} target`;
    $("tile-hit").className = "tile " + (p.collisions.length ? "bad" : "good");
    $("st-ms").textContent = res.ms < 10 ? res.ms.toFixed(1) + " ms" : Math.round(res.ms) + " ms";
    $("st-ms-s").textContent = res.mode >= 2
      ? `on this device · ${res.ctNodes.toLocaleString("en-US")} constraint-tree nodes`
      : res.mode === 1 ? `on this device · ${res.attempts} priority order${res.attempts === 1 ? "" : "s"} tried`
      : "on this device";
    const extra = prioritizedCost === null ? 0 : prioritizedCost - p.sumOfCosts;
    const vsPrioritized = prioritizedCost === null ? ""
      : ` Prioritized planning needs ${prioritizedCost}` + (extra > 0 ? ` (+${extra}).` : extra < 0 ? ` (${extra}).` : " here too.");
    $("compare").textContent = res.mode === 2 || res.mode === 4
      ? "Optimal: no collision-free plan has a lower sum of costs." + vsPrioritized
      : res.mode === 3
      ? `Bounded: no plan can beat ${res.lowerBound}, so this one (${p.sumOfCosts}) is at most ` +
        `${((p.sumOfCosts / Math.max(1, res.lowerBound) - 1) * 100).toFixed(1)}% above optimal (w = ${ecbsWeight()} allows ${((ecbsWeight() - 1) * 100).toFixed(0)}%).` + vsPrioritized
      : "";
    $("time").max = String(p.makespan);
    showStatus(res);
    $("map").setAttribute("aria-label", `Warehouse with ${state.n} robots, ${p.collisions.length} collisions in the current plan`);
  }

  function showStatus(res) {
    if (!res) return;
    $("status").textContent = `Planner: C++ compiled to WebAssembly${planner.inWorker() ? ", in a background worker" : ""} · ` +
      `${plannerName(res.mode)} · ${state.n - state.broken.size} working robots`;
  }

  // Chip labels and the selected-robot card; refreshed once per whole timestep.
  function showLive() {
    const p = state.plan;
    const t = Math.min(Math.floor(state.time), p.makespan);
    if (t === state.lastShownT) return;
    state.lastShownT = t;
    p.agents.forEach((a, i) => {
      const arrive = a.path.length - 1;
      const chip = $(`robot-${i}`);
      if (!chip) return;  // the fleet size is changing: the new plan is on its way
      chip.querySelector(".dot").style.background = isWorking(i) ? V.agentColor(i) : "var(--muted)";
      chip.querySelector(".st").textContent = !isWorking(i) ? "broken"
        : state.orders.running ? (orderOf(i) ? (t >= arrive ? "picking" : `${arrive - t} to parcel`) : "idle")
        : t >= arrive ? "arrived" : `${arrive - t} to go`;
    });
    const i = state.selected;
    const a = p.agents[i];
    const arrive = a.path.length - 1;
    let waits = 0;
    for (let k = 1; k <= arrive; k++) {
      if (a.path[k][0] === a.path[k - 1][0] && a.path[k][1] === a.path[k - 1][1]) waits++;
    }
    const [x, y] = V.posAt(a, t);
    $("sel-title").innerHTML = `<span class="sel-dot"></span>Robot ${i}`;
    $("sel-title").querySelector(".sel-dot").style.background = isWorking(i) ? V.agentColor(i) : "var(--muted)";
    $("sel-state").textContent = !isWorking(i) ? "broken down"
      : state.orders.running ? (orderOf(i) ? "fetching a parcel" : "idle") : "driving where you sent it";
    $("sel-now").textContent = `(${x}, ${y})`;
    $("sel-goal").textContent = isWorking(i) ? fmtXY(state.goals[i]) : "–";
    $("sel-arrive").textContent = isWorking(i) ? `t = ${arrive}` : "–";
    $("sel-waits").textContent = waits === 0 ? "none" : `${waits} step${waits === 1 ? "" : "s"}`;
    $("break").textContent = isWorking(i) ? `Break down robot ${i}` : `Repair robot ${i}`;
    if (state.orders.running) showOrders();
  }

  function layout() {
    if (!state.plan) return;
    const maxH = Math.max(240, Math.min(window.innerHeight * 0.66, 720));
    view.resize($("canvas-wrap").clientWidth, maxH);
    draw();
  }

  function draw() {
    const p = state.plan;
    if (!p) return;
    const orders = state.orders.list.map((o) => {
      const [x, y] = toXY(o.cell);
      return { x, y, color: o.robot >= 0 ? V.agentColor(o.robot) : null };
    });
    view.draw(state.time, {
      trails: $("trails").checked, selected: state.selected, orders, broken: new Set(state.broken.keys()),
    });
    const t = Math.min(Math.floor(state.time), p.makespan);
    $("time").value = String(t);
    $("time-label").textContent = `t = ${t} / ${p.makespan}`;
    showLive();
  }

  function setPlaying(on) {
    state.playing = on;
    $("play").textContent = on ? "Pause" : "Play";
    if (on) {
      if (state.time >= state.plan.makespan && !state.orders.running) state.time = 0;
      state.lastFrame = performance.now();
      requestAnimationFrame(tick);
    }
  }

  function tick(now) {
    if (!state.playing) return;
    const dt = Math.min(0.1, (now - state.lastFrame) / 1000);
    state.lastFrame = now;
    if (state.busy) {  // the next plan starts from the current positions: hold still until it's here
      requestAnimationFrame(tick);
      return;
    }
    state.time = Math.min(state.time + dt * Number($("speed").value), state.plan.makespan);
    if (state.orders.running) {
      orderTick(now);  // may replan and reset the time
    } else if (state.time >= state.plan.makespan) {
      setPlaying(false);  // everyone has arrived; the next tap plans from here
    }
    draw();
    if (state.playing) requestAnimationFrame(tick);
  }

  // ---------- start ----------

  async function start(M) {
    state.M = M;
    state.W = M._sg_width();
    state.H = M._sg_height();
    planner.start();
    readRows();
    M._sg_random_agents(state.n, state.seed);  // the same 10 robots as the dashboard's demo scene
    const starts = [];
    const goals = [];
    for (let i = 0; i < state.n; i++) { starts.push(M._sg_agent_start(i)); goals.push(M._sg_agent_goal(i)); }
    buildRobotButtons();
    select(0);
    syncPlannerControls();
    if (!(await tryReplan(starts, goals))) return;

    $("map").addEventListener("click", onMapTap);
    $("mode-goal").addEventListener("click", () => setTapMode("goal"));
    $("mode-start").addEventListener("click", () => setTapMode("start"));
    $("mode-shelf").addEventListener("click", () => setTapMode("shelf"));
    const replanHere = async () => {
      if (state.busy) return;
      if (await tryReplan(currentPositions(), state.goals.slice(), { orderLoop: state.orders.running })) setPlaying(true);
    };
    $("planner").addEventListener("change", () => { syncPlannerControls(); replanHere(); });
    $("weight").addEventListener("change", () => { if (plannerMode() === 3) replanHere(); });
    $("count").addEventListener("change", (e) => setRobotCount(Number(e.target.value)));
    $("assign").addEventListener("change", showOrders);
    $("cancel").addEventListener("click", () => planner.cancel());
    $("orders-toggle").addEventListener("click", () => setOrdersRunning(!state.orders.running));
    $("reset-map").addEventListener("click", resetMap);
    $("shuffle").addEventListener("click", () => shuffle(false));
    $("break").addEventListener("click", toggleBroken);
    $("play").addEventListener("click", () => setPlaying(!state.playing));
    $("restart").addEventListener("click", () => {
      if (state.orders.running) return;  // the order loop only moves forward
      state.time = 0;
      setPlaying(true);
    });
    $("time").addEventListener("input", (e) => {
      if (state.orders.running) { draw(); return; }
      setPlaying(false);
      state.time = Number(e.target.value);
      draw();
    });
    $("trails").addEventListener("change", draw);
    document.addEventListener("keydown", (e) => {
      const tag = (e.target && e.target.tagName) || "";
      if (tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA") return;
      if (e.key === " ") { e.preventDefault(); setPlaying(!state.playing); }
      else if (/^[0-9]$/.test(e.key) && Number(e.key) < state.n) select(Number(e.key));
    });
    let lastWidth = -1;
    const onResize = () => {
      const w = $("canvas-wrap").clientWidth;
      if (w !== lastWidth) { lastWidth = w; layout(); }
    };
    if (window.ResizeObserver) new ResizeObserver(onResize).observe($("canvas-wrap"));
    else window.addEventListener("resize", onResize);
    window.matchMedia("(prefers-color-scheme: dark)").addEventListener?.("change", draw);

    showHint();
    showOrders();
    if (!reduceMotion) setPlaying(true);
  }

  if (typeof window.createSwarmGridPlanner !== "function") {
    $("status").textContent = "Could not load the planner (swarmgrid-planner.js).";
    return;
  }
  window.createSwarmGridPlanner().then(start).catch((err) => {
    $("status").textContent = "The planner failed to start: " + err;
  });
})();
