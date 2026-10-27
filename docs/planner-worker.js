// Web Worker for docs/control.html: runs the WebAssembly planner off the main thread, so a long
// CBS or ECBS search never freezes the page (and can be cancelled by terminating the worker).
//
// Message in:  { id, blocked: Uint8Array (1 = shelf or broken robot), starts, goals, mode,
//                nodes, seconds, weight }
// Message out: { id, status, ms, ctNodes, limitReached, attempts, failed, error, lowerBound,
//                paths: [[cell, ...], ...] }  (paths only when status === 1)
"use strict";

importScripts("swarmgrid-planner.js");

const ready = createSwarmGridPlanner();

self.onmessage = async (event) => {
  const M = await ready;
  const q = event.data;
  for (let c = 0; c < q.blocked.length; c++) M._sg_set_obstacle(c, q.blocked[c]);
  M._sg_set_ecbs_weight(q.weight);
  M._sg_clear_agents();
  for (let i = 0; i < q.starts.length; i++) M._sg_add_agent(q.starts[i], q.goals[i]);
  const t0 = performance.now();
  const status = q.starts.length ? M._sg_plan_with_limits(q.mode, q.nodes, q.seconds) : 1;
  const out = {
    id: q.id,
    status,
    ms: performance.now() - t0,
    ctNodes: M._sg_ct_nodes(),
    limitReached: M._sg_limit_reached() === 1,
    attempts: M._sg_attempts(),
    failed: M._sg_failed_agent(),
    error: status < 0 ? M.UTF8ToString(M._sg_last_error()) : "",
    lowerBound: M._sg_lower_bound(),
    paths: null,
  };
  if (status === 1) {
    out.paths = [];
    for (let k = 0; k < q.starts.length; k++) {
      const len = M._sg_path_length(k);
      const p = new Array(len);
      for (let t = 0; t < len; t++) p[t] = M._sg_path_cell(k, t);
      out.paths.push(p);
    }
  }
  self.postMessage(out);
};
