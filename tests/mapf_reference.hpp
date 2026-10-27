#pragma once

// Test-only helper shared by the CBS and ECBS tests.

#include <cstdint>
#include <functional>
#include <queue>
#include <unordered_map>
#include <vector>

#include "swarmgrid/grid.hpp"

namespace swarmgrid_test {

using swarmgrid::CellId;
using swarmgrid::Grid;

// Independent reference for small instances: Dijkstra over the JOINT state of all agents.
// State = every agent's cell + which agents have "finished" (parked on their goal for good).
// Each timestep costs 1 per unfinished agent, so the cheapest way to finish everyone is the
// optimal sum of costs. Shares no code with CBS or space-time A*. Returns -1 if unsolvable.
inline long long referenceSumOfCosts(const Grid& g, const std::vector<CellId>& starts,
                              const std::vector<CellId>& goals) {
    const int k = static_cast<int>(starts.size());
    const auto V = static_cast<std::uint64_t>(g.numCells());
    const unsigned allDone = (1u << k) - 1;
    auto encode = [&](const std::vector<CellId>& pos, unsigned done) {
        std::uint64_t key = 0;
        for (CellId c : pos) key = key * V + static_cast<std::uint64_t>(c);
        return (key << k) | done;
    };
    struct Item {
        long long cost;
        std::vector<CellId> pos;
        unsigned done;
        bool operator>(const Item& o) const { return cost > o.cost; }
    };
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    std::unordered_map<std::uint64_t, long long> best;
    open.push({0, starts, 0u});
    best[encode(starts, 0u)] = 0;
    std::vector<CellId> nb;
    while (!open.empty()) {
        Item cur = open.top();
        open.pop();
        if (best[encode(cur.pos, cur.done)] < cur.cost) continue;
        if (cur.done == allDone) return cur.cost;
        auto relax = [&](const std::vector<CellId>& pos, unsigned done, long long cost) {
            const std::uint64_t key = encode(pos, done);
            auto it = best.find(key);
            if (it == best.end() || cost < it->second) {
                best[key] = cost;
                open.push({cost, pos, done});
            }
        };
        // Free action: an agent standing on its goal declares itself finished.
        for (int i = 0; i < k; ++i) {
            if (!(cur.done >> i & 1u) && cur.pos[static_cast<std::size_t>(i)] == goals[static_cast<std::size_t>(i)]) {
                relax(cur.pos, cur.done | (1u << i), cur.cost);
            }
        }
        // One timestep: every unfinished agent waits or moves; finished agents stay.
        std::vector<std::vector<CellId>> options(static_cast<std::size_t>(k));
        int unfinished = 0;
        for (int i = 0; i < k; ++i) {
            const CellId c = cur.pos[static_cast<std::size_t>(i)];
            options[static_cast<std::size_t>(i)] = {c};
            if (!(cur.done >> i & 1u)) {
                ++unfinished;
                g.neighbors(c, nb);
                for (CellId m : nb) options[static_cast<std::size_t>(i)].push_back(m);
            }
        }
        if (unfinished == 0) continue;
        std::vector<CellId> next(static_cast<std::size_t>(k));
        std::function<void(int)> choose = [&](int i) {
            if (i == k) {
                for (int a = 0; a < k; ++a) {
                    for (int b = a + 1; b < k; ++b) {
                        const auto ua = static_cast<std::size_t>(a), ub = static_cast<std::size_t>(b);
                        if (next[ua] == next[ub]) return;  // vertex conflict
                        if (next[ua] == cur.pos[ub] && next[ub] == cur.pos[ua]) return;  // swap
                    }
                }
                relax(next, cur.done, cur.cost + unfinished);
                return;
            }
            for (CellId c : options[static_cast<std::size_t>(i)]) {
                next[static_cast<std::size_t>(i)] = c;
                choose(i + 1);
            }
        };
        choose(0);
    }
    return -1;
}

}  // namespace swarmgrid_test
