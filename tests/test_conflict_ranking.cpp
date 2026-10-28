#include "doctest/doctest.h"

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "swarmgrid/cbs.hpp"
#include "swarmgrid/conflict_ranking.hpp"
#include "swarmgrid/conflicts.hpp"
#include "swarmgrid/warehouse_gen.hpp"
#include "mapf_reference.hpp"

using swarmgrid::CbsOptions;
using swarmgrid::CbsResult;
using swarmgrid::CellId;
using swarmgrid::ConflictRanker;
using swarmgrid::ConflictSelection;
using swarmgrid::Grid;

namespace {

void warehouseInstance(int k, std::uint32_t seed, Grid& g, std::vector<CellId>& s, std::vector<CellId>& t) {
    g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    const auto inst = swarmgrid::generateRandomInstance(g, k, seed);
    s.clear();
    t.clear();
    for (int i = 0; i < k; ++i) {
        s.push_back(g.toId(inst.starts[static_cast<std::size_t>(i)]));
        t.push_back(g.toId(inst.goals[static_cast<std::size_t>(i)]));
    }
}

CbsOptions with(ConflictSelection selection, bool bypass = true) {
    CbsOptions o;
    o.selection = selection;
    o.bypass = bypass;
    return o;
}

// The M5 rule written as a linear model: cardinal >> semi-cardinal >> non-cardinal, then earlier.
ConflictRanker m5AsLinearModel() {
    std::array<std::int64_t, swarmgrid::kConflictFeatures> w{};
    w[swarmgrid::kIsCardinal] = 1 << 20;
    w[swarmgrid::kIsSemiCardinal] = 1 << 19;
    w[swarmgrid::kTime] = -1;
    return ConflictRanker(w);
}

std::string tempPath(const char* name) { return std::string("swarmgrid_test_") + name + ".txt"; }

}  // namespace

TEST_CASE("ranker: feature names, scoring, save/load") {
    std::set<std::string> names;
    for (int i = 0; i < swarmgrid::kConflictFeatures; ++i) names.insert(swarmgrid::conflictFeatureName(i));
    CHECK(static_cast<int>(names.size()) == swarmgrid::kConflictFeatures);
    CHECK_THROWS_AS(swarmgrid::conflictFeatureName(swarmgrid::kConflictFeatures), std::out_of_range);

    std::array<std::int64_t, swarmgrid::kConflictFeatures> w{};
    for (int i = 0; i < swarmgrid::kConflictFeatures; ++i) w[static_cast<std::size_t>(i)] = i - 7;
    const ConflictRanker r(w);
    swarmgrid::ConflictFeatureVector f{};
    std::int64_t expected = 0;
    for (int i = 0; i < swarmgrid::kConflictFeatures; ++i) {
        f[static_cast<std::size_t>(i)] = 3 * i + 1;
        expected += (i - 7) * (3 * i + 1);
    }
    CHECK(r.score(f) == expected);

    const std::string path = tempPath("ranker");
    r.save(path);
    CHECK(ConflictRanker::load(path).weights() == w);
    std::remove(path.c_str());
}

TEST_CASE("ranker: malformed model files are rejected") {
    const std::string path = tempPath("bad_ranker");
    auto write = [&](const std::string& text) {
        std::ofstream out(path, std::ios::binary);
        out << text;
    };
    std::string all;
    for (int i = 0; i < swarmgrid::kConflictFeatures; ++i) all += std::string(swarmgrid::conflictFeatureName(i)) + " 1\n";
    write("# comment\r\n" + all);  // CRLF and comments are fine
    CHECK(ConflictRanker::load(path).weights()[0] == 1);
    write(all + "is_vertex 2\n");
    CHECK_THROWS_AS(ConflictRanker::load(path), std::invalid_argument);   // duplicate
    write(all.substr(all.find('\n') + 1));
    CHECK_THROWS_AS(ConflictRanker::load(path), std::invalid_argument);   // missing is_vertex
    write(all + "no_such_feature 3\n");
    CHECK_THROWS_AS(ConflictRanker::load(path), std::invalid_argument);   // unknown
    write("is_vertex one\n");
    CHECK_THROWS_AS(ConflictRanker::load(path), std::invalid_argument);   // not a number
    std::remove(path.c_str());
    CHECK_THROWS_AS(ConflictRanker::load(path), std::runtime_error);      // missing file
}

TEST_CASE("cbs: learned and oracle selection stay optimal (random small instances)") {
    swarmgrid::Rng rng(777);
    int checked = 0;
    for (int trial = 0; trial < 50; ++trial) {
        Grid g(4, 4);
        for (int i = rng.uniformInt(0, 3); i > 0; --i) {
            const int x = rng.uniformInt(0, 3);
            const int y = rng.uniformInt(0, 3);
            g.setObstacle(x, y, true);
        }
        const int k = rng.uniformInt(2, 3);
        swarmgrid::MapfInstance inst;
        try {
            inst = swarmgrid::generateRandomInstance(g, k, static_cast<std::uint32_t>(trial + 500));
        } catch (const std::invalid_argument&) {
            continue;
        }
        std::vector<CellId> s, t;
        for (int i = 0; i < k; ++i) {
            s.push_back(g.toId(inst.starts[static_cast<std::size_t>(i)]));
            t.push_back(g.toId(inst.goals[static_cast<std::size_t>(i)]));
        }
        const long long opt = swarmgrid_test::referenceSumOfCosts(g, s, t);
        for (ConflictSelection sel : {ConflictSelection::Learned, ConflictSelection::Oracle}) {
            for (bool bypass : {false, true}) {
                CbsOptions o = with(sel, bypass);
                o.maxNodes = 5000;
                const CbsResult r = swarmgrid::solveCbs(g, s, t, o);
                if (opt < 0) {
                    CHECK_FALSE(r.solved);
                    continue;
                }
                REQUIRE(r.solved);
                CHECK(swarmgrid::validatePlan(g, s, t, r.paths).ok);
                CHECK(r.sumOfCosts == opt);
            }
        }
        if (opt >= 0) ++checked;
    }
    CHECK(checked > 30);
}

TEST_CASE("cbs: learned selection follows its model exactly") {
    Grid g;
    std::vector<CellId> s, t;
    warehouseInstance(10, 42, g, s, t);
    // All weights zero: every score ties, so the earliest conflict wins, exactly like First.
    CbsOptions zero = with(ConflictSelection::Learned);
    zero.ranker = ConflictRanker(std::array<std::int64_t, swarmgrid::kConflictFeatures>{});
    const CbsResult a = swarmgrid::solveCbs(g, s, t, zero);
    const CbsResult b = swarmgrid::solveCbs(g, s, t, with(ConflictSelection::First));
    CHECK(a.nodesExpanded == b.nodesExpanded);
    CHECK(a.sumOfCosts == 208);
    // The M5 rule as weights picks the same conflicts as the M5 code.
    CbsOptions m5 = with(ConflictSelection::Learned);
    m5.ranker = m5AsLinearModel();
    const CbsResult c = swarmgrid::solveCbs(g, s, t, m5);
    const CbsResult d = swarmgrid::solveCbs(g, s, t, with(ConflictSelection::Cardinal));
    CHECK(c.nodesExpanded == d.nodesExpanded);
    CHECK(c.nodesGenerated == d.nodesGenerated);
    CHECK(d.nodesExpanded == 245);  // the pinned M5 number (test_cbs.cpp)
    CHECK(c.oracleReplans == 0);
}

TEST_CASE("cbs: shipped model and oracle on the 10-robot warehouse") {
    Grid g;
    std::vector<CellId> s, t;
    warehouseInstance(10, 42, g, s, t);
    for (ConflictSelection sel : {ConflictSelection::Learned, ConflictSelection::Oracle}) {
        const CbsResult r = swarmgrid::solveCbs(g, s, t, with(sel));
        REQUIRE(r.solved);
        CHECK(r.sumOfCosts == 208);
        CHECK(swarmgrid::validatePlan(g, s, t, r.paths).ok);
        CHECK(r.cardinalSplits + r.semiCardinalSplits + r.nonCardinalSplits == r.nodesExpanded - 1);
        if (sel == ConflictSelection::Oracle) CHECK(r.oracleReplans > 0);
        else CHECK(r.oracleReplans == 0);
    }
}

TEST_CASE("cbs: training samples agree with the MDD classification") {
    // Two independent computations: MDD cardinality (features) and real splits (oracle). A cardinal
    // conflict must make both children dearer, a semi-cardinal one exactly one, a non-cardinal none.
    int samples = 0;
    for (std::uint32_t seed : {42u, 7u, 3u}) {
        Grid g;
        std::vector<CellId> s, t;
        warehouseInstance(15, seed, g, s, t);
        std::vector<swarmgrid::ConflictSample> data;
        CbsOptions o = with(ConflictSelection::Cardinal);
        o.samples = &data;
        o.sampleMaxNodes = 30;
        o.sampleMaxConflicts = 8;
        o.maxNodes = 30;
        const CbsResult r = swarmgrid::solveCbs(g, s, t, o);
        CHECK(r.oracleReplans == 2 * static_cast<long long>(data.size()));
        std::vector<int> perNode(31, 0);
        for (const auto& smp : data) {
            REQUIRE(smp.node >= 1);
            REQUIRE(smp.node <= 30);
            ++perNode[static_cast<std::size_t>(smp.node)];
            const auto& f = smp.features;
            CHECK(f[swarmgrid::kIsVertex] + f[swarmgrid::kIsEdge] + f[swarmgrid::kIsTarget] == 1);
            CHECK(f[swarmgrid::kIsCardinal] + f[swarmgrid::kIsSemiCardinal] <= 1);
            CHECK(f[swarmgrid::kMddWidthMin] >= 1);
            CHECK(f[swarmgrid::kMddWidthMin] <= f[swarmgrid::kMddWidthMax]);
            CHECK(f[swarmgrid::kDelayMin] >= 0);
            CHECK(smp.minIncrease >= 0);
            if (f[swarmgrid::kIsCardinal] == 1) {
                CHECK(smp.minIncrease >= 1);
            } else if (f[swarmgrid::kIsSemiCardinal] == 1) {
                CHECK(smp.minIncrease == 0);
                CHECK(smp.sumIncrease >= 1);
            } else {
                CHECK(smp.sumIncrease == 0);
            }
            ++samples;
        }
        for (int count : perNode) CHECK(count <= 8);
        // Recording must not change the search.
        CbsOptions plain = with(ConflictSelection::Cardinal);
        plain.maxNodes = 30;
        CHECK(swarmgrid::solveCbs(g, s, t, plain).nodesGenerated == r.nodesGenerated);
    }
    CHECK(samples > 50);
}

TEST_CASE("cbs: bad sampling settings throw") {
    Grid g(3, 1);
    std::vector<swarmgrid::ConflictSample> data;
    CbsOptions o;
    o.samples = &data;
    o.sampleMaxConflicts = 0;
    CHECK_THROWS_AS(swarmgrid::solveCbs(g, {0}, {2}, o), std::invalid_argument);
}
