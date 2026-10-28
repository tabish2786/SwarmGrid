#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace swarmgrid {

// ML-guided conflict selection for CBS, in the spirit of Huang, Koenig & Dilkina (AAAI 2021,
// "Learning to Resolve Conflicts for Multi-Agent Path Finding with Conflict-Based Search").
//
// CBS may split on ANY conflict of a node and stays optimal; the choice only changes how big the
// tree gets. An expensive but good choice is the "oracle" (strong branching): split every conflict
// for real and pick the one whose cheaper child got the most expensive, i.e. the one that raises
// the lower bound most. Here a linear model learns to rank conflicts like that oracle from cheap
// features, offline (swarmgrid ml-data + scripts/train_conflict_ranker.py); CBS then splits on
// the conflict with the highest score.
//
// The features are integers and the weights are fixed-point integers, so a score is exact integer
// arithmetic: the same conflict is chosen with every compiler and CPU. (A floating-point dot
// product can differ in the last bit, for example when a compiler fuses multiply-adds, and a
// single different choice changes the whole search.)

enum ConflictFeature : int {
    kIsVertex,           // conflict type, one-hot
    kIsEdge,
    kIsTarget,
    kIsCardinal,         // both agents' MDDs have width 1 there: both children must cost more
    kIsSemiCardinal,     // exactly one of them
    kTime,               // timestep of the conflict
    kMddWidthMin,        // MDD width at that timestep, smaller / larger of the two agents
    kMddWidthMax,
    kAgentConflictsMin,  // how many conflicts of this node involve each agent
    kAgentConflictsMax,
    kPathCostMin,        // current path cost of each agent
    kPathCostMax,
    kDelayMin,           // path cost minus the agent's shortest-path distance
    kDelayMax,
    kConstraintsMin,     // constraints on each agent at this node
    kConstraintsMax,
    kGoalDistMin,        // distance from where each agent is at the conflict to its goal
    kGoalDistMax,
    kConflictFeatureCount
};

constexpr int kConflictFeatures = kConflictFeatureCount;
using ConflictFeatureVector = std::array<std::int64_t, kConflictFeatures>;

// Short snake_case name, used as the CSV column and in model files. Throws std::out_of_range.
const char* conflictFeatureName(int feature);

// Linear scorer with fixed-point weights: score = sum of weight[i] * feature[i].
class ConflictRanker {
public:
    // The model shipped with the code (src/conflict_ranker_model.inc, written by the training
    // script).
    ConflictRanker();
    explicit ConflictRanker(const std::array<std::int64_t, kConflictFeatures>& weights);

    // Model file: one "feature_name weight" pair per line, every feature exactly once; lines
    // starting with '#' are comments. Throws std::runtime_error if unreadable, std::invalid_argument
    // if malformed.
    static ConflictRanker load(const std::string& path);
    void save(const std::string& path) const;

    std::int64_t score(const ConflictFeatureVector& features) const;
    const std::array<std::int64_t, kConflictFeatures>& weights() const { return weights_; }

private:
    std::array<std::int64_t, kConflictFeatures> weights_{};
};

// One training example: the features of one conflict of one CT node, and what the oracle measured
// when it split that conflict. Examples with the same `node` (within one CBS run) are ranked
// against each other.
struct ConflictSample {
    int node = 0;           // 1-based index of the expanded CT node within the run
    int conflictIndex = 0;  // position of the conflict in the node's conflict list
    ConflictFeatureVector features{};
    int minIncrease = 0;    // cost of the cheaper child minus the node's cost (higher = better split)
    int sumIncrease = 0;    // both children's increases added (tie-break)
};

// Cost increase recorded for a child that has no solution at all.
constexpr int kInfeasibleIncrease = 1000;

}  // namespace swarmgrid
