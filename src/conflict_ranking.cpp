#include "swarmgrid/conflict_ranking.hpp"

#include <cstddef>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace swarmgrid {

namespace {

#include "conflict_ranker_model.inc"

constexpr const char* kFeatureNames[kConflictFeatures] = {
    "is_vertex",           "is_edge",           "is_target",       "is_cardinal",
    "is_semi_cardinal",    "time",              "mdd_width_min",   "mdd_width_max",
    "agent_conflicts_min", "agent_conflicts_max", "path_cost_min", "path_cost_max",
    "delay_min",           "delay_max",         "constraints_min", "constraints_max",
    "goal_dist_min",       "goal_dist_max",
};

}  // namespace

const char* conflictFeatureName(int feature) {
    if (feature < 0 || feature >= kConflictFeatures) {
        throw std::out_of_range("conflictFeatureName: no feature " + std::to_string(feature));
    }
    return kFeatureNames[feature];
}

ConflictRanker::ConflictRanker() : weights_(kBuiltInConflictWeights) {}

ConflictRanker::ConflictRanker(const std::array<std::int64_t, kConflictFeatures>& weights)
    : weights_(weights) {}

ConflictRanker ConflictRanker::load(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("ConflictRanker::load: cannot read " + path);
    std::array<std::int64_t, kConflictFeatures> weights{};
    std::array<bool, kConflictFeatures> seen{};
    std::string line;
    int lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;
        if (!line.empty() && line.back() == '\r') line.pop_back();  // CRLF files from Windows
        if (line.empty() || line[0] == '#') continue;
        std::istringstream fields(line);
        std::string name;
        long long value = 0;
        std::string extra;
        if (!(fields >> name >> value) || (fields >> extra)) {
            throw std::invalid_argument(path + ":" + std::to_string(lineNo) +
                                        ": expected 'feature_name weight'");
        }
        int index = -1;
        for (int i = 0; i < kConflictFeatures; ++i) {
            if (name == kFeatureNames[i]) index = i;
        }
        if (index < 0) throw std::invalid_argument(path + ": unknown feature '" + name + "'");
        if (seen[static_cast<std::size_t>(index)]) {
            throw std::invalid_argument(path + ": feature '" + name + "' appears twice");
        }
        seen[static_cast<std::size_t>(index)] = true;
        weights[static_cast<std::size_t>(index)] = value;
    }
    for (int i = 0; i < kConflictFeatures; ++i) {
        if (!seen[static_cast<std::size_t>(i)]) {
            throw std::invalid_argument(path + ": missing feature '" + kFeatureNames[i] + "'");
        }
    }
    return ConflictRanker(weights);
}

void ConflictRanker::save(const std::string& path) const {
    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("ConflictRanker::save: cannot write " + path);
    out << "# SwarmGrid conflict ranker: score = sum of weight * feature (fixed point)\n";
    for (int i = 0; i < kConflictFeatures; ++i) {
        out << kFeatureNames[i] << ' ' << weights_[static_cast<std::size_t>(i)] << '\n';
    }
}

std::int64_t ConflictRanker::score(const ConflictFeatureVector& features) const {
    std::int64_t s = 0;
    for (std::size_t i = 0; i < features.size(); ++i) s += weights_[i] * features[i];
    return s;
}

}  // namespace swarmgrid
