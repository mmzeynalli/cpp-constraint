#ifndef CONSTRAINTS_HPP
#define CONSTRAINTS_HPP

#include <vector>
#include <unordered_map>
#include <string>
#include <memory>
#include <functional>
#include <algorithm>
#include <limits>
#include <unordered_set>
#include <optional>
#include <cmath>

#include "domain.hpp"

class Constraint {
public:
    virtual ~Constraint() = default;

    /**
     * Perform the constraint checking.
     *
     * If the forwardcheck parameter is not false, besides telling if
     * the constraint is currently broken or not, the constraint
     * implementation may choose to hide values from the domains of
     * unassigned variables to prevent them from being used, and thus
     * prune the search space.
     *
     * @param variables Variables affected by that constraint,
     *        in the same order provided by the user
     * @param domains Dictionary mapping variables to their domains
     * @param assignments Dictionary mapping assigned variables to
     *        their current assumed value
     * @param forwardcheck Boolean value stating whether forward checking
     *        should be performed or not
     * @return Boolean value stating if this constraint is currently
     *         broken or not
     */
    virtual bool operator()(
        const std::vector<std::string>& variables,
        const std::unordered_map<std::string, std::vector<int>>& domains,
        const std::unordered_map<std::string, int>& assignments,
        bool forwardcheck = false
    ) const {
        return true;
    }

    /**
     * Preprocess variable domains.
     *
     * This method is called before starting to look for solutions,
     * and is used to prune domains with specific constraint logic
     * when possible. For instance, any constraints with a single
     * variable may be applied on all possible values and removed,
     * since they may act on individual values even without further
     * knowledge about other assignments.
     *
     * @param variables Variables affected by that constraint,
     *        in the same order provided by the user
     * @param domains Dictionary mapping variables to their domains
     * @param constraints List of pairs of (constraint, variables)
     * @param vconstraints Dictionary mapping variables to a list
     *        of constraints affecting the given variables.
     */
    virtual void preProcess(
        const std::vector<std::string>& variables,
        std::unordered_map<std::string, std::vector<int>>& domains,
        std::vector<std::pair<std::shared_ptr<Constraint>, std::vector<std::string>>>& constraints,
        std::unordered_map<std::string, std::vector<std::pair<std::shared_ptr<Constraint>, std::vector<std::string>>>>& vconstraints
    ) {
        if (variables.size() == 1) {
            const auto& variable = variables[0];
            auto& domain = domains[variable];
            for (auto it = domain.begin(); it != domain.end();) {
                if (!(*this)(variables, domains, {{variable, *it}})) {
                    it = domain.erase(it);
                } else {
                    ++it;
                }
            }
            constraints.erase(std::remove_if(constraints.begin(), constraints.end(),
                [this, &variables](const auto& pair) {
                    return pair.first.get() == this && pair.second == variables;
                }), constraints.end());
            auto& vconstraint = vconstraints[variable];
            vconstraint.erase(std::remove_if(vconstraint.begin(), vconstraint.end(),
                [this, &variables](const auto& pair) {
                    return pair.first.get() == this && pair.second == variables;
                }), vconstraint.end());
        }
    }

    /**
     * Helper method for generic forward checking.
     *
     * Currently, this method acts only when there's a single
     * unassigned variable.
     *
     * @param variables Variables affected by that constraint,
     *        in the same order provided by the user
     * @param domains Dictionary mapping variables to their domains
     * @param assignments Dictionary mapping assigned variables to
     *        their current assumed value
     * @return Boolean value stating if this constraint is currently
     *         broken or not
     */
    virtual bool forwardCheck(
        const std::vector<std::string>& variables,
        std::unordered_map<std::string, std::vector<int>>& domains,
        std::unordered_map<std::string, int>& assignments
    ) const {
        std::string unassignedVariable;
        for (const auto& variable : variables) {
            if (assignments.find(variable) == assignments.end()) {
                if (unassignedVariable.empty()) {
                    unassignedVariable = variable;
                } else {
                    return true;
                }
            }
        }
        if (!unassignedVariable.empty()) {
            auto& domain = domains[unassignedVariable];
            if (!domain.empty()) {
                for (auto it = domain.begin(); it != domain.end();) {
                    assignments[unassignedVariable] = *it;
                    if (!(*this)(variables, domains, assignments)) {
                        it = domain.erase(it);
                    } else {
                        ++it;
                    }
                }
                assignments.erase(unassignedVariable);
            }
            if (domain.empty()) {
                return false;
            }
        }
        return true;
    }
};



class FunctionConstraint : public Constraint {
public:
    /**
     * Constraint which wraps a function defining the constraint logic.
     *
     * Examples:
     *     Problem problem;
     *     problem.addVariables({"a", "b"}, {1, 2});
     *     auto func = [](int a, int b) { return b > a; };
     *     problem.addConstraint(func, {"a", "b"});
     *     auto solution = problem.getSolution();
     *     // solution should be: {'a': 1, 'b': 2}
     *
     *     Problem problem;
     *     problem.addVariables({"a", "b"}, {1, 2});
     *     auto func = [](int a, int b) { return b > a; };
     *     problem.addConstraint(FunctionConstraint(func), {"a", "b"});
     *     auto solution = problem.getSolution();
     *     // solution should be: {'a': 1, 'b': 2}
     */

    /**
     * Initialization method.
     *
     * @param func Function wrapped and queried for constraint logic
     * @param assigned Whether the function may receive unassigned
     *        variables or not
     */
    FunctionConstraint(std::function<bool(const std::vector<int>&)> func, bool assigned = true)
        : _func(std::move(func)), _assigned(assigned) {}

    bool operator()(
        const std::vector<std::string>& variables,
        const std::unordered_map<std::string, std::vector<int>>& domains,
        const std::unordered_map<std::string, int>& assignments,
        bool forwardcheck = false
    ) const override {
        std::vector<int> parms;
        int missing = 0;
        static const int _unassigned = std::numeric_limits<int>::min();

        for (const auto& x : variables) {
            auto it = assignments.find(x);
            if (it != assignments.end()) {
                parms.push_back(it->second);
            } else {
                parms.push_back(_unassigned);
                ++missing;
            }
        }

        if (missing > 0) {
            return (_assigned || _func(parms)) &&
                   (!forwardcheck || missing != 1 || forwardCheck(variables, domains, assignments));
        }
        return _func(parms);
    }

private:
    std::function<bool(const std::vector<int>&)> _func;
    bool _assigned;
};


class AllDifferentConstraint : public Constraint {
public:
    /**
     * Constraint enforcing that values of all given variables are different.
     *
     * Example:
     *     Problem problem;
     *     problem.addVariables({"a", "b"}, {1, 2});
     *     problem.addConstraint(std::make_shared<AllDifferentConstraint>());
     *     auto solutions = problem.getSolutions();
     *     // solutions should contain: [{'a': 1, 'b': 2}, {'a': 2, 'b': 1}]
     */

    bool operator()(
        const std::vector<std::string>& variables,
        const std::unordered_map<std::string, std::vector<int>>& domains,
        const std::unordered_map<std::string, int>& assignments,
        bool forwardcheck = false
    ) const override {
        std::unordered_set<int> seen;
        static const int _unassigned = std::numeric_limits<int>::min();

        for (const auto& variable : variables) {
            auto it = assignments.find(variable);
            if (it != assignments.end()) {
                int value = it->second;
                if (seen.find(value) != seen.end()) {
                    return false;
                }
                seen.insert(value);
            }
        }

        if (forwardcheck) {
            for (const auto& variable : variables) {
                if (assignments.find(variable) == assignments.end()) {
                    auto domainIt = domains.find(variable);
                    if (domainIt != domains.end()) {
                        auto& domain = domainIt->second;
                        for (const auto& value : seen) {
                            auto valueIt = std::find(domain.begin(), domain.end(), value);
                            if (valueIt != domain.end()) {
                                domain.erase(valueIt);
                                if (domain.empty()) {
                                    return false;
                                }
                            }
                        }
                    }
                }
            }
        }

        return true;
    }
};


class AllEqualConstraint : public Constraint {
    public:
        /**
         * Constraint enforcing that values of all given variables are equal.
         *
         * Example:
         *     Problem problem;
         *     problem.addVariables({"a", "b"}, {1, 2});
         *     problem.addConstraint(std::make_shared<AllEqualConstraint>());
         *     auto solutions = problem.getSolutions();
         *     // solutions should contain: [{'a': 1, 'b': 1}, {'a': 2, 'b': 2}]
         */

        bool operator()(
            const std::vector<std::string>& variables,
            const std::unordered_map<std::string, std::vector<int>>& domains,
            const std::unordered_map<std::string, int>& assignments,
            bool forwardcheck = false
        ) const override {
            static const int _unassigned = std::numeric_limits<int>::min();
            int singlevalue = _unassigned;

            for (const auto& variable : variables) {
                auto it = assignments.find(variable);
                int value = (it != assignments.end()) ? it->second : _unassigned;

                if (singlevalue == _unassigned) {
                    singlevalue = value;
                } else if (value != _unassigned && value != singlevalue) {
                    return false;
                }
            }

            if (forwardcheck && singlevalue != _unassigned) {
                for (const auto& variable : variables) {
                    if (assignments.find(variable) == assignments.end()) {
                        auto domainIt = domains.find(variable);
                        if (domainIt != domains.end()) {
                            auto& domain = domainIt->second;
                            if (std::find(domain.begin(), domain.end(), singlevalue) == domain.end()) {
                                return false;
                            }
                            domain.erase(std::remove_if(domain.begin(), domain.end(),
                                [singlevalue](int value) { return value != singlevalue; }),
                                domain.end());
                            if (domain.empty()) {
                                return false;
                            }
                        }
                    }
                }
            }

            return true;
        }
    };


class MaxSumConstraint : public Constraint {
public:
    /**
     * Constraint enforcing that values of given variables sum up to a given amount.
     *
     * Example:
     *     Problem problem;
     *     problem.addVariables({"a", "b"}, {1, 2});
     *     problem.addConstraint(std::make_shared<MaxSumConstraint>(3));
     *     auto solutions = problem.getSolutions();
     *     // solutions should contain: [{'a': 1, 'b': 1}, {'a': 1, 'b': 2}, {'a': 2, 'b': 1}]
     */

    MaxSumConstraint(double maxsum, std::optional<std::vector<double>> multipliers = std::nullopt)
        : _maxsum(maxsum), _multipliers(std::move(multipliers)) {}

    void preProcess(
        const std::vector<std::string>& variables,
        std::unordered_map<std::string, std::vector<int>>& domains,
        std::vector<std::pair<std::shared_ptr<Constraint>, std::vector<std::string>>>& constraints,
        std::unordered_map<std::string, std::vector<std::pair<std::shared_ptr<Constraint>, std::vector<std::string>>>>& vconstraints
    ) override {
        Constraint::preProcess(variables, domains, constraints, vconstraints);

        std::vector<bool> variable_contains_negative;
        std::optional<std::string> variable_with_negative;
        for (const auto& variable : variables) {
            bool contains_negative = std::any_of(domains[variable].begin(), domains[variable].end(),
                                                 [](int value) { return value < 0; });
            variable_contains_negative.push_back(contains_negative);
            if (contains_negative) {
                if (variable_with_negative.has_value()) {
                    return;
                }
                variable_with_negative = variable;
            }
        }

        if (_multipliers.has_value()) {
            for (size_t i = 0; i < variables.size(); ++i) {
                const auto& variable = variables[i];
                if (variable_with_negative.has_value() && *variable_with_negative != variable) {
                    continue;
                }
                auto& domain = domains[variable];
                domain.erase(std::remove_if(domain.begin(), domain.end(),
                    [this, i](int value) { return value * (*_multipliers)[i] > _maxsum; }),
                    domain.end());
            }
        } else {
            for (const auto& variable : variables) {
                if (variable_with_negative.has_value() && *variable_with_negative != variable) {
                    continue;
                }
                auto& domain = domains[variable];
                domain.erase(std::remove_if(domain.begin(), domain.end(),
                    [this](int value) { return value > _maxsum; }),
                    domain.end());
            }
        }
    }

    bool operator()(
        const std::vector<std::string>& variables,
        const std::unordered_map<std::string, std::vector<int>>& domains,
        const std::unordered_map<std::string, int>& assignments,
        bool forwardcheck = false
    ) const override {
        double sum = 0;
        if (_multipliers.has_value()) {
            for (size_t i = 0; i < variables.size(); ++i) {
                const auto& variable = variables[i];
                auto it = assignments.find(variable);
                if (it != assignments.end()) {
                    sum += it->second * (*_multipliers)[i];
                }
            }
            sum = std::round(sum * 1e10) / 1e10;
            if (sum > _maxsum) {
                return false;
            }
            if (forwardcheck) {
                for (size_t i = 0; i < variables.size(); ++i) {
                    const auto& variable = variables[i];
                    if (assignments.find(variable) == assignments.end()) {
                        auto domainIt = domains.find(variable);
                        if (domainIt != domains.end()) {
                            auto domain = domainIt->second;
                            domain.erase(std::remove_if(domain.begin(), domain.end(),
                                [this, &sum, i](int value) { return sum + value * (*_multipliers)[i] > _maxsum; }),
                                domain.end());
                            if (domain.empty()) {
                                return false;
                            }
                        }
                    }
                }
            }
        } else {
            for (const auto& variable : variables) {
                auto it = assignments.find(variable);
                if (it != assignments.end()) {
                    sum += it->second;
                }
            }
            sum = std::round(sum * 1e10) / 1e10;
            if (sum > _maxsum) {
                return false;
            }
            if (forwardcheck) {
                for (const auto& variable : variables) {
                    if (assignments.find(variable) == assignments.end()) {
                        auto domainIt = domains.find(variable);
                        if (domainIt != domains.end()) {
                            auto domain = domainIt->second;
                            domain.erase(std::remove_if(domain.begin(), domain.end(),
                                [this, &sum](int value) { return sum + value > _maxsum; }),
                                domain.end());
                            if (domain.empty()) {
                                return false;
                            }
                        }
                    }
                }
            }
        }
        return true;
    }

private:
    double _maxsum;
    std::optional<std::vector<double>> _multipliers;
};


class ExactSumConstraint : public Constraint {
public:
    /**
     * Constraint enforcing that values of given variables sum exactly to a given amount.
     *
     * Example:
     *     Problem problem;
     *     problem.addVariables({"a", "b"}, {1, 2});
     *     problem.addConstraint(std::make_shared<ExactSumConstraint>(3));
     *     auto solutions = problem.getSolutions();
     *     // solutions should contain: [{'a': 1, 'b': 2}, {'a': 2, 'b': 1}]
     */

    ExactSumConstraint(double exactsum, std::optional<std::vector<double>> multipliers = std::nullopt)
        : _exactsum(exactsum), _multipliers(std::move(multipliers)) {}

    void preProcess(
        const std::vector<std::string>& variables,
        std::unordered_map<std::string, std::vector<int>>& domains,
        std::vector<std::pair<std::shared_ptr<Constraint>, std::vector<std::string>>>& constraints,
        std::unordered_map<std::string, std::vector<std::pair<std::shared_ptr<Constraint>, std::vector<std::string>>>>& vconstraints
    ) override {
        Constraint::preProcess(variables, domains, constraints, vconstraints);

        if (_multipliers.has_value()) {
            for (size_t i = 0; i < variables.size(); ++i) {
                auto& domain = domains[variables[i]];
                domain.erase(std::remove_if(domain.begin(), domain.end(),
                    [this, i](int value) { return value * (*_multipliers)[i] > _exactsum; }),
                    domain.end());
            }
        } else {
            for (const auto& variable : variables) {
                auto& domain = domains[variable];
                domain.erase(std::remove_if(domain.begin(), domain.end(),
                    [this](int value) { return value > _exactsum; }),
                    domain.end());
            }
        }
    }

    bool operator()(
        const std::vector<std::string>& variables,
        const std::unordered_map<std::string, std::vector<int>>& domains,
        const std::unordered_map<std::string, int>& assignments,
        bool forwardcheck = false
    ) const override {
        double sum = 0;
        bool missing = false;

        if (_multipliers.has_value()) {
            for (size_t i = 0; i < variables.size(); ++i) {
                const auto& variable = variables[i];
                auto it = assignments.find(variable);
                if (it != assignments.end()) {
                    sum += it->second * (*_multipliers)[i];
                } else {
                    missing = true;
                }
            }
            sum = std::round(sum * 1e10) / 1e10;
            if (sum > _exactsum) {
                return false;
            }
            if (forwardcheck && missing) {
                for (size_t i = 0; i < variables.size(); ++i) {
                    const auto& variable = variables[i];
                    if (assignments.find(variable) == assignments.end()) {
                        auto domainIt = domains.find(variable);
                        if (domainIt != domains.end()) {
                            auto& domain = domainIt->second;
                            domain.erase(std::remove_if(domain.begin(), domain.end(),
                                [this, &sum, i](int value) { return sum + value * (*_multipliers)[i] > _exactsum; }),
                                domain.end());
                            if (domain.empty()) {
                                return false;
                            }
                        }
                    }
                }
            }
        } else {
            for (const auto& variable : variables) {
                auto it = assignments.find(variable);
                if (it != assignments.end()) {
                    sum += it->second;
                } else {
                    missing = true;
                }
            }
            sum = std::round(sum * 1e10) / 1e10;
            if (sum > _exactsum) {
                return false;
            }
            if (forwardcheck && missing) {
                for (const auto& variable : variables) {
                    if (assignments.find(variable) == assignments.end()) {
                        auto domainIt = domains.find(variable);
                        if (domainIt != domains.end()) {
                            auto& domain = domainIt->second;
                            domain.erase(std::remove_if(domain.begin(), domain.end(),
                                [this, &sum](int value) { return sum + value > _exactsum; }),
                                domain.end());
                            if (domain.empty()) {
                                return false;
                            }
                        }
                    }
                }
            }
        }

        if (missing) {
            return sum <= _exactsum;
        } else {
            return std::abs(sum - _exactsum) < 1e-10; // Using epsilon comparison for floating-point equality
        }
    }

private:
    double _exactsum;
    std::optional<std::vector<double>> _multipliers;
};


class MinSumConstraint : public Constraint {
public:
    /**
     * Constraint enforcing that values of given variables sum at least to a given amount.
     *
     * Example:
     *     Problem problem;
     *     problem.addVariables({"a", "b"}, {1, 2});
     *     problem.addConstraint(std::make_shared<MinSumConstraint>(3));
     *     auto solutions = problem.getSolutions();
     *     // solutions should contain: [{'a': 1, 'b': 2}, {'a': 2, 'b': 1}, {'a': 2, 'b': 2}]
     */

    MinSumConstraint(double minsum, std::optional<std::vector<double>> multipliers = std::nullopt)
        : _minsum(minsum), _multipliers(std::move(multipliers)) {}

    bool operator()(
        const std::vector<std::string>& variables,
        const std::unordered_map<std::string, std::vector<int>>& domains,
        const std::unordered_map<std::string, int>& assignments,
        bool forwardcheck = false
    ) const override {
        // check if each variable is in the assignments
        for (const auto& variable : variables) {
            if (assignments.find(variable) == assignments.end()) {
                return true;
            }
        }

        // with each variable assigned, sum the values
        double sum = 0;
        if (_multipliers.has_value()) {
            for (size_t i = 0; i < variables.size(); ++i) {
                sum += assignments.at(variables[i]) * (*_multipliers)[i];
            }
        } else {
            for (const auto& variable : variables) {
                sum += assignments.at(variable);
            }
        }
        sum = std::round(sum * 1e10) / 1e10;  // Round to 10 decimal places
        return sum >= _minsum;
    }

private:
    double _minsum;
    std::optional<std::vector<double>> _multipliers;
};


class MaxProdConstraint : public Constraint {
public:
    /**
     * Constraint enforcing that values of given variables create a product up to at most a given amount.
     */

    MaxProdConstraint(double maxprod) : _maxprod(maxprod) {}

    void preProcess(
        const std::vector<std::string>& variables,
        std::unordered_map<std::string, std::vector<int>>& domains,
        std::vector<std::pair<std::shared_ptr<Constraint>, std::vector<std::string>>>& constraints,
        std::unordered_map<std::string, std::vector<std::pair<std::shared_ptr<Constraint>, std::vector<std::string>>>>& vconstraints
    ) override {
        Constraint::preProcess(variables, domains, constraints, vconstraints);

        std::vector<bool> variable_contains_lt1;
        std::optional<std::string> variable_with_lt1;

        for (const auto& variable : variables) {
            bool contains_lt1 = std::any_of(domains[variable].begin(), domains[variable].end(),
                                            [](int value) { return value < 1; });
            variable_contains_lt1.push_back(contains_lt1);
            if (contains_lt1) {
                if (variable_with_lt1.has_value()) {
                    return;
                }
                variable_with_lt1 = variable;
            }
        }

        for (const auto& variable : variables) {
            if (variable_with_lt1.has_value() && *variable_with_lt1 != variable) {
                continue;
            }
            auto& domain = domains[variable];
            domain.erase(std::remove_if(domain.begin(), domain.end(),
                [this](int value) { 
                    return value > _maxprod || (value == 0 && _maxprod < 0);
                }),
                domain.end());
        }
    }

    bool operator()(
        const std::vector<std::string>& variables,
        const std::unordered_map<std::string, std::vector<int>>& domains,
        const std::unordered_map<std::string, int>& assignments,
        bool forwardcheck = false
    ) const override {
        double prod = 1;
        for (const auto& variable : variables) {
            auto it = assignments.find(variable);
            if (it != assignments.end()) {
                prod *= it->second;
            }
        }
        prod = std::round(prod * 1e10) / 1e10;  // Round to 10 decimal places

        if (prod > _maxprod) {
            return false;
        }

        if (forwardcheck) {
            for (const auto& variable : variables) {
                if (assignments.find(variable) == assignments.end()) {
                    auto domainIt = domains.find(variable);
                    if (domainIt != domains.end()) {
                        auto domain = domainIt->second;
                        domain.erase(std::remove_if(domain.begin(), domain.end(),
                            [this, prod](int value) { return prod * value > _maxprod; }),
                            domain.end());
                        if (domain.empty()) {
                            return false;
                        }
                    }
                }
            }
        }

        return true;
    }

private:
    double _maxprod;
};


class MinProdConstraint : public Constraint {
public:
    /**
     * Constraint enforcing that values of given variables create a product up to at least a given amount.
     */

    MinProdConstraint(double minprod) : _minprod(minprod) {}

    void preProcess(
        const std::vector<std::string>& variables,
        std::unordered_map<std::string, std::vector<int>>& domains,
        std::vector<std::pair<std::shared_ptr<Constraint>, std::vector<std::string>>>& constraints,
        std::unordered_map<std::string, std::vector<std::pair<std::shared_ptr<Constraint>, std::vector<std::string>>>>& vconstraints
    ) override {
        Constraint::preProcess(variables, domains, constraints, vconstraints);

        for (const auto& variable : variables) {
            auto& domain = domains[variable];
            domain.erase(std::remove_if(domain.begin(), domain.end(),
                [this](int value) {
                    return value == 0 && _minprod > 0;
                }),
                domain.end());
        }
    }

    bool operator()(
        const std::vector<std::string>& variables,
        const std::unordered_map<std::string, std::vector<int>>& domains,
        const std::unordered_map<std::string, int>& assignments,
        bool forwardcheck = false
    ) const override {
        // check if each variable is in the assignments
        for (const auto& variable : variables) {
            if (assignments.find(variable) == assignments.end()) {
                return true;
            }
        }

        // with each variable assigned, calculate the product
        double prod = 1;
        for (const auto& variable : variables) {
            prod *= assignments.at(variable);
        }
        prod = std::round(prod * 1e10) / 1e10;  // Round to 10 decimal places

        return prod >= _minprod;
    }

private:
    double _minprod;
};


class InSetConstraint : public Constraint {
public:
    /**
     * Constraint enforcing that values of given variables are present in the given set.
     *
     * Example:
     *     Problem problem;
     *     problem.addVariables({"a", "b"}, {1, 2});
     *     problem.addConstraint(std::make_shared<InSetConstraint>(std::unordered_set<int>{1}));
     *     auto solutions = problem.getSolutions();
     *     // solutions should contain: [{'a': 1, 'b': 1}]
     */

    InSetConstraint(std::unordered_set<int> set) : _set(std::move(set)) {}

    bool operator()(
        const std::vector<std::string>& variables,
        const std::unordered_map<std::string, std::vector<int>>& domains,
        const std::unordered_map<std::string, int>& assignments,
        bool forwardcheck = false
    ) const override {
        // preProcess() will remove it.
        throw std::runtime_error("Can't happen");
    }

    void preProcess(
        const std::vector<std::string>& variables,
        std::unordered_map<std::string, std::vector<int>>& domains,
        std::vector<std::pair<std::shared_ptr<Constraint>, std::vector<std::string>>>& constraints,
        std::unordered_map<std::string, std::vector<std::pair<std::shared_ptr<Constraint>, std::vector<std::string>>>>& vconstraints
    ) override {
        for (const auto& variable : variables) {
            auto& domain = domains[variable];
            domain.erase(std::remove_if(domain.begin(), domain.end(),
                [this](int value) { return _set.find(value) == _set.end(); }),
                domain.end());

            auto& vconstraint = vconstraints[variable];
            vconstraint.erase(std::remove_if(vconstraint.begin(), vconstraint.end(),
                [this, &variables](const auto& pair) {
                    return pair.first.get() == this && pair.second == variables;
                }),
                vconstraint.end());
        }

        constraints.erase(std::remove_if(constraints.begin(), constraints.end(),
            [this, &variables](const auto& pair) {
                return pair.first.get() == this && pair.second == variables;
            }),
            constraints.end());
    }

private:
    std::unordered_set<int> _set;
};

class NotInSetConstraint : public Constraint {
public:
    /**
     * Constraint enforcing that values of given variables are not present in the given set.
     *
     * Example:
     *     Problem problem;
     *     problem.addVariables({"a", "b"}, {1, 2});
     *     problem.addConstraint(std::make_shared<NotInSetConstraint>(std::unordered_set<int>{1}));
     *     auto solutions = problem.getSolutions();
     *     // solutions should contain: [{'a': 2, 'b': 2}]
     */

    NotInSetConstraint(std::unordered_set<int> set) : _set(std::move(set)) {}

    bool operator()(
        const std::vector<std::string>& variables,
        const std::unordered_map<std::string, std::vector<int>>& domains,
        const std::unordered_map<std::string, int>& assignments,
        bool forwardcheck = false
    ) const override {
        // preProcess() will remove it.
        throw std::runtime_error("Can't happen");
    }

    void preProcess(
        const std::vector<std::string>& variables,
        std::unordered_map<std::string, std::vector<int>>& domains,
        std::vector<std::pair<std::shared_ptr<Constraint>, std::vector<std::string>>>& constraints,
        std::unordered_map<std::string, std::vector<std::pair<std::shared_ptr<Constraint>, std::vector<std::string>>>>& vconstraints
    ) override {
        for (const auto& variable : variables) {
            auto& domain = domains[variable];
            domain.erase(std::remove_if(domain.begin(), domain.end(),
                [this](int value) { return _set.find(value) != _set.end(); }),
                domain.end());

            auto& vconstraint = vconstraints[variable];
            vconstraint.erase(std::remove_if(vconstraint.begin(), vconstraint.end(),
                [this, &variables](const auto& pair) {
                    return pair.first.get() == this && pair.second == variables;
                }),
                vconstraint.end());
        }

        constraints.erase(std::remove_if(constraints.begin(), constraints.end(),
            [this, &variables](const auto& pair) {
                return pair.first.get() == this && pair.second == variables;
            }),
            constraints.end());
    }

private:
    std::unordered_set<int> _set;
};

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

class SomeInSetConstraint : public Constraint {
public:
    /**
     * Constraint enforcing that at least some of the values of given variables must be present in a given set.
     *
     * Example:
     *     Problem problem;
     *     problem.addVariables({"a", "b"}, {1, 2});
     *     problem.addConstraint(std::make_shared<SomeInSetConstraint>(std::unordered_set<int>{1}));
     *     auto solutions = problem.getSolutions();
     *     // solutions should contain: [{'a': 1, 'b': 1}, {'a': 1, 'b': 2}, {'a': 2, 'b': 1}]
     */

    SomeInSetConstraint(std::unordered_set<int> set, int n = 1, bool exact = false)
        : _set(std::move(set)), _n(n), _exact(exact) {}

    bool operator()(
        const std::vector<std::string>& variables,
        const std::unordered_map<std::string, std::vector<int>>& domains,
        const std::unordered_map<std::string, int>& assignments,
        bool forwardcheck = false
    ) const override {
        int missing = 0;
        int found = 0;

        for (const auto& variable : variables) {
            auto it = assignments.find(variable);
            if (it != assignments.end()) {
                found += (_set.find(it->second) != _set.end());
            } else {
                missing++;
            }
        }

        if (missing > 0) {
            if (_exact) {
                if (!(found <= _n && _n <= missing + found)) {
                    return false;
                }
            } else {
                if (_n > missing + found) {
                    return false;
                }
            }

            if (forwardcheck && (_n - found == missing)) {
                for (const auto& variable : variables) {
                    if (assignments.find(variable) == assignments.end()) {
                        auto domainIt = domains.find(variable);
                        if (domainIt != domains.end()) {
                            auto& domain = domainIt->second;
                            domain.erase(std::remove_if(domain.begin(), domain.end(),
                                [this](int value) { return _set.find(value) == _set.end(); }),
                                domain.end());
                            if (domain.empty()) {
                                return false;
                            }
                        }
                    }
                }
            }
        } else {
            if (_exact) {
                if (found != _n) {
                    return false;
                }
            } else {
                if (found < _n) {
                    return false;
                }
            }
        }

        return true;
    }

private:
    std::unordered_set<int> _set;
    int _n;
    bool _exact;
};

class SomeNotInSetConstraint : public Constraint {
public:
    /**
     * Constraint enforcing that at least some of the values of given variables must not be present in a given set.
     *
     * Example:
     *     Problem problem;
     *     problem.addVariables({"a", "b"}, {1, 2});
     *     problem.addConstraint(std::make_shared<SomeNotInSetConstraint>(std::unordered_set<int>{1}));
     *     auto solutions = problem.getSolutions();
     *     // solutions should contain: [{'a': 1, 'b': 2}, {'a': 2, 'b': 1}, {'a': 2, 'b': 2}]
     */

    SomeNotInSetConstraint(std::unordered_set<int> set, int n = 1, bool exact = false)
        : _set(std::move(set)), _n(n), _exact(exact) {}

    bool operator()(
        const std::vector<std::string>& variables,
        const std::unordered_map<std::string, std::vector<int>>& domains,
        const std::unordered_map<std::string, int>& assignments,
        bool forwardcheck = false
    ) const override {
        int missing = 0;
        int found = 0;

        for (const auto& variable : variables) {
            auto it = assignments.find(variable);
            if (it != assignments.end()) {
                found += (_set.find(it->second) == _set.end());
            } else {
                missing++;
            }
        }

        if (missing > 0) {
            if (_exact) {
                if (!(found <= _n && _n <= missing + found)) {
                    return false;
                }
            } else {
                if (_n > missing + found) {
                    return false;
                }
            }

            if (forwardcheck && (_n - found == missing)) {
                for (const auto& variable : variables) {
                    if (assignments.find(variable) == assignments.end()) {
                        auto domainIt = domains.find(variable);
                        if (domainIt != domains.end()) {
                            auto& domain = domainIt->second;
                            domain.erase(std::remove_if(domain.begin(), domain.end(),
                                [this](int value) { return _set.find(value) != _set.end(); }),
                                domain.end());
                            if (domain.empty()) {
                                return false;
                            }
                        }
                    }
                }
            }
        } else {
            if (_exact) {
                if (found != _n) {
                    return false;
                }
            } else {
                if (found < _n) {
                    return false;
                }
            }
        }

        return true;
    }

private:
    std::unordered_set<int> _set;
    int _n;
    bool _exact;
};



#endif // CONSTRAINTS_HPP
