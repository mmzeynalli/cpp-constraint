#ifndef PROBLEM_HPP
#define PROBLEM_HPP

#include <iostream>
#include <vector>
#include <unordered_map>
#include <string>
#include <stdexcept>
#include <functional>
#include <cassert>

#include "constraints.hpp"
#include "domain.hpp"
#include "solver.hpp"

// Class definition
template <typename T>
class Problem
{
public:
    /**
     * Class used to define a problem and retrieve solutions.
     */
    Problem(Solver *solver = NULL) : _solver(solver ? solver : new OptimizedBacktrackingSolver())
    {
        /**
         * Initialization method.
         *
         * Args:
         *     solver: Problem solver (default OptimizedBacktrackingSolver)
         */
        // Check if solver is an instance of Solver
        // Warn for experimental parallel solver
        if (dynamic_cast<ParallelSolver *>(_solver) != nullptr)
        {
            std::cerr << "Warning: ParallelSolver is currently experimental, and unlikely to be faster than OptimizedBacktrackingSolver. Please report any issues." << std::endl;
        }
    }

    void reset()
    {
        /**
         * Reset the current problem definition.
         *
         * Example:
         *     Problem problem;
         *     problem.addVariable("a", {1, 2});
         *     problem.reset();
         */
        _constraints.clear();
        _variables.clear();
    }

    void setSolver(Solver *solver)
    {
        /**
         * Change the problem solver currently in use.
         *
         * Example:
         *     OptimizedBacktrackingSolver solver;
         *     Problem problem(solver);
         *
         * Args:
         *     solver: New problem solver
         */
        _solver = solver;
    }

    Solver *getSolver() const
    {
        /**
         * Obtain the problem solver currently in use.
         *
         * Returns:
         *     Solver currently in use
         */
        return _solver;
    }

    void addVariable(const std::string &variable, const Domain<T> &domain)
    {
        /**
         * Add a variable to the problem.
         *
         * Example:
         *     Problem<int> problem;
         *     problem.addVariable("a", Domain<int>({1, 2}));
         *
         * Args:
         *     variable: Object representing a problem variable
         *     domain: Set of items defining the possible values that the given variable may assume
         */
        if (_variables.find(variable) != _variables.end())
        {
            throw std::invalid_argument("Tried to insert duplicated variable " + variable);
        }
        if (domain.isEmpty())
        {
            throw std::invalid_argument("Domain is empty");
        }
        _variables[variable] = domain;
    }

    void addVariables(const std::vector<std::string> &variables, const Domain<T> &domain)
    {
        /**
         * Add one or more variables to the problem.
         *
         * Example:
         *     Problem<int> problem;
         *     problem.addVariables({"a", "b"}, Domain<int>({1, 2, 3}));
         *
         * Args:
         *     variables: Sequence of objects representing problem variables
         *     domain: Set of items defining the possible values that the given variables may assume
         */
        for (const auto &variable : variables)
            addVariable(variable, domain);
    }

    void addConstraint(const Constraint &constraint, const std::vector<std::string> &variables = {})
    {
        /**
         * Add a constraint to the problem.
         *
         * Example:
         *     Problem problem;
         *     problem.addVariables({"a", "b"}, {1, 2, 3});
         *     problem.addConstraint(lambda a, b: b == a+1, ["a", "b"])
         *
         * Args:
         *     constraint: Constraint to be included in the problem
         *     variables: Variables affected by the constraint (default to all variables)
         */
        _constraints.emplace_back(constraint, variables);
    }

    std::unordered_map<std::string, int> getSolution()
    {
        auto [domains, constraints, vconstraints] = _getArgs(_solver->requires_pickling());
        if (domains.empty())
        {
            return {};
        }
        return _solver->getSolution(domains, constraints, vconstraints);
    }

    std::vector<std::unordered_map<std::string, int>> getSolutions()
    {
        auto [domains, constraints, vconstraints] = _getArgs(_solver->requires_pickling());
        if (domains.empty())
        {
            return {};
        }
        return _solver->getSolutions(domains, constraints, vconstraints);
    }

    // For getSolutionIter, we'll return a custom iterator
    class SolutionIterator
    {
        // Implementation details omitted for brevity
    };

    SolutionIterator getSolutionIter()
    {
        auto [domains, constraints, vconstraints] = _getArgs(_solver->requires_pickling());
        if (domains.empty())
        {
            return SolutionIterator(); // Empty iterator
        }
        return SolutionIterator(_solver, domains, constraints, vconstraints);
    }

    std::vector<std::vector<int>> getSolutionsOrderedList(const std::vector<std::string> &order = {})
    {
        auto solutions = getSolutions();
        std::vector<std::vector<int>> result;

        if (order.empty() || order.size() == 1)
        {
            for (const auto &solution : solutions)
            {
                result.push_back(std::vector<int>(solution.begin(), solution.end()));
            }
        }
        else
        {
            for (const auto &solution : solutions)
            {
                std::vector<int> ordered_solution;
                for (const auto &key : order)
                {
                    ordered_solution.push_back(solution[key]);
                }
                result.push_back(ordered_solution);
            }
        }

        return result;
    }

    std::tuple<std::vector<std::vector<int>>, std::unordered_map<std::vector<int>, int>, int>
    getSolutionsAsListDict(const std::vector<std::string> &order = {}, bool validate = true)
    {
        auto solutions_list = getSolutionsOrderedList(order);
        int size_list = solutions_list.size();
        std::unordered_map<std::vector<int>, int> solutions_dict;

        for (int i = 0; i < size_list; ++i)
        {
            solutions_dict[solutions_list[i]] = i;
        }

        if (validate)
        {
            int size_dict = solutions_dict.size();
            if (size_list != size_dict)
            {
                throw std::runtime_error(std::to_string(size_list - size_dict) +
                                         " duplicate parameter configurations in searchspace, should not happen.");
            }
        }

        return std::make_tuple(solutions_list, solutions_dict, size_list);
    }

private:
    Solver *_solver;
    std::vector<std::pair<Constraint *, std::vector<std::string>>> _constraints;
    std::vector<std::string> _str_constraints;

    std::unordered_map<std::string, Domain<T> *> _variables;

    std::tuple<std::unordered_map<std::string, Domain<T> *>,
               std::vector<std::pair<Constraint *, std::vector<std::string>>>,
               std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>>>
    _getArgs(bool picklable = false)
    {
        auto domains = _variables;
        std::vector<std::string> allvariables;
        for (const auto &[var, _] : domains)
        {
            allvariables.push_back(var);
        }

        std::vector<std::pair<Constraint *, std::vector<std::string>>> constraints;

        // Parse string constraints
        if (!_str_constraints.empty())
        {
            // String constraint parsing would go here
        }

        // Add regular constraints
        for (const auto &[constraint, variables] : _constraints)
        {
            std::vector<std::string> vars = variables.empty() ? allvariables : variables;
            constraints.emplace_back(constraint, vars);
        }

        // Check for precompiled FunctionConstraints
        if (picklable)
        {
            for (const auto &[constraint, _] : constraints)
            {
                assert(dynamic_cast<FunctionConstraint *>(constraint) == nullptr &&
                       "You have used FunctionConstraints with ParallelSolver(process_mode=True). "
                       "Please use string constraints instead.");
            }
        }

        std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> vconstraints;
        for (const auto &[variable, _] : domains)
        {
            vconstraints[variable] = {};
        }

        for (const auto &[constraint, variables] : constraints)
        {
            for (const auto &variable : variables)
            {
                vconstraints[variable].emplace_back(constraint, variables);
            }
        }

        for (auto &[constraint, variables] : constraints)
        {
            constraint->preProcess(variables, domains, constraints, vconstraints);
        }

        for (auto &[_, domain] : domains)
        {
            domain->resetState();
            if (domain->isEmpty())
            {
                return {{}, {}, {}};
            }
        }

        return {domains, constraints, vconstraints};
    }
};

#endif // PROBLEM_HPP