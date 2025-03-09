#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <unordered_map>
#include <vector>
#include <string>
#include <stdexcept>
#include <functional>
#include <algorithm>
#include <string>
#include <random>
#include <stdexcept>
#include <future>
#include <thread>

#include "domain.hpp"
#include "constraints.hpp"

template <typename T>
class Solver
{
public:
    static constexpr bool requires_pickling = false;

    virtual ~Solver() = default;

    virtual std::unordered_map<std::string, int> getSolution(
        const std::unordered_map<std::string, Domain<T> *> &domains,
        const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints)
    {
        throw std::runtime_error(std::string(typeid(*this).name()) + " is an abstract class");
    }

    virtual std::vector<std::unordered_map<std::string, int>> getSolutions(
        const std::unordered_map<std::string, Domain<T> *> &domains,
        const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints)
    {
        throw std::runtime_error(std::string(typeid(*this).name()) + " provides only a single solution");
    }

    virtual std::vector<std::unordered_map<std::string, int>>::iterator getSolutionIter(
        const std::unordered_map<std::string, Domain<T> *> &domains,
        const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints)
    {
        throw std::runtime_error(std::string(typeid(*this).name()) + " doesn't provide iteration");
    }
};

template <typename T>
class BacktrackingSolver : public Solver
{
public:
    BacktrackingSolver(bool forwardcheck = true) : _forwardcheck(forwardcheck) {}

    std::vector<std::unordered_map<std::string, int>>::iterator getSolutionIter(
        const std::unordered_map<std::string, Domain<T> *> &domains,
        const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints) override
    {

        std::unordered_map<std::string, int> assignments;
        std::vector<std::tuple<std::string, std::vector<int>, std::vector<Domain<T> *>>> queue;

        auto solutions = std::make_shared<std::vector<std::unordered_map<std::string, int>>>();

        std::function<void()> backtrack = [&]()
        {
            while (true)
            {
                // Mix the Degree and Minimum Remaining Values (MRV) heuristics
                std::vector<std::tuple<int, int, std::string>> lst;
                for (const auto &[variable, domain] : domains)
                {
                    if (assignments.find(variable) == assignments.end())
                    {
                        lst.emplace_back(-vconstraints.at(variable).size(), domain->size(), variable);
                    }
                }
                std::sort(lst.begin(), lst.end());

                std::string variable;
                std::vector<int> values;
                std::vector<Domain<T> *> pushdomains;

                if (!lst.empty())
                {
                    variable = std::get<2>(lst[0]);
                    values = domains.at(variable)->getValues();
                    if (_forwardcheck)
                    {
                        for (const auto &[var, domain] : domains)
                        {
                            if (assignments.find(var) == assignments.end() && var != variable)
                            {
                                pushdomains.push_back(domain);
                            }
                        }
                    }
                }
                else
                {
                    // No unassigned variables. We've got a solution.
                    solutions->push_back(assignments);
                    if (queue.empty())
                    {
                        return;
                    }
                    std::tie(variable, values, pushdomains) = queue.back();
                    queue.pop_back();
                    if (!pushdomains.empty())
                    {
                        for (auto domain : pushdomains)
                        {
                            domain->popState();
                        }
                    }
                }

                while (true)
                {
                    if (values.empty())
                    {
                        assignments.erase(variable);
                        while (!queue.empty())
                        {
                            std::tie(variable, values, pushdomains) = queue.back();
                            queue.pop_back();
                            if (!pushdomains.empty())
                            {
                                for (auto domain : pushdomains)
                                {
                                    domain->popState();
                                }
                            }
                            if (!values.empty())
                            {
                                break;
                            }
                            assignments.erase(variable);
                        }
                        if (queue.empty())
                        {
                            return;
                        }
                    }

                    assignments[variable] = values.back();
                    values.pop_back();

                    if (!pushdomains.empty())
                    {
                        for (auto domain : pushdomains)
                        {
                            domain->pushState();
                        }
                    }

                    bool consistent = true;
                    for (const auto &[constraint, vars] : vconstraints.at(variable))
                    {
                        if (!(*constraint)(vars, domains, assignments, pushdomains))
                        {
                            consistent = false;
                            break;
                        }
                    }

                    if (consistent)
                    {
                        break;
                    }

                    if (!pushdomains.empty())
                    {
                        for (auto domain : pushdomains)
                        {
                            domain->popState();
                        }
                    }
                }

                queue.emplace_back(variable, values, pushdomains);
            }
        };

        backtrack();

        return solutions->begin();
    }

    std::unordered_map<std::string, int> getSolution(
        const std::unordered_map<std::string, Domain<T> *> &domains,
        const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints) override
    {

        auto iter = getSolutionIter(domains, constraints, vconstraints);
        if (iter != std::vector<std::unordered_map<std::string, int>>().end())
        {
            return *iter;
        }
        return {};
    }

    std::vector<std::unordered_map<std::string, int>> getSolutions(
        const std::unordered_map<std::string, Domain<T> *> &domains,
        const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints) override
    {

        auto iter = getSolutionIter(domains, constraints, vconstraints);
        std::vector<std::unordered_map<std::string, int>> solutions;
        while (iter != std::vector<std::unordered_map<std::string, int>>().end())
        {
            solutions.push_back(*iter);
            ++iter;
        }
        return solutions;
    }

private:
    bool _forwardcheck;
};

template <typename T>
class OptimizedBacktrackingSolver : public Solver
{
public:
    OptimizedBacktrackingSolver(bool forwardcheck = true) : _forwardcheck(forwardcheck) {}

    std::vector<std::unordered_map<std::string, int>>::iterator getSolutionIter(
        const std::unordered_map<std::string, Domain<T> *> &domains,
        const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints) override
    {

        std::unordered_map<std::string, int> assignments;
        auto sorted_variables = getSortedVariables(domains, vconstraints);
        std::vector<std::tuple<std::string, std::vector<int>, std::vector<Domain<T> *>>> queue;

        auto solutions = std::make_shared<std::vector<std::unordered_map<std::string, int>>>();

        std::function<void()> backtrack = [&]()
        {
            while (true)
            {
                std::string variable;
                std::vector<int> values;
                std::vector<Domain<T> *> pushdomains;

                for (const auto &var : sorted_variables)
                {
                    if (assignments.find(var) == assignments.end())
                    {
                        variable = var;
                        values = domains.at(variable)->getValues();
                        if (_forwardcheck)
                        {
                            for (const auto &[v, domain] : domains)
                            {
                                if (assignments.find(v) == assignments.end() && v != variable)
                                {
                                    pushdomains.push_back(domain);
                                }
                            }
                        }
                        break;
                    }
                }

                if (variable.empty())
                {
                    solutions->push_back(assignments);
                    if (queue.empty())
                    {
                        return;
                    }
                    std::tie(variable, values, pushdomains) = queue.back();
                    queue.pop_back();
                    if (!pushdomains.empty())
                    {
                        for (auto domain : pushdomains)
                        {
                            domain->popState();
                        }
                    }
                }

                while (true)
                {
                    if (values.empty())
                    {
                        assignments.erase(variable);
                        while (!queue.empty())
                        {
                            std::tie(variable, values, pushdomains) = queue.back();
                            queue.pop_back();
                            if (!pushdomains.empty())
                            {
                                for (auto domain : pushdomains)
                                {
                                    domain->popState();
                                }
                            }
                            if (!values.empty())
                            {
                                break;
                            }
                            assignments.erase(variable);
                        }
                        if (queue.empty())
                        {
                            return;
                        }
                    }

                    assignments[variable] = values.back();
                    values.pop_back();

                    if (!pushdomains.empty())
                    {
                        for (auto domain : pushdomains)
                        {
                            domain->pushState();
                        }
                    }

                    bool consistent = true;
                    for (const auto &[constraint, vars] : vconstraints.at(variable))
                    {
                        if (!(*constraint)(vars, domains, assignments, pushdomains))
                        {
                            consistent = false;
                            break;
                        }
                    }

                    if (consistent)
                    {
                        break;
                    }

                    if (!pushdomains.empty())
                    {
                        for (auto domain : pushdomains)
                        {
                            domain->popState();
                        }
                    }
                }

                queue.emplace_back(variable, values, pushdomains);
            }
        };

        backtrack();

        return solutions->begin();
    }

    std::vector<std::unordered_map<std::string, int>> getSolutionsList(
        const std::unordered_map<std::string, Domain<T> *> &domains,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints)
    {

        std::unordered_map<std::string, int> assignments;
        std::vector<std::pair<std::string, std::vector<int>>> queue;
        std::vector<std::unordered_map<std::string, int>> solutions;
        auto sorted_variables = getSortedVariables(domains, vconstraints);

        std::function<void()> backtrack = [&]()
        {
            while (true)
            {
                std::string variable;
                std::vector<int> values;

                for (const auto &var : sorted_variables)
                {
                    if (assignments.find(var) == assignments.end())
                    {
                        variable = var;
                        values = domains.at(variable)->getValues();
                        break;
                    }
                }

                if (variable.empty())
                {
                    solutions.push_back(assignments);
                    if (queue.empty())
                    {
                        return;
                    }
                    std::tie(variable, values) = queue.back();
                    queue.pop_back();
                }

                while (true)
                {
                    if (values.empty())
                    {
                        assignments.erase(variable);
                        while (!queue.empty())
                        {
                            std::tie(variable, values) = queue.back();
                            queue.pop_back();
                            if (!values.empty())
                            {
                                break;
                            }
                            assignments.erase(variable);
                        }
                        if (queue.empty())
                        {
                            return;
                        }
                    }

                    assignments[variable] = values.back();
                    values.pop_back();

                    bool consistent = true;
                    for (const auto &[constraint, vars] : vconstraints.at(variable))
                    {
                        if (!(*constraint)(vars, domains, assignments, nullptr))
                        {
                            consistent = false;
                            break;
                        }
                    }

                    if (consistent)
                    {
                        break;
                    }
                }

                queue.emplace_back(variable, values);
            }
        };

        backtrack();

        return solutions;
    }

    std::vector<std::unordered_map<std::string, int>> getSolutions(
        const std::unordered_map<std::string, Domain<T> *> &domains,
        const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints) override
    {

        if (_forwardcheck)
        {
            auto iter = getSolutionIter(domains, constraints, vconstraints);
            std::vector<std::unordered_map<std::string, int>> solutions;
            while (iter != std::vector<std::unordered_map<std::string, int>>().end())
            {
                solutions.push_back(*iter);
                ++iter;
            }
            return solutions;
        }
        return getSolutionsList(domains, vconstraints);
    }

    std::unordered_map<std::string, int> getSolution(
        const std::unordered_map<std::string, Domain<T> *> &domains,
        const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints) override
    {

        auto iter = getSolutionIter(domains, constraints, vconstraints);
        if (iter != std::vector<std::unordered_map<std::string, int>>().end())
        {
            return *iter;
        }
        return {};
    }

private:
    bool _forwardcheck;

    std::vector<std::string> getSortedVariables(
        const std::unordered_map<std::string, Domain<T> *> &domains,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints)
    {

        std::vector<std::tuple<int, int, std::string>> lst;
        for (const auto &[variable, domain] : domains)
        {
            lst.emplace_back(-vconstraints.at(variable).size(), domain->size(), variable);
        }
        std::sort(lst.begin(), lst.end());

        std::vector<std::string> result;
        for (const auto &[_, __, variable] : lst)
        {
            result.push_back(variable);
        }
        return result;
    }
};

template <typename T>
class RecursiveBacktrackingSolver : public Solver
{
public:
    RecursiveBacktrackingSolver(bool forwardcheck = true) : _forwardcheck(forwardcheck) {}

    std::vector<std::unordered_map<std::string, int>> recursiveBacktracking(
        std::vector<std::unordered_map<std::string, int>> &solutions,
        std::unordered_map<std::string, Domain<T> *> &domains,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints,
        std::unordered_map<std::string, int> &assignments,
        bool single)
    {

        std::vector<std::tuple<int, int, std::string>> lst;
        for (const auto &[variable, domain] : domains)
        {
            lst.emplace_back(-vconstraints.at(variable).size(), domain->size(), variable);
        }
        std::sort(lst.begin(), lst.end());

        std::string variable;
        for (const auto &[_, __, var] : lst)
        {
            if (assignments.find(var) == assignments.end())
            {
                variable = var;
                break;
            }
        }

        if (variable.empty())
        {
            solutions.push_back(assignments);
            return solutions;
        }

        assignments[variable] = 0; // Placeholder value

        std::vector<Domain<T> *> pushdomains;
        if (_forwardcheck)
        {
            for (const auto &[var, domain] : domains)
            {
                if (assignments.find(var) == assignments.end())
                {
                    pushdomains.push_back(domain);
                }
            }
        }

        for (int value : domains[variable]->getValues())
        {
            assignments[variable] = value;
            if (!pushdomains.empty())
            {
                for (auto domain : pushdomains)
                {
                    domain->pushState();
                }
            }

            bool consistent = true;
            for (const auto &[constraint, variables] : vconstraints.at(variable))
            {
                if (!(*constraint)(variables, domains, assignments, pushdomains))
                {
                    consistent = false;
                    break;
                }
            }

            if (consistent)
            {
                recursiveBacktracking(solutions, domains, vconstraints, assignments, single);
                if (!solutions.empty() && single)
                {
                    return solutions;
                }
            }

            if (!pushdomains.empty())
            {
                for (auto domain : pushdomains)
                {
                    domain->popState();
                }
            }
        }

        assignments.erase(variable);
        return solutions;
    }

    std::unordered_map<std::string, int> getSolution(
        std::unordered_map<std::string, Domain<T> *> &domains,
        const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints) override
    {

        std::vector<std::unordered_map<std::string, int>> solutions;
        std::unordered_map<std::string, int> assignments;
        solutions = recursiveBacktracking(solutions, domains, vconstraints, assignments, true);
        return solutions.empty() ? std::unordered_map<std::string, int>{} : solutions[0];
    }

    std::vector<std::unordered_map<std::string, int>> getSolutions(
        std::unordered_map<std::string, Domain<T> *> &domains,
        const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints) override
    {

        std::vector<std::unordered_map<std::string, int>> solutions;
        std::unordered_map<std::string, int> assignments;
        return recursiveBacktracking(solutions, domains, vconstraints, assignments, false);
    }

    std::vector<std::unordered_map<std::string, int>>::iterator getSolutionIter(
        const std::unordered_map<std::string, Domain<T> *> &domains,
        const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints) override
    {

        throw std::runtime_error("RecursiveBacktrackingSolver doesn't provide iteration");
    }

private:
    bool _forwardcheck;
};

template <typename T>
class MinConflictsSolver : public Solver
{
public:
    MinConflictsSolver(int steps = 1000, std::mt19937 *rand = nullptr)
        : _steps(steps), _rand(rand ? rand : new std::mt19937(std::random_device{}())) {}

    ~MinConflictsSolver()
    {
        if (_rand)
            delete _rand;
    }

    std::unordered_map<std::string, int> getSolution(
        std::unordered_map<std::string, Domain<T> *> &domains,
        const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints) override
    {

        std::unordered_map<std::string, int> assignments;

        // Initial assignment
        for (const auto &[variable, domain] : domains)
        {
            std::uniform_int_distribution<> dis(0, domain->size() - 1);
            assignments[variable] = domain->getValues()[dis(*_rand)];
        }

        for (int step = 0; step < _steps; ++step)
        {
            bool conflicted = false;
            std::vector<std::string> variables;
            for (const auto &[variable, _] : domains)
            {
                variables.push_back(variable);
            }
            std::shuffle(variables.begin(), variables.end(), *_rand);

            for (const auto &variable : variables)
            {
                // Check if variable is not in conflict
                bool in_conflict = false;
                for (const auto &[constraint, vars] : vconstraints.at(variable))
                {
                    if (!(*constraint)(vars, domains, assignments))
                    {
                        in_conflict = true;
                        break;
                    }
                }
                if (!in_conflict)
                    continue;

                // Variable has conflicts. Find values with less conflicts.
                int mincount = vconstraints.at(variable).size();
                std::vector<int> minvalues;

                for (int value : domains[variable]->getValues())
                {
                    assignments[variable] = value;
                    int count = 0;
                    for (const auto &[constraint, vars] : vconstraints.at(variable))
                    {
                        if (!(*constraint)(vars, domains, assignments))
                        {
                            ++count;
                        }
                    }
                    if (count == mincount)
                    {
                        minvalues.push_back(value);
                    }
                    else if (count < mincount)
                    {
                        mincount = count;
                        minvalues.clear();
                        minvalues.push_back(value);
                    }
                }

                // Pick a random one from these values.
                std::uniform_int_distribution<> dis(0, minvalues.size() - 1);
                assignments[variable] = minvalues[dis(*_rand)];
                conflicted = true;
            }

            if (!conflicted)
            {
                return assignments;
            }
        }

        return {};
    }

    std::vector<std::unordered_map<std::string, int>> getSolutions(
        std::unordered_map<std::string, Domain<T> *> &domains,
        const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints) override
    {
        throw std::runtime_error("MinConflictsSolver provides only a single solution");
    }

    std::vector<std::unordered_map<std::string, int>>::iterator getSolutionIter(
        const std::unordered_map<std::string, Domain<T> *> &domains,
        const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints) override
    {
        throw std::runtime_error("MinConflictsSolver doesn't provide iteration");
    }

private:
    int _steps;
    std::mt19937 *_rand;
};

template <typename T>
class ParallelSolver : public Solver
{
public:
    ParallelSolver(bool process_mode = false) : _process_mode(process_mode)
    {
        requires_pickling = process_mode;
    }

    std::unordered_map<std::string, int> getSolution(
        std::unordered_map<std::string, Domain<T> *> &domains,
        const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints) override
    {
        throw std::runtime_error("ParallelSolver only provides all solutions");
    }

    std::vector<std::unordered_map<std::string, int>> getSolutions(
        std::unordered_map<std::string, Domain<T> *> &domains,
        const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints) override
    {
        return getSolutionsList(domains, vconstraints);
    }

    std::vector<std::unordered_map<std::string, int>> getSolutionsList(
        std::unordered_map<std::string, Domain<T> *> &domains,
        const std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &vconstraints)
    {

        // Precompute constraints lookup per variable
        std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> constraint_lookup;
        for (const auto &[var, _] : domains)
        {
            constraint_lookup[var] = vconstraints.count(var) ? vconstraints.at(var) : std::vector<std::pair<Constraint *, std::vector<std::string>>>{};
        }

        // Sort variables by domain size (heuristic)
        std::vector<std::string> sorted_vars;
        for (const auto &[var, _] : domains)
        {
            sorted_vars.push_back(var);
        }
        std::sort(sorted_vars.begin(), sorted_vars.end(),
                  [&domains](const std::string &a, const std::string &b)
                  {
                      return domains[a]->size() < domains[b]->size();
                  });

        // Split parallel and sequential parts
        std::string first_var = sorted_vars[0];
        std::vector<std::string> remaining_vars(sorted_vars.begin() + 1, sorted_vars.end());

        // Create the parallel function arguments and solutions vector
        std::vector<std::future<std::vector<std::unordered_map<std::string, int>>>> futures;
        std::vector<std::unordered_map<std::string, int>> solutions;

        // Execute in parallel
        std::vector<std::thread> threads;
        for (int val : domains[first_var]->getValues())
        {
            if (_process_mode)
            {
                // In a real implementation, you'd need to use some inter-process communication mechanism here
                throw std::runtime_error("Process mode not implemented in this C++ version");
            }
            else
            {
                futures.push_back(std::async(std::launch::async, &ParallelSolver::parallel_worker, this,
                                             requires_pickling, std::ref(domains), std::ref(constraint_lookup), first_var, val, remaining_vars));
            }
        }

        for (auto &future : futures)
        {
            auto result = future.get();
            solutions.insert(solutions.end(), result.begin(), result.end());
        }

        return solutions;
    }

private:
    bool _process_mode;
    bool requires_pickling;

    std::vector<std::unordered_map<std::string, int>> parallel_worker(
        bool requires_pickling,
        std::unordered_map<std::string, Domain<T> *> &domains,
        std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &constraint_lookup,
        const std::string &first_var,
        int val,
        std::vector<std::string> remaining_vars)
    {

        // Implementation of parallel_worker goes here
        // This would involve solving a subproblem with the first variable fixed to 'val'
        // You'd use a local solver (like OptimizedBacktrackingSolver) to solve this subproblem

        // Placeholder implementation:
        std::vector<std::unordered_map<std::string, int>> solutions;
        std::unordered_map<std::string, int> solution;
        solution[first_var] = val;
        solutions.push_back(solution);
        return solutions;
    }
};

// Helper functions
template <typename T>
bool is_valid(const std::unordered_map<std::string, int> &assignment,
              const std::vector<std::pair<Constraint *, std::vector<std::string>>> &constraints_lookup,
              const std::unordered_map<std::string, Domain<T> *> &domains)
{
    for (const auto &[constraint, vars_involved] : constraints_lookup)
    {
        bool all_vars_assigned = std::all_of(vars_involved.begin(), vars_involved.end(),
                                             [&assignment](const std::string &v)
                                             { return assignment.find(v) != assignment.end(); });

        if (all_vars_assigned && !(*constraint)(vars_involved, domains, assignment))
        {
            return false;
        }
    }
    return true;
}

template <typename T>
std::vector<std::unordered_map<std::string, int>> sequential_recursive_backtrack(
    std::unordered_map<std::string, int> &assignment,
    std::vector<std::string> &unassigned_vars,
    std::unordered_map<std::string, Domain<T> *> &domains,
    std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &constraint_lookup)
{

    if (unassigned_vars.empty())
    {
        return {assignment};
    }

    std::string var = unassigned_vars.back();
    unassigned_vars.pop_back();

    std::vector<std::unordered_map<std::string, int>> solutions;
    for (int value : domains[var]->getValues())
    {
        assignment[var] = value;
        if (is_valid(assignment, constraint_lookup[var], domains))
        {
            auto sub_solutions = sequential_recursive_backtrack(assignment, unassigned_vars, domains, constraint_lookup);
            solutions.insert(solutions.end(), sub_solutions.begin(), sub_solutions.end());
        }
        assignment.erase(var);
    }

    unassigned_vars.push_back(var);
    return solutions;
}

template <typename T>
std::vector<std::unordered_map<std::string, int>> sequential_optimized_backtrack(
    std::unordered_map<std::string, int> &assignment,
    std::vector<std::string> &unassigned_vars,
    std::unordered_map<std::string, Domain<T> *> &domains,
    std::unordered_map<std::string, std::vector<std::pair<Constraint *, std::vector<std::string>>>> &constraint_lookup)
{

    std::vector<std::pair<std::string, std::vector<int>>> queue;
    std::vector<std::unordered_map<std::string, int>> solutions;

    while (true)
    {
        std::string variable;
        std::vector<int> values;

        for (const auto &var : unassigned_vars)
        {
            if (assignment.find(var) == assignment.end())
            {
                variable = var;
                values = domains[var]->getValues();
                break;
            }
        }

        if (variable.empty())
        {
            solutions.push_back(assignment);
            if (queue.empty())
            {
                return solutions;
            }
            std::tie(variable, values) = queue.back();
            queue.pop_back();
        }

        while (true)
        {
            if (values.empty())
            {
                assignment.erase(variable);
                while (!queue.empty())
                {
                    std::tie(variable, values) = queue.back();
                    queue.pop_back();
                    if (!values.empty())
                    {
                        break;
                    }
                    assignment.erase(variable);
                }
                if (queue.empty())
                {
                    return solutions;
                }
            }

            assignment[variable] = values.back();
            values.pop_back();

            bool consistent = true;
            for (const auto &[constraint, variables] : constraint_lookup[variable])
            {
                if (!(*constraint)(variables, domains, assignment))
                {
                    consistent = false;
                    break;
                }
            }

            if (consistent)
            {
                break;
            }
        }

        queue.emplace_back(variable, values);
    }
}

#endif // SOLVER_HPP