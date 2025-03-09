#ifndef DOMAIN_HPP
#define DOMAIN_HPP

#include <string>
#include <iostream>
#include <vector>

// ----------------------------------------------------------------------
// Variables
// ----------------------------------------------------------------------

class Variable
{
public:
    std::string name;

    Variable(std::string name)
    {
        this->name = name;
    }

    friend std::ostream &operator<<(std::ostream &os, const Variable &obj)
    {
        os << "Variable(" << obj.name << ")";
        return os;
    }
};

Variable Unassigned = Variable("Unassigned");

// ----------------------------------------------------------------------
// Domains
// ----------------------------------------------------------------------

#include <vector>
#include <algorithm>

template <typename T>
class Domain : public std::vector<T>
{
private:
    std::vector<T> _hidden;
    std::vector<size_t> _states;

public:
    Domain(const std::vector<T> &set) : std::vector<T>(set) {}

    void resetState()
    {
        this->insert(this->end(), _hidden.begin(), _hidden.end());
        _hidden.clear();
        _states.clear();
    }

    void pushState()
    {
        _states.push_back(this->size());
    }

    void popState()
    {
        if (_states.empty())
            return;

        size_t diff = _states.back() - this->size();
        _states.pop_back();

        if (diff > 0)
        {
            auto start = _hidden.end() - diff;
            this->insert(this->end(), start, _hidden.end());
            _hidden.erase(start, _hidden.end());
        }
    }

    void hideValue(const T &value)
    {
        auto it = std::find(this->begin(), this->end(), value);
        if (it != this->end())
        {
            _hidden.push_back(*it);
            this->erase(it);
        }
    }
};

#endif // DOMAIN_HPP
