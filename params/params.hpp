#pragma once
#include <iostream>
#include <any>
#include <map>

class ICPParams: public std::map<std::string, std::any> {
using std::map<std::string, std::any>::map;
public:
    struct NoValueException: public std::runtime_error {
        NoValueException(const char* message): std::runtime_error(message) {}
    };
    template <typename T>
    T get(std::string key) {
        if (count(key) == 0) {
            if constexpr (std::is_same_v<T, double> || std::is_same_v<T, int> || std::is_same_v<T, bool>) {
                return T(1);
            } else {
                throw NoValueException(("Key [" + key + "] has no default value given.").c_str());
            }
        } else {
            return std::any_cast<T>(this->operator[](key));
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const ICPParams& params) {
        for (const std::pair<std::string, std::any>& pr: params) {
            os << pr.first << " ";
        }
    }
};