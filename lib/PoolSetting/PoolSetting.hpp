#ifndef POOLTESTING_POOLSETTING_HPP
#define POOLTESTING_POOLSETTING_HPP
#pragma once

#include <vector>
#include <functional>
#include "Arduino.h"

//#include "./Homie/Datatypes/Callbacks.hpp"

namespace PoolInternals {
//    class HomieClass;
//    class Config;
//    class Validation;
//    class BootConfig;

    class IPoolSetting {
    public:
//        static std::vector<IPoolSetting*> settings;
        static std::vector<IPoolSetting*> settingsConfig;
        static std::vector<IPoolSetting*> settingsProbeOffset;
        static std::vector<IPoolSetting*> settingsProbe;
        static std::vector<IPoolSetting*> settingsTime;
        static std::vector<IPoolSetting*> settingsLocation;

        bool isRequired() const;
        const char *getName() const;
        const char *getDescription() const;
        virtual bool isBool() const { return false; }
        virtual bool isLong() const { return false; }
        virtual bool isShort() const { return false; }
        virtual bool isUShort() const { return false; }
        virtual bool isDouble() const { return false; }
        virtual bool isConstChar() const { return false; }
        virtual const char *getType() const { return "unknown"; }

    protected:
        explicit IPoolSetting(const char *name, const char *description);
        const char *_name;
        const char *_description;
        bool _required;
        bool _provided;
//    public:
//        std::vector<IPoolSetting *> *_settings;
    };
}  // namespace PoolInternals

template<class T>
class PoolSetting : public PoolInternals::IPoolSetting {
//    friend PoolInternals::HomieClass;
//    friend PoolInternals::Config;
//    friend PoolInternals::Validation;
//    friend PoolInternals::BootConfig;

public:
    PoolSetting(const char *name, const char *description, std::vector<IPoolSetting *> *settings);
    T get() const;
    void set(T value);
    bool wasProvided() const;
    PoolSetting<T> &setDefaultValue(T defaultValue);
    PoolSetting<T> &setValidator(const std::function<bool(T candidate)> &validator);

private:
    T _value;
    std::function<bool(T candidate)> _validator;
    bool validate(T candidate) const;
    bool isBool() const;
    bool isLong() const;
    bool isShort() const;
    bool isUShort() const;
    bool isDouble() const;
    bool isConstChar() const;
    void ffree() const;
    const char *getType() const;
};

#endif //POOLTESTING_POOLSETTING_HPP
