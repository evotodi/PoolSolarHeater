#pragma once

#include <vector>
#include <functional>
#include "Arduino.h"

namespace PoolInternals {
    class IPoolSetting {
    public:
        const char* getName() const;
        const char* getDescription() const;
        bool isRequired() const;

        virtual bool isBool() const { return false; }
        virtual bool isLong() const { return false; }
        virtual bool isShort() const { return false; }
        virtual bool isUShort() const { return false; }
        virtual bool isDouble() const { return false; }
        virtual bool isConstChar() const { return false; }

        virtual const char* getType() const { return "unknown"; }

    protected:
        explicit IPoolSetting(const char* name, const char* description, std::vector<IPoolSetting*> * settings);
        const char* _name;
        const char* _description;
        std::vector<IPoolSetting*>* _settings;
        bool _provided;
        bool _required;
    };
}  // namespace PoolInternals

template <class T>
class PoolSetting : public PoolInternals::IPoolSetting {
public:
    PoolSetting(const char* name, const char* description, std::vector<IPoolSetting*> * settings);

    T get() const;
    void set(T value);
    PoolSetting<T>& setDefaultValue(T defaultValue);
    PoolSetting<T>& setValidator(const std::function<bool(T candidate)>& validator);
    bool wasProvided() const;

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

    const char* getType() const;
};