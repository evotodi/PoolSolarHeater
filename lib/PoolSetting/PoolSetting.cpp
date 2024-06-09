#include "PoolSetting.hpp"

using namespace PoolInternals;

//std::vector<IPoolSetting*> __attribute__((init_priority(101))) IPoolSetting::settings;
std::vector<IPoolSetting*> __attribute__((init_priority(101))) IPoolSetting::settingsConfig;
std::vector<IPoolSetting*> __attribute__((init_priority(101))) IPoolSetting::settingsProbeOffset;
std::vector<IPoolSetting*> __attribute__((init_priority(101))) IPoolSetting::settingsProbe;
std::vector<IPoolSetting*> __attribute__((init_priority(101))) IPoolSetting::settingsTime;
std::vector<IPoolSetting*> __attribute__((init_priority(101))) IPoolSetting::settingsLocation;

PoolInternals::IPoolSetting::IPoolSetting(const char *name, const char *description)
        : _name(name), _description(description), _required(true), _provided(false) {
}

bool IPoolSetting::isRequired() const {
    return _required;
}

const char *IPoolSetting::getName() const {
    return _name;
}

const char *IPoolSetting::getDescription() const {
    return _description;
}




template<class T>
PoolSetting<T>::PoolSetting(const char *name, const char *description, std::vector<IPoolSetting *> *settings)
        : IPoolSetting(name, description), _value(), _validator([](T candidate) { return true; }) {
//    IPoolSetting::settings.push_back(this);
//    IPoolSetting::_settings->push_back(this);
    settings->push_back(this);
}

template<class T>
T PoolSetting<T>::get() const {
    return _value;
}

template<class T>
bool PoolSetting<T>::wasProvided() const {
    return _provided;
}

template<class T>
PoolSetting<T> &PoolSetting<T>::setDefaultValue(T defaultValue) {
    _value = defaultValue;
    _required = false;
    return *this;
}

template<class T>
PoolSetting<T> &PoolSetting<T>::setValidator(const std::function<bool(T candidate)> &validator) {
    _validator = validator;
    return *this;
}

template<class T>
bool PoolSetting<T>::validate(T candidate) const {
    return _validator(candidate);
}

template<class T>
void PoolSetting<T>::set(T value) {
    _value = value;
    _provided = true;
}

template<class T>
bool PoolSetting<T>::isBool() const { return false; }

template<class T>
bool PoolSetting<T>::isLong() const { return false; }

template<class T>
bool PoolSetting<T>::isShort() const { return false; }

template<class T>
bool PoolSetting<T>::isUShort() const { return false; }

template<class T>
bool PoolSetting<T>::isDouble() const { return false; }

template<class T>
bool PoolSetting<T>::isConstChar() const { return false; }

template<>
bool PoolSetting<bool>::isBool() const { return true; }

template<>
const char *PoolSetting<bool>::getType() const { return "bool"; }

template<>
bool PoolSetting<long>::isLong() const { return true; }

template<>
const char *PoolSetting<long>::getType() const { return "long"; }

template<>
bool PoolSetting<int16_t>::isShort() const { return true; }

template<>
const char *PoolSetting<int16_t>::getType() const { return "short"; }

template<>
bool PoolSetting<uint16_t>::isUShort() const { return true; }

template<>
const char *PoolSetting<uint16_t>::getType() const { return "u_short"; }

template<>
bool PoolSetting<double>::isDouble() const { return true; }

template<>
const char *PoolSetting<double>::getType() const { return "double"; }

template<>
bool PoolSetting<const char *>::isConstChar() const { return true; }

template<>
const char *PoolSetting<const char *>::getType() const { return "string"; }

template<>
PoolSetting<const char *> &PoolSetting<const char *>::setDefaultValue(const char *defaultValue) {
    //free any potentially prior set value
    free(const_cast<char *>(_value));
    //duplicate the value, so we own it, and might free when required
    _value = strdup(defaultValue);;
    _required = false;
    return *this;
}

template<>
void PoolSetting<const char *>::set(const char *value) {
    //free any potentially prior set value (that was copied)
    free(const_cast<char *>(_value));
    //duplicate the value, so we own it, and might free when required
    _value = strdup(value);
    _required = false;
}

// Needed because otherwise undefined reference to
template
class PoolSetting<bool>;

template
class PoolSetting<long>;

template
class PoolSetting<int16_t>;

template
class PoolSetting<uint16_t>;

template
class PoolSetting<double>;

template
class PoolSetting<const char *>;
