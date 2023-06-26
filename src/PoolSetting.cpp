#include "PoolSetting.h"

using namespace PoolInternals;

PoolInternals::IPoolSetting::IPoolSetting(const char * name, const char * description, std::vector<IPoolSetting*> * settings)
        : _name(name)
        , _description(description)
        , _settings(settings)
        , _provided(false)
        , _required(true) {
}

const char* IPoolSetting::getName() const {
    return _name;
}

const char* IPoolSetting::getDescription() const {
    return _description;
}

bool IPoolSetting::isRequired() const {
    return _required;
}

template<class T>
PoolSetting<T>::PoolSetting(const char *name, const char *description, std::vector<IPoolSetting *> *settings)
    :IPoolSetting(name, description, settings)
    , _value() {

    IPoolSetting::_settings->push_back(this);
}

template <class T>
T PoolSetting<T>::get() const {
    return _value;
}

template <class T>
PoolSetting<T>& PoolSetting<T>::setDefaultValue(T defaultValue) {
    _value = defaultValue;
    _required = false;
    return *this;
}

template <class T>
PoolSetting<T>& PoolSetting<T>::setValidator(const std::function<bool(T candidate)>& validator) {
    _validator = validator;
    return *this;
}

template <class T>
bool PoolSetting<T>::validate(T candidate) const {
    return _validator(candidate);
}

template <class T>
bool PoolSetting<T>::wasProvided() const {
    return _provided;
}

template <class T>
void PoolSetting<T>::set(T value) {
    ffree();
    _value = value;
    _provided = true;
}

template <class T>
bool PoolSetting<T>::isBool() const { return false; }

template <class T>
bool PoolSetting<T>::isLong() const { return false; }

template <class T>
bool PoolSetting<T>::isShort() const { return false; }

template <class T>
bool PoolSetting<T>::isUShort() const { return false; }

template <class T>
bool PoolSetting<T>::isDouble() const { return false; }

template <class T>
bool PoolSetting<T>::isConstChar() const { return false; }

template<>
bool PoolSetting<bool>::isBool() const { return true; }
template<>
const char* PoolSetting<bool>::getType() const { return "bool"; }
template<>
void PoolSetting<bool>::ffree() const {}

template<>
bool PoolSetting<long>::isLong() const { return true; }
template<>
const char* PoolSetting<long>::getType() const { return "long"; }
template<>
void PoolSetting<long>::ffree() const {}

template<>
bool PoolSetting<int16_t>::isShort() const { return true; }
template<>
const char* PoolSetting<int16_t>::getType() const { return "short"; }
template<>
void PoolSetting<int16_t>::ffree() const {}

template<>
bool PoolSetting<uint16_t>::isUShort() const { return true; }
template<>
const char* PoolSetting<uint16_t>::getType() const { return "u_short"; }
template<>
void PoolSetting<uint16_t>::ffree() const {}

template<>
bool PoolSetting<double>::isDouble() const { return true; }
template<>
const char* PoolSetting<double>::getType() const { return "double"; }
template<>
void PoolSetting<double>::ffree() const {}

template<>
bool PoolSetting<const char*>::isConstChar() const { return true; }
template<>
const char* PoolSetting<const char*>::getType() const { return "string"; }
template<>
void PoolSetting<const char*>::ffree() const { free(const_cast<char*>(_value)); }

// Needed because otherwise undefined reference to
template class PoolSetting<bool>;
template class PoolSetting<long>;
template class PoolSetting<int16_t>;
template class PoolSetting<uint16_t>;
template class PoolSetting<double>;
template class PoolSetting<const char*>;
