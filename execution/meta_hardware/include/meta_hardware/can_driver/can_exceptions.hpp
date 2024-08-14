#ifndef META_HARDWARE_CAN_DRIVER_CAN_EXCEPTIONS_HPP
#define META_HARDWARE_CAN_DRIVER_CAN_EXCEPTIONS_HPP

#include <stdexcept>

namespace meta_hardware {

class CanException : public std::runtime_error {
  public:
    explicit CanException(const std::string &msg) : std::runtime_error(msg) {}
};

class CanIOException : public CanException {
  public:
    explicit CanIOException(const std::string &msg) : CanException(msg) {}
};

class CanIOTimedOutException : public CanIOException {
  public:
    explicit CanIOTimedOutException(const std::string &msg)
        : CanIOException(msg) {}
};

} // namespace meta_hardware

#endif // META_HARDWARE_CAN_DRIVER_CAN_EXCEPTIONS_HPP