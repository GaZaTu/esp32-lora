#include "AsyncRadio.hpp"
#include "tser.hpp"
#include "unishox2.h"

namespace rf {
class string : public std::string {
public:
  using std::string::basic_string;
  using std::string::operator=;
};
} // namespace rf

namespace rf {
struct SignalStrengthRequest {};

struct SignalStrengthResponse {
  // strength but positive instead of negative
  uint8_t rssi = 0;
  // signal to noise ratio
  int8_t snr = 0;
  // frequency error in Hz x 100
  int8_t ferr = 0;
};

struct Message {
  std::variant<std::monostate,
    // Possible messages
    SignalStrengthRequest,
    SignalStrengthResponse
  > variant;
};
} // namespace rf

namespace tser {
void operator<<(const rf::string& string, tser::BinaryArchive& bin);
void operator>>(rf::string& string, tser::BinaryArchive& bin);
} // namespace tser

namespace rf {
namespace detail {
template <typename... Ts>
struct overload : Ts... {
  using Ts::operator()...;
};
} // namespace detail

template <typename Driver, typename Message>
auto transmit(AsyncRadio<Driver>& radio, Message&& message) {
  return radio.transmit(tser::serialize(rf::Message{std::move(message)}));
}

template <typename... Ts>
using handlers = detail::overload<Ts...>;

template <typename Driver, typename... Handlers>
void handle(AsyncRadio<Driver>& radio, handlers<Handlers...>&& handlers) {
  auto message = tser::deserialize<rf::Message>(radio.readString());
  std::visit(handlers, message.variant);
}
}; // namespace rf
