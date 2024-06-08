#pragma once

#include <algorithm>
#include <array>
#include <cstring>
#include <ostream>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>

namespace tser {
namespace detail {
template <typename T>
concept aggregate = std::is_aggregate_v<T>;

template <typename T, typename... Args>
concept aggregate_initializable = aggregate<T> && requires { T{std::declval<Args>()...}; };

namespace detail {
struct any {
  template <typename T>
  constexpr operator T&() const noexcept;
  template <typename T>
  constexpr operator T&&() const noexcept;
};
} // namespace detail

namespace detail {
template <std::size_t I>
using indexed_any = any;

template <aggregate T, typename Indices>
struct aggregate_initializable_from_indices;

template <aggregate T, std::size_t... Indices>
struct aggregate_initializable_from_indices<T, std::index_sequence<Indices...>>
    : std::bool_constant<aggregate_initializable<T, indexed_any<Indices>...>> {};
} // namespace detail

template <typename T, std::size_t N>
concept aggregate_initializable_with_n_args =
    aggregate<T> && detail::aggregate_initializable_from_indices<T, std::make_index_sequence<N>>::value;

template <template <std::size_t> typename Predicate, std::size_t Beg, std::size_t End>
struct binary_search;

template <template <std::size_t> typename Predicate, std::size_t Beg, std::size_t End>
using binary_search_base = std::conditional_t<(End - Beg <= 1), std::integral_constant<std::size_t, Beg>,
    std::conditional_t<Predicate<(Beg + End) / 2>::value, binary_search<Predicate, (Beg + End) / 2, End>,
        binary_search<Predicate, Beg, (Beg + End) / 2>>>;

template <template <std::size_t> typename Predicate, std::size_t Beg, std::size_t End>
struct binary_search : binary_search_base<Predicate, Beg, End> {};

template <template <std::size_t> typename Predicate, std::size_t Beg, std::size_t End>
constexpr std::size_t binary_search_v = binary_search<Predicate, Beg, End>::value;

template <template <std::size_t> typename Predicate, std::size_t N>
struct forward_search : std::conditional_t<Predicate<N>::value, std::integral_constant<std::size_t, N>,
                            forward_search<Predicate, N + 1>> {};

template <template <std::size_t> typename Predicate, std::size_t N>
struct backward_search : std::conditional_t<Predicate<N>::value, std::integral_constant<std::size_t, N>,
                             backward_search<Predicate, N - 1>> {};

template <template <std::size_t> typename Predicate>
struct backward_search<Predicate, (std::size_t)-1> {};

namespace detail {
template <typename T>
  requires std::is_aggregate_v<T>
struct aggregate_inquiry {
  template <std::size_t N>
  struct initializable : std::bool_constant<aggregate_initializable_with_n_args<T, N>> {};
};

template <aggregate T>
struct minimum_initialization : forward_search<detail::aggregate_inquiry<T>::template initializable, 0> {};

template <aggregate T>
constexpr auto minimum_initialization_v = minimum_initialization<T>::value;
} // namespace detail

template <aggregate T>
struct num_aggregate_fields : binary_search<detail::aggregate_inquiry<T>::template initializable,
                                  detail::minimum_initialization_v<T>, 8 * sizeof(T) + 1> {};

template <aggregate T>
constexpr std::size_t num_aggregate_fields_v = num_aggregate_fields<T>::value;

template <std::size_t N>
struct struct_to_tuple_t;

#define TO_TUPLE_T(SIZE, FIELDS...)        \
  template <>                              \
  struct struct_to_tuple_t<SIZE> {         \
    template <typename S>                  \
    static auto tie(S&& s) {               \
      auto& [FIELDS] = std::forward<S>(s); \
      return std::tie(FIELDS);             \
    }                                      \
  };

TO_TUPLE_T(16, f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14, f15);
TO_TUPLE_T(15, f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14);
TO_TUPLE_T(14, f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13);
TO_TUPLE_T(13, f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12);
TO_TUPLE_T(12, f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11);
TO_TUPLE_T(11, f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10);
TO_TUPLE_T(10, f0, f1, f2, f3, f4, f5, f6, f7, f8, f9);
TO_TUPLE_T(9, f0, f1, f2, f3, f4, f5, f6, f7, f8);
TO_TUPLE_T(8, f0, f1, f2, f3, f4, f5, f6, f7);
TO_TUPLE_T(7, f0, f1, f2, f3, f4, f5, f6);
TO_TUPLE_T(6, f0, f1, f2, f3, f4, f5);
TO_TUPLE_T(5, f0, f1, f2, f3, f4);
TO_TUPLE_T(4, f0, f1, f2, f3);
TO_TUPLE_T(3, f0, f1, f2);
TO_TUPLE_T(2, f0, f1);
TO_TUPLE_T(1, f0);

#undef TO_TUPLE_T
} // namespace detail

template <typename S>
auto struct_to_tuple(S&& s) {
  constexpr auto num_fields = detail::num_aggregate_fields_v<std::decay_t<S>>;
  return detail::struct_to_tuple_t<num_fields>::tie(std::forward<S>(s));
}
} // namespace tser

namespace tser {
template <typename T>
size_t encode_varint(T value, char* output) {
  size_t i = 0;
  if constexpr (std::is_signed_v<T>) {
    value = static_cast<T>(value << 1 ^ (value >> (sizeof(T) * 8 - 1)));
  }
  for (; value > 127; ++i, value >>= 7) {
    output[i] = static_cast<char>(static_cast<uint8_t>(value & 127) | 128);
  }
  output[i++] = static_cast<uint8_t>(value) & 127;
  return i;
}

template <typename T>
size_t decode_varint(T& value, const char* const input) {
  size_t i = 0;
  for (value = 0; i == 0 || (input[i - 1] & 128); i++) {
    value |= static_cast<T>(input[i] & 127) << (7 * i);
  }
  if constexpr (std::is_signed_v<T>) {
    value = (value & 1) ? -static_cast<T>((value + 1) >> 1) : (value + 1) >> 1;
  }
  return i;
}
} // namespace tser

namespace tser {
// implementation details for C++20 is_detected
namespace detail {
struct ns {
  ~ns() = delete;
  ns(ns const&) = delete;
};

template <class Default, class AlwaysVoid, template <class...> class Op, class... Args>
struct detector {
  using value_t = std::false_type;
  using type = Default;
};
template <class Default, template <class...> class Op, class... Args>
struct detector<Default, std::void_t<Op<Args...>>, Op, Args...> {
  using value_t = std::true_type;
  using type = Op<Args...>;
};

template <class T>
struct is_array : std::is_array<T> {};
template <template <typename, size_t> class TArray, typename T, size_t N>
struct is_array<TArray<T, N>> : std::true_type {};
} // namespace detail

// we need a bunch of template metaprogramming for being able to differentiate
// between different types
template <template <class...> class Op, class... Args>
constexpr bool is_detected_v = detail::detector<detail::ns, void, Op, Args...>::value_t::value;

class BinaryArchive;

template <class T>
using has_begin_t = decltype(*std::begin(std::declval<T>()));
template <class T>
using has_members_t = decltype(T::_serializable);
template <class T>
using has_smaller_t = decltype(std::declval<T>() < std::declval<T>());
template <class T>
using has_equal_t = decltype(std::declval<T>() == std::declval<T>());
template <class T>
using has_nequal_t = decltype(std::declval<T>() != std::declval<T>());
template <class T>
using has_outstream_op_t = decltype(std::declval<std::ostream>() << std::declval<T>());
template <class T>
using has_tuple_t = std::tuple_element_t<0, T>;
template <class T>
using has_optional_t = decltype(std::declval<T>().has_value());
template <class T>
using has_element_t = typename T::element_type;
template <class T>
using has_mapped_t = typename T::mapped_type;
template <class T>
using has_custom_save_t = decltype(std::declval<T>().save(std::declval<BinaryArchive&>()));
template <class T>
using has_free_save_t = decltype(std::declval<const T&>() << std::declval<BinaryArchive&>());
template <class T>
constexpr bool is_container_v = is_detected_v<has_begin_t, T>;
template <class T>
constexpr bool is_tuple_v = is_detected_v<has_tuple_t, T>;
template <class T>
constexpr bool is_tser_t_v = is_detected_v<has_members_t, T>;
template <class T>
constexpr bool is_pointer_like_v =
    std::is_pointer_v<T> || is_detected_v<has_element_t, T> || is_detected_v<has_optional_t, T>;

class BinaryArchive {
private:
  std::string _bytes = std::string(1024, '\0');
  size_t _bufferSize = 0;
  size_t _readOffset = 0;

public:
  explicit BinaryArchive(const size_t initialSize = 1024) : _bytes(initialSize, '\0') {
  }

  template <typename T, std::enable_if_t<!std::is_integral_v<T>, int> = 0>
  explicit BinaryArchive(const T& t) {
    save(t);
  }

  template <typename T>
  void save(const T& t) {
    if constexpr (is_detected_v<has_free_save_t, T>) {
      operator<<(t, *this);
    } else if constexpr (is_detected_v<has_custom_save_t, T>) {
      t.save(*this);
    } else if constexpr (is_tuple_v<T>) {
      std::apply(
          [&](auto&... tVal) {
            (save(tVal), ...);
          },
          t);
    } else if constexpr (is_pointer_like_v<T>) {
      save(static_cast<bool>(t));
      if (t) {
        save(*t);
      }
    } else if constexpr (is_container_v<T>) {
      if constexpr (!detail::is_array<T>::value) {
        save(t.size());
      }
      for (auto& val : t) {
        save(val);
      }
    } else {
      if (_bufferSize + sizeof(T) + sizeof(T) / 4 > _bytes.size()) {
        _bytes.resize((_bufferSize + sizeof(T)) * 2);
      }
      if constexpr (std::is_integral_v<T> && sizeof(T) > 2) {
        _bufferSize += encode_varint(t, _bytes.data() + _bufferSize);
      } else if constexpr (std::is_integral_v<T> || std::is_same_v<T, std::monostate>) {
        std::memcpy(_bytes.data() + _bufferSize, std::addressof(t), sizeof(T));
        _bufferSize += sizeof(T);
      } else if constexpr (detail::num_aggregate_fields_v<T> > 0) {
        auto tuple = struct_to_tuple(t);
        save(tuple);
      }
    }
  }

  template <typename T>
  void load(T& t) {
    using V = std::decay_t<T>;
    if constexpr (is_detected_v<has_free_save_t, V>) {
      operator>>(t, *this);
    } else if constexpr (is_detected_v<has_custom_save_t, T>) {
      t.load(*this);
    } else if constexpr (is_tuple_v<V>) {
      std::apply(
          [&](auto&... tVal) {
            (load(tVal), ...);
          },
          t);
    } else if constexpr (is_pointer_like_v<T>) {
      if constexpr (std::is_pointer_v<T>) {
        t = load<bool>() ? (t = new std::remove_pointer_t<T>(), load(*t), t) : nullptr;
      } else if constexpr (is_detected_v<has_optional_t, T>) {
        t = load<bool>() ? T(load<typename V::value_type>()) : T();
      } else {
        t = T(load<has_element_t<V>*>());
      }
    } else if constexpr (is_container_v<T>) {
      if constexpr (!detail::is_array<T>::value) {
        const auto size = load<decltype(t.size())>();
        using VT = typename V::value_type;
        for (size_t i = 0; i < size; ++i)
          if constexpr (!is_detected_v<has_mapped_t, V>) {
            t.insert(t.end(), load<VT>());
          } else {
            t.emplace(VT{load<typename V::key_type>(), load<typename V::mapped_type>()});
          }
      } else {
        for (auto& val : t) {
          load(val);
        }
      }
    } else {
      if constexpr (std::is_integral_v<T> && sizeof(T) > 2) {
        _readOffset += decode_varint(t, _bytes.data() + _readOffset);
      } else if constexpr (std::is_integral_v<T> || std::is_same_v<T, std::monostate>) {
        std::memcpy(&t, _bytes.data() + _readOffset, sizeof(T));
        _readOffset += sizeof(T);
      } else if constexpr (detail::num_aggregate_fields_v<T> > 0) {
        auto tuple = struct_to_tuple(t);
        load(tuple);
      }
    }
  }

  template <typename T>
  T load() {
    std::remove_const_t<T> t{};
    load(t);
    return t;
  }

  template <typename T>
  friend BinaryArchive& operator<<(BinaryArchive& ba, const T& t) {
    ba.save(t);
    return ba;
  }

  template <typename T>
  friend BinaryArchive& operator>>(BinaryArchive& ba, T& t) {
    ba.load(t);
    return ba;
  }

  void reset() {
    _bufferSize = 0;
    _readOffset = 0;
  }

  void initialize(std::string_view str) {
    _bytes = str;
    _bufferSize = str.size();
    _readOffset = 0;
  }

  std::string_view get_buffer() const {
    return std::string_view(_bytes.data(), _bufferSize);
  }

  std::string extract_buffer() {
    _bytes.resize(_bufferSize);
    return std::move(_bytes);
  }
};

template <class Base, typename Derived>
std::conditional_t<std::is_const_v<Derived>, const Base, Base>& base(Derived* thisPtr) {
  return *thisPtr;
}

template <typename T>
T deserialize(std::string_view encoded) {
  BinaryArchive ba(0);
  ba.initialize(encoded);
  return ba.load<T>();
}

template <typename T>
std::string serialize(const T& object) {
  BinaryArchive ba(256);
  ba.save(object);
  return ba.extract_buffer();
}
} // namespace tser

namespace tser {
template <typename _Variant, size_t _Idx>
void save_variant(const _Variant& variant, tser::BinaryArchive& bin) {
  if (variant.index() == _Idx) {
    const auto& value = std::get<_Idx>(variant);
    bin.save(value);
  } else {
    if constexpr ((_Idx + 1) < std::variant_size_v<_Variant>) {
      save_variant<_Variant, _Idx + 1>(variant, bin);
    }
  }
}

template <typename... _Types>
void operator<<(const std::variant<_Types...>& variant, tser::BinaryArchive& bin) {
  bin.save((size_t)variant.index());

  save_variant<std::variant<_Types...>, 0>(variant, bin);
}

template <typename _Variant, size_t _Idx>
void load_variant(_Variant& variant, size_t variant_index, tser::BinaryArchive& bin) {
  if (variant_index == _Idx) {
    std::variant_alternative_t<_Idx, _Variant> value;
    bin.load(value);

    variant = std::move(value);
  } else {
    if constexpr ((_Idx + 1) < std::variant_size_v<_Variant>) {
      load_variant<_Variant, _Idx + 1>(variant, variant_index, bin);
    }
  }
}

template <typename... _Types>
void operator>>(std::variant<_Types...>& variant, tser::BinaryArchive& bin) {
  size_t variant_index;
  bin.load(variant_index);

  load_variant<std::variant<_Types...>, 0>(variant, variant_index, bin);
}
} // namespace tser
