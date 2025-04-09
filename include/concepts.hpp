#pragma once
#include "surface/surface_panel.hpp"
#include <ranges>
#include "aerocalcs/aerocalcsingle.hpp"
#include <type_traits>

template <typename Iterator, typename T>
concept IteratorOfType = requires(Iterator it) {
  typename std::iterator_traits<Iterator>::value_type;
  requires std::same_as<typename std::iterator_traits<Iterator>::value_type, T>;
};

template <typename Iterator>
concept SurfaceContainerIterator = IteratorOfType<Iterator, SurfacePanel>;

template <class T>
concept SurfaceType = std::convertible_to<T *, SurfacePanel *>;

template <typename R>
concept constant_integral_range = std::ranges::constant_range<R> &&
                                  std::integral<std::ranges::range_value_t<R>>;
#define RANGE(n) std::views::iota(0, (int)n)

template <typename R>
concept AeroResults_range =
    std::ranges::range<R> &&
    std::is_same_v<std::ranges::range_value_t<R>, AeroResults>;

template <typename R>
concept AeroResults_range_range =
    std::ranges::range<R> && AeroResults_range<std::ranges::range_value_t<R>>;
