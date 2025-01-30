#pragma once
#include "surface/surface_panel.hpp"
#include <type_traits>

template <typename Iterator, typename T>
concept IteratorOfType = requires(Iterator it) {
	typename std::iterator_traits<Iterator>::value_type;
	requires std::same_as<typename std::iterator_traits<Iterator>::value_type, T>;
};

template <typename Iterator>
concept SurfaceContainerIterator = IteratorOfType<Iterator, SurfacePanel>;

template<class T>
concept SurfaceType = std::convertible_to<T*, SurfacePanel*>;