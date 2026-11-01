// =============================================================================
//  mpegmech/span.hpp —— 轻量 span（C++17 兼容）
// =============================================================================
//  文件    : mpegmech/span.hpp
//  用途    : 提供 std::span 的最小替代品。
//  编码    : UTF-8
//
//  ── 为什么不用 std::span ────────────────────────────────────────────────────
//  `std::span` 是 **C++20** 特性。本库与 rtpmech / netmech 一样坚持 **C++17**
//  （目标平台包含嵌入式板卡，工具链可能不支持 C++20）。
//
//  本实现只提供实际用到的子集：
//    * 从裸指针+长度 或 容器/数组 构造
//    * size() / data() / empty() / operator[] / begin() / end()
//    * subspan() / first()
//  不提供只读/可变分离（统一为 `span<T>`，由 T 是否 const 决定可写性）。
//
//  ── 与 rtpmech 的关系 ──────────────────────────────────────────────────────
//  两份 span.hpp **内容等价、命名空间不同**。刻意不共享，避免 mpegmech 对
//  rtpmech 产生依赖（媒体处理与网络栈没有耦合理由）。
// =============================================================================
#pragma once

#include <cstddef>
#include <iterator>
#include <type_traits>
#include <utility>
#include <vector>

namespace mpegmech {

/// 连续内存的轻量视图（std::span 的 C++17 子集替代）。
///
/// @tparam T 元素类型；`span<const uint8_t>` 即只读视图。
///
/// @warning 本类**不拥有**内存。被指向的容器必须比视图活得更久。
template <typename T>
class span {
public:
    using element_type = T;
    using value_type   = std::remove_cv_t<T>;
    using pointer      = T*;
    using reference    = T&;
    using size_type    = std::size_t;
    using iterator     = T*;

    /// 默认构造：空视图。
    constexpr span() noexcept = default;

    /// 从指针 + 长度构造。
    constexpr span(T* data, size_type len) noexcept : data_(data), size_(len) {}

    /// 从容器构造（要求 data() 与 size()）。
    template <typename Container,
              typename = std::void_t<decltype(std::declval<Container&>().data()),
                                     decltype(std::declval<Container&>().size())>>
    constexpr span(Container& c) noexcept : data_(c.data()), size_(c.size())
    {
    }

    /// 从 const 容器构造（生成只读视图）。
    template <typename Container,
              typename = std::void_t<decltype(std::declval<const Container&>().data()),
                                     decltype(std::declval<const Container&>().size())>>
    constexpr span(const Container& c) noexcept : data_(c.data()), size_(c.size())
    {
    }

    /// 从 C 数组构造。
    template <std::size_t N>
    constexpr span(T (&arr)[N]) noexcept : data_(arr), size_(N)
    {
    }

    /// 从 span<U> 构造（U -> T 可转换，支持去掉 const）。
    template <typename U,
              typename = std::enable_if_t<std::is_convertible_v<U*, T*>>>
    constexpr span(const span<U>& other) noexcept
        : data_(other.data()), size_(other.size())
    {
    }

    [[nodiscard]] constexpr T*        data() const noexcept { return data_; }
    [[nodiscard]] constexpr size_type size() const noexcept { return size_; }
    [[nodiscard]] constexpr size_type size_bytes() const noexcept
    {
        return size_ * sizeof(T);
    }
    [[nodiscard]] constexpr bool empty() const noexcept { return size_ == 0; }

    [[nodiscard]] constexpr T& operator[](size_type i) const noexcept
    {
        return data_[i];
    }

    [[nodiscard]] constexpr iterator begin() const noexcept { return data_; }
    [[nodiscard]] constexpr iterator end() const noexcept { return data_ + size_; }

    /// 取子视图（越界自动截断，不抛异常）。
    [[nodiscard]] constexpr span subspan(size_type offset,
                                         size_type count) const noexcept
    {
        const size_type off = offset > size_ ? size_ : offset;
        const size_type max = size_ - off;
        return span(data_ + off, count > max ? max : count);
    }

    /// 从头取 count 个元素。
    [[nodiscard]] constexpr span first(size_type count) const noexcept
    {
        return span(data_, count > size_ ? size_ : count);
    }

private:
    T*        data_ = nullptr;
    size_type size_ = 0;
};

} // namespace mpegmech
