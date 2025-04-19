#pragma once
#include <memory>
#include <mutex>
#include <stdexcept>

template <typename T>
class Singleton
{
public:
  template <typename... Args>
  static void init(Args &&... args)
  {
    static_assert(
      std::is_constructible_v<T, Args &&...>,
      "Derived cannot be constructed with the provided arguments.");

    [[maybe_unused]] auto _ = std::make_unique<T>(std::forward<Args>(args)...);

    std::call_once(init_flag_, [args = std::make_tuple(std::forward<Args>(args)...)]() mutable {
      instance_.reset(nullptr);  // 防止重复调用时抛异常
      std::apply(
        [](auto &&... params) {
          instance_.reset(new T(std::forward<decltype(params)>(params)...));
        },
        std::move(args));
    });
  }

  static T & getInstance()
  {
    if (!instance_) {
      throw std::runtime_error("Singleton not initialized. Call init() first.");
    }
    return *instance_;
  }

  Singleton(const Singleton &) = delete;
  Singleton & operator=(const Singleton &) = delete;

protected:
  Singleton() = default;
  virtual ~Singleton() = default;

private:
  static std::unique_ptr<T> instance_;
  static std::once_flag init_flag_;
};

template <typename T>
std::unique_ptr<T> Singleton<T>::instance_ = nullptr;

template <typename T>
std::once_flag Singleton<T>::init_flag_;
