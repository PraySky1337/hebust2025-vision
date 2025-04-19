#pragma once
#include <cassert>
#include <functional>
#include <iostream>
#include <map>

namespace at {
template <typename StateType, typename EventType>
class StateMachine
{
public:
  using Action = std::function<void()>;

  // 添加状态转移
  void add_transition(StateType from, EventType event, StateType to)
  {
    transitions[from][event] = to;
  }

  // 设置状态进入动作
  void set_on_enter(StateType state, Action action) { on_enter[state] = std::move(action); }

  // 设置状态退出动作
  void set_on_exit(StateType state, Action action) { on_exit[state] = std::move(action); }

  // 设置初始状态
  void set_initial_state(StateType state)
  {
    current = state;
    if (on_enter[state]) on_enter[state]();
  }

  // 处理事件
  void handle_event(EventType event)
  {
    auto it = transitions.find(current);
    if (it != transitions.end()) {
      auto ev = it->second.find(event);
      if (ev != it->second.end()) {
        StateType next = ev->second;
        if (on_exit[current]) on_exit[current]();
        current = next;
        if (on_enter[current]) on_enter[current]();
        return;
      }
    }
    std::cout << "[No transition] Current: " << current << " Event: " << event << std::endl;
  }

  // 获取当前状态
  StateType get_state() const { return current; }

private:
  StateType current;
  std::map<StateType, std::map<EventType, StateType>> transitions;
  std::map<StateType, Action> on_enter;
  std::map<StateType, Action> on_exit;
};
} // namespace at
