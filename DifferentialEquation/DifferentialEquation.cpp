#define _CRT_SECURE_NO_WARNINGS
#include <utility>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include <fmt/core.h>

template <typename Container, typename Func, typename Stepper>
auto create_iterators(Container &state, Stepper &stepper, Func &func, double t0,
                      double tmax, double dt) {
  using namespace boost::numeric::odeint;

  return std::make_pair(
      make_const_step_time_iterator_begin(stepper, func, state, t0, tmax, dt),
      make_const_step_time_iterator_end(stepper, func, state));
}

template <typename Container, typename Func>
void solve_and_print(Container &state, Func &func, double t0, double tmax,
                     double dt) {
  boost::numeric::odeint::runge_kutta4<Container> stepper;

  auto [ode_begin_iterator, ode_end_iterator] =
      create_iterators(state, stepper, func, t0, tmax, dt);
  for (auto ode_iterator = ode_begin_iterator; ode_iterator != ode_end_iterator;
       ++ode_iterator) {
    const auto &[current_state, time] = *ode_iterator;
    fmt::print("t = {} x = {} p = {}\n", time, current_state[0],
               current_state[1]);
  }
}

int main() {
  std::vector<double> state{0.0, 1.0};

  const auto harmonic_oscillator = [gamma = 0.15](const std::vector<double> &x,
                                                  std::vector<double> &dxdt,
                                                  double /* t */) {
    dxdt[0] = x[1];
    dxdt[1] = -x[0] - gamma * x[1];
  };

  solve_and_print(state, harmonic_oscillator, 0.0, 10.0, 0.1);
}
