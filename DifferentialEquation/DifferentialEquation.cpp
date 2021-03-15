#define _CRT_SECURE_NO_WARNINGS
#include <boost/numeric/odeint.hpp>
#include <fmt/core.h>
#include <vector>

int main() {
  constexpr auto gamma = 0.15;
  std::vector<double> state{0.0, 1.0};
  boost::numeric::odeint::runge_kutta4<std::vector<double>> stepper;
  const auto harmonic_oscillator = [&gamma](const std::vector<double> &x,
                                            std::vector<double> &dxdt,
                                            double t) {
    dxdt[0] = x[1];
    dxdt[1] = -x[0] - gamma * x[1];
  };

  const auto ode_begin_iterator =
      boost::numeric::odeint::make_const_step_time_iterator_begin(
          stepper, harmonic_oscillator, state, 0.0, 10.0, 0.1);
  const auto ode_end_iterator =
      boost::numeric::odeint::make_const_step_time_iterator_end(
          stepper, harmonic_oscillator, state);
  for (auto ode_iterator = ode_begin_iterator; ode_iterator != ode_end_iterator;
       ++ode_iterator) {
    const auto &[current_state, time] = *ode_iterator;
    fmt::print("t = {} x = {} p = {}\n", time, current_state[0],
               current_state[1]);
  }
}
