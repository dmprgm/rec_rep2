#include "rec_rep2_servo/friction_observer.hpp"

namespace rec_rep2_servo
{

FrictionObserver::FrictionObserver(const Vector7d & L, const Vector7d & Lp)
: L_(L), Lp_(Lp), xi_(Vector7d::Zero())
{
}

void FrictionObserver::reset()
{
  xi_.setZero();
}

Vector7d FrictionObserver::update(
  const Vector7d & tau_measured, const Vector7d & g_q, const Vector7d & b_qdot, double dt)
{
  const Vector7d residual = tau_measured - g_q - b_qdot;
  const Vector7d d_xi = -(L_ + Lp_).cwiseProduct(xi_) + L_.cwiseProduct(residual);
  xi_ += dt * d_xi;
  return xi_;
}

}  // namespace rec_rep2_servo
