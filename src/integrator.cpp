#include "integrator.hpp"

thread_local TMode Integrator::transportMode = 
  Integrator::TransportMode::FromCamera;

void Integrator::setTransportMode(TransportMode tmode) {
  transportMode = tmode;
}
TMode Integrator::getTransportMode() {
  return transportMode;
}