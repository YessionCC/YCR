#include "sampler.hpp"

thread_local GeneralSampler threadSampler;

GeneralSampler& GeneralSampler::getThreadSampler() {
  return threadSampler;
}