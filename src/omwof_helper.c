#include "omwof_helper.h"



uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax)
{
  return ((((au32_IN - au32_INmin) * (au32_OUTmax - au32_OUTmin)) / (au32_INmax - au32_INmin)) + au32_OUTmin);
}

uint32_t Constrain(uint32_t au32_IN, uint32_t au32_MIN, uint32_t au32_MAX)
{
  if (au32_IN < au32_MIN)
  {
    return au32_MIN;
  }
  else if (au32_IN > au32_MAX)
  {
    return au32_MAX;
  }
  else
  {
    return au32_IN;
  }
}