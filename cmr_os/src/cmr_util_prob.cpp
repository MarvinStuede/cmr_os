#include "cmr_os/cmr_util_prob.h"

namespace cmr_os{
namespace util_prob{
double expCumDist(double rate, double x){return 1 - exp(-rate*x);}
double expDist(double rate, double x){
  if(x<0) return 0;
  return rate * exp(-rate*x);
}

}

}
