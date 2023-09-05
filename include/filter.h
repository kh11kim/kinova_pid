#include "core.h"

class LPF{
private:
  Vector7d prev;
  int dof;
  double alpha;

public:
  LPF(int filter_dof, double sampling_rate, double cutoff_frequency)
  {
    double tau = 1/(2*M_PI*cutoff_frequency);
    dof = filter_dof;
    alpha = (tau)/(tau+sampling_rate);
  }
  void Init(const Vector7d& init){
    prev = init;
  }
  Vector7d Apply(const Vector7d& raw){
    prev = prev*alpha + raw*(1.-alpha);
    return prev;
  }
};