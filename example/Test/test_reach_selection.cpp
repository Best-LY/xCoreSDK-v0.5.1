#include <iostream>
#include <opencv2/core.hpp>
#include "mannequin_reach_planner.hpp"
using Model=rokae::xMateModel<6>;

int main() {
  try {
    using namespace mannequin_reach;
    Candidate rejected; rejected.name="unreachable_short"; rejected.tcpLength=.1;
    Candidate a; a.name="reachable_long"; a.passed=true; a.tcpLength=2; a.jointTravel=3;
    Candidate b=a; b.name="reachable_short"; b.tcpLength=1;
    require(best({rejected,a,b})==2,"reachability must precede length");
    Candidate c=b; c.jointTravel=2;
    require(best({b,c})==1,"joint travel tie-break");
    bool failed=false;
    try { best({rejected}); } catch(...) { failed=true; }
    require(failed,"no-feasible route must fail");
    Limits limits{}; for(auto& bound:limits) bound={-6.28,6.28};
    Joints q{},reference{}; q.fill(2*M_PI+.1); reference.fill(.1);
    require(staged_motion::jointError(legalTurns(q,reference,limits),reference)<1e-9,"nearest legal turn");
    q[0]=std::numeric_limits<double>::quiet_NaN(); failed=false;
    try { legalTurns(q,reference,limits); } catch(...) { failed=true; }
    require(failed,"nonfinite IK accepted");
    std::cout<<"[PASS] 5 reach-selection tests; no SDK connection or motion\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<"[FAIL] "<<e.what()<<'\n'; return 1; }
}
