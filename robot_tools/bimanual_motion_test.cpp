#include "bimanual_motion.hpp"
#include <cassert>
#include <limits>
#include <iostream>
using namespace bimanual;
int main(){
 Body b{},saved{};for(int i=0;i<15;++i)b[i]=saved[i]=.01f*i;
 Arm a{},z{};for(int i=0;i<14;++i)z[i]=(i+1)*.01f;
 apply(b,a,z,.5f);
 for(int i=0;i<15;++i)assert(b[i]==saved[i]);
 for(int i=15;i<29;++i)assert(std::abs(b[i]-z[i-15]*.5f)<1e-6);
 apply(b,a,z,1);assert(b[15]==z[0]&&b[21]==z[6]&&b[22]==z[7]&&b[28]==z[13]);
 Request r;std::string prefix="test_run 1 5 ",values;for(int i=0;i<14;++i)values+="0.01 ";
 assert(parse(prefix+values,"test_run",r));assert(valid(r,Body{}));
 assert(!parse("old_run 1 5 "+values,"test_run",r));
 assert(!parse("test_run 1 5 0 0 0 0 0 0 0","test_run",r));
 assert(!parse(prefix+values+" 0","test_run",r));
 assert(!parse("test_run 1 nan "+values,"test_run",r));
 Request bad=r;bad.q[7]=std::numeric_limits<float>::quiet_NaN();assert(!valid(bad,Body{}));
 bad=r;bad.q[8]=.81f;assert(!valid(bad,Body{}));
 bad=r;bad.q[1]=-.81f;assert(!valid(bad,Body{}));
 bad=r;bad.duration=3;bad.q[13]=.20f;assert(!valid(bad,Body{}));
 bad=r;bad.duration=5;bad.q[13]=.20f;assert(valid(bad,Body{}));
 std::cout<<"PASS mapping_14_to_29 preserved_lower15 strict_run_id exact_14_finite asymmetric_limits speed_limit\n";
}
