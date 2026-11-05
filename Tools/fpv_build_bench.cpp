// Bench for Source/DroneSimUE/Sim/FpvBuild.h without Unreal:
//   g++ -O2 -std=c++17 -I../Source/DroneSimUE/Sim fpv_build_bench.cpp -o fpv_build_bench && ./fpv_build_bench
#include "FpvBuild.h"
#include <cstdio>
using namespace FpvBuild;
void run(Selection sel,const char*tag){
  Physical P=Derive(sel); Summary Sm=Summarize(sel,P);
  printf("== %s: mass %.0fg TWR %.2f hover %.0f%% hoverA %.1f maxA %.0f motorA %.0f hoverMin %.1f vmax %.0fkm/h I=(%.5f %.5f %.5f) J=%.2e\n  %s\n",tag,Sm.MassG,Sm.ThrustToWeight,Sm.HoverThrottle*100,Sm.HoverCurrentA,Sm.MaxCurrentA,Sm.MotorMaxCurrentA,Sm.HoverMinutes,Sm.MaxSpeedKmh,P.Inertia[0],P.Inertia[1],P.Inertia[2],P.RotorInertia,Sm.Warnings.c_str());
  FpvModel::Params R; Pids Pid; FpvModel::State B; PhysState S;
  double thr=PrimeHover(S,P,B); FpvModel::Input In; In.Throttle=thr;
  for(int i=0;i<200;i++) Step(B,S,In,R,P,Pid,0.005);
  printf("  hover 1s: vz %.2f rate %.1f %.1f %.1f V %.1f\n",B.Vel.Z,B.Rate.X*57.3,B.Rate.Y*57.3,B.Rate.Z*57.3,S.PackVolts);
  // roll step full stick for 0.3 s
  In.Roll=1.0; double peak=0,t63=-1; double target=FpvModel::ActualRateDps(1,R.Roll);
  for(int i=0;i<300;i++){Step(B,S,In,R,P,Pid,0.001); double r=B.Rate.X*57.3; if(r>peak)peak=r; if(t63<0&&r>0.63*target)t63=i;}
  printf("  roll step target %.0f: t63 %.0f ms peak %.0f end %.0f\n",target,t63,peak,B.Rate.X*57.3);
  In.Roll=0; for(int i=0;i<300;i++)Step(B,S,In,R,P,Pid,0.001);
  printf("  release 0.3s: rate %.1f\n",B.Rate.X*57.3);
  In.Yaw=1; double ty=-1;double yt=FpvModel::ActualRateDps(1,R.Yaw);
  for(int i=0;i<400;i++){Step(B,S,In,R,P,Pid,0.001); if(ty<0&&B.Rate.Z*57.3>0.63*yt)ty=i;}
  printf("  yaw step t63 %.0f ms end %.0f\n",ty,B.Rate.Z*57.3); In.Yaw=0;
  // punch
  B=FpvModel::State(); S=PhysState(); PrimeHover(S,P,B); In.Throttle=1;
  for(int i=0;i<1000;i++)Step(B,S,In,R,P,Pid,0.001);
  printf("  punch 1s: vz %.1f V %.1f A %.0f\n",B.Vel.Z,S.PackVolts,S.PackCurrentA);
}
int main(){
  run(Selection{},"5in default");
  run(Selection{2,3,2,3,1},"5in 6S");
  run(Selection{4,4,4,4,1},"7in LR liion");
  run(Selection{5,5,5,5,2},"10in");
  run(Selection{0,0,0,0,0},"3in whoop");
  run(Selection{1,1,1,0,0},"3.5");
  run(Selection{2,2,2,5,2},"5in heavy liion");
}
