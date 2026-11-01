from pathlib import Path
import ast, json, math, textwrap, sys, subprocess, os
import numpy as np
import argparse, tempfile, shutil
parser=argparse.ArgumentParser()
parser.add_argument('--compiler',default=shutil.which('g++') or shutil.which('clang++'))
parser.add_argument('--output',type=Path,default=Path(tempfile.mkdtemp(prefix='icv-tests-')))
args=parser.parse_args()
if not args.compiler: parser.error('Specify a C++14 compiler with --compiler (g++, clang++, or zig).')
C=Path(__file__).resolve().parents[1];T=args.output;T.mkdir(parents=True,exist_ok=True)
localization=C/'Chapter11/CollabPlanner/nokov_localization/script/nokov_localization.py'
source=localization.read_text(encoding='utf8');tree=ast.parse(source)
classes=[n for n in tree.body if isinstance(n,ast.ClassDef) and n.name in ['MarkerSet','Vehicle']]
class RotationStub:
    @staticmethod
    def from_quat(q):
        class R:
            def as_euler(self,seq):return 0.,0.,math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
        return R()
ns={'np':np,'Rotation':RotationStub,'rad_offset':0.,'VehicleParams':{'marker_height':.1,'height':.2,'rear_hang':.03,'front_hang':.04,'wheel_base':.143,'width':.19}}
exec(compile(ast.Module(body=classes,type_ignores=[]),str(localization),'exec'),ns)
from types import SimpleNamespace as S
def message(x,y,yaw=0.):return S(pose=S(position=S(x=x,y=y,z=0),orientation=S(x=0,y=0,z=math.sin(yaw/2),w=math.cos(yaw/2))))
v=ns['Vehicle']();v.check_ready();assert not v.ready
v.markers={0:message(0,0)};v.check_ready();assert not v.ready
v.markers={0:message(0,0),1:None};v.check_ready();assert not v.ready
v.markers={0:message(0,0),1:message(2,0),2:message(-1,0)};v.check_ready();assert v.ready and v.rear_center==2
assert v.get_pose()==(-1,0,0.)
v=ns['Vehicle']();v.markers={0:message(0,0,math.pi/2),1:message(0,2),2:message(0,-1)};v.check_ready();assert v.ready and v.rear_center==2
assert 'check_ready' in ns['Vehicle'].__dict__

cpp=(C/'Chapter11/CollabPlanner/pure_pursuit/src/pure_pursuit.cpp').read_text(encoding='utf8')
def block(s,signature):
    start=s.index(signature); b=s.index('{',start);depth=1;e=b+1
    while depth:
        if s[e]=='{':depth+=1
        elif s[e]=='}':depth-=1
        e+=1
    return s[start:e]
header=r'''
#include <algorithm>
#include <array>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>
using std::hypot; using std::sin; using std::cos; using std::atan2; using std::fabs;
using std::next_permutation;
namespace ros { struct TimerEvent {}; struct Time { static Time now(){return {};} double toSec()const{return 0.;} }; }
namespace std_msgs { struct Int32 {int data=0;}; }
struct TrajectoryPoint {double time=0,x=0,y=0,yaw=0,velocity=0,acceleration=0;};
struct Trajectory {int target=0;std::vector<TrajectoryPoint> points;};
struct AprilObject {enum {VEHICLE=1};int id=0,type=VEHICLE;double x=0,y=0,yaw=0;};
struct ChassisCommand {int target=0;double throttle=0,steering=0;};
using AprilObjectConstPtr=std::shared_ptr<const AprilObject>;
using TrajectoryConstPtr=std::shared_ptr<const Trajectory>;
struct Pub {std::vector<ChassisCommand> commands;std::vector<int> reached;
 void publish(const ChassisCommand& x){commands.push_back(x);}
 void publish(const std_msgs::Int32& x){reached.push_back(x.data);}
};
struct Visual {struct {std::vector<int> points;} line_strip_;void visualize(double,double){};};
const double wheel_base_=0.143,center_to_rear_=0.0825,approaching_tolerance_=0.05,lfw_=0.,lrv_=0.;
'''
helpers=block(cpp,'TrajectoryPoint Interpolate(')+'\n'+block(cpp,'TrajectoryPoint InterpolateWithTime(')
fields=r'''
class Tracker { public:
 Trajectory trajectory_;AprilObject object_;std::vector<std::pair<int,int>> ranges_;
 int current_range_=0,tracking_object_=0,car_index_=0,lookahead_index_=0;
 double start_time_=0.,lookahead_distance_=0.18,lookahead_gain_=1.;
 double longitude_kp_=1.8,longitude_ki_=0.,longitude_output_=0.,longitude_perror_=0.;
 bool pose_initialized_=false,approached_=false,final_approached_=false;
 Visual visualization_;Pub command_publisher_,reached_publisher_;
'''
signatures=['    void ObjectCallback(','    void TrajCallback(','    bool ValidRange(','    void PublishNeutralCommand(','    inline double DistanceToTraj(','    void UpdateLookahead(','    void SwitchRange(','    double LongitudeController(','    void Controller(','    void ControlCallback(']
methods='\n'.join(block(cpp,x) for x in signatures)
assign=(C/'Chapter11/CollabPlanner/collab_planner/collab_node.cpp').read_text(encoding='utf8')
assignment=r'''
struct Pose {double px=0,py=0,heading=0;Pose()=default;Pose(double x,double y,double a=0):px(x),py(y),heading(a){} double x()const{return px;}double y()const{return py;}double theta()const{return heading;}};
typedef std::array<Pose,8> Poses;
class Assignment {public:
'''
assignment+='\n'.join(block(assign,x) for x in ['    double EuclideanDistance(','    double CalculateVariance(','    struct VariancePairing'])+';\n'+block(assign,'    Poses FindOptimalPairing(')+'\n};\n'
test=r'''
int main(){
 Tracker t;auto p=std::make_shared<AprilObject>();p->x=0.;p->y=0.;t.ObjectCallback(p);
 assert(t.pose_initialized_ && t.object_.x==0. && t.object_.y==0.);
 p->x=100.;p->y=200.;t.ObjectCallback(p);assert(std::abs(t.object_.x-.1)<1e-12);
 p->x=2000.;p->y=2000.;t.ObjectCallback(p);assert(std::abs(t.object_.x-.1)<1e-12 && std::abs(t.object_.y-.2)<1e-12);
 p->x=std::numeric_limits<double>::quiet_NaN();t.ObjectCallback(p);assert(std::abs(t.object_.x-.1)<1e-12);
 auto tr=std::make_shared<Trajectory>();tr->points={{0,0,0,0,.1,0},{1,1,0,0,.1,0}};
 t.TrajCallback(tr);assert(t.ValidRange() && t.ranges_[0].second==2);
 assert(t.trajectory_.points.at(t.ranges_[0].second-1).x==1.);
 t.object_.x=1.;t.object_.y=0.;t.ControlCallback({});assert(t.approached_);
 assert(t.command_publisher_.commands.size()==1 && t.command_publisher_.commands.back().throttle==0.);
 assert(t.reached_publisher_.reached.size()==1);
 auto bad=std::make_shared<Trajectory>();t.TrajCallback(bad);assert(t.trajectory_.points.size()==2);
 bad->points={{0,0,0,0,.1,0},{0,1,0,0,.1,0}};t.TrajCallback(bad);assert(t.trajectory_.points[1].time==1.);
 tr->points={{0,0,0,0,.1,0},{1,.1,0,0,.1,0},{2,.1,0,0,-.1,0},{3,0,0,0,-.1,0}};
 t.TrajCallback(tr);assert(t.ranges_.size()==2 && t.current_range_==0);
 t.object_.x=.1;t.object_.y=0.;t.SwitchRange();assert(t.current_range_==1 && t.car_index_==2 && t.lookahead_index_==2);
 t.TrajCallback(tr);assert(t.current_range_==0 && t.car_index_==0 && !t.approached_);
 t.current_range_=9;t.ControlCallback({});assert(t.command_publisher_.commands.back().throttle==0.);
 TrajectoryPoint a,b;a.yaw=3.13;b.yaw=-3.13;assert(std::abs(Interpolate(a,b,.5).yaw)>3.);
 Assignment as;Poses from{},to{};
 for(int i=0;i<8;i++){from[i]=Pose(i*.17,i*i*.031);to[i]=Pose(std::sin(i*.83),std::cos(i*.71),i*.1);}
 auto result=as.FindOptimalPairing(from,to);
 std::vector<int> indices(8);std::iota(indices.begin(),indices.end(),0);
 std::vector<std::pair<double,double>> scores;
 do{std::vector<double>d;for(int i=0;i<8;i++)d.push_back(as.EuclideanDistance(from[i],to[indices[i]]));scores.emplace_back(as.CalculateVariance(d),std::accumulate(d.begin(),d.end(),0.));}while(std::next_permutation(indices.begin(),indices.end()));
 std::sort(scores.begin(),scores.end());double expected=0.;for(int i=0;i<50;i++)expected=std::max(expected,scores[i].second);
 double actual=0.;for(int i=0;i<8;i++)actual+=as.EuclideanDistance(from[i],result[i]);assert(std::abs(actual-expected)<1e-10);
 // Exercise every retained-candidate position, including the final heap entry.
 std::priority_queue<Assignment::VariancePairing> q;for(int i=0;i<50;i++)q.push({double(i),i==0?100.:double(i),{}});
 auto best=q.top();q.pop();int visited=1;while(!q.empty()){auto candidate=q.top();q.pop();visited++;if(candidate.distanceSum>best.distanceSum)best=candidate;}
 assert(visited==50 && best.distanceSum==100.);
 std::cout<<"PASS: pose units, zero coordinates, rejected measurements, range endpoints, completion, replacement, heading interpolation, fixed-array assignment, all 50 candidates\n";
}
'''
harness=T/'runtime_regression.cpp';harness.write_text(header+helpers+fields+methods+'\n};\n'+assignment+test,encoding='utf8')
env=dict(os.environ,ZIG_GLOBAL_CACHE_DIR=str(T/'zig-global-cache'),ZIG_LOCAL_CACHE_DIR=str(T/'zig-local-cache'))
compiler=Path(args.compiler);exe=T/('runtime_regression.exe' if os.name=='nt' else 'runtime_regression')
result=subprocess.run([str(compiler)]+(['c++'] if compiler.stem=='zig' else [])+['-std=c++14','-O0',str(harness),'-o',str(exe)],env=env,capture_output=True,text=True)
(T/'cpp-build.log').write_text(result.stdout+result.stderr,encoding='utf8');assert result.returncode==0,result.stderr
run=subprocess.run([str(exe)],capture_output=True,text=True);assert run.returncode==0,run.stdout+run.stderr
report={'status':'PASS','python':'Actual MarkerSet/Vehicle class definitions exercised with test messages; unavailable, empty and rotated-marker cases checked.',
        'cpp':run.stdout.strip(),'scope':'Portable regression harness compiles actual changed function bodies with interface stand-ins; no ROS installation or physical execution.'}
(T/'runtime-regression.json').write_text(json.dumps(report,indent=2),encoding='utf8')
print(json.dumps(report,indent=2))
