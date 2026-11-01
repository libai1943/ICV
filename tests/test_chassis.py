from pathlib import Path
import argparse,tempfile,shutil,os,subprocess,json
parser=argparse.ArgumentParser()
parser.add_argument('--compiler',default=shutil.which('g++') or shutil.which('clang++'))
parser.add_argument('--output',type=Path,default=Path(tempfile.mkdtemp(prefix='icv-chassis-')))
args=parser.parse_args()
if not args.compiler:parser.error('Specify a C++14 compiler with --compiler.')
C=Path(__file__).resolve().parents[1];T=args.output;T.mkdir(parents=True,exist_ok=True)
source=(C/'Chapter11/CollabPlanner/chassis/src/chassis_node.cpp').read_text(encoding='utf8')
source='\n'.join(l for l in source.splitlines() if not l.startswith('#include')).replace('int main(int argc, char **argv)','int application_main(int argc, char **argv)')
header=r'''
#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>
#include <cstdlib>
#undef assert
#define assert(x) do { if (!(x)) { std::cerr << "Assertion " << __LINE__ << std::endl; std::exit(1); } } while(0)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
std::atomic<int> reads{0};bool publisher_created=false,port_closed=false;
int protocol_starts=0,protocol_ends=0,publications=0;double feedback=0;
namespace std_msgs { struct Float64{double data=0;}; }
namespace sandbox_msgs {struct ChassisCommand {double steering=0,throttle=0;int target=0;};using ChassisCommandConstPtr=std::shared_ptr<const ChassisCommand>;}
namespace ros {
struct Publisher{void publish(const std_msgs::Float64&v){assert(publisher_created);publications++;feedback=v.data;}};
struct Subscriber{};
struct NodeHandle{template<class T>Publisher advertise(const char*,int,bool){publisher_created=true;return {};}template<class F>Subscriber subscribe(const char*,int,F){return {};}};
void init(int,char**,const char*){std::cerr << "Starting protocol fixture" << std::endl;}
void spin(){for(int i=0;i<1000&&reads.load()==0;i++)std::this_thread::sleep_for(std::chrono::milliseconds(1));assert(reads.load()>0);}
}
using tiny_serial_handle_t=int;const int TINY_SERIAL_INVALID=-1;
int tiny_serial_open(const char*,int){return 1;}
int tiny_serial_send(int,const void*,int n){return n;}
int tiny_serial_read(int,void*,int){return 0;}
void tiny_serial_close(int){assert(protocol_ends==2);port_closed=true;}
namespace Tiny {
template<int N>struct Packet{char storage[N]{};void*data(){return storage;}};
struct ProtoLight{
 bool active=false,checksum=false;std::vector<char> last;
 void enableCheckSum(){checksum=true;}
 void begin(int(*)(void*,const void*,int),int(*)(void*,void*,int)){assert(checksum);active=true;protocol_starts++;}
 void write(char*p,int n){last.assign(p,p+n);}
 template<int N>int read(Packet<N>&p){assert(active && publisher_created && protocol_starts==2);p.storage[0]=char(0xe3);int16_t v=125;std::memcpy(p.storage+1,&v,2);std::this_thread::sleep_for(std::chrono::milliseconds(1));reads++;return 3;}
 void end(){assert(active);active=false;protocol_ends++;}
};
}
'''
tail=r'''
int main(){char*argv[]={nullptr};assert(application_main(0,argv)==0);assert(port_closed&&publications>0&&feedback==1.25);assert(!rx_protocol.active&&!tx_protocol.active);
 auto msg=std::make_shared<sandbox_msgs::ChassisCommand>();msg->target=3;msg->throttle=.4;msg->steering=.2;CmdVelCb(msg);
 assert(tx_protocol.last.size()==6 && static_cast<unsigned char>(tx_protocol.last[0])==3 && static_cast<unsigned char>(tx_protocol.last[1])==0xe1);
 int16_t velocity;std::memcpy(&velocity,tx_protocol.last.data()+2,2);assert(velocity==25);
 std::cout<<"PASS: publisher initialized before receive, checksum setup, independent framing, bounded receiver shutdown, protocol/port lifetime, packet units and saturation\n";
}
'''
cpp=T/'chassis_regression.cpp';cpp.write_text(header+source+tail,encoding='utf8')
compiler=Path(args.compiler);exe=T/('chassis_regression.exe' if os.name=='nt' else 'chassis_regression')
env=dict(os.environ,ZIG_GLOBAL_CACHE_DIR=str(T/'zig-global-cache'),ZIG_LOCAL_CACHE_DIR=str(T/'zig-local-cache'))
cmd=[str(compiler)]+(['c++','-mcpu=baseline'] if compiler.stem=='zig' else [])+['-std=c++14',str(cpp),'-o',str(exe)]
if os.name!='nt':cmd+=['-pthread']
r=subprocess.run(cmd,capture_output=True,text=True,env=env);assert r.returncode==0,r.stderr
r=subprocess.run([str(exe)],capture_output=True,text=True);assert r.returncode==0,r.stdout+r.stderr
(T/'chassis-regression.json').write_text(json.dumps({'status':'PASS','result':r.stdout.strip(),'scope':'Actual source with portable ROS/serial fixtures.'},indent=2),encoding='utf8')
print(r.stdout)



