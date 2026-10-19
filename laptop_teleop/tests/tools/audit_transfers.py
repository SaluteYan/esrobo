"""Read-only, test-only observations of dashboard, PICO and passive robot CAN."""
import argparse
from collections import Counter, defaultdict
import getpass
import json
from pathlib import Path
import threading
import time
import urllib.request


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=int,default=60)
    p.add_argument('--output',required=True)
    p.add_argument('--robot',action='store_true')
    args=p.parse_args()
    op=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    result=dict(test_only=True,observations=[],robot_can=None,
                not_tested=['headset internal processing','inactive input sources'])
    worker=None
    if args.robot:
        import paramiko
        ssh=paramiko.SSHClient();ssh.load_system_host_keys()
        ssh.connect('192.168.10.100',username='esrobo',password=getpass.getpass('Robot SSH password: '),timeout=8)
        def read_robot():
            command=r"""python3 - <<'R'
import socket,select,time,struct,json,subprocess,collections
links=json.loads(subprocess.check_output(['ip','-s','-j','-d','link','show'],text=True))
links=[l for l in links if l.get('link_type')=='can']
sockets={};times=collections.defaultdict(list);enable={}
for link in links:
 if link.get('operstate')!='DOWN' and 'UP' in link.get('flags',[]):
  s=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
  s.bind((link['ifname'],));sockets[s]=link['ifname']
start=time.monotonic();end=start+15
while sockets and time.monotonic()<end:
 ready,_,_=select.select(list(sockets),[],[],min(.05,end-time.monotonic()))
 for s in ready:
  raw=s.recv(16);cid,n,b=struct.unpack('=IB3x8s',raw);cid&=0x1fffffff
  key=sockets[s]+':'+hex(cid);times[key].append(time.monotonic())
  if 0x261<=cid<=0x267:enable[key]={'enabled':bool(b[5]&64),'received':time.monotonic()}
now=time.monotonic()
for s in sockets:s.close()
stats={}
for key,values in times.items():
 gaps=[b-a for a,b in zip(values,values[1:])]
 stats[key]={'frames':len(values),'hz':len(values)/(now-start),'max_gap_ms':max(gaps)*1000 if gaps else None,'gaps_over_100ms':sum(g>.1 for g in gaps)}
print(json.dumps({'read_only':True,'duration_s':now-start,'can_links':links,'streams':stats,'driver_states':{k:{'enabled':v['enabled'],'age_s':now-v['received']} for k,v in enable.items()},'processes':subprocess.run(['pgrep','-af','esrobo_link.gateway|hand_ros|linker_hand|rosmaster|waist'],capture_output=True,text=True).stdout}))
R"""
            try:
                _,out,err=ssh.exec_command(command,timeout=25)
                result['robot_can']=json.loads(out.read().decode())
                result['robot_can']['stderr']=err.read().decode()
            except Exception as exc:result['robot_can']=dict(error=str(exc))
            finally:ssh.close()
        worker=threading.Thread(target=read_robot);worker.start()
    start=time.monotonic()
    while time.monotonic()-start<args.seconds:
        sample=dict(monotonic_s=time.monotonic())
        for name,url in [('dashboard','http://127.0.0.1:8080/api/state'),('pico','http://127.0.0.1:8765/api/state')]:
            begin=time.perf_counter()
            try:
                state=json.load(op.open(url,timeout=2))
                if name=='dashboard':
                    r=state.get('robot',{})
                    sample[name]={k:r.get(k) for k in ('mode','age_s','feedback_cache_age_s','state_send_gap_s','state_send_gap_max_s','feedback')}
                    sample['processes']={k:v.get('running') for k,v in state.get('processes',{}).items()}
                else:sample[name]={k:state.get(k) for k in ('age_ms','source_update_hz','acquisition_diagnostics','acquisition_diagnostics_age_ms')}
            except Exception as exc:sample[name]=dict(error=str(exc))
            sample[name]['http_ms']=(time.perf_counter()-begin)*1000
        result['observations'].append(sample)
        time.sleep(.2)
    if worker:worker.join(1.)
    result['duration_s']=time.monotonic()-start
    Path(args.output).write_text(json.dumps(result,indent=2,ensure_ascii=False))
    print(json.dumps(dict(output=args.output,observations=len(result['observations']),robot_streams=len((result['robot_can'] or {}).get('streams',{})))))


if __name__=='__main__':main()
