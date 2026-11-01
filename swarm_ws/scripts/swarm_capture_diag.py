#!/usr/bin/env python3
"""Read-only system/ROS/Gazebo sampler and diagnostic ZIP exporter (stdlib only).
Usage: python3 swarm_capture_diag.py record --label recording -- [ros2 launch ...]
       python3 swarm_capture_diag.py record --label recording
       python3 swarm_capture_diag.py mark --session /path --event recording_started
       python3 swarm_capture_diag.py stop --session /path
       python3 swarm_capture_diag.py pack --session /path
"""
import argparse, csv, datetime as dt, json, os, pathlib, re, shutil, signal, subprocess, sys, threading, time, uuid, zipfile

PROCESS_WORDS = ('gz sim','gz-server','gz-gui','gz sim server','ruby','ros2 launch','path_follower','stage_coordinator','run_prepared_mission','run_auto_experiment','run_unique_mission','gnome-shell','gnome-screencast','wf-recorder','obs','ffmpeg','pipewire','wireplumber','Xwayland','nvidia','intel_gpu_top','amdgpu')
FIELDS = ['wall_iso','elapsed_wall_s','clock_sim_s','clock_rtf_approx','cpu_total_pct','mem_available_mb','mem_used_pct','swap_used_mb','load1','process_count','gz_pid','gz_cpu_pct','gz_rss_mb','gz_status','gz_threads','gz_io_write_mb','recorder_processes','recorder_cpu_pct','recorder_rss_mb','gnome_shell_cpu_pct','ros_cpu_pct','ros_rss_mb','gpu_util_pct','gpu_mem_mb','gpu_temp_c','gpu_power_w','gpu_busy_pct_sysfs','cpu_pressure_some_avg10','memory_pressure_some_avg10','io_pressure_some_avg10','cgroup_oom_count']

def run(cmd, timeout=3):
    try:
        p=subprocess.run(cmd, stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=timeout,check=False)
        return {'exit':p.returncode,'out':p.stdout[-120000:],'err':p.stderr[-3000:]}
    except Exception as exc:return {'error':str(exc)}

def read(path, limit=300000):
    try:
        with open(path,'rb') as f:return f.read(limit).decode('utf-8',errors='replace')
    except Exception:return ''

def proc_snapshot():
    out={}
    for p in pathlib.Path('/proc').iterdir():
        if not p.name.isdigit():continue
        pid=int(p.name)
        try:
            raw=(p/'stat').read_text(); a=raw.rfind(')'); name=raw[raw.find('(')+1:a]; s=raw[a+2:].split()
            cmd=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode('utf-8','replace').strip()
            # Avoid arguments with potential secrets in output: store only process type / comm.
            if not any(w in (name+' '+cmd).lower() for w in PROCESS_WORDS):continue
            statm=(p/'statm').read_text().split(); rss=int(statm[1])*os.sysconf('SC_PAGE_SIZE')/1048576
            out[pid]={'pid':pid,'name':name,'kind':kind(name,cmd),'ticks':int(s[11])+int(s[12]),'rss_mb':round(rss,1),'threads':int(s[17]),'state':s[0],'ppid':int(s[1]),'io_write_mb':get_write_bytes(p)}
        except (FileNotFoundError,ProcessLookupError,PermissionError,ValueError,IndexError,OSError):pass
    return out

def kind(name, cmd):
    q=(name+' '+cmd).lower()
    if 'gz sim' in q or 'gz-server' in q or 'gz sim server' in q or 'gz sim -' in q:return 'gazebo'
    if any(x in q for x in ('gnome-screencast','wf-recorder','obs-studio','obs --','simplescreenrecorder','kooha','gpu-screen-recorder','ffmpeg')):return 'recorder'
    if 'gnome-shell' in q:return 'gnome-shell'
    if any(x in q for x in ('path_follower','stage_coordinator','run_prepared_mission','run_auto_experiment','run_unique_mission','ros2 launch')):return 'ros'
    return 'other'

def get_write_bytes(p):
    try:
        t=(p/'io').read_text(); m=re.search(r'^write_bytes:\s*(\d+)',t,re.M);return round(int(m.group(1))/1048576,1) if m else ''
    except Exception:return ''

def cpu_stat():
    t=read('/proc/stat',2000).splitlines()[0].split()[1:];n=list(map(int,t)); return sum(n),n[3]+(n[4] if len(n)>4 else 0)

def mem_stats():
    m={}
    for line in read('/proc/meminfo',8000).splitlines():
        if ':' in line:
            k,v=line.split(':',1)
            try:m[k]=int(v.strip().split()[0])/1024
            except Exception:pass
    return m

def pressure(kind):
    m=re.search(r'some\s+avg10=([0-9.]+)',read(f'/proc/pressure/{kind}',1000))
    return float(m.group(1)) if m else ''

def oom_count():
    raw=read('/sys/fs/cgroup/memory.events',3000)
    m=re.search(r'^oom_kill\s+(\d+)',raw,re.M)
    return int(m.group(1)) if m else ''

def gpu_stats():
    d={}; gpu=run(['nvidia-smi','--query-gpu=utilization.gpu,memory.used,temperature.gpu,power.draw','--format=csv,noheader,nounits'],timeout=1.5) if shutil.which('nvidia-smi') else {}
    if gpu.get('exit')==0 and gpu.get('out'):
        try:
            x=gpu['out'].strip().splitlines()[0].split(',');d.update(zip(['gpu_util_pct','gpu_mem_mb','gpu_temp_c','gpu_power_w'],[float(v.strip()) if v.strip() not in ('[Not Supported]','N/A') else '' for v in x]))
        except Exception:pass
    for p in pathlib.Path('/sys/class/drm').glob('card*/device/gpu_busy_percent'):
        try:d['gpu_busy_pct_sysfs']=float(p.read_text().strip());break
        except Exception:pass
    return d

class Clock:
    def __init__(self, log):self.value=None;self.updated=None;self.enabled=False;self.log=log;self.node=None;self.thread=None
    def start(self):
        try:
            import rclpy
            from rosgraph_msgs.msg import Clock as ClockMsg
            self.rclpy=rclpy
            rclpy.init(args=None)
            self.node=rclpy.create_node('swarm_capture_diag_clock_'+uuid.uuid4().hex[:6])
            self.node.create_subscription(ClockMsg,'/clock',self.cb,10)
            self.enabled=True
            def spin():
                try:rclpy.spin(self.node)
                except Exception as e:self.log.write('clock_spin: '+str(e)+'\n');self.log.flush()
            self.thread=threading.Thread(target=spin,daemon=True);self.thread.start()
        except Exception as e:
            self.log.write('clock not available: '+str(e)+'; run monitor in ROS-sourced terminal for /clock.\n');self.log.flush()
    def cb(self,msg):self.value=msg.clock.sec+msg.clock.nanosec*1e-9;self.updated=time.monotonic()
    def close(self):
        if self.enabled:
            try:self.rclpy.shutdown()
            except Exception:pass

def add_event(folder,event,details=''):
    with (folder/'events.tsv').open('a') as f:f.write(f'{dt.datetime.now().astimezone().isoformat()}\t{time.monotonic():.4f}\t{event}\t{details}\n')

def copy_small(src,dst,limit):
    if src.exists() and src.is_file():
        try:
            size=src.stat().st_size
            with src.open('rb') as f:
                if size>limit:f.seek(-limit,2)
                data=f.read(limit)
            dst.parent.mkdir(parents=True,exist_ok=True)
            dst.write_bytes((f'--- LAST {len(data)} BYTES OF {size} ---\n'.encode() if size>limit else b'')+data)
        except Exception:pass

def diagnostics(folder, start_wall, experiment_root=None):
    meta={}
    for x,cmd in {'uname':['uname','-a'],'os_release':['cat','/etc/os-release'],'ros_distro':['sh','-c','printf "%s" "$ROS_DISTRO"'],'nvidia':['nvidia-smi','-L'],'gpu_pci':['sh','-c','lspci | grep -Ei "VGA|3D|Display"'],'gnome':['gnome-shell','--version'],'docker':['sh','-c','test -f /.dockerenv && echo in_container || echo host'],'journal_kernel':['journalctl','-k','--since',dt.datetime.fromtimestamp(start_wall).strftime('%Y-%m-%d %H:%M:%S'),'--no-pager','-n','300']}.items():
        if cmd[0] not in ('cat','sh') and not shutil.which(cmd[0]):continue
        meta[x]=run(cmd,8)
    (folder/'environment.json').write_text(json.dumps(meta,indent=2))
    r=meta.get('journal_kernel',{});(folder/'kernel_last.txt').write_text(r.get('out','')+'\n'+r.get('err',''))
    # Include recent ROS launch logs (bounded) and run reports (not huge odom data)
    if experiment_root:
        exp=pathlib.Path(experiment_root).expanduser()
        if exp.exists():
            candidates=sorted((p for p in exp.iterdir() if p.is_dir() and p.stat().st_mtime>=start_wall-120),key=lambda p:p.stat().st_mtime,reverse=True)[:3]
            for d in candidates:
                for pattern in ('*.json','*.log','events.csv','recorder.log','summary.json','automation_report.json'):
                    for file in d.glob(pattern):
                        copy_small(file,folder/'experiments'/d.name/file.name,700000)
                # Odom is potentially large; include when within a 15 MiB ceiling.
                odom=d/'odom.csv'
                if odom.exists() and odom.stat().st_size<=15*1024*1024:
                    copy_small(odom,folder/'experiments'/d.name/'odom.csv',15*1024*1024)
    gzdir=pathlib.Path.home()/'.gz'/'sim'/'log'
    if gzdir.exists():
        recentgz=sorted((p for p in gzdir.rglob('*') if p.is_file() and p.stat().st_mtime>=start_wall-120 and p.suffix in ('.log','.txt')),key=lambda p:p.stat().st_mtime,reverse=True)[:16]
        for f in recentgz:copy_small(f,folder/'gazebo_logs'/f.relative_to(gzdir),500000)
    rosdir=pathlib.Path.home()/'.ros'/'log';dest=folder/'ros_logs'
    if rosdir.exists():
        recent=sorted((p for p in rosdir.iterdir() if p.is_dir() and p.stat().st_mtime>=start_wall-90),key=lambda p:p.stat().st_mtime,reverse=True)[:4]
        for d in recent:
            for f in d.rglob('*'):
                if f.is_file() and f.suffix in ('.log','.txt') and len(f.relative_to(d).parts)<4:
                    copy_small(f,dest/d.name/f.relative_to(d),500000)

def zip_folder(folder):
    dest=folder.with_suffix('.zip')
    with zipfile.ZipFile(dest,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for file in folder.rglob('*'):
            if file.is_file():z.write(file,arcname=str(file.relative_to(folder)))
    return dest

def record(a):
    root=pathlib.Path(a.output_root).expanduser().resolve();root.mkdir(parents=True,exist_ok=True)
    folder=root/(dt.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+re.sub(r'[^A-Za-z0-9_-]','_',a.label)[:50]+'_'+uuid.uuid4().hex[:6]);folder.mkdir()
    start_wall=time.time();begin=time.monotonic();(folder/'meta.json').write_text(json.dumps({'start_iso':dt.datetime.now().astimezone().isoformat(),'label':a.label,'interval_s':a.interval,'mode':'wrapper' if a.command else 'monitor','command_program':a.command[0] if a.command else None,'container':pathlib.Path('/.dockerenv').exists()},indent=2))
    print('DIAG_SESSION:',folder,flush=True)
    print('MARK:',f'python3 {pathlib.Path(__file__).resolve()} mark --session {folder} --event recording_started',flush=True)
    print('STOP:',f'python3 {pathlib.Path(__file__).resolve()} stop --session {folder}',flush=True)
    add_event(folder,'monitor_started')
    child=None; logfile=None
    if a.command:
        cmd=a.command[1:] if a.command[0]=='--' else a.command
        logfile=(folder/'launch_output.log').open('w',buffering=1)
        child=subprocess.Popen(cmd,stdout=logfile,stderr=subprocess.STDOUT,start_new_session=True)
        add_event(folder,'launch_started',f'pid={child.pid}')
    note=(folder/'monitor_notes.txt').open('w',buffering=1)
    clock=Clock(note);clock.start()
    stop=[False]
    def signal_stop(sig,frame):
        stop[0]=True
        if child and child.poll() is None:
            try:os.killpg(child.pid,signal.SIGINT)
            except ProcessLookupError:pass
    signal.signal(signal.SIGINT,signal_stop);signal.signal(signal.SIGTERM,signal_stop)
    previous=proc_snapshot();total,idle=cpu_stat();oldclock=None;oldmonotonic=None
    with (folder/'samples.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=FIELDS);w.writeheader()
        while not stop[0]:
            now=time.monotonic();p=proc_snapshot();t,i=cpu_stat();delta=max(now-begin,0.1)
            cpu=round(100*(1-(i-idle)/max(1,t-total)),1);total,idle=t,i
            mem=mem_stats();dticks=now-getattr(record,'last',now);record.last=now
            ticks_per_sec=os.sysconf('SC_CLK_TCK'); cpus=os.cpu_count() or 1
            for pid,v in p.items():
                pv=previous.get(pid)
                v['cpu_pct']=round((v['ticks']-pv['ticks'])/ticks_per_sec/max(0.01,dticks)*100,1) if pv else 0.
            previous=p
            bykind=lambda k:[v for v in p.values() if v['kind']==k]
            gz=bykind('gazebo');rec=bykind('recorder');ros=bykind('ros');shell=bykind('gnome-shell')
            get=lambda seq,k:sum(float(v.get(k) or 0) for v in seq)
            sim=clock.value; rtf=''
            if sim is not None:
                if oldclock is not None and sim>=oldclock and oldmonotonic is not None:rtf=round((sim-oldclock)/max(.01,now-oldmonotonic),3)
                oldclock,oldmonotonic=sim,now
            row=dict(wall_iso=dt.datetime.now().astimezone().isoformat(),elapsed_wall_s=round(now-begin,3),clock_sim_s=round(sim,3) if sim is not None else '',clock_rtf_approx=rtf,cpu_total_pct=cpu,mem_available_mb=round(mem.get('MemAvailable',0),1),mem_used_pct=round(100*(1-mem.get('MemAvailable',0)/max(1,mem.get('MemTotal',1))),1),swap_used_mb=round(mem.get('SwapTotal',0)-mem.get('SwapFree',0),1),load1=round(os.getloadavg()[0],2),process_count=len(p),gz_pid=';'.join(str(v['pid']) for v in gz),gz_cpu_pct=round(get(gz,'cpu_pct'),1),gz_rss_mb=round(get(gz,'rss_mb'),1),gz_status=';'.join(v['state'] for v in gz),gz_threads=int(get(gz,'threads')),gz_io_write_mb=round(get(gz,'io_write_mb'),1),recorder_processes=';'.join(v['name'] for v in rec),recorder_cpu_pct=round(get(rec,'cpu_pct'),1),recorder_rss_mb=round(get(rec,'rss_mb'),1),gnome_shell_cpu_pct=round(get(shell,'cpu_pct'),1),ros_cpu_pct=round(get(ros,'cpu_pct'),1),ros_rss_mb=round(get(ros,'rss_mb'),1))
            row.update(gpu_stats());row.update(cpu_pressure_some_avg10=pressure('cpu'),memory_pressure_some_avg10=pressure('memory'),io_pressure_some_avg10=pressure('io'),cgroup_oom_count=oom_count());w.writerow({field: row.get(field, '') for field in FIELDS});f.flush()
            if (folder/'STOP').exists():add_event(folder,'stop_file_seen');break
            if child and child.poll() is not None:
                add_event(folder,'launch_exit',str(child.returncode));break
            if a.duration and delta>=a.duration:add_event(folder,'duration_elapsed');break
            time.sleep(a.interval)
    if child and child.poll() is None:
        try:os.killpg(child.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:child.wait(timeout=12)
        except subprocess.TimeoutExpired:add_event(folder,'launch_still_running_after_SIGINT')
    if child:add_event(folder,'launch_returncode',str(child.poll()))
    clock.close();note.close()
    if logfile:logfile.close()
    diagnostics(folder,start_wall,a.experiment_root)
    add_event(folder,'monitor_finished')
    zipped=zip_folder(folder)
    print('DIAG_DIRECTORY:',folder,flush=True)
    print('DIAG_ZIP:',zipped,flush=True)
    if child and child.poll() not in (None,0):sys.exit(1)

def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='action',required=True)
    r=sub.add_parser('record');r.add_argument('--label',default='test');r.add_argument('--output-root',default='~/swarm_diagnostics');r.add_argument('--interval',type=float,default=1.0);r.add_argument('--duration',type=float,default=0);r.add_argument('--experiment-root',default='~/swarm_ws/scripts/resultados/experiments');r.add_argument('command',nargs=argparse.REMAINDER)
    m=sub.add_parser('mark');m.add_argument('--session',required=True);m.add_argument('--event',required=True);m.add_argument('--detail',default='')
    s=sub.add_parser('stop');s.add_argument('--session',required=True)
    z=sub.add_parser('pack');z.add_argument('--session',required=True)
    a=p.parse_args()
    if a.action=='record':
        if a.interval<.2:p.error('--interval must be >= 0.2')
        record(a)
    elif a.action=='mark':add_event(pathlib.Path(a.session),a.event,a.detail);print('EVENT_RECORDED')
    elif a.action=='stop':(pathlib.Path(a.session)/'STOP').touch();print('STOP_REQUESTED')
    else:print('DIAG_ZIP:',zip_folder(pathlib.Path(a.session)))
if __name__=='__main__':main()
