#!/usr/bin/env python3
"""Regenerate verified AMR-BIP BT figures and summary from a ROS2 status log.

Usage:
  python3 tools/plot_bt_evidence.py \
      --input results/2026-10-18_bt_gazebo/bt_status.jsonl \
      --out results/2026-10-18_bt_gazebo/figures

Supports a pure JSONL file or `ros2 topic echo /executive/status --field data`
terminal capture. Does not extrapolate results beyond the captured timestamps.
"""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import numpy as np

STATES=['MISSION','CHARGE_IDLE','CHARGE_NAVIGATING','CHARGE_REQUESTING','CHARGE_CHARGING']
COLOR={'MISSION':'#2464a3','CHARGE_IDLE':'#dc9b34','CHARGE_NAVIGATING':'#d27c37','CHARGE_REQUESTING':'#bd6250','CHARGE_CHARGING':'#209b85'}

def load_rows(path):
    data=[]
    for line in Path(path).read_text(errors='replace').splitlines():
        if not line.lstrip().startswith('{'): continue
        try: r=json.loads(line.strip())
        except json.JSONDecodeError: continue
        if r.get('engine')=='bt' and isinstance(r.get('t'),(float,int)) and 'battery' in r:
            data.append(r)
    data.sort(key=lambda r:r['t'])
    if not data: raise ValueError('No parsed BT status records found')
    return data

def make_plots(rows,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    times=np.array([r['t']-rows[0]['t'] for r in rows]); batt=np.array([r['battery'] for r in rows]); states=[r['state'] for r in rows]
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.titlesize':15,'axes.labelsize':11,'axes.edgecolor':'#bdc8d3','axes.spines.top':False,'axes.spines.right':False,'grid.alpha':0.18,'figure.facecolor':'white','savefig.facecolor':'white'})

    fig,ax=plt.subplots(figsize=(11.2,4.4))
    ax.plot(times,batt,lw=2.2,color='#17659f',label='Battery (simulated)')
    ax.axhline(25,color='#d47c34',ls='--',lw=1.3,label='Charge trigger (~25%)')
    ax.fill_between(times,batt,0,color='#2c82b2',alpha=.09)
    ax.set(xlabel='Seconds from first logged status',ylabel='Battery (%)',title='Battery-aware preemption and autonomous recharge')
    ax.set_xlim(times[0],times[-1]);ax.set_ylim(0,100);ax.grid(True,axis='y')
    for label,i in [('Dock / charge request',next((i for i,r in enumerate(rows) if r['state']=='CHARGE_REQUESTING'),None)),('Mission resumed',next((i for i,r in enumerate(rows) if i>0 and r['state']=='MISSION' and rows[i-1]['state']!='MISSION'),None))]:
        if i is not None:
            ax.axvline(times[i],ls=':',lw=1,color='#8c98a6')
            ax.annotate(f'{label}\n{batt[i]:.1f}%',(times[i],batt[i]),xytext=(8,15 if 'Dock' in label else -38),textcoords='offset points',fontsize=8,color='#303c4b',arrowprops=dict(arrowstyle='-',color='#64748b'))
    ax.legend(loc='upper center',ncol=2,frameon=False)
    fig.tight_layout();fig.savefig(out/'battery_profile.png',dpi=190,bbox_inches='tight');plt.close(fig)

    intervals=[]; prev=0
    for i in range(1,len(rows)):
        if rows[i]['state']!=rows[i-1]['state']:
            intervals.append((times[prev],times[i],rows[i-1]['state']));prev=i
    intervals.append((times[prev],times[-1],rows[-1]['state']))
    fig,ax=plt.subplots(figsize=(11.2,3.35))
    for a,b,s in intervals:
        ax.broken_barh([(a,max(.02,b-a))],(.7,.6),facecolors=COLOR.get(s,'#777'))
    ax.set(xlim=(times[0],times[-1]),ylim=(0,2),yticks=[1],yticklabels=['Behavior Tree'],xlabel='Seconds from first logged status',title='Executive state timeline: one observed BT trace')
    ax.grid(True,axis='x')
    from matplotlib.patches import Patch
    used=[s for s in STATES if s in states]
    fig.subplots_adjust(bottom=.32,top=.82)
    fig.legend([Patch(color=COLOR[s]) for s in used],[s.replace('_',' ') for s in used],loc='lower center',bbox_to_anchor=(.5,.03),ncol=3,frameon=False,fontsize=8)
    fig.savefig(out/'executive_state_timeline.png',dpi=190,bbox_inches='tight');plt.close(fig)

    fig,ax=plt.subplots(figsize=(11.2,3.5))
    steps=[r['step'] for r in rows]
    ax.step(times,steps,where='post',lw=2,color='#3075aa')
    ax.set(xlim=(times[0],times[-1]),ylim=(-.2,max(steps)+.4),yticks=sorted(set(steps)),xlabel='Seconds from first logged status',ylabel='Delivery mission step',title='Interrupted delivery resumes at the same step')
    nav_a=next((times[i] for i,r in enumerate(rows) if r['state']=='CHARGE_NAVIGATING'),None)
    resume=next((times[i] for i,r in enumerate(rows) if i>0 and r['state']=='MISSION' and rows[i-1]['state']!='MISSION'),None)
    if nav_a is not None and resume is not None:
        ax.axvspan(nav_a,resume,color='#df9d41',alpha=.16,label='Charging interruption')
    ax.grid(True,axis='y');ax.legend(loc='lower right',frameon=False)
    fig.tight_layout();fig.savefig(out/'mission_resume.png',dpi=190,bbox_inches='tight');plt.close(fig)
    return intervals

def main():
    p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--out',required=True)
    args=p.parse_args();rows=load_rows(args.input);make_plots(rows,args.out)
    print(f'Generated plots from {len(rows)} BT records, duration {rows[-1]["t"]-rows[0]["t"]:.2f} s')
    print('Last state:',rows[-1]['state'],'completed:',rows[-1]['completed'],'failed:',rows[-1]['failed'])
if __name__=='__main__':main()
