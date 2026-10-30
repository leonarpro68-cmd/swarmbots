#!/usr/bin/env python3
"""D9.7D: read-only audit of D9.7C.1 follower CSV and JSON.

CSV fields are running peak errors, NOT instantaneous error. No actual speed,
acceleration or measured finish time is derivable from this CSV.
"""
import argparse, csv, json, math, statistics
from pathlib import Path

ROBOTS=('summit0','summit1','summit2')
COLUMNS=('t_s','peak_err_0_m','peak_err_1_m','peak_err_2_m','separation_m')

def load_csv(path):
    with path.open(newline='') as handle:
        reader=csv.DictReader(handle)
        if not set(COLUMNS).issubset(set(reader.fieldnames or [])):
            raise ValueError('CSV_COLUMNS_MISSING')
        rows=[]
        for n,row in enumerate(reader,2):
            try:
                r={k:float(row[k]) for k in COLUMNS}
            except (ValueError,TypeError) as exc:
                raise ValueError(f'INVALID_CSV_ROW_{n}') from exc
            if not all(math.isfinite(x) for x in r.values()):
                raise ValueError(f'NONFINITE_CSV_ROW_{n}')
            rows.append(r)
    if len(rows)<2:
        raise ValueError('INSUFFICIENT_SAMPLES')
    for a,b in zip(rows,rows[1:]):
        # Several callbacks can observe the same simulation-time stamp.
        # Equal stamps are valid; only a real rewind is rejected.
        if b['t_s']<a['t_s']-1e-9:
            raise ValueError('TIME_WENT_BACKWARDS')
        for i in range(3):
            k=f'peak_err_{i}_m'
            if b[k]+1e-7<a[k]:
                raise ValueError('RUNNING_PEAK_DECREASED')
    if any(r['separation_m']<0 for r in rows):
        raise ValueError('NEGATIVE_SEPARATION')
    return rows

def evaluate(follower, relay, rows):
    issues=[]
    if follower.get('status')!='FINISHED_TRAJECTORY_NOT_MISSION': issues.append('FOLLOWER_NOT_COMPLETED')
    if follower.get('faults'): issues.append('FOLLOWER_FAULTS')
    if not follower.get('final_stop_confirmed'): issues.append('STOP_NOT_CONFIRMED')
    if not follower.get('end_signal_sent'): issues.append('END_SIGNAL_MISSING')
    if relay is None: issues.append('RELAY_REPORT_MISSING')
    else:
        if relay.get('status')!='NORMAL_END_STOP_CONFIRMED' or relay.get('faults') or not relay.get('normal_end'):
            issues.append('RELAY_NOT_NORMAL_END')
    peak={name:rows[-1][f'peak_err_{i}_m'] for i,name in enumerate(ROBOTS)}
    reported=follower.get('peak_tracking_error_m') or {}
    for name in ROBOTS:
        if name not in reported or abs(float(reported[name])-peak[name])>2e-6:
            issues.append('PEAK_MISMATCH_'+name)
    min_sep=min(r['separation_m'] for r in rows)
    reported_sep=follower.get('min_measured_separation_m')
    if reported_sep is None or abs(min_sep-float(reported_sep))>2e-6:
        issues.append('SEPARATION_MISMATCH')
    if min_sep<1.15: issues.append('SEPARATION_BELOW_1_15M')
    times=[r['t_s'] for r in rows]
    intervals=[b-a for a,b in zip(times,times[1:]) if b-a>1e-9]
    duplicate_stamps=sum(1 for a,b in zip(times,times[1:]) if abs(b-a)<=1e-9)
    if not intervals:
        raise ValueError('NO_TIME_PROGRESS')
    horizon=float(follower.get('horizon_s',0))
    if times[0]<-0.05 or times[-1]>horizon+.10:issues.append('TIME_OUTSIDE_PLAN_HORIZON')
    final_speed=follower.get('final_observed_speed_m_s') or {}
    if any(name not in final_speed or final_speed[name] is None or float(final_speed[name])>=.03 for name in ROBOTS):
        issues.append('FINAL_SPEED_NOT_CONFIRMED')
    return {
        'schema':'swarm.d97d.readonly_audit.v1',
        'status':'PASS_RECORDED_FOLLOWER_AUDIT' if not issues else 'REVIEW_REQUIRED',
        'issues':issues,'sample_count':len(rows),
        'duplicate_timestamp_pairs':duplicate_stamps,
        'unique_timestamps':len(rows)-duplicate_stamps,
        'recorded_reference_time_start_s':times[0],
        'recorded_reference_time_end_s':times[-1],
        'recorded_reference_span_s':times[-1]-times[0],
        'planned_horizon_s':horizon,
        'sample_step_median_s':statistics.median(intervals),
        'sample_step_max_s':max(intervals),
        'peak_tracking_error_m':peak,
        'minimum_sampled_center_separation_m':min_sep,
        'final_observed_speed_m_s':final_speed,
        'relay_status':None if relay is None else relay.get('status'),
        'mission_pickup_confirmed':False,
        'execution_authorized':False,
        'measurement_scope':{
            'tracking_error':'Only running maxima are logged; instantaneous error/RMSE cannot be recovered.',
            'separation':'Minimum of recorded samples; not a continuous-time certified bound.',
            'elapsed_time':'CSV t_s is planned reference clock; measured real finish time is NOT available.',
            'speed_acceleration':'No timestamped velocity vectors or acceleration observations in this CSV; cannot certify actual dynamic limits.',
            'baseline':'D8 comparison not valid until same spatial plan and consistent elapsed-time definitions are provided.'
        }
    }

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--stage',default='pickup1')
    p.add_argument('--relay-report',type=Path)
    a=p.parse_args()
    follower_path=a.run/'d97c'/f'{a.stage}_d97c_report.json'
    follower=json.loads(follower_path.read_text())
    csv_path=a.run/'d97c'/f'{a.stage}_d97c_report.csv'
    rows=load_csv(csv_path)
    relay=json.loads(a.relay_report.read_text()) if a.relay_report else None
    result=evaluate(follower,relay,rows)
    result['source_csv']=str(csv_path)
    result['source_follower_report']=str(follower_path)
    result['source_relay_report']=str(a.relay_report) if a.relay_report else None
    target=a.run/'d97d'/f'{a.stage}_evaluation.json'
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(result,indent=2,ensure_ascii=False))
    print('OUTPUT:',target)

if __name__=='__main__':main()
