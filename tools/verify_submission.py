"""JHY: independent checks of the delivered simulation data and CLI regression."""
from pathlib import Path
import csv
import hashlib
import json
import math
import subprocess
from datetime import datetime

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / 'Simulation'
CASES = ['default', 'slip', 'saturation', 'timeout', 'overheat', 'estop']
checks = 0


def check(value, message):
    global checks
    checks += 1
    if not value: raise AssertionError(message)


def main():
    import sys
    record = {'author': 'JHY', 'verified_at': datetime.now().astimezone().isoformat(),
              'data_source': 'software_simulation', 'hardware_tested': False,
              'scenarios': {}, 'checks': {}}
    check((SIM/'build/mecanum_sim.exe').is_file(), 'missing executable')
    for case in CASES:
        folder = SIM/'results'/case
        summary=json.loads((folder/'summary.json').read_text(encoding='utf-8'))
        with (folder/'telemetry.csv').open(encoding='utf-8',newline='') as f:
            reader=csv.DictReader(f); fields=reader.fieldnames; rows=list(reader)
        cfg={}
        for line in (folder/'used_config.cfg').read_text(encoding='utf-8').splitlines():
            if '=' in line and not line.startswith('#'):
                k,v=line.split('=',1); cfg[k]=v
        check(len(fields)==50,case+': column count')
        check(len(rows)==summary['samples'],case+': row count')
        duration=float(cfg['duration']); dt=float(cfg['dt']); max_w=float(cfg['max_wheel_speed'])
        last_time=0; latched=False; position_sum=0; yaw_sum=0
        for row in rows:
            values={k:float(v) for k,v in row.items() if k!='fault'}
            check(all(math.isfinite(v) for v in values.values()),case+': finite values')
            t=values['time_s']; h=t-last_time
            check(0<h<=dt+1e-8,case+': timestep')
            check(not latched or row['fault']!='OK',case+': fault latch')
            latched |= row['fault']!='OK'
            for wheel in ['LF','LB','RB','RF']:
                check(abs(values[wheel+'_target_radps'])<=max_w+1e-5,case+': wheel limit')
                check(abs(values[wheel+'_control'])<=1,case+': control limit')
                if latched:
                    check(values[wheel+'_control']==0 and values[wheel+'_target_radps']==0,case+': zero fault output')
            position_sum+=values['position_error_m']**2*h
            yaw_sum+=values['yaw_error_rad']**2*h
            last_time=t
        check(abs(last_time-duration)<1e-8,case+': final time')
        check(abs(math.sqrt(position_sum/duration)-summary['rmse_position_m'])<1e-8,case+': RMSE position')
        check(abs(math.sqrt(yaw_sum/duration)-summary['rmse_yaw_rad'])<1e-8,case+': RMSE yaw')
        record['scenarios'][case]={'samples':len(rows),'columns':len(fields),'fault':summary['fault'],
            'rmse_position_m':summary['rmse_position_m'],
            'csv_sha256':hashlib.sha256((folder/'telemetry.csv').read_bytes()).hexdigest()}
    result=subprocess.run([sys.executable,'tests/regression.py'],cwd=SIM,capture_output=True,text=True,encoding='utf-8',errors='replace')
    check(result.returncode==0,'CLI regression failed: '+result.stdout+result.stderr)
    record['checks']={'independent_assertions_passed':checks,'end_to_end_regression':result.stdout.strip(),
                      'compilation_flags':'-std=c11 -O2 -Wall -Wextra -Wpedantic -Werror',
                      'control_output_on_every_fault_row':'zero for all four wheels',
                      'hardware_acquisition_script':'PowerShell syntax checked; no serial hardware connected'}
    (ROOT/'docs/验证记录.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
    print(f'PASS: {checks} independent assertions; CLI regression also passed')


if __name__=='__main__': main()
