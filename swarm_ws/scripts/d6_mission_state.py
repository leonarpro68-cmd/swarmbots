#!/usr/bin/env python3
"""D6 operator-gated mission ledger. Never infers physical pickup/delivery from path_follower done."""
import argparse
import hashlib
import json
from pathlib import Path
from datetime import datetime, timezone

STAGES=('pickup1','delivery1','pickup2','delivery2')

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def write(path,state):
    path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix('.tmp')
    tmp.write_text(json.dumps(state,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    tmp.replace(path)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--state',required=True,type=Path)
    sub=p.add_subparsers(dest='action',required=True)
    init=sub.add_parser('init');init.add_argument('--scenario',required=True,type=Path)
    confirm=sub.add_parser('confirm');confirm.add_argument('--stage',choices=STAGES,required=True)
    confirm.add_argument('--report',required=True,type=Path)
    confirm.add_argument('--controls',required=True,type=Path)
    confirm.add_argument('--confirm-physical',action='store_true',help='Operador verificó éxito físico y finalización de 3 robots')
    show=sub.add_parser('show')
    a=p.parse_args()
    if a.action=='init':
        if a.state.exists():p.error('Estado ya existe; no se sobrescribe una misión')
        if not a.scenario.is_file():p.error('Escenario no existe')
        state={'version':'swarmbots.d6.v1','scenario':str(a.scenario.resolve()),'scenario_sha256':sha(a.scenario),
               'completed':[],'collected':[],'delivered':[],'round1_names':[],'round2_names':[],'events':[]}
        write(a.state,state)
        print('D6 INITIALIZED; next=pickup1; ningún movimiento autorizado')
        return
    state=json.loads(a.state.read_text(encoding='utf-8'))
    if sha(state['scenario'])!=state['scenario_sha256']:p.error('Escenario cambiado desde inicio de misión')
    if a.action=='show':
        print(json.dumps(state,indent=2,ensure_ascii=False));return
    if not a.confirm_physical:p.error('Se requiere --confirm-physical tras verificar en Gazebo las tres acciones físicas')
    completed=state['completed']
    if len(completed)>=4 or a.stage!=STAGES[len(completed)]:p.error('Orden incorrecto de etapa')
    report=json.loads(a.report.read_text(encoding='utf-8'))['stages'][a.stage]
    meta=report['hybrid_v2']
    if not(report.get('certified') and meta.get('certified') and meta.get('status')=='CERTIFIED'):
        p.error('Etapa no certificada')
    if str(Path(meta['scenario_json']).resolve())!=state['scenario']:
        p.error('Escenario de reporte distinto')
    if not a.controls.is_file():p.error('NPZ no encontrado')
    if a.stage.startswith('pickup'):
        names=report.get('assigned_names',[])
        if len(names)!=3 or len(set(names))!=3 or any(name in state['collected'] for name in names):
            p.error('Nombres recogidos inválidos/repetidos; requiere reporte del planificador D6')
        state['collected'].extend(names)
        state['round1_names' if a.stage=='pickup1' else 'round2_names']=names
    else:
        names=report.get('delivered_names',[])
        expected=state['round1_names' if a.stage=='delivery1' else 'round2_names']
        if names!=expected:p.error('Entregas no coinciden con los objetos cargados')
        state['delivered'].extend(names)
    state['completed'].append(a.stage)
    state['events'].append({'stage':a.stage,'utc':datetime.now(timezone.utc).isoformat(),
                            'report_sha256':sha(a.report),'controls_sha256':sha(a.controls),
                            'operator_physical_confirmation':True,'names':names})
    write(a.state,state)
    print('CONFIRMED',a.stage,'nombres:',names,'siguiente:',STAGES[len(state['completed'])] if len(state['completed'])<4 else 'MISSION COMPLETE')

if __name__=='__main__':main()
