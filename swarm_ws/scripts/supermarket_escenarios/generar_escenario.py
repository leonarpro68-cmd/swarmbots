#!/usr/bin/env python3
"""Generador offline e independiente para supermarket. No modifica SwarmBots.

Uso: python3 generar_escenario.py --seed 100 --output-dir resultados/seed_100
Requiere matplotlib para la figura; la selección solo usa la biblioteca estándar.
"""
from __future__ import annotations
import argparse, json, math, random
from pathlib import Path
from collections import deque

ROOT=Path(__file__).resolve().parent
COLORS={'verde':'#17a34a','naranja':'#ea580c','azul':'#2563eb'}
RECT=(-13.75,13.75,-23.75,22.50)
LABELS=[('V1','verde'),('V2','verde'),('N1','naranja'),('N2','naranja'),('A1','azul'),('A2','azul')]

def point_rect_distance(x,y,o):
    dx=max(o['xmin']-x,0,x-o['xmax']);dy=max(o['ymin']-y,0,y-o['ymax'])
    return math.hypot(dx,dy)

def close_to_rect_segment(a,b,o,clearance):
    # Distancia mínima segmento-rectángulo; minimización convexa en parámetro del segmento
    ax,ay=a;bx,by=b
    def d2(t):
        x=ax+t*(bx-ax); y=ay+t*(by-ay)
        return point_rect_distance(x,y,o)**2
    lo,hi=0.,1.
    for _ in range(40):
        m1=lo+(hi-lo)/3;m2=hi-(hi-lo)/3
        if d2(m1)<d2(m2):hi=m2
        else:lo=m1
    return min(d2(0),d2(1),d2((lo+hi)/2)) >= (clearance-1e-8)**2

def load(graph_file=None, geometry_file=None):
    graph=json.loads(Path(graph_file or ROOT/'supermarket_grafo.json').read_text())
    geometry=json.loads(Path(geometry_file or ROOT/'supermarket_obstaculos.json').read_text())
    vertices={tuple((n['x'],n['y'])):n for n in graph['nodes']}
    adj={v:set() for v in vertices}
    for a,b in graph['edges']:
        a,b=tuple(a),tuple(b);adj[a].add(b);adj[b].add(a)
    # Usar la componente mayor, sin restringir el grado del nodo.
    unseen=set(vertices); components=[]
    while unseen:
        s=min(unseen);unseen.remove(s);seen={s};queue=deque([s])
        while queue:
            for q in sorted(adj[queue.popleft()] & unseen):
                unseen.remove(q);seen.add(q);queue.append(q)
        components.append(seen)
    component=max(components,key=len)
    return graph,geometry,vertices,adj,component

def choose(seed, min_sep=5., clearance=.75, attempts=3000,
           graph_file=None, geometry_file=None, world='supermarket'):
    graph,geometry,vertices,adj,component=load(graph_file,geometry_file)
    if world=='supermarket':
        spawn_rect=RECT  # Preserva los experimentos originales y sus semillas.
    else:
        # La zona de basuras se obtiene del grafo real, sin asumir dimensiones fijas.
        # Se usa un margen interior en los límites físicos del mapa.
        b=geometry['bounds']
        xs=[v[0] for v in vertices];ys=[v[1] for v in vertices]
        spawn_rect=(max(min(xs),b['xmin']+clearance),min(max(xs),b['xmax']-clearance),
                    max(min(ys),b['ymin']+clearance),min(max(ys),b['ymax']-clearance))
        if spawn_rect[0]>=spawn_rect[1] or spawn_rect[2]>=spawn_rect[3]:
            raise ValueError('Mapa demasiado estrecho para generar basuras')
    obstacles=geometry['obstacles'];valid_nodes=sorted(component)
    rng=random.Random(seed)
    if len(valid_nodes)<6:raise ValueError('No hay nodos suficientes')
    # Todos los candidatos son nodos aprobados del grafo, incluso en pasillos.
    for trial in range(1,attempts+1):
        chosen=[]
        pool=valid_nodes[:];rng.shuffle(pool)
        for p in pool:
            if all(math.dist(p,q)>=min_sep for q in chosen):
                chosen.append(p)
                if len(chosen)==6:break
        if len(chosen)<6:continue
        # Se generan basuritas en coordenadas CONTINUAS, no en nodos.
        trash=[]; used=chosen[:]
        for label,color in LABELS:
            found=None
            for _ in range(3500):
                p=(rng.uniform(spawn_rect[0],spawn_rect[1]),rng.uniform(spawn_rect[2],spawn_rect[3]))
                if not all(point_rect_distance(*p,o)>=clearance for o in obstacles):continue
                if not all(math.dist(p,q)>=min_sep for q in used):continue
                # Identificar un nodo de aproximación sin cruzar obstáculos ni su banda de 0,75 m.
                # No se exige que la basurita esté en el mismo nodo.
                nearest=sorted(valid_nodes,key=lambda q:(math.dist(p,q),q))
                anchor=None
                for q in nearest[:25]:
                    if all(close_to_rect_segment(p,q,o,clearance) for o in obstacles):
                        anchor=q;break
                if anchor is None:continue
                found=(p,anchor);break
            if found is None:break
            (p,anchor)=found
            used.append(p);trash.append({'name':f'trash_{label}_{color}','type':'trash','color':color,'xy_m':[round(p[0],5),round(p[1],5)],'approach_node_xy_m':list(anchor),'approach_grid':[vertices[anchor]['i'],vertices[anchor]['j']]})
        if len(trash)!=6:continue
        robots=[{'name':f'summit{i}','xy_m':list(p),'grid':[vertices[p]['i'],vertices[p]['j']]} for i,p in enumerate(chosen[:3])]
        deposits=[{'name':f'deposit_{c}','type':'deposit','color':c,'xy_m':list(p),'grid':[vertices[p]['i'],vertices[p]['j']]} for c,p in zip(COLORS,chosen[3:])]
        all_pos=[tuple(a['xy_m']) for a in robots+deposits+trash]
        assert all(math.dist(a,b)>=min_sep-1e-6 for i,a in enumerate(all_pos) for b in all_pos[i+1:])
        scenario={'version':'supermarket_offline_v1','world':world,'seed':seed,'attempt':trial,'units':'m',
        'trash_spawn_rectangle':{'xmin':spawn_rect[0],'xmax':spawn_rect[1],'ymin':spawn_rect[2],'ymax':spawn_rect[3]},
        'rules':{'min_pair_separation_m':min_sep,'trash_obstacle_clearance_m':clearance,'robots_deposits_on_graph':True,'trash_continuous_xy':True},
        'robots':robots,'deposits':deposits,'trash':trash,
        'validation':{'geometry_and_pairwise_separation':True,'connected_graph':len(component)==len(vertices),'trash_approach_lines_checked':True,'astar_cbs_verified':False,'physical_maneuvers_verified':False,'gazebo_spawn_verified':False}}
        return graph,geometry,scenario
    raise RuntimeError(f'No se obtuvo escenario válido tras {attempts} intentos')

def draw(graph,geometry,scenario,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from matplotlib.lines import Line2D
    fig,ax=plt.subplots(figsize=(9.5,14))
    # Obstáculos e indicación tenue de banda de seguridad (rectángulo ampliado conservador).
    for o in geometry['obstacles']:
        ax.add_patch(Rectangle((o['xmin']-.75,o['ymin']-.75),o['xmax']-o['xmin']+1.5,o['ymax']-o['ymin']+1.5,color='#f59e0b',alpha=.12,zorder=1))
        ax.add_patch(Rectangle((o['xmin'],o['ymin']),o['xmax']-o['xmin'],o['ymax']-o['ymin'],color='#6b7280',alpha=.95,zorder=3))
    for a,b in graph['edges']:
        ax.plot((a[0],b[0]),(a[1],b[1]),color='#cbd5e1',lw=.42,alpha=.48,zorder=1)
    ax.scatter([p['x'] for p in graph['nodes']],[p['y'] for p in graph['nodes']],s=5,color='#0f766e',alpha=.5,zorder=2)
    sr=scenario['trash_spawn_rectangle'];ax.add_patch(Rectangle((sr['xmin'],sr['ymin']),sr['xmax']-sr['xmin'],sr['ymax']-sr['ymin'],fill=False,edgecolor='#2563eb',ls='--',lw=1.7,zorder=4))
    for i,r in enumerate(scenario['robots']):
        x,y=r['xy_m'];ax.scatter([x],[y],s=180,c='#111827',marker='o',edgecolors='white',linewidths=1.1,zorder=8);ax.text(x,y,str(i),color='white',ha='center',va='center',weight='bold',fontsize=9,zorder=9)
    for dep in scenario['deposits']:
        x,y=dep['xy_m'];c=COLORS[dep['color']];ax.scatter([x],[y],s=150,c=c,marker='D',edgecolors='black',linewidths=.8,zorder=8);ax.annotate('D-'+dep['color'][0].upper(),(x,y),xytext=(5,7),textcoords='offset points',fontsize=9,weight='bold',color=c,zorder=9,bbox={'facecolor':'white','alpha':.78,'edgecolor':'none','pad':1})
    for t in scenario['trash']:
        x,y=t['xy_m'];c=COLORS[t['color']];ax.scatter([x],[y],s=125,c=c,marker='s',edgecolors='white',linewidths=1,zorder=8);ax.annotate(t['name'].split('_')[1],(x,y),xytext=(5,7),textcoords='offset points',fontsize=9,weight='bold',color=c,zorder=9,bbox={'facecolor':'white','alpha':.78,'edgecolor':'none','pad':1})
    b=geometry['bounds'];ax.set(xlim=(b['xmin']-1,b['xmax']+1),ylim=(b['ymin']-1,b['ymax']+1),xlabel='X (m)',ylabel='Y (m)');ax.set_aspect('equal');ax.grid(alpha=.14)
    ax.set_title(f'SwarmBots · {scenario["world"]} · semilla {scenario["seed"]}\n3 Summit · 6 basuras continuas · 3 depósitos · separación ≥ {scenario["rules"]["min_pair_separation_m"]:g} m',fontsize=12,weight='bold')
    ax.legend(handles=[Line2D([0],[0],marker='o',color='w',markerfacecolor='#111827',markersize=9,label='Robots (nodos)'),Line2D([0],[0],marker='D',color='w',markerfacecolor='#ea580c',markersize=9,label='Depósitos (nodos)'),Line2D([0],[0],marker='s',color='w',markerfacecolor='#16a34a',markersize=9,label='Basuras (continuas)'),Line2D([0],[0],color='#2563eb',ls='--',label='Rectángulo de basuras')],loc='lower center',bbox_to_anchor=(.5,-.11),ncol=2,frameon=False,fontsize=9)
    fig.savefig(path,dpi=170,bbox_inches='tight',facecolor='white');plt.close(fig)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seed',type=int,default=100)
    ap.add_argument('--output-dir',type=Path,default=Path('resultados/supermarket/seed_100'))
    ap.add_argument('--min-separation',type=float,default=5.)
    ap.add_argument('--trash-clearance',type=float,default=.75)
    ap.add_argument('--graph',type=Path,help='grafo.json del mapa seleccionado')
    ap.add_argument('--geometry',type=Path,help='obstaculos.json del mapa seleccionado')
    ap.add_argument('--world',default='supermarket')
    args=ap.parse_args()
    graph,geometry,scenario=choose(args.seed,args.min_separation,args.trash_clearance,
                                    graph_file=args.graph,geometry_file=args.geometry,world=args.world)
    args.output_dir.mkdir(parents=True,exist_ok=True)
    (args.output_dir/'scenario.json').write_text(json.dumps(scenario,ensure_ascii=False,indent=2)+'\n')
    (args.output_dir/'validacion.json').write_text(json.dumps(scenario['validation'],ensure_ascii=False,indent=2)+'\n')
    draw(graph,geometry,scenario,args.output_dir/'escenario.png')
    print('Escenario generado:',args.output_dir,'| intento:',scenario['attempt'])
    print('Nota: validación geométrica offline; no valida CBS, agarre ni Gazebo.')
if __name__=='__main__':main()
