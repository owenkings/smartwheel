#!/usr/bin/env python3
"""Read-only native map/bag audit; optional bounded offline ROS publication.

No devices, bag replay, calibration update, or original-map writes. --output
must name a new project-contained JSON report. --publish only publishes actual
saved geometry on /wc_mapping/offline in ROS domain 84, after the audit passes.
It creates an adjacent, new RViz configuration for the two saved map displays.
All ROS imports are delayed until that explicitly requested publication step.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import sqlite3
import time

import numpy as np
import yaml


from wc_runtime.map_viewer import (
    MAX_FILE, MAX_POINTS, MAX_CELLS, SCALARS, require, contained, fingerprint,
    read_json, ply_header, ply_blocks, inspect_ply, pgm_token, read_pgm,
    inspect_grid, sqlite_read, node_signature, offline_rviz, publish_saved,
)


def audit(root, session):
    session=contained(root,session);export=contained(session,'export')
    request=read_json(session/'session.json');result=read_json(export/'result.json')
    identifier=request.get('session_id');mode=request.get('mode')
    require(isinstance(identifier,str) and re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}',identifier),'Invalid session ID')
    require(mode in ('left','right','all') and result.get('session_id')==identifier and result.get('mode')==mode,'Session/export identity mismatch')
    require(result.get('status')=='EXPORTED_EXPERIMENTAL_MAP','Export did not finish successfully')
    runtime=contained(root,Path('.phase1_runtime/sessions')/identifier/'mapping_app')
    manifest=read_json(runtime/'manifest.json')
    require(manifest.get('session_id')==identifier and manifest.get('role')=='mapping_app'
            and manifest.get('state')=='STOPPED' and manifest.get('exit_code')==0
            and not manifest.get('cleanup_errors'),'Session has not closed normally')
    health=read_json(session/'health/status.json')
    require(health.get('session_id')==identifier and not health.get('failure'),'Health/session mismatch or failure')
    database=contained(session,'slam/rtabmap.db')
    native,original_hash=sqlite_read(database,node_signature)
    require(original_hash['sha256']==result.get('database_sha256'),'Original native DB hash differs from export result')
    copied,_=sqlite_read(contained(export,'native_export_copy.db'),node_signature)
    require(native==copied,'Export copy is not the same native node/timestamp set')
    closure=(runtime/'process-1.log').read_text(errors='replace')
    require(any('Saving database/long-term memory...done!' in line and str(database) in line
                for line in closure.splitlines()),'Missing native normal DB closure log')
    files=result.get('files',{})
    require(isinstance(files,dict) and files,'No exported file inventory')
    for name,expected in files.items():
        require(fingerprint(contained(export,name))==expected,'Export file changed: '+name)
    clouds=[inspect_ply(contained(export,p)) for p in sorted(export.glob('*.ply'))]
    maps=[inspect_grid(contained(export,p))[0] for p in sorted(export.glob('*.yaml'))]
    require(clouds and maps,'Both real 3D and 2D export files are required')
    bag_dir=contained(session,'bag')
    metadata=yaml.safe_load((bag_dir/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    declared={row['topic_metadata']['name']:int(row['message_count']) for row in metadata['topics_with_message_count']}
    measured={};bag_files=[]
    def bag_counts(connection):
        return connection.execute('SELECT topics.name,COUNT(messages.id) FROM topics LEFT JOIN messages ON topics.id=messages.topic_id GROUP BY topics.id').fetchall()
    for name in metadata['relative_file_paths']:
        path=contained(bag_dir,name);rows,digest=sqlite_read(path,bag_counts)
        bag_files.append({'file':str(path),**digest})
        for topic,count in rows:measured[topic]=measured.get(topic,0)+count
    require(bag_files and measured==declared,'Bag SQL counts differ from closed metadata')
    sides=('left','right') if mode=='all' else (mode,)
    required=['/wc_mapping/imu/source_frame','/wc_mapping/wheel/feedback_raw']
    required+=['/wc_mapping/app/'+topic for topic in ('input_cloud','scan_cloud','prior_odom','odom','odom_info','cloud_map','grid_map')]
    required+=['/wc_mapping/lidar_'+side+'/'+topic for side in sides for topic in ('source_frame','source_frame_filtered')]
    require(all(measured.get(topic,0)>0 for topic in required),'Required hardware/map evidence has no bag messages')
    plan=read_json(runtime/'plan.json')
    require(set(measured)<=set(plan['bag_topics']),'Bag contains a topic outside session whitelist')
    export_log=(export/'export.log').read_text(errors='replace')
    return {'schema_version':1,'status':'PASS','validation_level':'REAL_FILES',
            'session_id':identifier,'mode':mode,'session_root':str(session),
            'native_database':{'file':str(database),**native,**original_hash},
            'closed_export_copy_same_node_set':True,'clouds':clouds,'maps':maps,
            'bag_files':bag_files,'bag_message_counts':measured,
            'stored_optimized_poses_loaded':'Loading optimized poses from database... done' in export_log,
            'stored_2d_map_loaded':'Loading optimized 2D occupancy grid from database... done!' in export_log,
            'bag_cdr_identity_validation':False,
            'scope':'File integrity, geometry availability and same-session evidence; not metric accuracy, floor correctness or route coverage',
            'navigation_validated':False,'geometry_accuracy_validated':False,
            'warning':None if any(m['occupied_cells'] for m in maps) else 'No occupied 2D cells; inspect scene classification'}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root',type=Path,default=Path('/home/nvidia/wheelchair'))
    parser.add_argument('--session-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True,help='New project-contained audit JSON')
    parser.add_argument('--publish',action='store_true',help='After successful audit, publish saved files only in domain 84')
    parser.add_argument('--duration',type=int,default=600)
    parser.add_argument('--max-view-points',type=int,default=1_000_000)
    args=parser.parse_args(argv)
    require(10<=args.duration<=3600 and 1000<=args.max_view_points<=2_000_000,'Invalid viewer bounds')
    root=args.project_root.absolute();session=contained(root,args.session_root);output=contained(root,args.output)
    require(output.parent.is_dir() and not output.exists(),'Output must be a new JSON in an existing directory')
    require(output.suffix=='.json','Output must have .json extension')
    rviz=output.with_suffix('.rviz');require(not rviz.exists(),'Offline RViz configuration already exists')
    try:
        report=audit(root,session)
        report['offline_view']={'rviz_config':str(rviz),'domain':84,'frame_id':'mapping_map',
            'command_note':'Run this script with --publish and a NEW report path after sourcing ROS; run rviz2 -d <that report.rviz> with ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1. No bag replay or device driver is needed.'}
    except Exception as error:
        report={'status':'FAIL','session_root':str(session),'reason':type(error).__name__+': '+str(error),
                'validation_level':'REAL_FILES','navigation_validated':False}
    with output.open('x',encoding='utf-8') as stream:
        json.dump(report,stream,indent=2,ensure_ascii=False,allow_nan=False);stream.write('\n')
    if report['status']=='PASS':
        with rviz.open('x',encoding='utf-8') as stream:yaml.safe_dump(offline_rviz(),stream,sort_keys=False)
    print(json.dumps({'status':report['status'],'report':str(output)},ensure_ascii=False),flush=True)
    if report['status']!='PASS':return 1
    if args.publish:publish_saved(report,args.duration,args.max_view_points)
    return 0


if __name__=='__main__':raise SystemExit(main())
