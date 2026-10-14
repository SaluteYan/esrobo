"""Offline left-hand zero-pose audit. No SDK, CAN, reset or motion interfaces.

Nominal mesh calculations below deliberately omit uncertainty to isolate model
and envelope effects. They are diagnostics, never operational certificates.
"""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import struct
import time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull
from esrobo_link.backends import ForwardModel
from esrobo_teleop.config import load_config
from esrobo_teleop.robot.hand_geometry import geometry_state
from esrobo_teleop.robot.linker_hand_driver import L10_PHYSICAL_JOINT_NAMES
from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard


def audit(root, fixture):
    cfg = load_config(str(root/'teleoperation/config/teleop_config.yaml'))
    cfg.ik.urdf_path = str(root/'teleoperation/urdf/esrobo_waist_with_head.urdf')
    g = TorsoCollisionGuard(ForwardModel(cfg.ik), 'left', cfg.robot.collision_package_dirs,
                            cfg.robot.torso_collision_margin_m, cfg.robot.shoulder_collision_margin_m)
    xml = ET.parse(cfg.ik.urdf_path).getroot()
    mimics = {j.get('name'):j.find('mimic').attrib for j in xml.findall('joint') if j.find('mimic') is not None}
    thumb = next(i for i,o in enumerate(g.geometry.geometryObjects) if o.name=='left_thumb_distal_0')
    waist = next(i for i,o in enumerate(g.geometry.geometryObjects) if o.name=='waist_link1_0')
    # Original STL convex hull, not repeated interval enclosures.
    mesh = xml.find("./link[@name='left_thumb_distal']/collision/geometry/mesh")
    filename = mesh.get('filename').removeprefix('package://')
    roots = [Path(p) for p in cfg.robot.collision_package_dirs] + [root/'teleoperation/assets']
    path = next(p/filename for p in roots if (p/filename).is_file())
    raw = path.read_bytes(); count = struct.unpack_from('<I',raw,80)[0]
    dtype=np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attr','<u2')])
    vertices=np.frombuffer(raw,dtype=dtype,count=count,offset=84)['vertices'].reshape(-1,3).astype(float)
    vertices *= np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
    # Guard construction moved the STL AABB centre into the URDF collision
    # origin. Its replacement Box is centred at zero; undo that shift for
    # the original STL hull to avoid counting the centre twice.
    center=(vertices.min(axis=0)+vertices.max(axis=0))/2
    hull=ConvexHull(vertices); cloud=g.coal.StdVec_Vec3s()
    for point in vertices[hull.vertices]-center:cloud.append(point)
    shape=g.coal.Convex.convexHull(cloud,True,'Qt')
    data=g.model.createData(); gd=g.pin.GeometryData(g.geometry)
    def nominal_mesh(arm, nominal):
        q=np.zeros(g.model.nq);q[g.indices[:7]]=arm
        values=dict(nominal)
        for name, spec in mimics.items():
            if spec['joint'] in values:
                values[name]=values[spec['joint']]*float(spec.get('multiplier',1))+float(spec.get('offset',0))
        for name,value in values.items():
            j=g.model.getJointId(name);qi=g.model.idx_qs[j]
            q[qi]=np.clip(value,g.model.lowerPositionLimit[qi],g.model.upperPositionLimit[qi])
        g.pin.forwardKinematics(g.model,data,q);g.pin.updateGeometryPlacements(g.model,data,g.geometry,gd)
        def transform(i):
            p=gd.oMg[i];c=g.shapes[i][1]
            return g.coal.Transform3s(p.rotation,p.translation+p.rotation@c)
        result=g.coal.DistanceResult()
        d=g.coal.distance(shape,transform(thumb),g.shapes[waist][0],transform(waist),g.distance_request,result)
        return float(d)
    variants={'recorded_hand':fixture['hand_raw'],
              'configured_open_feedback_HYPOTHETICAL':list(cfg.hand.left_open_feedback)}
    result=[]; mappings={}
    for variant,hand in variants.items():
        now=time.monotonic()
        state=geometry_state(hand,now,'left',cfg.hand.geometry_feedback_calibration,
                             L10_PHYSICAL_JOINT_NAMES,0.,cfg.hand.geometry_feedback_timeout_s,now=now)
        assert state.valid,state.reason
        nominal={n:(lo+hi)/2 for n,(lo,hi) in state.intervals.items()}
        mappings[variant]={n:float(np.degrees(v)) for n,v in nominal.items()}
        for label,arm in fixture['arm_poses'].items():
            full=np.zeros(14);full[:7]=arm
            exact=replace(state,intervals={n:(v,v) for n,v in nominal.items()})
            for kind,hs in [('nominal_with_production_rounding_NOT_safety',exact),('calibration_error_age_zero',state)]:
                g.hand_state_provider=lambda horizon,hs=hs:hs
                ok=g(full,full)
                result.append(dict(hand=variant,arm=label,kind=kind,accepted=ok,
                    **{k:g.last_diagnostics.get(k) for k in ('category','pair','clearance_m','required_m')}))
            result.append(dict(hand=variant,arm=label,kind='original_thumb_mesh_nominal_NOT_safety',
                               clearance_m=nominal_mesh(arm,nominal),required_m=cfg.robot.torso_collision_margin_m,
                               pair=['left_thumb_distal','waist_link1']))
    return dict(physical_motion=False,fixture=fixture,mapped_degrees=mappings,results=result,
                limitations=__doc__+' Configured open feedback is a hypothetical comparison, not a verified real hand pose.')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=Path.cwd());p.add_argument('--fixture',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();report=audit(a.root,json.loads(a.fixture.read_text()));a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
