"""Whole-parent pair proofs against full real-mesh subdivision; no hardware."""
import json
from pathlib import Path
import time
from unittest import mock
import numpy as np
import pytest
from esrobo_link.backends import ForwardModel
from esrobo_teleop.config import load_config
from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard
from esrobo_teleop.robot.hand_geometry import geometry_state
from esrobo_teleop.robot.linker_hand_driver import L10_PHYSICAL_JOINT_NAMES

ROOT=Path(__file__).resolve().parents[1]
FIXTURE=json.loads((ROOT/'tests/fixtures/entry_geometry_boxes_20261010.json').read_text())

@pytest.fixture(scope='module')
def geometry():
    cfg=load_config(str(ROOT/'config/teleop_config.yaml'))
    cfg.ik.urdf_path=str(ROOT/'urdf/esrobo_waist_with_head.urdf')
    g=TorsoCollisionGuard(ForwardModel(cfg.ik),'right',cfg.robot.collision_package_dirs,
                         cfg.robot.torso_collision_margin_m,cfg.robot.shoulder_collision_margin_m)
    def state():
        now=time.monotonic()
        return geometry_state(FIXTURE['hand_unit'],now-.01,'right',cfg.hand.geometry_feedback_calibration,
                              L10_PHYSICAL_JOINT_NAMES,0.,.1,now=now)
    g.hand_state_provider=lambda horizon:state()
    return g


@pytest.mark.parametrize('box',FIXTURE['boxes'],ids=lambda row:str(row['seq']))
def test_parent_proof_reuse_matches_full_pair_checking(geometry,box):
    g=geometry;full_clearance=g._clearance
    results=[]
    for reuse in (False,True):
        g.begin_entry_certificates()
        def clearance(*a,**kw):
            if not reuse:kw.pop('active_pairs',None)
            return full_clearance(*a,**kw)
        try:
            with mock.patch.object(g,'_clearance',side_effect=clearance), \
                 mock.patch.object(g.coal,'distance',wraps=g.coal.distance) as distance:
                accepted=g(np.array(box['lower']),np.array(box['upper']))
                results.append((accepted,distance.call_count,g.last_diagnostics['checked_boxes']))
        finally:g.end_entry_certificates()
    assert results[0][0] == results[1][0]
    assert results[0][0]
    assert results[1][1] <= results[0][1]
    assert results[0][2] == results[1][2]



@pytest.mark.parametrize('value',[float('nan'),float('inf')])
def test_nonfinite_pair_cannot_create_parent_certificate(geometry,value):
    g=geometry;box=FIXTURE['boxes'][0]
    g.begin_entry_certificates()
    try:
        with mock.patch.object(g.coal,'distance',return_value=value):
            assert not g(np.array(box['lower']),np.array(box['upper']))
        assert not g._entry_certificates
    finally:g.end_entry_certificates()


def test_reused_parent_proof_does_not_hide_child_collision(geometry):
    g=geometry
    # Large motion boxes and tighter gaps must produce the same refusal as
    # complete pair checking, even after harmless pairs have been eliminated.
    full_clearance=g._clearance
    old_margin=g.margin
    try:
        g.margin=.25
        box=FIXTURE['boxes'][0]
        results=[]
        for reuse in (False,True):
            g.begin_entry_certificates()
            def clearance(*a,**kw):
                if not reuse:kw.pop('active_pairs',None)
                return full_clearance(*a,**kw)
            with mock.patch.object(g,'_clearance',side_effect=clearance):
                results.append(g(np.array(box['lower']),np.array(box['upper'])))
            g.end_entry_certificates()
        assert results==[False,False]
    finally:
        g.margin=old_margin
        g.end_entry_certificates()
