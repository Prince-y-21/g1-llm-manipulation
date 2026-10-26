"""Meaningful no-I/O check: moving previews cannot approve release or send motion."""
from pathlib import Path
import unittest,tempfile,json,time,sys,importlib.util,contextlib,io
S=Path(__file__).parent;sys.path.insert(0,str(S))
spec=importlib.util.spec_from_file_location('remote_v4',S/'retry_remote_v4.py');remote=importlib.util.module_from_spec(spec);spec.loader.exec_module(remote)
class Preview(unittest.TestCase):
    def setup_fixture(self,root,age=0):
        r=Path(root);(r/'images').mkdir()
        (r/'body_status.json').write_text(json.dumps(dict(steady=time.monotonic(),moving=1,releasing=0,fault=0,seq=1)))
        for side in ('left','right'):
            p=r/('gripper_'+side);p.mkdir()
            (p/'status.json').write_text(json.dumps(dict(protocol_version=2,run=remote.run,written_steady=time.monotonic())))
        stamp=time.monotonic()-age
        for name in ('head','wrist0','wrist8'):
            (r/'images'/(name+'_frames.csv')).write_text('seq,steady_end\n1,'+str(stamp)+'\n')
            (r/'images'/(name+'_1.jpg')).write_bytes(b'completed-test-frame')
        (r/'images/depth_1.png').write_bytes(b'completed-test-depth')
        (r/'images/camera_metadata.json').write_text('{}');remote.r=r
    def test_moving_preview_is_observation_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.setup_fixture(tmp);sys.argv=['remote','preview','moving_test']
            with contextlib.redirect_stdout(io.StringIO()):remote.main()
            manifest=json.loads((Path(tmp)/'checkpoints/moving_test/visual_manifest.json').read_text())
            self.assertEqual(manifest['kind'],'moving_observation_not_release_evidence')
            self.assertNotIn('body_version',manifest)
            self.assertFalse((Path(tmp)/'request.txt').exists())
            self.assertFalse(any(Path(tmp).glob('verified_*')))
    def test_old_images_are_not_presented_as_live(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.setup_fixture(tmp,age=5);sys.argv=['remote','preview','old_test']
            with self.assertRaises(AssertionError):remote.main()
            self.assertFalse((Path(tmp)/'request.txt').exists())
if __name__=='__main__':unittest.main()
