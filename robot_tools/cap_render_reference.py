from cap_cad_geometry import *
import struct,zlib

def png(path,rgb):
    def chunk(t,d):return struct.pack('>I',len(d))+t+d+struct.pack('>I',zlib.crc32(t+d)&0xffffffff)
    h,w=rgb.shape[:2];raw=b''.join(b'\0'+rgb[y].tobytes() for y in range(h))
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',w,h,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))

sc=Scene();q=np.array([j['q'] for j in json.loads((OUT/'post_microtest_state.json').read_text())['joints']]);sc.setq(q)
for side in ['left','right']:
    cam=mujoco.MjvCamera();mujoco.mjv_defaultCamera(cam);cam.type=mujoco.mjtCamera.mjCAMERA_FREE;cam.lookat[:]=[.1,0,.2];cam.distance=1.6;cam.azimuth=45;cam.elevation=-15
    ren=mujoco.Renderer(sc.m,height=480,width=640);ren.update_scene(sc.d,cam)
    wrist=sc.m.body(side+'_wrist_yaw_link').id;r=sc.d.xmat[wrist].reshape(3,3);pos=sc.d.xpos[wrist]+r@np.array([.0915,0,.075])
    x=r@np.array([0,-1,0]);up=r@(np.array([.66,0,.75])/np.linalg.norm([.66,0,.75]));forward=np.cross(up,x)
    for c in ren.scene.camera:
        c.pos[:]=pos;c.forward[:]=forward;c.up[:]=up;c.frustum_near=.01;c.frustum_far=5;c.frustum_bottom=-.01*np.tan(np.radians(95)/2);c.frustum_top=-c.frustum_bottom
    rgb=ren.render();png(OUT/(side+'_wrist_reference.png'),rgb);ren.close()
print('WRIST_REFERENCE_RENDERED')
