import vpython as vp


N = 100
m = 8
vp.scene.width = vp.scene.height = N*m
vp.scene.center = vp.vec(N*m/2,N*m/2,0)
vp.scene.range = 0.5*N*m
v = []
quads = []
for x in range(N):
    for y in range(N):
        c = vp.color.hsv_to_rgb(vp.vec(x/N,1,1))
        v.append(vp.vertex(pos=vp.vec(x,y,0), color=c, normal=vp.vec(0,0,1)))
for x in range(N-1):
    for y in range(N-1):
        quads.append(vp.quad(vs=[v[N*x+y], v[N*x+N+y], v[N*x+N+y+1], v[N*x+y+1]]))
q = vp.compound(quads)
clones = []
n = N-1
for x in range(m):
    for y in range(m):
        clones.append(q.clone(pos=vp.vec(n*x+N/2,n*y+N/2,0)))
q.visible = False # make the original object invisible

