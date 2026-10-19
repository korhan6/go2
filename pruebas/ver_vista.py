import os, numpy as np
d=np.load(os.path.expanduser("~/.go2_nav/vista.npz"))
v,o,res,N=d["visto"],d["ocup"],float(d["res"]),int(d["n"])
ii,jj=np.nonzero(v); i0,i1,j0,j1=ii.min()-3,ii.max()+4,jj.min()-3,jj.max()+4
px,py=d["pose"][:2]; pi,pj=int(np.floor(py/res))+N//2,int(np.floor(px/res))+N//2
print("vetadas",d["vetadas"].round(2).tolist())
for i in range(i1-1,i0-1,-1):
    row=""
    for j in range(j0,j1):
        if (i,j)==(pi,pj): row+="@"
        elif o[i,j]>=4: row+="#"
        elif v[i,j]>=2: row+="."
        elif v[i,j]==1: row+=","
        else: row+=" "
    print(row)
