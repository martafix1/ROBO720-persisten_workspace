import numpy as np



Q = np.array([1,1,1])



center = np.array([0,0,0])
dir = np.array([0,1,0])

planeX = np.array([1,0,0])

planeY = np.cross(planeX, dir)

distance = np.dot(Q - center, dir)

Qprojected = Q - np.dot(Q - center, dir) * dir
r = Qprojected-center

planeXY = np.array([np.dot(r, planeX), np.dot(r, planeY)])


print(f"Q   : {Q}")

print(f"dir   : {dir}")
print(f"center: {center}")

print(f"planeX: {planeX}")
print(f"planeY: {planeY}")

print(f"distance: {distance}")

print(f"Qprojected: {Qprojected}")
print(f"r         : {r}")
print(f"planeXY   : {planeXY}")

#print(planeXY)





