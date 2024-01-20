import numpy as np
from dataclasses import dataclass
import math
import pywavefront
import time

#real coordinates
@dataclass
class Point:
    x: float
    y: float
    z: float

#screen coordinates
@dataclass
class Point2D:
    x: float
    y: float

#triangles
@dataclass
class Face:
    v1: Point
    v2: Point
    v3: Point

model = input("Model Filename: ")
X_RES = int(input("X Resolution: "))
Y_RES = int(input("Y Resolution: "))
x_rot=0
y_rot=0
z_rot=0
if input("Rotate Model?: y/N ") not in 'yY':
    x_rot = float(input("X Rotation: "))
    y_rot = float(input("Y Rotation: "))
    z_rot = float(input("Z Rotation: "))

t0=time.time()

scene = pywavefront.Wavefront(model, collect_faces=True)

camera = Point(0.0,0.0,-5.0)



def rotatePoint(point, xR, yR, zR):
    point = np.array([point[0] , point[1] , point[2]])
    
    xMatrix = np.array([ [1,0,0] ,
                         [0,math.cos(xR),-1*math.sin(xR)] ,
                         [0,math.sin(xR),math.cos(xR)   ] ])
    
    yMatrix = np.array([ [math.cos(yR),0,math.sin(yR)] ,
                         [0,1,0] ,
                         [-1*math.sin(yR),0,math.cos(yR)] ])
    
    zMatrix = np.array([ [math.cos(zR),-1*math.sin(zR),0   ] ,
                         [math.sin(zR),math.cos(zR),0   ] ,
                         [0,0,1] ])
    
    point = xMatrix.dot(point)
    point = yMatrix.dot(point)
    point = zMatrix.dot(point)
    #convert to regular float because its faster
    return (point[0].item(),point[1].item(),point[2].item())

def formatPoint(point):
    #tuple -> Point dataclass
    return Point(point[0], point[1], point[2])

def getFaceCenter(face):
    return Point((face.v1.x+face.v2.x+face.v3.x)/3,(face.v1.y+face.v2.y+face.v3.y)/3,(face.v1.z+face.v2.z+face.v3.z)/3)

def getDistance(p1 , p2):
    return np.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2)

def projectPoint(point, camera):
    dX = camera.x - point.x
    dY = camera.y - point.y
    dZ = camera.z - point.z
    return Point2D(dX*X_RES/dZ, dY*Y_RES/dZ)

def eFunc(p, v0, v1):
    #cross product
    return (p.x - v0.x)*(v1.y - v0.y) - (p.y - v0.y)*(v1.x - v0.x)

# Initial setup
zbuffer = []

for y in range(Y_RES):
    temp=[]
    for x in range(X_RES):
        temp.append(-1000000)
    zbuffer.append(temp)

screencolor=[]
for y in range(Y_RES):
    temp=[]
    for x in range(X_RES):
        temp.append((255,255,255))
    screencolor.append(temp)


for i in range(len(scene.vertices)):
        scene.vertices[i]=rotatePoint(scene.vertices[i] , x_rot, y_rot, z_rot+math.pi)


faceList=[]
for face in scene.mesh_list[0].faces: # assuming only one object in the scene
            
            faceList.append(Face(projectPoint(formatPoint(scene.vertices[face[0]]),camera), projectPoint(formatPoint(scene.vertices[face[1]]),camera), projectPoint(formatPoint(scene.vertices[face[2]]),camera)))

# Rendering

cy=0
for y in range(0,Y_RES):
    print(str(cy)+"/"+str(Y_RES))
    cx=0
    for x in range(0,X_RES):
        currentPixel = Point2D(x-X_RES//2 , y-Y_RES//2)
        cf=0
        for face in scene.mesh_list[0].faces:
            fv = faceList[cf]
            w0=eFunc(currentPixel, fv.v2, fv.v3)
            w1=eFunc(currentPixel, fv.v3, fv.v1)
            w2=eFunc(currentPixel, fv.v1, fv.v2)
            a = eFunc(fv.v3,fv.v1,fv.v2)
            if w0 >=0 and w1 >=0 and w2 >=0:
                yReciprocal = scene.vertices[face[0]][1]*w0/a + scene.vertices[face[1]][1]*w1/a + scene.vertices[face[2]][1]*w2/a
                if yReciprocal == 0:
                    yReciprocal=0.01
                truey = 1/yReciprocal
                if(truey > zbuffer[cy][cx]):
                    zbuffer[cy][cx]=truey
                    screencolor[cy][cx]= (100,100,100)
            cf+=1
        cx+=1
    cy+=1



from PIL import Image

pixels=screencolor

array=np.array(pixels , dtype=np.uint8)

img = Image.fromarray(array)
img.save('img.png')


t1=time.time()

print("Time elapsed: " + str(t1-t0))

    
