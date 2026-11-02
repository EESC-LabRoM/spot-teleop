"""Small manual Cartesian moves for the simulation teaching interface."""
import math
import numpy as np
import xml.etree.ElementTree as ET


def multiply(q, r):
    x,y,z,w=q; a,b,c,d=r
    return np.array([w*a+x*d+y*c-z*b,w*b-x*c+y*d+z*a,w*c+x*b-y*a+z*d,w*d-x*a-y*b-z*c])


def manual_target(position, quaternion, axis, amount):
    p,q=np.array(position,float).copy(),np.array(quaternion,float).copy()
    if isinstance(amount,bool):raise ValueError('Passo inválido')
    amount=float(amount)
    linear=axis in ('reach','height','x','y')
    if axis not in ('reach','height','base','roll','pitch','yaw','x','y'):
        raise ValueError('Eixo manual desconhecido')
    limit=.05 if linear else math.radians(15)
    if not math.isfinite(amount) or abs(amount)>limit+1e-10:
        raise ValueError('Use passos de até 5 cm ou 15 graus')
    if axis=='reach':
        radius=np.linalg.norm(p[:2])
        if radius+amount<.03:raise ValueError('Muito perto da base')
        p[:2]*=(radius+amount)/max(radius,1e-8)
    elif axis=='height':p[2]+=amount
    elif axis=='x':p[0]+=amount
    elif axis=='y':p[1]+=amount
    else:
        vector=np.zeros(3);vector[{'base':2,'roll':2,'pitch':1,'yaw':0}[axis]]=math.sin(amount/2)
        delta=np.r_[vector,math.cos(amount/2)]
        if axis=='base':
            x,y=p[:2];c,s=math.cos(amount),math.sin(amount)
            p[:2]=[c*x-s*y,s*x+c*y]
            q=multiply(delta,q)
        else:q=multiply(q,delta)
    if np.any(p<np.array([.02,-.5,.04])) or np.any(p>np.array([.8,.5,.8])):
        raise ValueError('Esse passo sai do espaço da bancada; reduza ou volte um passo')
    return p,q/np.linalg.norm(q)


def rotation(axis, angle):
    axis=np.asarray(axis,float);axis=axis/np.linalg.norm(axis)
    x,y,z=axis;cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*cross+(1-math.cos(angle))*(cross@cross)


def quaternion_rotation(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


class ManualKinematics:
    """URDF FK/Jacobian and local damped IK for operator demonstrations."""
    def __init__(self, urdf):
        root=ET.parse(urdf).getroot();by_child={j.find('child').get('link'):j for j in root.findall('joint')}
        chain=[];link='fr3_hand_tcp'
        while link!='fr3_link0':
            joint=by_child[link];chain.append(joint);link=joint.find('parent').get('link')
        self.chain=[];self.lower=[];self.upper=[]
        for joint in reversed(chain):
            origin=joint.find('origin');matrix=np.eye(4)
            if origin is not None:
                matrix[:3,3]=list(map(float,origin.get('xyz','0 0 0').split()))
                r,p,y=map(float,origin.get('rpy','0 0 0').split())
                matrix[:3,:3]=rotation([0,0,1],y)@rotation([0,1,0],p)@rotation([1,0,0],r)
            moving=joint.get('type')=='revolute';axis=joint.find('axis')
            axis=np.array(list(map(float,axis.get('xyz','0 0 1').split()))) if axis is not None else np.array([0.,0.,1.])
            self.chain.append((matrix,axis,moving))
            if moving:
                bounds=joint.find('limit');self.lower.append(float(bounds.get('lower')));self.upper.append(float(bounds.get('upper')))
        self.lower=np.array(self.lower);self.upper=np.array(self.upper)

    def forward(self,q):
        transform=np.eye(4);origins=[];axes=[];i=0
        for origin,axis,moving in self.chain:
            transform=transform@origin
            if moving:
                origins.append(transform[:3,3].copy());axes.append(transform[:3,:3]@axis)
                rotate=np.eye(4);rotate[:3,:3]=rotation(axis,q[i]);transform=transform@rotate;i+=1
        position=transform[:3,3];jacobian=np.zeros((6,len(axes)))
        for i,axis in enumerate(axes):
            jacobian[:3,i]=np.cross(axis,position-origins[i]);jacobian[3:,i]=axis
        return position,transform[:3,:3],jacobian

    def solve(self,position,quaternion,seed):
        q=np.clip(np.array(seed,float),self.lower+1e-5,self.upper-1e-5)
        target=quaternion_rotation(quaternion)
        for _ in range(90):
            p,r,j=self.forward(q);delta=target@r.T
            theta=math.acos(float(np.clip((np.trace(delta)-1)/2,-1,1)))
            vector=np.array([delta[2,1]-delta[1,2],delta[0,2]-delta[2,0],delta[1,0]-delta[0,1]])
            orient=vector*.5 if theta<1e-6 else vector*(theta/(2*math.sin(theta)))
            error=np.r_[np.asarray(position)-p,orient]
            if np.linalg.norm(error[:3])<.0002 and np.linalg.norm(orient)<.001:
                return q
            step=j.T@np.linalg.solve(j@j.T+np.eye(6)*.0001,error)
            largest=np.max(np.abs(step))
            if largest>.12:step*=.12/largest
            q=np.clip(q+step,self.lower+1e-5,self.upper-1e-5)
        raise ValueError('Pose não alcançável neste passo. Volte ou tente um passo menor.')
