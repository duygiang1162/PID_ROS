#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt 

def D_robot(m1,m2,l1,l2,lc1,lc2,Izz1,Izz2,th):
  D=np.zeros((2,2))
  t1=th[0,0]
  t2=th[1,0]
  D[0,0]=Izz1+Izz2+lc1**2*m1+l1**2*m2+2*np.cos(t2)*l1*lc2*m2+lc2**2*m2
  D[0,1]=Izz2+np.cos(t2)*l1*lc2*m2+lc2**2*m2
  D[1,0]=Izz2+np.cos(t2)*l1*lc2*m2+lc2**2*m2
  D[1,1]=Izz2+lc2**2*m2
  return D

def C_robot(m1,m2,l1,l2,lc1,lc2,Izz1,Izz2,th,dth):
  C=np.zeros((2,2))
  t1=th[0,0]
  t2=th[1,0]

  dt1=dth[0,0]
  dt2=dth[1,0]
  
  C[0,0]=-2 * np.sin(t2) * l1 * lc2 * m2 * dt2
  C[0,1]=-np.sin(t2) * l1 * lc2 * m2 * dt2
  C[1,0]= np.sin(t2) * l1 * lc2 * m2 * dt1
  C[1,1]= 0.0

  return C

def g_robot(m1,m2,l1,l2,lc1,lc2,Izz1,Izz2,th,grav):
  gravity=np.zeros((2,1))
  
  t1=th[0,0]
  t2=th[1,0]

  gravity[0,0]=0.0
  gravity[1,0]=0.0

  return gravity