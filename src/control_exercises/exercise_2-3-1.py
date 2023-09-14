#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt 

#  inertia matrix
def D_robot(m1,m2,l1,l2,lc1,lc2,Izz1,Izz2,th):
  D=np.zeros((2,2))
  t1=th[0,0]
  t2=th[1,0]
  
  # start
  D[0,0]=
  D[0,1]=
  D[1,0]=
  D[1,1]=
  # end
    
  return D

#  coriolis term      
def C_robot(m1,m2,l1,l2,lc1,lc2,Izz1,Izz2,th,dth):
  C=np.zeros((2,2))
  t1=th[0,0]
  t2=th[1,0]

  dt1=dth[0,0]
  dt2=dth[1,0]
  
  # startC[0,0]=0
  C[0,1]=0
  C[1,0]=0
  C[1,1]=0
  # endreturn C

  return C

# gravity vector
def g_robot(m1,m2,l1,l2,lc1,lc2,Izz1,Izz2,th,grav):
  gravity=np.zeros((2,1))
  
  t1=th[0,0]
  t2=th[1,0]
  
  # startgravity[0,0]=
  gravity[1,0]=
  # end
	return gravity