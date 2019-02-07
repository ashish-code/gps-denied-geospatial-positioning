import numpy as np
import scipy as sp
import scipy.optimize.minpack as spmin
import os, errno, heapq

__halflogmax = np.log(np.finfo(1.0).max)/2.0

class SpatialAngleNMS:
    N = None
    maxCutoff = None
    cutoffPctThresh = None
    spatialRad = None
    angleThresh = None
    
    totalElems = None
    valHeap = None
    maxVal = None
    
    def __init__(self,N,maxCutoff = -np.inf,spatialRad = 0,angleThresh = np.inf,cutoffPctThresh = 0):
        self.N = N
        self.maxCutoff = maxCutoff
        self.cutoffPctThresh = cutoffPctThresh
        self.spatialRad = spatialRad
        self.angleThresh = angleThresh
    
    def add(self,val,pos,ang,data):
        if self.valHeap == None:
            self.valHeap = []
            self.maxVal = -np.inf
            self.totalElems = 0

        self.totalElems += 1

        if val > self.maxCutoff and val > self.cutoffPctThresh*self.maxVal:
            if val > self.maxVal:
                self.maxVal = val
                while len(self.valHeap) > 0 and self.valHeap[0][0] < self.cutoffPctThresh*self.maxVal:
                    heapq.heappop(self.valHeap)

            heapElem = (val,self.totalElems,pos,ang,data)
            if len(self.valHeap) >= 2*self.N:
                heapq.heappushpop(self.valHeap,heapElem)
            else:
                heapq.heappush(self.valHeap,heapElem)

    def getModes(self):
        if self.valHeap == None:
            return []

        if self.spatialRad > 0:
            # Perform spatial non-max suppression
            modeList = [heapq.heappop(self.valHeap) for i in range(len(self.valHeap))]
            retList = []
            for cmode in reversed(modeList):
                closeFound = False
                for omode in retList:
                    angleDiff = cmode[3] - cmode[3]
                    while angleDiff > np.pi:
                        angleDiff += -2.0*np.pi
                    while angleDiff < -np.pi:
                        angleDiff += 2.0*np.pi                    
                    if np.sqrt(np.sum(np.power(cmode[2] - omode[2],2.0))) < self.spatialRad and np.abs(angleDiff) < self.angleThresh:
                        closeFound = True
                        break
                if not closeFound:
                    retList.append(cmode)
        else:
            retList = [heapq.heappop(self.valHeap) for i in range(len(self.valHeap))]

#        if self.cutoffPctThresh > 0:
#            # Double check the cutoff percentage
#            oldRetList = retList
#            retList = []
#            for mode in oldRetList:
#                if mode[0] >= self.cutoffPctThresh*self.maxVal:
#                    retList.append(mode)

        if len(retList) > self.N:
            return retList[0:self.N]
        else:
            return retList
        
        

def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc: # Python >2.5
        if exc.errno == errno.EEXIST:
            pass
        else: raise

def logsumexp(x,axis = 0):
    xShape = x.shape
    if len(xShape) == 0 or xShape[axis] == 0:
        return -np.inf
    alpha = np.amax(x,axis) + (2.0*np.log(x.shape[axis]) - __halflogmax)
    badx = np.logical_not(np.isfinite(x))
    xx = np.exp(x - alpha.reshape(xShape[0:axis] + (1,) +  xShape[(axis+1):]))
    xx[badx] = 0
    sumexpx = np.sum(xx,axis)
    if np.isscalar(sumexpx):
        if sumexpx > 0:
            return alpha + np.log(sumexpx)
        else:
            return -np.inf
    else:
        ret = np.empty_like(sumexpx)
        ret[sumexpx > 0] = np.log(sumexpx[sumexpx>0])
        ret[sumexpx <= 0] = -np.inf 
        return alpha + ret

def logdet_chol(L):
    """log(det(A)) where A = L*L^T with L lower triangular"""
    return 2.0*np.sum(np.log(np.diag(L)))

from scipy.linalg.lapack import get_lapack_funcs
potrs, = get_lapack_funcs(('potrs',),(np.array([0.0]),))

def solve_chol(L,b):
    """inv(A)*b where A = L*L^T with L lower triangular"""
#    potrs, = get_lapack_funcs(('potrs',),(L,))
    b, info = potrs(L,b,lower=True,overwrite_b=False)
    if info < 0:
        msg = "Argument %d to lapack's ?potrs() has an illegal value." % info
        raise TypeError, msg
    if info > 0:
        msg = "Unknown error occured int ?potrs(): error code = %d" % info
        raise TypeError, msg
    return b

def inv_chol(L):
    """inv(A) where A = L*L^T with L lower triangular"""
    invL = np.linalg.inv(L)
    return np.dot(invL.T,invL)

def circumcircle(A,B,C):
    Ap = A - C
    Bp = B - C
    normAp = np.sqrt(np.sum(np.power(Ap,2.0)))
    normBp = np.sqrt(np.sum(np.power(Bp,2.0)))
    Theta = np.arccos(np.dot(Ap.T/normAp,Bp/normBp))
    sinTheta = np.sin(Theta)
    if np.abs(sinTheta) > 1e-8:
        r = np.sqrt(np.sum(np.power(Ap - Bp,2.0)))/(2.0*sinTheta)
        t = normAp**2 * Bp  - normBp**2 * Ap
        center = (np.dot(t.T,Bp)*Ap - np.dot(t.T,Ap)*Bp)/(2.0*(normAp*normBp*sinTheta)**2) + C
    else:
        n = np.array([Ap[1]/normAp,-Ap[0]/normAp]) 
        d0 = np.dot(n.T,C)
        r = np.inf
        center = (n,d0)

    return (r,center)

#def fit_quadratic(A,B,C):
#    offset = (1.0/3.0)*(A+B+C).reshape(2)
#    Ap0 = A.reshape(2)-offset
#    Bp0 = B.reshape(2)-offset
#    Cp0 = C.reshape(2)-offset
##    scale = 1.0
#    scale = (1.0/6.0)*np.sum(np.abs(Ap0) + np.abs(Bp0) + np.abs(Cp0))
#    Ap = Ap0/scale
#    Bp = Bp0/scale
#    Cp = Cp0/scale
#    sumsqAp = np.sum(np.power(Ap,2.0))
#    sumsqBp = np.sum(np.power(Bp,2.0))
#    sumsqCp = np.sum(np.power(Cp,2.0))
#    aM = np.array(((Ap[0],Ap[1],1), \
#                   (Bp[0],Bp[1],1), \
#                   (Cp[0],Cp[1],1)))
#    a = np.linalg.det(aM)
#    
#    bM = np.array(((Ap[0],Ap[1],sumsqAp), \
#                   (Bp[0],Bp[1],sumsqBp), \
#                   (Cp[0],Cp[1],sumsqCp)))
#    b = np.linalg.det(bM)
#    
#    SxM = 0.5*np.array(((sumsqAp,Ap[1],1.0), \
#                        (sumsqBp,Bp[1],1.0), \
#                        (sumsqCp,Cp[1],1.0)))
#    SyM = 0.5*np.array(((Ap[0],sumsqAp,1.0), \
#                        (Bp[0],sumsqBp,1.0), \
#                        (Cp[0],sumsqCp,1.0)))
#    S = np.array((np.linalg.det(SxM), np.linalg.det(SyM)))
#    
#    return (a,b,S,offset,scale)

def line_intersection_2d(p1,p2,p3,p4):
    M12 = np.array([p1.reshape(2),p2.reshape(2)])
    det12 = np.linalg.det(M12)

    M34 = np.array([p3.reshape(2),p3.reshape(2)])
    det34 = np.linalg.det(M34)

    Mx = np.array([(det12,p1[0] - p2[0]),(det34,p3[0] - p4[0])]) 
    My = np.array([(det12,p1[1] - p2[1]),(det34,p3[1] - p4[1])]) 
    
    Mden = np.array([(p1-p2).reshape(2),(p3-p4).reshape(2)])
    den = np.linalg.det(Mden)
    
    return np.array((np.linalg.det(Mx)/den,np.linalg.det(My)/den))

def quadratic_fit_objective(x,r,center):
    if np.isfinite(r):
        d = np.sqrt(np.sum(np.power(x - center.reshape(2,1),2.0),0)) - r
    else:
        n = center[0]
        d0 = center[1]
        d = np.dot(n.T,x) - d0
    return d

def quadratic_error(y,x,A,B):
    (r,center) = circumcircle(A,B,y)
    es = quadratic_fit_objective(x,r,center)
    return es

def quadratic_error_aparam(a,x,A,B):
    d = B - A
    n = np.array([d[1],-d[0]])
    y = 0.5*(A+B) + a*n
    (r,center) = circumcircle(A,B,y)
    es = quadratic_fit_objective(x,r,center)
    return es

def quadratic_error_alphaParam(a,x,A,B):
#    (r,center) = circumcircle(A,B,y)
    d = B - A
    n = np.array([d[1],-d[0]])
    if np.abs(a) < 1e-16:
        normd = np.sqrt(np.sum(np.power(d,2.0)))
        es = np.dot(n.T/normd,x-0.5*(A+B).reshape(2,1))
    else:
        C = 0.5*(A+B) + (1.0/a)*n
        r = 0.5*(np.sqrt(np.sum(np.power(A.reshape(2) - C.reshape(2),2.0))) + \
                 np.sqrt(np.sum(np.power(B.reshape(2) - C.reshape(2),2.0))))
        es = np.sqrt(np.sum(np.power(x - C.reshape(2,1),2.0),0)) - r
    return es

#def quadratic_error_alphaParam_deriv(a,x,A,B):
    

def fit_circle_with_endpoints(x,A,B):
    """ Fit a circle which always passes through A and B and is as close as possible to the points in x """
    if x.ndim > 1 and x.shape[1] > 1:
        x0 = x[:,x.shape[1]/2].reshape(2)
        (x1, flag) = spmin.leastsq(quadratic_error, x0, (x,A,B), maxfev = 5000)
        (r,C) = circumcircle(A,B,x1)
        es = quadratic_fit_objective(x,r,C)

#        x0 = x[:,x.shape[1]/2].reshape(2)
#        d = B - A
#        n = np.array([d[1],-d[0]])
#        a0 = np.dot(n.T,x0 - 0.5*(A+B))
#        (a1, flag) = spmin.leastsq(quadratic_error_aparam, a0, (x,A,B), maxfev = 2000)
#        x1 = 0.5*(A+B) + a1*n
#        (r,C) = circumcircle(A,B,x1)
#        es = quadratic_fit_objective(x,r,C)

#        a0 = 0
#        (a1, flag) = spmin.leastsq(quadratic_error_alphaParam, a0, (x,A,B), maxfev = 2000)
#        d = B - A
#        n = np.array([d[1],-d[0]])
#        if np.abs(a1) < 1e-16:
#            normd = np.sqrt(np.sum(np.power(d,2.0)))
#            r = np.inf
#            C = None
#            es = np.dot(n.T/normd,x-0.5*(A+B).reshape(2,1))
#        else:
#            C = 0.5*(A+B) + (1.0/a1)*n
#            r = 0.5*(np.sqrt(np.sum(np.power(A.reshape(2) - C.reshape(2),2.0))) + \
#                     np.sqrt(np.sum(np.power(B.reshape(2) - C.reshape(2),2.0))))
#            es = np.sqrt(np.sum(np.power(x - C.reshape(2,1),2.0),0)) - r
    else:
        x1 = x.reshape(2)
        (r,C) = circumcircle(A,B,x1)
        es = quadratic_fit_objective(x,r,C)

    return (C,r,es)

if __name__ == '__main__':
    A = 2*np.array([1.0, 0.0]) + 5
    x = 2*np.array([1.0, 1.0]/np.sqrt(2.0)) + 5
    B = 2*np.array([0.0, 1.0]) + 5
#    (a,b,S,offset,scale) = fit_quadratic(A,B,x)
#    (C,r,es) = fit_circle_with_endpoints(x,A,B)
    rp = circumcircle(A,B,x)
    print rp
    
    
