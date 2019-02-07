import numpy as np
import scipy as sp
import scipy.optimize as opt
import scipy.stats as stats
import math
from Utils import logsumexp,inv_chol,logdet_chol,solve_chol
from copy import deepcopy

def gaussian_kl(mu1,Sigma1,mu2,Sigma2,cholSigma1 = None,cholSigma2 = None):
    if cholSigma1 == None:
        cholSigma1 = np.linalg.cholesky(Sigma1)
    if cholSigma2 == None:
        cholSigma2 = np.linalg.cholesky(Sigma2)
    d = len(mu1)

    dmu = np.linalg.solve(cholSigma1,mu1 - mu2)
    Lhat = np.linalg.solve(cholSigma2,cholSigma1) 

#    dmu = sp.linalg.solve_triangular(cholSigma1,mu1 - mu2,lower=True)
#    Lhat = sp.linalg.solve_triangular(cholSigma2,cholSigma1,lower=True)

    logDetSigmaRatio = 2.0*(np.sum(np.log(np.diag(cholSigma2))) - np.sum(np.log(np.diag(cholSigma1))))
    trSigmaInvSigma = np.sum(np.power(Lhat,2)) - float(d)
    dmuTerm = np.sum(np.power(dmu,2),0)
    kl = 0.5*(logDetSigmaRatio + trSigmaInvSigma + dmuTerm)
    if kl <= -1e-10:
        print kl, logDetSigmaRatio, trSigmaInvSigma, dmuTerm
        assert(kl >= -1e-10)
    elif kl < 0:
        kl = 0
    return kl

def gaussian_kl_vec(mu1,Sigma1,mu2,Sigma2,cholSigma1 = None,cholSigma2 = None):
    d = mu1.shape[0]
    N = mu1.shape[1]
    
    if cholSigma1 == None:
        cholSigma1 = np.empty_like(Sigma1)
        for i in range(N):
            cholSigma1[:,:,i] = np.linalg.cholesky(Sigma1[:,:,i])
    if cholSigma2 == None:
        cholSigma2 = np.linalg.cholesky(Sigma2)

    SigmaInvSigma = solve_chol(cholSigma2,Sigma1.reshape((d,-1))).reshape((d,d,N))
    trSigmaInvSigma = np.trace(SigmaInvSigma) - float(d) 

    muDiff = mu1 - mu2
    dmuDiff = np.copy(muDiff)
    for i in range(N):
        dmuDiff[:,i] = solve_chol(cholSigma1[:,:,i], muDiff[:,i])
    dmuTerm = np.sum(np.multiply(muDiff,dmuDiff),0)

    logDetSigmaRatio = 2.0*(np.sum(np.log(np.diagonal(cholSigma2)).reshape((1,d)),1) - np.sum(np.log(np.diagonal(cholSigma1)).reshape((N,d)),1))

    kl = np.maximum(0,0.5*(logDetSigmaRatio + trSigmaInvSigma + dmuTerm))
    
    return kl


def gaussian_logprob(x,mu,Sigma,cholSigma = None,normalized = True):
    if cholSigma == None:
        cholSigma = np.linalg.cholesky(Sigma)

    if mu == None:
        dx = np.linalg.solve(cholSigma,x)
    else:
        dx = np.linalg.solve(cholSigma,x - mu)
    
    quadForm = np.sum(np.power(dx,2),0)
#    quadForm = np.dot(dx.T,dx)
    
    if normalized:
        return -0.5*(np.sum(math.log(2.0*math.pi) + 2*np.log(np.diag(cholSigma))) + quadForm)
    else:
        return -0.5*quadForm

def gaussian_logprob_grad(x,mu,Sigma,cholSigma = None):
    if cholSigma == None:
        cholSigma = np.linalg.cholesky(Sigma)

    if mu == None:
        return -0.5*solve_chol(cholSigma,x).T
    else:
        return -0.5*solve_chol(cholSigma,x-mu).T
    
def gaussian_prec_logprob(x,mu,InvSigma,cholInvSigma = None,normalized = True):
    if mu == None:
        dx = x
    else:
        dx = x - mu
    
    quadForm = np.sum(np.multiply(dx,np.dot(InvSigma,dx)))
#    quadForm = np.dot(dx.T,np.dot(InvSigma,dx))
    
    if normalized:
        if cholInvSigma == None:
            cholInvSigma = np.linalg.cholesky(InvSigma)

        return -0.5*(np.sum(math.log(2.0*math.pi) - 2*np.log(np.diag(cholInvSigma))) + quadForm)
    else:
        return -0.5*quadForm

def gaussian_dist_xmu1xmu2_product_x(mu1,Sigma1,mu2,Sigma2):
    """Compute distribution of N(x|mu1,Sigma1)N(x|mu2,Sigma2)"""
    InvSigmaHat = np.linalg.inv(Sigma1) + np.linalg.inv(Sigma2)
    SigmaHat = np.linalg.inv(InvSigmaHat)
    muHat = np.dot(SigmaHat,np.linalg.solve(Sigma1, mu1) + np.linalg.solve(Sigma2,mu2))
    logC = gaussian_logprob(mu1,mu2,Sigma1 + Sigma2)
    return (logC,muHat,SigmaHat)


def gaussian_dist_yxxmu_product_x(y,Ay,by,Sigmay,mu,Sigma,cholSigmay = None,cholSigma = None,InvSigma = None):
    """Compute distribution p(x) \propto N(y|Ay*x+by,Sigmay)N(x|mu,Sigma)"""
    if cholSigma == None:
        cholSigma = np.linalg.cholesky(Sigma)
#    if InvSigma == None:
#        InvSigma = inv_chol(cholSigma)
    if cholSigmay == None:
        cholSigmay = np.linalg.cholesky(Sigmay)
    InvSigmayAy = solve_chol(cholSigmay, Ay)

    yDim = len(y)
    xDim = len(mu)

    SigmaBar = Sigmay + np.dot(Ay,np.dot(Sigma,Ay.T))
    # SigmaHat = (Ay.T * Sigma_y^-1 * Ay + Sigma^-1)^-1
    #          = Sigma - Sigma * Ay.T * inv(Sigma_y + A*Sigma*A.T) * A * Sigma (woodbury)
    tmpSigmaHat = np.identity(Sigma.shape[0]) - np.dot(Sigma, np.dot(Ay.T,np.linalg.solve(SigmaBar,Ay)))
    SigmaHat = np.dot(tmpSigmaHat,Sigma)
    
    muHat = np.dot(SigmaHat, np.dot(InvSigmayAy.T, y - by)) + np.dot(tmpSigmaHat,mu)
    logC = gaussian_logprob(y,np.dot(Ay,mu)+by,SigmaBar,normalized = False) \
            + np.sum(np.log(np.diag(np.linalg.cholesky(SigmaHat)))) \
            - np.sum(np.log(np.diag(cholSigma))) \
            - 0.5*yDim*np.log(2.0*math.pi) - np.sum(np.log(np.diag(cholSigmay)))

    return (logC,muHat,SigmaHat)
    
def gaussian_dist_yxxz_intxproduct_z(y,Ay,by,Sigmay,Ax,bx,Sigmax,mu,Sigma):
    """
    Compute distribution of p(z) = C * N(z|mu,Sigma) \int N(y|Ay*x + by,Sigmay)N(x|Ax*z + bx,Sigmax) dx
                                 = N(m,S)
    
    Returns (logC,muBar,SigmaBar).
    """
    cholSigmax = np.linalg.cholesky(Sigmax)
    cholSigmay = np.linalg.cholesky(Sigmay)
    cholSigma = np.linalg.cholesky(Sigma)
    InvSigmax = inv_chol(cholSigmax)
    InvSigmay = inv_chol(cholSigmay)
    InvSigma = inv_chol(cholSigma)
    InvSigmayAy = np.dot(InvSigmay,Ay)
    
    mhatxy = np.dot(Ay.T,np.dot(InvSigmay,y - by)) - np.dot(InvSigmax,bx)
    Pxy = np.dot(Ay.T, InvSigmayAy) + InvSigmax
    cholPxy = np.linalg.cholesky(Pxy)
#    Pzy = np.dot(Ax.T,np.dot(InvSigmax - np.dot(InvSigmax,solve_chol(cholPxy,InvSigmax)),Ax)) + InvSigma
    Pzy = np.dot(Ax.T,np.dot(InvSigmax,Ax)) - np.dot(Ax.T,np.dot(InvSigmax,solve_chol(cholPxy,InvSigmax)),Ax) + InvSigma
    Pzym = np.dot(Ax.T,solve_chol(cholSigmax,solve_chol(cholPxy, (np.dot(Ay.T,solve_chol(cholSigmay,y)) - solve_chol(cholSigmax,bx)) ))) + solve_chol(cholSigma,mu)
    cholPzy = np.linalg.cholesky(Pzy)
    m = solve_chol(cholPzy,Pzym)
    S = inv_chol(cholPzy)
    rt2pi = math.sqrt(2.0*math.pi)
    logC = gaussian_logprob(bx,0,Sigmax) + gaussian_logprob(mu,0,Sigma) + gaussian_logprob(y,by,Sigmay) - 0.5*(logdet_chol(rt2pi*cholPxy) + logdet_chol(rt2pi*cholPzy))\
           + 0.5*(np.dot(mhatxy.T,solve_chol(cholPxy,mhatxy)) + np.dot(m.T,Pzym))
#    print y - by,bx
#    print mhatxy
#    print Pxy
#    print Pzy
#    print m
#    print 'quadBits = ',0.5*(np.dot(mhatxy.T,solve_chol(cholPxy,mhatxy))), 0.5*np.dot(m.T,Pzym)
    return (logC,m,S)


#class GaussianDistribution:
#    __mu = None
#    __Sigma = None
#    __CholSigma = None

def klDiv_var(Phi,logPhi,logPsi,KLdivs):
    finitePhiInds = np.nonzero(np.isfinite(logPhi))
    D_upper = np.sum(np.multiply(Phi[finitePhiInds],logPhi[finitePhiInds] - logPsi[finitePhiInds])) + np.sum(np.multiply(Phi[finitePhiInds],KLdivs[finitePhiInds]))
    return D_upper

def klDiv_varit(Phi,logPhi,logPsi,logweights_ref,logweights_other,KLdivs,upd_other_logweights = False):
    tmp = logPsi - KLdivs
    lsetmp = logsumexp(tmp,0)
    lsetmp[np.logical_not(np.isfinite(lsetmp))] = 0
    logPhi = logweights_ref.reshape((1,-1)) + (tmp - lsetmp)
    Phi = np.exp(logPhi)
    if upd_other_logweights:
        logweights_other = logsumexp(logPhi,1)
    logPsi = logweights_other.reshape((-1,1)) + logPhi - logsumexp(logPhi,1).reshape((-1,1))
    
    D_upper = klDiv_var(Phi,logPhi,logPsi,KLdivs)
    
    if upd_other_logweights:
        return (D_upper,Phi,logPhi,logPsi,logweights_other)
    else:
        return (D_upper,Phi,logPhi,logPsi)

class GaussianMixtureModel:
    logweights = None
    mus = None
    Sigmas = None
    dim = 0
    
    def __init__(self,dim,nComps = 0):
        self.logweights = np.empty((nComps))
        self.mus = np.asarray(np.empty((dim,nComps)))
        self.Sigmas = np.asarray(np.empty((dim,dim,nComps)))
        self.dim = dim
    
    def klDivergence_varupper(self,other):
        """ Returns an (upper bound) approximation to KL(self||other). """
        D = self.dim
        Nref = self.numComps()
        Nother = other.numComps()

        if KLdivs == None:
            cholSigmaSelf = np.empty((D,D,Nref))
            for b in range(Nref):
                cholSigmaSelf[:,:,b] = np.linalg.cholesky(self.getSigma(b))

            KLdivs = np.empty((Nother,Nref))
            for b in range(Nother):
                mub = other.getMu(b)
                Sigmab = other.getSigma(b)
                cholSigmab = np.linalg.cholesky(other.getSigma(b))
    
                KLdivs[b,:] = gaussian_kl_vec(self.mus,self.Sigmas,mub,Sigmab,cholSigma1 = cholSigmaSelf,cholSigma2 = cholSigmab)

        logPhi = other.logweights.reshape(Nother,1) + self.logweights.reshape(1,Nref)
        logPsi = self.logweights.reshape(Nother,1) + logPhi - logsumexp(logPhi,1).reshape((Nother,1))
        Phi = np.exp(logPhi)

        done = False
        itNum = 0
        D_upper = np.inf
        while not done:
            itNum += 1
            last_D_upper = D_upper

            (D_upper,Phi,logPhi,logPsi) = klDiv_varit(Phi,logPhi,logPsi,self.logweights,other.logweights,KLdivs)

            assert(np.isfinite(D_upper))
            print '{0}: D_upper = {2}'.format(itNum,D_upper)
            if D_upper - last_D_upper > 1e-6:
                break
        
        return D_upper

    def simplify(self,ref = None,natThresh = None,minNumComps = 1):
        """ Remove components and adjust the mixture until just before the KL divergence exceeds natThresh """
        maxNumSubVarIts = 5
        maxNumVarIts = 20
        maxNumDelPerIt = 1
        
        
        D = self.dim
        Nself = self.numComps()
        Nnew = Nself

        # Compute Cholesky decompositions
        cholSigmaSelf = np.empty((D,D,Nself))
        for b in range(Nself):
            cholSigmaSelf[:,:,b] = np.linalg.cholesky(self.getSigma(b))

        if ref == None:
            ref = deepcopy(self)
            cholSigmaRef = deepcopy(cholSigmaSelf)
            Nref = Nself
            # Initialize the variational parameters
            Phi = np.diag(np.exp(self.logweights.reshape(Nnew)))
#            logPhi = np.log(Phi)
            logPhi = np.empty_like(Phi)
            logPhi[Phi > 0] = np.log(Phi[Phi > 0])
            logPhi[Phi <= 0] = -np.inf
#            logPsi = np.log(Phi)
            logPsi = np.copy(logPhi)
        else:
            Nref = ref.numComps()
            cholSigmaRef = np.empty((D,D,Nref))
            for a in range(Nref):
                cholSigmaRef[:,:,a] = np.linalg.cholesky(ref.getSigma(a))
            # Initialize the variational parameters
            logPhi = self.logweights.reshape(Nnew,1) + ref.logweights.reshape(1,Nref)
            logPsi = self.logweights.reshape(Nnew,1) + logPhi - logsumexp(logPhi,1).reshape((Nnew,1))
            Phi = np.exp(logPhi)

        # Compute KL divergence between components
        KLdivs = np.empty((Nself,Nref))
        for b in range(Nself):
            mub = self.getMu(b)
            Sigmab = self.getSigma(b)
            cholSigmab = cholSigmaSelf[:,:,b]

            KLdivs[b,:] = gaussian_kl_vec(ref.mus,ref.Sigmas,mub,Sigmab,cholSigma1 = cholSigmaRef,cholSigma2 = cholSigmab)
        del cholSigmaSelf

        new_D_upper = klDiv_var(Phi,logPhi,logPsi,KLdivs)

        newActiveComps = [i for i in range(0,Nself)]
        newActiveComps.sort(key=lambda x: self.logweights[x])

        globalVarItCnt = 0        
        itNum = 0
        while Nnew-maxNumDelPerIt >= minNumComps:
            itNum += 1
            
            currNumDel = 1
#            delLogW = logsumexp(self.logweights[newActiveComps[0:2]])
#            while delLogW < np.log(1e-4) and currNumDel < maxNumDelPerIt:
#                currNumDel += 1
#                delLogW = logsumexp(self.logweights[newActiveComps[0:(currNumDel+1)]])
#            print '{0}: numDel = {1}'.format(itNum,currNumDel)
            delCompI = newActiveComps[0:currNumDel]
            propComps = newActiveComps[currNumDel:]
            numDel = len(delCompI)
            Nprop = Nnew-numDel
            
            propLogWeights = np.logaddexp(self.logweights[propComps], logsumexp(self.logweights[delCompI]) - np.log(Nprop))
            propMus = self.mus[:,propComps].reshape((D,Nprop))
            propSigmas = self.Sigmas[:,:,propComps].reshape((D,D,Nprop))

            propLogPhi = np.logaddexp(logPhi[propComps,:],logsumexp(logsumexp(logPhi[delCompI,:],1),0) - np.log(Nprop))
            propPhi = np.exp(propLogPhi)
            prevNormPropPhi = None
            propKLdivs = KLdivs[propComps,:]
            propLogPsi = propLogWeights.reshape(Nprop,1) + propLogPhi - logsumexp(propLogPhi,1).reshape((Nprop,1))

            D_upper = klDiv_var(propPhi,propLogPhi,propLogPsi,propKLdivs)

            varItNum = 0
#            while varItNum < maxNumVarIts and (itNum == 0 or D_upper >= natThresh):
            while varItNum < maxNumVarIts and (varItNum == 0 or D_upper >= natThresh):
                varItNum += 1
                globalVarItCnt += 1
                
                last_propLogWeights = propLogWeights
                lastVarIt_D_upper = D_upper

                # Minimize the upper bound wrt the variational parameters and log weights
                subVarItNum = 0
                while subVarItNum < maxNumSubVarIts and D_upper >= natThresh:
                    lastsub_propLogWeights = propLogWeights
                    lastSubVarIt_D_upper = D_upper
                    (D_upper,propPhi,propLogPhi,propLogPsi,propLogWeights) = klDiv_varit(propPhi,propLogPhi,propLogPsi,ref.logweights,propLogWeights,propKLdivs,True)
                    if np.abs(D_upper - lastSubVarIt_D_upper) < 1e-6:
                        break
                    
                if D_upper >= natThresh:
                    normPropPhi = np.exp(propLogPhi - logsumexp(propLogPhi,1).reshape(Nprop,1))
                    if prevNormPropPhi != None:
#                        updPropComps = np.nonzero(np.sum(np.abs(normPropPhi - prevNormPropPhi),1).ravel() > 1e-2)
#                        updPropComps = updPropComps[0]
                        updPropComps = np.array([np.argmax(np.sum(np.abs(normPropPhi - prevNormPropPhi),1).ravel())])
                    else:
                        updPropComps = range(Nprop)
#                    updPropComps = range(Nprop)
#                    updPropComps = np.array([np.argmax(logsumexp(propLogPhi,1).ravel())])
                else:
                    updPropComps = []

                if len(updPropComps) > 0:
                    if prevNormPropPhi == None:
                        prevNormPropPhi = normPropPhi
                    else:
                        prevNormPropPhi[updPropComps,:] = normPropPhi[updPropComps,:]
                    propMus[:,updPropComps] = np.dot(ref.mus,normPropPhi[updPropComps,:].T)
                    dmus = propMus[:,updPropComps].reshape(D,-1,1) - ref.mus.reshape(D,1,Nref)
                    dmuCovs = np.multiply(dmus.reshape(D,1,-1,Nref),dmus.reshape(1,D,-1,Nref))
                    #updPropSigmas = np.sum(np.multiply((ref.Sigmas.reshape(D,D,1,Nref) + dmuCovs).reshape((D,D,-1,Nref)),normPropPhi[updPropComps,:].reshape((1,1,-1,Nref))),3)
                    updPropSigmas = np.sum(np.multiply(ref.Sigmas.reshape(D,D,1,Nref),normPropPhi[updPropComps,:].reshape((1,1,-1,Nref))),3) + np.sum(np.multiply(dmuCovs.reshape((D,D,-1,Nref)),normPropPhi[updPropComps,:].reshape((1,1,-1,Nref))),3)
                    propSigmas[:,:,updPropComps] = updPropSigmas

                    prev___D_upper = D_upper
                    prev___KLDivs = propKLdivs[updPropComps,:]
                    for (tInd,propInd) in enumerate(updPropComps):
                        mub = propMus[:,propInd].reshape((D,1))
                        Sigmab = propSigmas[:,:,propInd]
                        propKLdivs[propInd,:] = gaussian_kl_vec(ref.mus,ref.Sigmas,mub,Sigmab,cholSigma1 = cholSigmaRef)
                    
#                    (D_upper,propPhi,propLogPhi,propLogPsi,propLogWeights) = klDiv_varit(propPhi,propLogPhi,propLogPsi,ref.logweights,propLogWeights,propKLdivs,True)
                    D_upper = klDiv_var(propPhi,propLogPhi,propLogPsi,propKLdivs)
                    if False and D_upper - prev___D_upper >= 1e-6:
                        print '{0} -> {1}'.format(prev___D_upper,D_upper)
                        print 'prevKLDivs = {0}'.format(prev___KLDivs)
                        print 'currKLDivs = {0}'.format(propKLdivs[updPropComps,:])
                        assert(D_upper - prev___D_upper < 1e-6)
#                    print '{0} -> {1}'.format(prev___D_upper,D_upper)
                    if prev___D_upper >= D_upper + 1e-8:
                        break

#                (D_upper,propPhi,propLogPhi,propLogPsi,propLogWeights) = klDiv_varit(propPhi,propLogPhi,propLogPsi,ref.logweights,propLogWeights,propKLdivs,True)
#                print '  {1}: D_upper = {0}, subVarItNums = {2}'.format(D_upper,varItNum,subVarItNum)


                if np.abs(D_upper - lastVarIt_D_upper) < 1e-6 and np.all(np.abs(last_propLogWeights - propLogWeights) < 1e-6):
                    break



#            print '{2}: D_upper = {0}, varItNums = {1}'.format(D_upper,varItNum,itNum)
            
            if D_upper > natThresh:
                break
            else:
                Nnew = Nprop
                new_D_upper = D_upper

                KLdivs[propComps,:] = propKLdivs
                logPhi[propComps,:] = propLogPhi
                Phi[propComps,:] = propPhi
                logPsi[propComps,:] = propLogPsi

                self.logweights[delCompI] = -np.inf
                self.logweights[propComps] = propLogWeights
                self.mus[:,propComps] = propMus
                self.Sigmas[:,:,propComps] = propSigmas

                while numDel > 0:
                    newActiveComps.pop(0)
                    numDel += -1
                newActiveComps.sort(key=lambda x: self.logweights[x])

        if Nnew < Nself:
            self.logweights = self.logweights[newActiveComps]
            self.mus = self.mus[:,newActiveComps].reshape(D,Nnew)
            self.Sigmas = self.Sigmas[:,:,newActiveComps].reshape(D,D,Nnew)

        return new_D_upper

    def getLogW(self,compI):
        return self.logweights[compI]
    
    def getMu(self,compI):
        if isinstance(compI,(int, long)):
            return np.reshape(self.mus[:,compI],(self.dim,1))
        else:
            return np.reshape(self.mus[:,compI],(self.dim,compI.size))

    def getSigma(self,compI):
        if isinstance(compI,(int, long)):
            return np.reshape(self.Sigmas[:,:,compI],(self.dim,self.dim))
        else:
            return np.reshape(self.Sigmas[:,:,compI],(self.dim,self.dim,compI.size))

    def drawSamples(self,N):
        Ws = np.exp(self.logweights - logsumexp(self.logweights))
        sampleComps = np.random.multinomial(N,Ws)
        samples = np.empty((self.dim,N))
        startInd = 0
        for i in range(len(sampleComps)):
            cN = sampleComps[i]
            if cN < 0:
                print cN
                print sampleComps
                print Ws
                print self.logweights
            assert(cN >= 0)
            assert(startInd >= 0)
            samples[:,startInd:(startInd+cN)] = self.getMu(i) + np.dot(np.linalg.cholesky(self.Sigmas[:,:,i]),np.random.randn(self.dim,cN))
            startInd += cN
        return samples

    def logProb(self,x):
        N = self.numComps()
        if len(x.shape) == 1:
            x = x.reshape((self.dim,-1))
        lps = np.empty((N,x.shape[1]))
        for i in range(N):
            lps[i,:] = gaussian_logprob(x,self.getMu(i),self.Sigmas[:,:,i]) + self.logweights[i]
        return logsumexp(lps,0)
    
    def dlogProb(self,x):
        N = self.numComps()
        origxshape = x.shape
        if len(x.shape) == 1:
            x = x.reshape((self.dim,1))
        lps = np.empty((N,1))
        grads = np.empty((N,self.dim))
        for i in range(N):
            cmu = self.getMu(i)
            cSigma = self.Sigmas[:,:,i]
            cholSigma = np.linalg.cholesky(cSigma)
            lps[i,0] = gaussian_logprob(x,cmu,cSigma,cholSigma) + self.logweights[i]
            grads[i,:] = gaussian_logprob_grad(x,cmu,cSigma,cholSigma)
        return np.sum(np.multiply(np.exp(lps-logsumexp(lps,0)),grads),0).reshape(origxshape)
    
    def findMaximum(self,x0 = None):
        if x0 == None:
            lp = self.logProb(self.mus)
            minI = np.argmax(lp.reshape(-1))
            x0 = self.getMu(minI)
#        optRes = opt.fmin(lambda x: -self.logProb(x), x0, disp=False)
        optRes = opt.fmin_bfgs(lambda x: -self.logProb(x), x0, disp=False, fprime = lambda x: -self.dlogProb(x))
        return optRes

    def productGaussianDist(self,mu,Sigma):
        newDist = self
        for i in range(self.numComps()):
            (clogC, cmu, cSigma) = gaussian_dist_xmu1xmu2_product_x(mu,Sigma,self.getMu(i),self.Sigmas[:,:,i])
            newDist.mus[:,[i]] = cmu
            newDist.Sigmas[:,:,i] = cSigma
            newDist.logweights[i] += clogC
        logC = newDist.normalizeWeights()
        return (logC,newDist)

    def numComps(self):
        return len(self.logweights)
    
    def addMixture(self,w,mixDist):
        self.logweights = np.hstack((self.logweights,mixDist.logweights + np.log(w)))
        self.mus = np.hstack((self.mus,mixDist.mus))
        self.Sigmas = np.dstack((self.Sigmas,mixDist.Sigmas))

    def addMixtureLogW(self,logw,mixDist):
        self.logweights = np.hstack((self.logweights,mixDist.logweights + logw))
        self.mus = np.hstack((self.mus,mixDist.mus))
        self.Sigmas = np.dstack((self.Sigmas,mixDist.Sigmas))
        
    def addComponent(self,w,mu,Sigma):
        self.logweights = np.hstack((self.logweights,np.log(w)))
        self.mus = np.hstack((self.mus,mu))
        self.Sigmas = np.dstack((self.Sigmas,Sigma))
        
    def addComponentLogW(self,logw,mu,Sigma):
        logw = np.asarray(logw)
        self.logweights = np.hstack((self.logweights,logw))
        self.mus = np.hstack((self.mus,mu))
        self.Sigmas = np.dstack((self.Sigmas,np.asarray(Sigma)))

    def setComponentLogW(self,compI,logw,mu,Sigma):
        logw = np.asarray(logw)
        compI = np.asarray(compI).ravel()
        self.logweights[compI] = logw
        self.mus[:,compI] = mu.reshape((self.dim,compI.size))
        self.Sigmas[:,:,compI] = Sigma.reshape((self.dim,self.dim,compI.size))

    def normalizeWeights(self):
        N = self.numComps()
        if N == 0:
            return -np.inf
        elif N == 1:
            normFactor = self.logweights[0]
            self.logweights[0] = 0
#            print 'Normalizing single component mixture, normFactor = {0}, lw = {1}'.format(normFactor,self.logweights[0,0])
            return normFactor
        else:
            normFactor = logsumexp(self.logweights)
            assert(np.isfinite(normFactor))
            self.logweights -= normFactor
            normFactor2 = logsumexp(self.logweights)
            if np.abs(normFactor2) > 1e-8:
                normFactor += normFactor2
                self.logweights -= normFactor2
            return normFactor
    
    def transform(self,A,b):
        newDim = A.shape[0]
        transMM = GaussianMixtureModel(newDim,self.numComps())
        transMM.logweights = deepcopy(self.logweights)
        transMM.mus = np.dot(A,self.mus) + b
        for compI in range(self.numComps()):
            transMM.Sigmas[:,:,compI] = np.dot(np.dot(A,self.Sigmas[:,:,compI]),A.T)
        return transMM

#    def cdf(self,ub):
#        assert(self.dim == 1) # Unimplmemented for multivariate gaussians
#        return np.exp(self.logcdf(ub))
#
#    def logcdf(self,ub):
#        assert(self.dim == 1) # Unimplmemented for multivariate gaussians
#        logcdfs = deepcopy(self.logweights)
#        for compI in range(self.numComps()):
#            logcdfs[compI] += stats.norm.logcdf(ub,self.mus[0,compI],np.sqrt(self.Sigmas[0,0,compI]))
#        return logsumexp(logcdfs)
    def logcdf(self,ub):
        assert(self.dim == 1) # Unimplmemented for multivariate gaussians
        return np.log(self.cdf(ub))

    def cdf(self,ub):
        assert(self.dim == 1) # Unimplmemented for multivariate gaussians
        ret = np.zeros_like(ub)
        for compI in range(self.numComps()):
            ccdf = stats.norm.cdf(ub,self.mus[0,compI],np.sqrt(self.Sigmas[0,0,compI]))
            ret[ccdf > 0] += np.exp(self.logweights[compI] + np.log(ccdf[ccdf > 0]))
        return ret
    
    def removeUnusedComponents(self):
        goodComps = np.flatnonzero(np.isfinite(self.logweights))
        self.logweights = self.logweights[goodComps]
        self.mus = np.reshape(self.mus[:,goodComps],(self.dim,len(goodComps)))
        self.Sigmas = np.reshape(self.Sigmas[:,:,goodComps],(self.dim,self.dim,len(goodComps)))
    
    def computeMeanCovar(self):
        weights = np.exp(self.logweights - logsumexp(self.logweights))
        mu = np.dot(self.mus,weights).reshape((self.dim,1))
        Ex2 = np.reshape(np.dot(self.Sigmas,weights),(self.dim,self.dim)) + np.asarray(np.dot(np.multiply(self.mus,weights),self.mus.T))
        Sigma = Ex2 - np.dot(mu, mu.T)
        return (mu,Sigma)

    def computeMean(self):
        weights = np.exp(self.logweights - logsumexp(self.logweights))
        mu = np.dot(self.mus,weights).reshape((self.dim,1))
        return mu
    
    def printDist(self):
        for i in range(len(self.logweights)):
            print 'Component {0}: log(w) = {1}'.format(i,self.logweights[i])
            print '  mu       sigma'
            for d in range(self.dim):
                print '  {0} {1}'.format(self.mus[d,i],self.Sigmas[d,:,i])
                
        
        
        
