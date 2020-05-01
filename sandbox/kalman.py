from numpy import *
from numpy.linalg import inv,det
import matplotlib.pyplot as plt

#Function to project the state and the error covariance ahead
def kf_predict(X, P, A, Q, B, U):
    X = dot(A, X) + dot(B, U)
    P = dot(A, dot(P, A.T)) + Q
    return(X,P)

#function to update the measurement

def kf_update(X, P, Y, H, R):
    IM = dot(H, X)
    IS = R + dot(H, dot(P, H.T))
    K = dot(P, dot(H.T, inv(IS)))
    X = X + dot(K, (Y-IM))
    P = P - dot(K, dot(IS, K.T))
    LH = gauss_pdf(Y, IM, IS)
    return (X,P,K,IM,IS,LH)

def gauss_pdf(X, M, S):
    if M.shape[1] == 1:
        DX = X - tile(M, X.shape[1])
        E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)
        E = E + 0.5 * M.shape[0] * log(2 * pi) + 0.5 * log(det(S))
        P = exp(-E)
    elif X.shape[1] == 1:
        DX = tile(X, M.shape[1])- M
        E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)
        E = E + 0.5 * M.shape[0] * log(2 * pi) + 0.5 * log(det(S))
        P = exp(-E)
    else:
        DX = X-M
        E = 0.5 * dot(DX.T, dot(inv(S), DX))
        E = E + 0.5 * M.shape[0] * log(2 * pi) + 0.5 * log(det(S))
        P = exp(-E)
    return (P[0],E[0])



#time step of mobile movement
dt = 0.01
#Initialization of state matrices
X= array([[0.0], [0.0], [0.1], [0.1]])
print(X)
P= diag((0.01, 0.01, 0.01, 0.01))
A= array([[1, 0, dt , 0], [0, 1, 0, dt], [0, 0, 1.1, 0], [0, 0, 0,0.1]])
Q = eye(X.shape[0])
B = eye(X.shape[0])
U = zeros(X.shape[0])


# Measurement matrices
Y = array([[X[0,0] + abs(random.randn(1)[0])], [X[1,0] +abs(random.randn(1)[0])]])
print(abs(random.randn(1)[0]))
H = array([[1, 0, 0, 0], [0, 1, 0, 0]])
R = eye(Y.shape[0])*1


# Number of iterations in Kalman Filter
N_iter = 100

# Applying the Kalman Filter
for i in arange(0, N_iter):
    (X, P) = kf_predict(X, P, A, Q, B, U)
    (X, P, K, IM, IS, LH) = kf_update(X, P, Y, H, R)
    Y = array([[X[0,0] + abs(0.1*random.randn(1)[0])],[X[1, 0] +abs(0.1 * random.randn(1)[0])]])
    plt.plot(X[0,0],X[1,0],'y*')
    plt.plot(Y[0],Y[1],'ro')
    
    
plt.title("Base robot Omnidirezionale")
plt.xlabel(" x [m] ")
plt.ylabel(" y [m] ")
plt.legend(('Estimated','Measured'),'upper center',shadow=True)
plt.grid(True)
plt.show()