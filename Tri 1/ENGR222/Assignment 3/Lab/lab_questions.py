import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumtrapz
from scipy.interpolate import UnivariateSpline
from scipy.integrate import dblquad
from scipy.integrate import tplquad


def a_i():
    theta = lambda u : np.pi * np.sin( np.log(1 + u**2))
    
    n = 1001
    s = np.linspace(0, 10, n)
    x = cumtrapz(np.cos(theta(s)), s, initial=0)
    y = cumtrapz(np.sin(theta(s)), s, initial=0)

    print(f"The coordinates at s = {s[-1]} are: $(x,y) = ({x[-1]},{y[-1]})$")

    plt.plot(x,y)
    plt.grid()
    plt.show()


def a_ii():
    theta = lambda u : np.pi * np.sin( np.log(1 + u**2))

    s_0 = 5
    steps = [0.01, 0.005, 0.001, 0.0005]

    for h in steps:
        df = (theta(s_0 + h) - theta(s_0 - h))/(2 * h)
        
        curvature = np.abs(df)
        print(f"For $h = {h}$ : $K(s=5) = {curvature}$")


def b_i():
    ti = [0.0, 0.6 , 1.1 , 1.5 , 1.8 , 2.1 , 2.3 , 2.5 , 2.8 , 3.2 ]
    xi = [0.0,-0.44,-0.69,-0.63,-0.62,-0.55,-0.63,-0.67,-0.44,-0.10]
    yi = [0.0,-0.15,-0.41,-0.15,-0.11,-0.31,-0.20,-0.15,-0.23,-0.21]
    zi = [0.0, 0.36, 0.11, 0.09, 0.16, 0.06,-0.04, 0.12, 0.01, 0.05]

    f = UnivariateSpline(ti,xi,s=0)
    g = UnivariateSpline(ti,yi,s=0)
    h = UnivariateSpline(ti,zi,s=0)    
    t = np.linspace(ti[0],ti[-1],101)

    print(f"The coordinates at t = 2 are: $(x,y,z) = ({f(2)},{g(2)},{h(2)})$")

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(f(t),g(t),h(t))
    ax.plot(f(2),g(2),h(2), 'o')
    ax.plot(xi,yi,zi,'o')

    plt.show()

def b_ii():
    ti = [0.0, 0.6 , 1.1 , 1.5 , 1.8 , 2.1 , 2.3 , 2.5 , 2.8 , 3.2 ]
    xi = [0.0,-0.44,-0.69,-0.63,-0.62,-0.55,-0.63,-0.67,-0.44,-0.10]
    yi = [0.0,-0.15,-0.41,-0.15,-0.11,-0.31,-0.20,-0.15,-0.23,-0.21]
    zi = [0.0, 0.36, 0.11, 0.09, 0.16, 0.06,-0.04, 0.12, 0.01, 0.05]

    f = UnivariateSpline(ti,xi,s=0)
    g = UnivariateSpline(ti,yi,s=0)
    h = UnivariateSpline(ti,zi,s=0)    
    t = np.linspace(ti[0],ti[-1],101)

    r = lambda t:np.array([f(t),g(t),h(t)]).T
    dfdt = f.derivative()
    dgdt = g.derivative()
    dhdt = h.derivative()
    v = lambda t:np.array([dfdt(t),dgdt(t),dhdt(t)]).T
    d2fdt2 = dfdt.derivative()
    d2gdt2 = dgdt.derivative()
    d2hdt2 = dhdt.derivative()
    a = lambda t:np.array([d2fdt2(t),d2gdt2(t),d2hdt2(t)]).T

    rk = r(2)
    vk = v(2)
    ak = a(2)

    Tk = vk/np.linalg.norm(vk)
    Nk = ak*np.dot(vk,vk)-vk*np.dot(ak,vk)
    Nk /= np.linalg.norm(Nk)
    Bk = np.cross(Tk,Nk)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d') 
    ax.plot([rk[0],rk[0]+Tk[0]],[rk[1],rk[1]+Tk[1]],[rk[2],rk[2]+Tk[2]],'k-')
    ax.plot([rk[0],rk[0]+Nk[0]],[rk[1],rk[1]+Nk[1]],[rk[2],rk[2]+Nk[2]],'r-')
    ax.plot([rk[0],rk[0]+Bk[0]],[rk[1],rk[1]+Bk[1]],[rk[2],rk[2]+Bk[2]],'g-')
    ax.plot(f(t),g(t),h(t))
    plt.show()

def c_i():

    f = lambda y, x : np.cos(x) * np.exp(y)
    g1 = lambda x : x**2
    g2 = lambda x : np.sin(x) + 10

    print(f"The integral evaluates to: {dblquad(f,-3, 3, g1, g2)[0]}")
    

def c_ii():

    f = lambda x,y,z : 4 / (1 + x**2 + y**2 + z**2)
    def F(r,t,p):
        x = r*np.cos(t)*np.sin(p)
        y = r*np.sin(t)*np.sin(p)
        z = r*np.cos(p)
        return f(x,y,z)*r**2*np.sin(p)

    print(f"The integral evaluates to: {tplquad(F,0,np.pi,0,2*np.pi,0,1)[0]}")

c_ii()