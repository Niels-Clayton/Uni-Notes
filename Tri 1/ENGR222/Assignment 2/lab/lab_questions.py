import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import quad

# Question a 
# Section i
def q_a_i():
    f = lambda x  : np.exp(np.cos(np.pi * x**2))

    x_0 = 1/np.sqrt(2)
    h = 0.1**np.linspace(1, 18, 100)
    df = (f(x_0 + h) - f(x_0))/h
    error = np.abs(df - (-np.sqrt(2) * np.pi))
    
    print("The step size that provides minimum error is: " + str(h[np.argmin(error)]))
    print("The minimum error is: " + str(error[np.argmin(error)]))

    plt.loglog(h, error)
    plt.gca().invert_xaxis()
    plt.grid()
    plt.xlabel("Step Size [h]")
    plt.ylabel("Error")
    plt.show()

# Section ii
def q_a_ii():
    f = lambda x  : np.exp(np.cos(np.pi * x**2))

    x_0 = 1/np.sqrt(2)
    h = 0.1**np.linspace(1, 18, 100)
    df = (f(x_0 + h) - f(x_0-h))/(2*h)
    error = np.abs(df - (-np.sqrt(2) * np.pi))
    
    print("The step size that provides minimum error is: " + str(h[np.argmin(error)]))
    print("The minimum error is: " + str(error[np.argmin(error)]))

    plt.loglog(h, error)
    plt.gca().invert_xaxis()
    plt.grid()
    plt.xlabel("Step Size [h]")
    plt.ylabel("Error")
    plt.show()

# Section iii
def q_a_iii():
    f = lambda x  : np.exp(np.cos(np.pi * x**2))

    x_0 = 1/np.sqrt(2)
    h = 0.1**np.linspace(1, 18, 100)
    ddf = ( f(x_0 + h) - 2*f(x_0) + f(x_0-h) )/h**2
    error = np.abs(ddf - (2*np.pi * (np.pi - 1)))
    
    print("\nThe step size that provides minimum error is: " + str(h[np.argmin(error)]))
    print("The minimum error is: " + str(error[np.argmin(error)]))

    plt.loglog(h, error)
    plt.gca().invert_xaxis()
    plt.grid()
    plt.xlabel("Step Size [h]")
    plt.ylabel("Error")
    plt.show()

# Question B
# Section i

def q_b_i():
    f = lambda x : x * np.exp(-np.sqrt(x))
    
    a = 0
    b = 10
    steps = np.array([10,20,40,80,160])
    
    for n in steps:
        x = np.linspace(a, b, n+1)
        h = np.abs(a-b) / n
        y = f(x)
        trap_rule = (y[1:]+y[:-1]).sum()*h/2
        print(f"For a sub-interval of {n} the integral is {trap_rule}")


def q_b_ii():
    f = lambda x : x * np.exp(-np.sqrt(x))
    
    a = 0
    b = 10
    steps = np.array([10,20,40,80,160])
    
    for n in steps:
        x = np.linspace(a, b, n+1)
        h = np.abs(a-b) / n
        y = f(x)
        simp_rule = h/3*(y[0]+y[-1])+4*h/3*y[1::2].sum()+2*h/3*y[2:-1:2].sum()
        print(f"For a sub-interval of {n} the integral is {simp_rule}")

def q_b_iii():
    f = lambda x : x * np.exp(-np.sqrt(x))
    
    a = 0
    upper = np.array([10.0, 100.0, 1000.0])

    for b in upper:
        print(f"At the upper bound of {b} the integral evaulates to {quad(f,a,b)[0]}")
    
    print(f"At an infinite upper bound the integral evaluates to {quad(f,a,np.inf)[0]}")



q_a_i()
q_a_ii()
q_a_iii()

q_b_i()
q_b_ii()
q_b_iii()
