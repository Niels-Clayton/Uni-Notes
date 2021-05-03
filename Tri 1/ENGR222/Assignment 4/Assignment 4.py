#%% Question 3
from sympy import linsolve, Matrix, symbols
x,y,z,w = symbols("x,y,z,w")
M  = Matrix(([5,-1,2,3,7],
            [-1,3,2,0,-2],
            [0,4,2,-1,1],
            [2,0,-3,7,-3]))
M

#%%
linsolve(M,x,y,z,w)


#%% Question 4
from sympy import linsolve, Matrix, symbols
x,y,z,w = symbols("x,y,z,w")
aug  = Matrix(([1,-1,2,4,2],
             [3,-3,1,2,1],
             [2,-1,1,0,-1],
             [2,-6,1,10,9]))

linsolve(aug,x,y,z,w)

#%% Question 5
from sympy import linsolve, Matrix, symbols
x,y,z,w = symbols("x,y,z,w")
M  = Matrix(([1,2,2,3,0],
             [-4,-8,-8,-9,0],
             [2,4,1,0,3],
             [1,2,-2,-7,4]))
             
linsolve(M,x,y,z,w)

#%% Question 7
from sympy import linsolve, Matrix, symbols
a,b,c,d = symbols("a,b,c,d")
M  = Matrix(([1,0,-1,0,0],
             [4,0,0,-2,0],
             [0,2,-2,-1,0]))
M
#%%
linsolve(M,a,b,c,d)

# %%
Matrix([1,2,1,2]).transpose()

# %%
