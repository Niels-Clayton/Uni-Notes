#%% Question 3
from sympy import linsolve, Matrix, symbols
x,y,z,w = symbols("x,y,z,w")
M  = Matrix(([3,3,-1,-2,0],
            [2,2,1,2,5],
            [1,1,0,0,1]))
linsolve(M,x,y,z,w)



#%% Question 5
from sympy import linsolve, Matrix, symbols
x,y,z,w = symbols("x,y,z,w")
M = Matrix(([29,-16,-58,-9],
            [-16,38,32,27],
            [-58,32,116,18]))
linsolve(M,x,y,z)

M.rref()# %%


#%% Question 7

A =  Matrix(([1,0],
            [1,-1],
            [2,1]))
v =  Matrix(([-2],
            [1],
            [4]))

x = A.pinv() * v
x
# %%
