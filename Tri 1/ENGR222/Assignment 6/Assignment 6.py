# %% 
from sympy import Matrix
from sympy.functions.elementary.miscellaneous import sqrt

# %% 1
A = Matrix([[-1,2],[2,-3],[-1,3]])
v = Matrix([4,1,2])
x = A.pinv()*v
x[0]*x[1]

# %% 2
A = Matrix([[1,1,0],[1,1,0],[1,0,1],[1,0,1]])
v = Matrix([1,3,8,2])
x = A.pinv()*v
x

# %% 3
A = Matrix([[1,1,0],[1,1,0],[1,0,1],[1,0,1]])
v = Matrix([3,2,1,1])
x = A.pinv()*v
d = v-(A*x)
dist = sqrt(d[0]**2+d[1]**2+d[2]**2+d[3]**2)
dist

# %% 4
A = Matrix([[2,1],[3,1],[5,1],[8,1]])
v = Matrix([4,2,1,-1])
x = A.pinv()*v
x
# %% 5