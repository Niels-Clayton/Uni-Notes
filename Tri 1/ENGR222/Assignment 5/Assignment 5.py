#%% 
from sympy import Matrix

M = Matrix([ [-2, 1, -1], [19, -5, 4], [43, -13, 12] ])
a = M.eigenvals()
v = M.eigenvects()


# %%
