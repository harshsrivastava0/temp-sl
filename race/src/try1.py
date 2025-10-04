from scipy.linalg import block_diag
import numpy as np

# Define A and B (based on your matrices; please substitute placeholders with actual values)
A = np.array([[ 0, 0, 0, 0.96445057,  0, 0, 0],
              [ 0, 0, 0, -0.2642633,  0, 0, 0],
              [ 0, 0, 0, 0, 0, 0, 0 ],
              [ 0, 0, 0, 0, 0, 0, 0 ],
              [ 0, 0, 0, 0, 0, 0, 0 ],
              [ 0, 0, 0,-0, 0, 0, 0 ],
              [ 0, 0, 0, 0, 0, 0, 0 ]])
B = np.array([[0,         0        ],
 [0,         0        ],
 [1,         0        ],
 [0,         1        ],
 [0,         0        ],
 [0,         0        ],
 [0.51923077, 0       ]])

# Compute controllability matrix
ctrb_matrix = B
for i in range(1, A.shape[0]):
    ctrb_matrix = np.hstack((ctrb_matrix, np.linalg.matrix_power(A, i) @ B))

# Check rank
ctrb_rank = np.linalg.matrix_rank(ctrb_matrix)
print("Controllability Matrix Rank:", ctrb_rank)

eigvals = np.linalg.eigvals(A)
print("Eigenvalues of A:", eigvals)

unstable_eigvals = [ev for ev in eigvals if np.real(ev) > 0]
print("Unstable Eigenvalues of A:", unstable_eigvals)
