# Starter code for those trying to use LQR. Your
# K matrix controller should come from a call to lqr(A,B,Q,R),
# which we have provided. Below this are "dummy" matrices of the right
# type and size. If you fill in these with values you derive by hand
# they should work correctly to call the function.

# Here is the provided LQR function
import scipy.linalg
import numpy as np


def lqr(A, B, Q, R):
    x = scipy.linalg.solve_continuous_are(A, B, Q, R)
    k = np.linalg.inv(R) * np.dot(B.T, x)
    return k


# FOR YOU TODO: Fill in the values for A, B, Q and R here.
# Note that they should be matrices not scalars.
# Then, figure out how to apply the resulting k
# to solve for a control, u, within the policyfn that balances the cartpole.
l = 0.5  # pole length
m = 0.5  # pole mass
M = 0.5  # Cart mass
b = 0.1  # friction constant
g = 9.82  # gravity - I thought it was 9.81 but aight I guess

A = np.array(
    [
        [0, 1, 0, 0],
        [0, -(4 * b) / (m + 4 * M), 0, (3 * m * g) / (m + 4 * M)],
        [0, -6 * b / (l * (m + 4 * M)), 0, 6 *
         ((M + m) * g) / ((m + 4 * M) * l)],
        [0, 0, 1, 0],
    ]
)

B = np.array([[0.0, 4 / (m + 4 * M), 6 / (l * (m + 4 * M)), 0.0]])
B.shape = (4, 1)

Q = np.array([[1, 1, 1, 1], [1, 1, 0, 0], [1, 0, 1, 0], [1, 0, 0, 1]])

R = np.array([[1]])
# print("A holds:", A)
# print("B holds:", B)
# print("Q holds:", Q)
# print("R holds:", R)

# Uncomment this to get the LQR gains k once you have
# filled in the correct matrices.
k = lqr(A, B, Q, R)
# print("k holds:", k)


def policyfn(x):
    val = np.subtract(x, np.array([0, 0, 0, np.pi]))
    u = -np.dot(k, val)
    print(np.array([u]))
    return np.array([u])
