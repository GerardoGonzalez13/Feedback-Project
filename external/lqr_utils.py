import numpy as np

try:
    from scipy.linalg import eigvals, solve_continuous_are
except Exception as exc:  # pragma: no cover - import-time dependency guard
    raise RuntimeError(
        "Continuous-time LQR requires scipy.linalg.solve_continuous_are, "
        "but SciPy is not available in this environment."
    ) from exc


def continuous_lqr(a_matrix, b_matrix, q_matrix, r_matrix):
    """
    Solve the continuous-time LQR problem for x_dot = A x + B u.

    Returns the state-feedback gain K, Riccati matrix P, and closed-loop
    eigenvalues of A - B K.
    """
    p_matrix = solve_continuous_are(a_matrix, b_matrix, q_matrix, r_matrix)
    k_matrix = np.linalg.solve(r_matrix, b_matrix.T @ p_matrix)
    closed_loop_eigs = eigvals(a_matrix - b_matrix @ k_matrix)
    return k_matrix, p_matrix, closed_loop_eigs
