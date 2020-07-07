function [u, u_dot, u_ddot] = deriv_unit_vector(q, q_dot, q_ddot)

nq = norm(q);
u = q / nq;
u_dot = q_dot / nq - q * dot(q, q_dot) / nq^3;

u_ddot = q_ddot / nq - q_dot / nq^3 * (2 * dot(q, q_dot))...
    - q / nq^3 * (dot(q_dot, q_dot) + dot(q, q_ddot))...
    + 3 * q / nq^5 * dot(q, q_dot)^2;

end