function a = generate_trajectory(q0, qf, q0_dot, qf_dot, t0, tf)
    syms a0 a1 a2 a3;
    % A x a = q
    % where A = 4x4 cubic polynomial matrix, 
    % a = a0, a1, a2, a3
    % q = q0 q0_dot qf qf_dot

    % a = inv(A) * q

    A = [1  t0    t0^2   t0^3; 
         0  1     2*t0   3*t0^2; 
         1  tf    tf^2   tf^3; 
         0  1     2*tf   3*tf^2];
    
    q = [q0 qf q0_dot qf_dot]';

    a = [a0; a1; a2; a3];

    sol = solve([A*a==q],a);
    
    a = [sol.a0; sol.a1; sol.a2; sol.a3];

end
