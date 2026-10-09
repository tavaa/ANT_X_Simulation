   import casadi.*
   x   = MX.sym('x', 2);
   nlp = struct('x', x, 'f', (x(1)-1)^2 + (x(2)-2)^2);
   S   = nlpsol('S', 'ipopt', nlp);
   sol = S('x0', [0; 0]);
   disp(full(sol.x))   % deve stampare [1; 2]