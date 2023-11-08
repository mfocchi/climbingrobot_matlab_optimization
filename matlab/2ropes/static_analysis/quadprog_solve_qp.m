
%minimize (1/2)x.T*G*x+g*x
%s.t. Cx≤d
%     Ax=b

function [X,FVAL,EXITFLAG] = quadprog_solve_qp(G, g, C, d, A, b)

% matlab quadprog requires
%min 0.5*x'*H*x + f'*x   subject to:  A*x <= b 
%            x    
 
if nargin >7
    disp('wrong number  of arguments');
    return
end

H = G;
f = g';

if (isempty(C) && isempty(d) && isempty(A) && isempty(b))
   disp('Unconstrained');
   [X,FVAL,EXITFLAG] = quadprog(H,f,[],[],[],[]);


elseif ( isempty(A) && isempty(b))
    ineq_matrix = C;
    ineq_vector = d;
    disp('ineq constraint set');
    [X,FVAL,EXITFLAG] = quadprog(H,f,ineq_matrix,ineq_vector,[],[]);
 
    
elseif (isempty(C) && isempty(d))

    Aeq = A;
    beq= b;
    disp('eq  constraint set');
    [X,FVAL,EXITFLAG] = quadprog(H,f,[],[],Aeq,beq);
    
else
    ineq_matrix = C;
    ineq_vector = d;
    Aeq = A;
    beq= b;
    %disp('eq /ineq constraint set');
    [X,FVAL,EXITFLAG] = quadprog(H,f,ineq_matrix,ineq_vector,Aeq,beq);
    
  

end