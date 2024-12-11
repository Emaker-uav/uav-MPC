function Fu = uavNMPC(X_states,X_des,Q,R,N,dT_MPC)
tic
global u0;
X_ref = generateReference(X_states,X_des,N);
fun = @(u)costFunction(u,X_states,X_ref, Q, R, N,dT_MPC);
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
Fu = fmincon(fun,u0,[],[],[],[],[],[],[],options);
u0 = Fu;
toc
end
%% 
function x_ref = generateReference(x,x_des,h)
    x_ref = zeros(size(x,1), h);
    for i = 1 : size(x,1)
        x_ref(i,:) = linspace(x(i), x_des(i), h);
    end
end

