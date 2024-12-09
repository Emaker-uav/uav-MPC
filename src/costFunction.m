function J = costFunction(u,X_states,X_ref, Q, R, N,dt_MPC)
    U = reshape(u(1:20,1),size(R,1),N);
    X = zeros(size(X_states,1),N); 
    X(:,1) = X_states;                                   
for k = 1 : N-1
    dX = uavDynamics(X(:, k), U(:, k));
    X(:, k+1) = X(:, k) + dt_MPC*dX ;
end
J = 0; 
for k = 1 : N
    state_cost = (X_ref(:,k) - X(:,k))' * Q * (X_ref(:,k) - X(:,k));
    input_cost = U(:,k)' * R * U(:,k);
    J = J + state_cost + input_cost;
end
end
