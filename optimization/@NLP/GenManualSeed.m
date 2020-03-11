function [q_seed,qd_seed,qdd_seed,t_seed] = GenManualSeed(obj, rbm)

arguments 
    obj (1,1) NLP
    rbm (1,1) DynamicalSystem
end
    

% Guess initial pose
[qGuess, tGuess] = Opt.ManualInit();


validateattributes(tGuess, {'double'}, {'size', [1,1]})
mustBePositive(tGuess)

validateattributes(qGuess, {'double'}, {'size', [numel(rbm.States.q.sym),3]})

ncp = obj.Settings.ncp;

t_seed = linspace(0,tGuess,ncp);


t_seed1 = t_seed(1:ceil(ncp/2));
[q_start2half, qd_start2half] = InterpSeed1(obj, t_seed1, qGuess(:,1), qGuess(:,2));

% interpolate from t_half to t_end

t_seed2 = t_seed(ceil(ncp/2):end);
[q_half2end, qd_half2end] = InterpSeed2(obj, t_seed2, qGuess(:,2), qGuess(:,3));

q_seed = [q_start2half, q_half2end(:,2:end)];
qd_seed = [qd_start2half, qd_half2end(:,2:end)];

validateattributes(q_seed, {'double'}, {'size',[numel(rbm.States.q.sym), ncp]})        
validateattributes(qd_seed, {'double'}, {'size',[numel(rbm.States.q.sym), ncp]})        


qdd_seed = zeros(numel(rbm.States.q.sym),ncp);
for i = 1:numel(rbm.States.q.sym)
    qdd_seed(i,:) = gradient(qd_seed(i,:),t_seed);
end

                

end
