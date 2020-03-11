function [nlp, rbm] = CheckFeasibility(nlp, rbm, data)

if ~all(all(rbm.States.q.LowerBound < data.q))
    warning('Initial guess: seed.q < q.LB')
end
if ~all(all(rbm.States.q.UpperBound > data.q))
    warning('Initial guess: seed.q > q.UB')
end
rbm.States.q.Seed = data.q;


if ~all(all(rbm.States.dq.LowerBound < data.qd))
    warning('Initial guess: seed.qd < qd.LB')
end
if ~all(all(rbm.States.dq.UpperBound > data.qd))
    warning('Initial guess: seed.qd > qd.UB')
end
rbm.States.dq.Seed = data.qd;


if ~all(all(rbm.States.ddq.LowerBound < data.qdd))
    warning('Initial guess: seed.qdd < qdd.LB')
end
if ~all(all(rbm.States.ddq.UpperBound > data.qdd))
    warning('Initial guess: seed.qdd > qdd.UB')
end
rbm.States.ddq.Seed = data.qdd;


if ~all(all(rbm.Inputs.u.LowerBound < data.u))
    warning('Initial guess: seed.u < u.LB')
end
if ~all(all(rbm.Inputs.u.UpperBound > data.u))
    warning('Initial guess: seed.u > u.UB')
end
rbm.Inputs.u.Seed = data.u;


if ~(nlp.Problem.FinalTime.LowerBound < data.t(end))
    warning('Initial guess: seed.tf < tf.LB')
end
if ~(nlp.Problem.FinalTime.UpperBound > data.t(end))
    warning('Initial guess: seed.tf > tf.UB')
end
nlp.Problem.FinalTime.Value = data.t(end);


if nlp.Problem.SlewRate.Bool
    if ~all(all(rbm.Inputs.du.LowerBound < data.du))
        warning('Initial guess: seed.du < du.LB')
    end
    if ~all(all(rbm.Inputs.du.UpperBound > data.du))
        warning('Initial guess: seed.du > du.UB')
    end
    rbm.Inputs.du.Seed = data.du;
end


if nlp.Problem.Trajectory.Bool
    if ~all(all(nlp.Problem.Trajectory.PolyCoeff.LowerBound < data.a))
        warning('Initial guess: seed.a < a.LB')
    end
    if ~all(all(nlp.Problem.Trajectory.PolyCoeff.UpperBound > data.a))
        warning('Initial guess: seed.a > a.UB')
    end
    nlp.Problem.Trajectory.PolyCoeff.Seed = data.a;
end


for i = 1:numel(rbm.Contacts)
    if ~all(all(rbm.Contacts{i}.Fc.LowerBound < data.(['Fc_', num2str(i)])))
        warning('Initial guess: seed.Fc < Fc.LB')
    end
    if ~all(all(rbm.Contacts{i}.Fc.UpperBound > data.(['Fc_', num2str(i)])))
        warning('Initial guess: seed.Fc > Fc.UB')
    end
    rbm.Contacts{i}.Fc.Seed = data.(['Fc_', num2str(i)]);
end

end