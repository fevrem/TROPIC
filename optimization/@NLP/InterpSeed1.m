function [q_start2half,qd_start2half] = InterpSeed1(obj, t1, q_start, q_end)

% -------------------- interpolate between t=0 & intm ------------------- %
q_start2half  = zeros(numel(q_start), numel(t1));
qd_start2half = zeros(numel(q_start), numel(t1));

for m = 1:numel(q_start)
    q_start2half(m,:)  = q_start(m) + t1/(t1(end)-t1(1))*(q_end(m)-q_start(m));
    qd_start2half(m,:) = 1/(t1(end)-t1(1))*(q_end(m)-q_start(m)) * ones(1,length(t1));
end



end