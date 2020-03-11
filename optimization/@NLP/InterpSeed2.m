function [q_half2end,qd_half2end] = InterpSeed2(obj, t2, q_start, q_end)

% ------------------ interpolate between midpoint & end ----------------- %
q_half2end  = zeros(numel(q_start), numel(t2));
qd_half2end = zeros(numel(q_start), numel(t2));
for m = 1:numel(q_start)
    q_half2end(m,:)  = q_start(m) + (t2-t2(1))/(t2(end)-t2(1))*(q_end(m)-q_start(m));
    qd_half2end(m,:) = 1/(t2(end)-t2(1))*(q_end(m)-q_start(m)) * ones(1,length(t2));
end


end