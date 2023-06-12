function  state = computeStateFromCartesian(p)
global p_a1 p_a2
    psi = atan2(p(1), -p(3));
    l1 = norm(p - p_a1);
    l2 = norm(p - p_a2);
    phid = 0;
    l1d = 0.0;
    l2d = 0.0;
    state = [psi; l1; l2; phid; l1d; l2d];
end