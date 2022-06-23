function int_targets = compute_intermediate_targets(target, N)

    for i=1:N
        int_targets(:,i) = target/N*i;
    end

end