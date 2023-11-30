%% [System module]
%  configurate termination condition

function term_config = init_term_config(mode, threshold)
    % sanity check
    switch mode
        case Termination_Method.det
            if length(threshold) ~= 1
                error('Error: more than one threshold provided');
            end
        case Termination_Method.std
            if length(threshold) ~= 1
                error('Error: more than one threshold provided');
            end
        case Termination_Method.eigen
            if length(threshold) ~= 1
                error('Error: more than one threshold provided');
            end
        case Termination_Method.circle
            if length(threshold) ~= 2
                error('Error: not enough threshold (p, r), provided');
            end
        otherwise
            error('Error: Unexpected Termination Method');
    end
    
    term_config.mode = mode;
    term_config.threshold = threshold;
   
end