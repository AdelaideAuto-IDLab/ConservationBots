function z = remove_invalid_id(pulses, id_list)
    if ~isempty(pulses)
        valid_id = boolean(zeros(height(pulses), 1));
        for n = 1:height(pulses)
            if sum(ismember(id_list, pulses.Target_ID(n)))
                valid_id(n) = true;
            end
        end
        z = pulses(valid_id, :);
    else
        z = [];
    end
end