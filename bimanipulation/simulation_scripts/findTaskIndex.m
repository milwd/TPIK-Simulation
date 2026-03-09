function idx = findTaskIndex(task_set, id_to_find)
    % Extract all identifiers into a cell array
    identifiers = cellfun(@(x) x.ID, task_set, 'UniformOutput', false);
    identifiers = cellfun(@char, identifiers, 'UniformOutput', false);
    % Find *all* matching indices (can be multiple)
    idx = find(strcmp(identifiers, id_to_find));
end