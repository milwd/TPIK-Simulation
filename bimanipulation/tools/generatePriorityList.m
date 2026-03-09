function unifiedList = generatePriorityList(A1, A2)
    % generateUnifiedList generates a merged priority list with transition states
    % inputs:
    %   A1: cell array of task names (previous action) e.g., {'a', 'b', 'd'}
    %   A2: cell array of task names (current action)  e.g., {'a', 'c', 'b', 'd'}
    % output:
    %   unifiedList: a struct containing id and type
    %   type 0  :  Constant   (active in both)
    %   type -1 :  Decreasing (fading out)
    %   type 1  :  Increasing (fading in )

    n1 = length(A1);
    n2 = length(A2);
    maxLevels = max(n1, n2);

    unifiedList = struct('id', {}, 'type', {});
    count = 1;

    for k = 1:maxLevels
        if k <= n1
            t1 = A1{k};
        else
            t1 = '';
        end

        if k <= n2
            t2 = A2{k};
        else
            t2 = '';
        end

        % the LOGIC
        
        if strcmp(t1, t2)
            unifiedList(count).id = t1;
            unifiedList(count).type = 0; 
            count = count + 1;
            
        else
            if ~isempty(t1)
                unifiedList(count).id = t1;
                unifiedList(count).type = -1;
                count = count + 1;
            end
            
            if ~isempty(t2)
                unifiedList(count).id = t2;
                unifiedList(count).type = 1; 
                count = count + 1;
            end
        end
    end
end