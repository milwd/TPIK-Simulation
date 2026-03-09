classdef Task < handle

    properties
        xdotbar = [] % reference task velocity
        J = []       % task Jacobian
        A = []       % task internal activation function
        ID
        task_name
    end

    methods (Abstract)
        updateReference(obj)
        updateJacobian(obj)
        updateActivation(obj)
    end
end