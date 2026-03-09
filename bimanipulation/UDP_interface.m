classdef UDP_interface < handle
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        hudprLeft
        hudprRight
        hudpsLeft
        hudpsRight
        hudps
        setup
    end

    methods
        function obj = UDP_interface(setup)
            obj.setup=setup;
            if obj.setup == true
                obj.hudprLeft = dsp.UDPReceiver('LocalIPPort',1501,'MaximumMessageLength',255);
                obj.hudprRight = dsp.UDPReceiver('LocalIPPort',1503,'MaximumMessageLength',255);
                obj.hudpsLeft = dsp.UDPSender('RemoteIPPort',1500);
                obj.hudpsLeft.RemoteIPAddress = '127.0.0.1';
                obj.hudpsRight = dsp.UDPSender('RemoteIPPort',1502);
                obj.hudpsRight.RemoteIPAddress = '127.0.0.1';
            else
                obj.hudps = dsp.UDPSender('RemoteIPPort',1505);
                obj.hudps.RemoteIPAddress = '127.0.0.1';
            end
        end

        function [qL,qR] = udp_receive(obj,t)
            if obj.setup == true
                dataLeft = step(obj.hudprLeft);
                dataRight = step(obj.hudprRight);
                % wait for data (to decide)
                if t == 0
                    while(isempty(dataLeft))
                        dataLeft = step(obj.hudprLeft);
                        pause(deltat);
                    end
                    while(isempty(dataRight))
                        dataRight = step(obj.hudprRight);
                        pause(deltat);
                    end
                end
                qL = typecast(dataLeft, 'double');
                qR = typecast(dataRight, 'double');
            else
                qL=zeros(1,7);
                qR=zeros(1,7);
            end
        end
        function send(obj,t,bms)
            if obj.setup  == true
                step(obj.hudpsLeft,[t;bms.left_arm.qdot]);
                step(obj.hudpsRight,[t;bms.right_arm.qdot]);
            else
                step(obj.hudps,[bms.left_arm.q,bms.right_arm.q])
            end
        
        end
    end
end