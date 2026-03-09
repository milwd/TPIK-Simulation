classdef UnityInterface < handle
    %UNITYINTERFACE Handles UDP communication with Unity viewer.
    %   Encapsulates setup, sending, and cleanup of UDP connections for
    %   both the arm and vehicle, as well as rotation handling.
    
    properties (Access = private)
        uArm         % arm joints socket
        uVehicle     % vehicle pose socket
        uAltitude    % altitude feedback socket
        wuRw         % world rotation (unity vs sim)
        vRvu         % vehicle rotation (unity vs sim)
        unityIP      % target IP address
        armPort      % target port for arm
        vehiclePort  % target port for vehicle
        altitudePort % local port for receiving altitude
    end
    
    methods
        function obj = UnityInterface(unityIP)
            if nargin < 1
                obj.unityIP = "127.0.0.1";
            else
                obj.unityIP = unityIP;
            end

            % Define ports
            obj.armPort     = 15000;
            obj.vehiclePort = 15001;
            obj.altitudePort= 15003;

            % Rotation matrices
            obj.wuRw = rotation(0, -pi/2, pi/2);
            obj.vRvu = rotation(-pi/2, 0, -pi/2);

            % Initialize UDP ports
            obj.uArm = udpport("datagram", "IPV4", "ByteOrder", "big-endian", 'OutputDatagramSize', 28);
            obj.uVehicle = udpport("datagram", "IPV4", "ByteOrder", "big-endian", 'OutputDatagramSize', 24);
            obj.uAltitude = udpport("datagram", "IPV4", "LocalPort", obj.altitudePort);
        end

       function send(obj, uvms)
            %SEND Sends arm and vehicle data to Unity.

            % --- Arm data ---
            packetArm = single(uvms.q);
            write(obj.uArm, packetArm, "single", obj.unityIP, 15000);

            % --- Vehicle rotation ---
            unity_vehicle = obj.wuRw * uvms.wTv(1:3,1:3) * obj.vRvu;
            [r, p, y] = RotMatrix2RPY(unity_vehicle);

            % --- Vehicle translation ---
            trasl = uvms.wTv(1:3,4);
            unity_vehicle_trasl = obj.wuRw * trasl;

            packetVehicle = single([ ...
                unity_vehicle_trasl(1), ...
               -unity_vehicle_trasl(2), ...
                unity_vehicle_trasl(3), ...
               -r, p, -y]);

            write(obj.uVehicle, packetVehicle, "single", obj.unityIP, 15001);
        end

        function altitude = receiveAltitude(obj, uvms)
            %RECEIVEALTITUDE Reads one altitude measurement from Unity
            altitude = uvms.altitude;  % default: keep previous value

            if obj.uAltitude.NumDatagramsAvailable > 0
                % Read all available datagrams
                dgrams = read(obj.uAltitude, obj.uAltitude.NumDatagramsAvailable, "single");

                % Convert last datagram back to single
                latestData = typecast(dgrams(end).Data, 'double');
                sensorDistance = latestData(end);  % last float

                % Transform to world frame
                w_kw = [0 0 1]';
                v_kw = uvms.vTw(1:3,1:3) * w_kw;
                altitude = v_kw' * [0 0 double(sensorDistance)]';
            end
        end

        function delete(obj)
            % Destructor: cleanly release UDP ports
            try
                clear obj.uArm obj.uVehicle obj.uAltitude
            catch
            end
        end
    end
end