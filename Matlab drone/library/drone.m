classdef drone < handle
    properties
        %% --- SECTION 1: PHYSICAL CONSTANTS ---
        g, t, dt, tf, m, l, I, x, r, dr, euler, w, dx, u, T, M
        
        %% --- SECTION 2: PID ERROR VARIABLES ---
        phi_des, phi_err, phi_err_prev, phi_err_sum
        thetta_des, thetta_err, thetta_err_prev, thetta_err_sum
        psi_des, psi_err, psi_err_prev, psi_err_sum
        zdot_des, zdot_err, zdot_err_prev, zdot_err_sum
        
        x_err_sum = 0; x_err_prev = 0;
        y_err_sum = 0; y_err_prev = 0;
        z_err_sum = 0; z_err_prev = 0;
        
        kP_pos = 0.15; kD_pos = 0.4;
        %% --- SECTION 3: PID GAINS ---
        kP_phi, kI_phi, kD_phi
        kP_thetta, kI_thetta, kD_thetta
        kP_psi, kD_psi
        kP_zdot, kI_zdot, kD_zdot
    end
    
    methods
        function obj = drone(params, initStates, initInputs, gains, simTime)
            obj.g = 9.81; obj.t = 0.0; obj.dt = 0.01; obj.tf = simTime;
            obj.m = params('Mass'); obj.l = params('armlength');
            % Ch 2.12: Inertia matrix represents physical agility and moment resistance
            obj.I = [params('Ixx'), 0, 0; 0, params('Iyy'), 0; 0, 0, params('Izz')]; 
            obj.x = initStates(:); 
            obj.dx = zeros(12, 1); 
            obj.r = obj.x(1:3); obj.dr = obj.x(4:6); 
            obj.euler = obj.x(7:9); obj.w = obj.x(10:12); 
            obj.u = initInputs(:); obj.T = obj.u(1); obj.M = obj.u(2:4); 
            
            fields = {'phi_err_sum','phi_err_prev','thetta_err_sum','thetta_err_prev','psi_err_sum','psi_err_prev'};
            for f = 1:length(fields), obj.(fields{f}) = 0; end
            
            obj.kP_phi = gains('P_phi'); obj.kI_phi = gains('I_phi'); obj.kD_phi = gains('D_phi');
            obj.kP_thetta = gains('P_thetta'); obj.kI_thetta = gains('I_thetta'); obj.kD_thetta = gains('D_thetta');
            obj.kP_psi = gains('P_psi'); obj.kD_psi = gains('D_psi');
        end
        
        function state = GetState(obj), state = obj.x; end 

        %% --- SECTION 5: PHYSICS ENGINE (EOM) ---
        function obj = EvalEOM(obj, disturbance)
            if nargin < 2, disturbance = [0;0;0]; end 
            % Ch 4.9: Transformation between Body {B} and Inertial {E} frames via Rotation Matrix
            R = RPY2Rot(obj.euler); 
            
            % Ch 4.10: Translational Dynamics based on Newton’s Second Law
            obj.dx(1:3, 1) = obj.dr; % d(Position) = Velocity
            % Ch 3.1: m*z'' = -mg + F_thrust (expressed in vector form: F = ma)
            obj.dx(4:6, 1) = 1/obj.m * ([0; 0; obj.m*obj.g] + R*obj.T*[0;0;-1]); 
            

             % Ch 2.12: Rotational Dynamics (Euler's Equations) for rigid body motion
            total_moments = obj.M(:) + disturbance(:); 
            obj.dx(10:12, 1) = obj.I \ (total_moments - cross(obj.w(:), obj.I * obj.w(:)));
            
            % Ch 4: Mapping Matrix converts Angular Velocity (p,q,r) to Euler Rates (phi', theta', psi')
            phi = obj.euler(1); thetta = obj.euler(2);
            T_mat = [1, sin(phi)*tan(thetta), cos(phi)*tan(thetta);
                     0, cos(phi), -sin(phi);
                     0, sin(phi)*sec(thetta), cos(phi)*sec(thetta)];
            obj.dx(7:9, 1) = T_mat * obj.w(:);
            
           
        end
        
        %% --- SECTION 6: STATE UPDATE ---
        function obj = UpdateState(obj, disturbance)
            obj.t = obj.t + obj.dt; 
            obj.EvalEOM(disturbance); 
            % Numerical integration to find current state (x = x + dx*dt)
            obj.x = obj.x + obj.dx(:) * obj.dt; 
            obj.r = obj.x(1:3); obj.dr = obj.x(4:6); obj.euler = obj.x(7:9); obj.w = obj.x(10:12);
        end
        
%% --- SECTION 7: ATTITUDE CONTROLLER ---
        function obj = AttitudeCtrl(obj, refSig)
            % refSig = [Thrust, Roll_des, Pitch_des, Yaw_des]
            obj.phi_des = refSig(2); obj.thetta_des = refSig(3); obj.psi_des = refSig(4);
            
            % --- ROLL (Phi) PID ---
            obj.phi_err = obj.phi_des - obj.euler(1); 
            obj.phi_err_sum = obj.phi_err_sum + obj.phi_err * obj.dt; 
            u2 = obj.kP_phi*obj.phi_err + obj.kI_phi*obj.phi_err_sum + ...
                 obj.kD_phi*(obj.phi_err - obj.phi_err_prev)/obj.dt; 
            
            % --- PITCH (Theta) PID ---
            obj.thetta_err = obj.thetta_des - obj.euler(2); 
            obj.thetta_err_sum = obj.thetta_err_sum + obj.thetta_err * obj.dt; 
            u3 = obj.kP_thetta*obj.thetta_err + obj.kI_thetta*obj.thetta_err_sum + ...
                 obj.kD_thetta*(obj.thetta_err - obj.thetta_err_prev)/obj.dt; 
            
            % --- YAW (Psi) PD ---
            u4 = obj.kP_psi*(obj.psi_des - obj.euler(3)) + obj.kD_psi*(0 - obj.w(3));
            
            % Store results
            obj.phi_err_prev = obj.phi_err; 
            obj.thetta_err_prev = obj.thetta_err; 
            
            % Final Outputs to Physics Engine
            obj.u(1) = refSig(1); 
            obj.u(2:4) = [u2, u3, u4]; 
            obj.T = obj.u(1); obj.M = obj.u(2:4);
        end

%% --- SECTION 8: POSITION CONTROLLER ---
        function refSig = PositionCtrl(obj, targetPos)
            % targetPos is [0, 0, -6]
           
            x_err = targetPos(1) - obj.r(1);
            y_err = targetPos(2) - obj.r(2);
            theta_des = (0.12 * x_err) + (0.6 * -obj.dr(1)); 
            phi_des   = -((0.12 * y_err) + (0.6 * -obj.dr(2)));
            
            limit = 12 * (pi/180); 
            theta_des = max(min(theta_des, limit), -limit);
            phi_des   = max(min(phi_des, limit), -limit);
            
            % 2. Z Control 
            % Z is r(3). 
            z_alt_error = obj.r(3) - targetPos(3); 
            
            
            if ~isprop(obj, 'z_err_sum'), obj.addprop('z_err_sum'); end
            obj.z_err_sum = obj.z_err_sum + z_alt_error * obj.dt;
            
            T_correction = (10.0 * z_alt_error) + (2.0 * obj.z_err_sum) + (5.0 * obj.dr(3));
            
            % Total Thrust = Baseline to hover + Correction
            T_total = (obj.m * obj.g) + T_correction; 
            
            % Prevent the motors from turning off or exploding
            T_total = max(min(T_total, 30), 0); 
            
            refSig = [T_total, phi_des, theta_des, 0]; 
        end
    end
end