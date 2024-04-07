classdef MX_RobotSpatialDyn < handle
    properties
        tree
        q
        dq
        S
        XBase
        VBody
        Xup          
    end
    
    methods
        function obj = MX_RobotSpatialDyn(model)
            obj.tree = model;
            obj.q = [];
            obj.dq = [];
            obj.S = [];
            obj.XBase = [];
            obj.VBody = [];
            obj.Xup = [];
        end
                
        function transFloatBase(obj)            
            obj.tree = mx_floatBase_New(obj.tree);            
        end
        
        function updateRobState(obj, q_rel, dq_rel)
            obj.q = q_rel;
            obj.dq = dq_rel;
            [obj.S, obj.XBase, obj.VBody, obj.Xup] = mx_getCommomProperty(obj.tree, obj.q, obj.dq);
        end
        
        function [Pos, R] = getPointWorldPosRotm(obj, bodyN, rotAxis, pos_w)
            rotR = getRotRuseAxis(rotAxis);
            pos = rotR * pos_w;

            X_0_b = obj.XBase{bodyN};

            R_0_b = X_0_b(1:3, 1:3);
            R_b_0 = R_0_b';

            temp1 = X_0_b(4:6, 1:3);
            P_b_0 = mx_skewInv((R_b_0 * temp1)');

            Pos = P_b_0 + R_b_0 * pos;

            if nargout > 1
                R = R_b_0 * rotR; 
            end
        end
        
        function VEL = getPointWorldVel(obj, bodyN, rotAxis, pos_w)
            Pos = obj.getPointWorldPosRotm(bodyN, rotAxis, pos_w);
            Vb = obj.XBase{bodyN} \ obj.VBody{bodyN};

            Omega = Vb(1:3);

            Vel = Vb(4:6) + mx_skew(Omega) * Pos; 

            VEL = [Omega; Vel];            
        end
        
        function Jac = getPointWorldJac(obj, bodyN, rotAxis, pos_w)
            Pos = obj.getPointWorldPosRotm(bodyN, rotAxis, pos_w);
            POS = Xtrans(Pos);

            e = obj.getTreeNodes(bodyN);
            NB = obj.tree.NB;
            Jac = zeros(6, NB);

            for i = 1:NB
                if e(i)
                    Jac(:, i) = POS * obj.XBase{i}^(-1) * obj.S{i};
                end    
            end            
        end
        
        function dJac = getPointWorldJacDot(obj, bodyN, rotAxis, pos_w)
            Pos = obj.getPointWorldPosRotm(bodyN, rotAxis, pos_w);
            POS = Xtrans(Pos);

            Vel_6D = obj.getPointWorldVel(bodyN, rotAxis, pos_w);
            Vel = Vel_6D(4:6);

            VEL = [zeros(3, 6); -mx_skew(Vel), zeros(3)];
            
            e = obj.getTreeNodes(bodyN);

            NB = obj.tree.NB;
            dJac = zeros(6, NB);
            for i = 1:NB
                if e(i)
                    X_i_0 = obj.XBase{i}^(-1);
                    dX = X_i_0 * mx_vCross(obj.VBody{i});      
                    dJac(:, i) = VEL * X_i_0 * obj.S{i} + POS * dX * obj.S{i};
                end    
            end            
        end
         
        function [cG, hG, AG, mass, vG, X_G_0, IG] = getCentroidalProperty(obj)
            I_0_C = zeros(6, 6);
            Ic = obj.tree.I;

            NB = obj.tree.NB;
            for i = NB:-1:1
                j = obj.tree.parent(i);
                if j == 0
                    I_0_C = I_0_C + (obj.Xup{i})' * Ic{i} * obj.Xup{i};
                else
                    Ic{j} = Ic{j} + (obj.Xup{i})' * Ic{i} * obj.Xup{i};  
                end
            end

            mass = I_0_C(6, 6);

            cG = mx_skewInv(I_0_C(1:3, 4:6)/mass);

            X_G_0 = [eye(3), zeros(3); mx_skew(cG), eye(3)];

            hG = zeros(6, 1);
            AG = zeros(6, NB);

            for i = 1:NB
                X_G_i = obj.XBase{i} * X_G_0;

                AG(:, i) = (X_G_i)' * Ic{i} * obj.S{i};

                hG = hG + AG(:, i) * obj.dq(i);    
            end

            IG = X_G_0' * I_0_C * X_G_0; 

            vG = IG \ hG;         
        end
        
        function [dAG, AG] = getAGDot(obj)
            I_0_C = zeros(6, 6);
            Ic = obj.tree.I;

            NB = obj.tree.NB;
            dIc = cell(1, NB);

            for i = 1:NB
                dIc{i} = zeros(6, 6);
            end

            for i = NB:-1:1
                j = obj.tree.parent(i);
                if j == 0
                    I_0_C = I_0_C + (obj.Xup{i})' * Ic{i} * obj.Xup{i};
                else
                    Ic{j} = Ic{j} + (obj.Xup{i})' * Ic{i} * obj.Xup{i};
                    dXup = obj.Xup{i} * mx_vCross(obj.VBody{j}) - mx_vCross(obj.VBody{i}) * obj.Xup{i};
                    dIc{j} = dIc{j} + dXup'* Ic{i} * obj.Xup{i} + (obj.Xup{i})' * dIc{i} * obj.Xup{i} + (obj.Xup{i})' * Ic{i} * dXup;
                end
            end
            
            mass = I_0_C(6, 6);
            cG = mx_skewInv(I_0_C(1:3, 4:6)/mass);

            X_G_0 = [eye(3), zeros(3); mx_skew(cG), eye(3)];

            hG = zeros(6, 1);
            AG = zeros(6, NB);
            
            for i = 1:NB
                X_G_i = obj.XBase{i} * X_G_0;

                AG(:, i) = (X_G_i)' * Ic{i} * obj.S{i};
                
                hG = hG + AG(:, i) * obj.dq(i);    
            end

            IG = X_G_0' * I_0_C * X_G_0;

            vG = IG \ hG;

            dAG = zeros(6, NB);

            for i = 1:NB                
                X_G_i = obj.XBase{i} * X_G_0;
                vG = [zeros(3, 1); vG(4:6)];
                dX = X_G_i * mx_vCross(vG) - mx_vCross(obj.VBody{i}) * X_G_i;

                dAG(:, i) = dX' * Ic{i} * obj.S{i} + X_G_i' * dIc{i} * obj.S{i};
            end            
        end
        
        function [H, C] = getDynamicsHandC(obj, f_ext, grav_accn)
            if nargin < 3
              a_grav = [0;0;0;0;0;-9.81];
            else
              a_grav = [0;0;0;grav_accn(1);grav_accn(2);grav_accn(3)];
            end

            external_force = (nargin > 1 && ~isempty(f_ext));

            NB = obj.tree.NB;
            avp = cell(1, NB);
            fvp = cell(1, NB);

            for i = 1:NB
                j = obj.tree.parent(i);
                if j == 0
                    avp{i} = obj.Xup{i} * -a_grav;
                else
                    avp{i} = obj.Xup{i} * avp{j} + crm(obj.VBody{i}) * obj.S{i} * obj.dq(i);
                end
                fvp{i} = obj.tree.I{i} * avp{i} + crf(obj.VBody{i}) * obj.tree.I{i} * obj.VBody{i};
                if external_force
                    fvp{i} = fvp{i} - f_ext(:, i);
                end
            end

            IC = obj.tree.I;
            C = zeros(NB, 1);

            for i = NB:-1:1
                C(i,1) = obj.S{i}' * fvp{i};
                j = obj.tree.parent(i);
                if j ~= 0
                    fvp{j} = fvp{j} + obj.Xup{i}' * fvp{i};
                    IC{j} = IC{j} + obj.Xup{i}' * IC{i} * obj.Xup{i};
                end
            end

            H = zeros(NB, NB);

            for i = 1:NB
                n = i;
                fh = IC{i} * obj.S{i};
                H(n, n) = obj.S{i}' * fh;
                j = i;
                while obj.tree.parent(j) > 0
                    fh = obj.Xup{j}' * fh;
                    j = obj.tree.parent(j);
                    np = j;
                    H(n, np) = obj.S{j}' * fh;
                    H(np, n) = H(n, np);
                end
            end            
        end
                
    end
    
    methods (Access = private)
        function e = getTreeNodes(obj, bodyN)
            NB = obj.tree.NB;
            e = zeros(1, NB);
            body = bodyN;
            while body ~= 0
                e(body) = 1;
                body = obj.tree.parent(body);
            end            
        end
        
        function Jb = getJacBody(obj, bodyN)            
            e = obj.getTreeNodes(bodyN);

            NB = obj.tree.NB;
            J = zeros(6, NB);
            for i = 1:NB
                if e(i)
                    J(:, i) = obj.XBase{i}^(-1) * obj.S{i};
                end    
            end

            Jb = obj.XBase{bodyN} * J;            
        end
        
        function dJb = getJacBodyDot(obj, bodyN)
            e = obj.getTreeNodes(bodyN);

            NB = obj.tree.NB;
            dJb = zeros(6, NB);
            for i = 1:NB
                if e(i)
                    if i == bodyN
                        dJb(:, i) = zeros(6, 1);
                    else
                        Xtemp = obj.XBase{bodyN} * (obj.XBase{i})^(-1);
                        dX = Xtemp * mx_vCross(obj.VBody{i}) - mx_vCross(obj.VBody{bodyN}) * Xtemp;
                        dJb(:, i) = dX * obj.S{i};
                    end
                end    
            end
        end
        
        function projDot = getProjDot(obj, X_G_i, vG, bodyN)
            I = obj.tree.I{bodyN};
            vi = obj.VBody{bodyN};

            dX = X_G_i * mx_vCross(vG) - mx_vCross(vi) * X_G_i;

            projDot = dX' * I;
        end
        
    end

end

function rotR = getRotRuseAxis(rotAxis)
    switch rotAxis
        case 'x'
            rotR = mx_Roty_3D(-pi/2);
        case 'y'
            rotR = mx_Rotx_3D(pi/2);
        case 'z'
            rotR = eye(3);
        otherwise
            error('Invalid rotAxis!!!');
    end
end