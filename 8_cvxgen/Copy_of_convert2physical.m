function [x_trajec, u_trajec] = convert2physical(x_trajec, u_trajec, param)
u_trajec = [u_trajec; zeros(1, size(u_trajec,2))];

I = param.I;
m_L = param.MASS_L;
m_Q = param.MASS_Q;
g = param.G;
l = param.LENGTH;
e3 = [0 0 1].';
psi = 0;
psi_dot1 = 0;
psi_dot2 = 0;

length = size(x_trajec,2);
for i=1:length
    if(i~=length)
        x_L = x_trajec(1:3,i);
        x_L_dot1 = x_trajec(4:6,i);
        x_L_dot2 = x_trajec(7:9,i);
        x_L_dot3 = x_trajec(10:12,i);
        x_L_dot4 = x_trajec(13:15,i);
        x_L_dot5 = x_trajec(16:18,i);
        x_L_dot6 = u_trajec(1:3,i);
        
        T = m_L*norm(x_L_dot2 + g*e3); %Tension
        p = -(x_L_dot2 + g*e3)/norm(x_L_dot2 + g*e3); %cable direction
        T_dot1 = -m_L*dot(x_L_dot3,p);
        p_dot1 = -(m_L*x_L_dot3 + T_dot1*p)/T;
        omega = cross(m_L/T*x_L_dot3, p);
        x_Q = x_L - l*p;
        T_dot2 = -m_L*(dot(x_L_dot4,p) + dot(x_L_dot3,p_dot1));
        p_dot2 = -(m_L*x_L_dot4 + 2*T_dot1*p_dot1 + T_dot2*p)/T;
        omega_dot1 = m_L*(cross(x_L_dot4,p) + cross(x_L_dot3, p_dot1));
        T_dot3 = -m_L*(dot(x_L_dot5,p) + 2*dot(x_L_dot4, p_dot1) + dot(x_L_dot3, p_dot2));
        p_dot3 = -(m_L*x_L_dot4 + 3*T_dot2*p_dot1 + 3*T_dot1*p_dot2 + T_dot3*p)/T;
        omega_dot2 = m_L*(cross(x_L_dot5,p) + 2*cross(x_L_dot4,p_dot1) + cross(x_L_dot3,p_dot2));
        T_dot4 = -m_L*(dot(x_L_dot6,p) + 3*dot(x_L_dot5,p_dot1) + 3*dot(x_L_dot4,p_dot2) + dot(x_L_dot3,p_dot3));
        p_dot4 = -(m_L*x_L_dot6+4*T_dot3*p_dot1+6*T_dot2+p_dot2+4*T_dot1*p_dot3+T_dot4*p)/T;
        omega_dot3 = m_L*(cross(x_L_dot6,p) + 3*cross(x_L_dot5,p_dot1) + 3*cross(x_L_dot4,p_dot2) + cross(x_L_dot3,p_dot3));
        
        x_Q_dot1 = x_L_dot1 - l*p_dot1;
        x_Q_dot2 = x_L_dot2 - l*p_dot2;
        x_Q_dot3 = x_L_dot3 - l*p_dot3;
        x_Q_dot4 = x_L_dot4 - l*p_dot4;
        
        b3 = (m_L*(x_L_dot2 + g*e3) + m_Q*(x_Q_dot2 + g*e3))/norm(m_L*(x_L_dot2 + g*e3) + m_Q*(x_Q_dot2 + g*e3));
        c1 = [cos(psi) sin(psi) 0].';
        c2 = [-sin(psi) cos(psi) 0].';
        b1 = cross(c2,b3);
        b1 = b1/norm(b1);
        b2 = cross(b3,b1);
        R = [b1 b2 b3];
        euler = rotm2eul(R,'ZYX').';
        euler = euler(3:-1:1);
        
        f = norm(m_L*(x_L_dot2 + g*e3) + m_Q*(x_Q_dot2 + g*e3));
        f_dot1 = dot(m_Q*x_Q_dot3 + m_L*x_L_dot3,b3);
        
        theta_dot1 = (dot(m_Q/f*x_Q_dot3 - f_dot1/f*b3, b1)-psi_dot1*dot(e3,b2))/dot(c2,b2);
        phi_dot1 = -dot(m_Q/f*x_Q_dot3 - f_dot1/f*b3, b2)-psi_dot1*dot(e3,b2);
        
        
        p_dot1 = cross(omega, p);
        omega_dot1 = cross(m_L*x_L_dot4/T, p) + cross(m_L*x_L_dot3/T, p_dot1);
%         p_dot2 = cross(omega_dot1, p) + cross(omega, p_dot1);
        p_dot2 = -(m_L*x_L_dot4 + 2*T_dot1*p_dot1 + T_dot2*p)/T;
        omega_dot2 = m_L/T*(cross(x_L_dot5, p) + 2*cross(x_L_dot4, p_dot1) + cross(x_L_dot3, p_dot2));
        p_dot3 = cross(omega_dot2,p) + 2*cross(omega_dot1,p_dot1) + cross(omega,p_dot2);
        omega_dot3 = m_L/T*(cross(x_L_dot6, p) + 3*cross(x_L_dot5, p_dot1) + 3*cross(x_L_dot4, p_dot2) + cross(x_L_dot3, p_dot3));
        p_dot4 = cross(omega_dot3, p) + 3*cross(omega_dot2, p_dot1) + 3*cross(omega_dot1, p_dot2) + cross(omega, p_dot3);
        
        x_Q_dot2 = x_L_dot2 - l*p_dot2;
        x_Q_dot3 = x_L_dot3 - l*p_dot3;
        x_Q_dot4 = x_L_dot4 - l*p_dot4;
        
        f = norm(m_Q*x_Q_dot2 + m_Q*g*e3 - T*p);
        b3 = m_Q*x_Q_dot2 + m_Q*g*e3 - T*p;
        b3 = b3/norm(b3);
        R = [b1 b2 b3];
        euler = rotm2eul(R,'ZYX').';
        euler = euler(3:-1:1);
        
        T_dot1 = -m_L*dot(x_L_dot3, p);
        f_dot1 = m_Q*dot(x_Q_dot3, b3) - dot(T_dot1*p + T*p_dot1,b3);
        theta_dot1 = (dot((m_Q/f*x_Q_dot3 - f_dot1/f*b3 - (T_dot1*p + T*p_dot1)/f),b1)-psi_dot1*dot(e3,b2))/dot(c2,b2);
        phi_dot1 = -dot((m_Q/f*x_Q_dot3 - f_dot1/f*b3 - (T_dot1*p + T*p_dot1)/f),b2)-dot(psi_dot1*e3+theta_dot1*c2,b1);
        pqr = Dynamics.rpyDot2Pqr([phi_dot1 theta_dot1 psi_dot1].', euler);
        
%         f_dot2 = dot(m_Q*x_Q_dot4 + m_L*x_L_dot4,b3) -2*f_dot1*cross(pqr,b3) - f*cross(pqr,cross(pqr,b3));
        
        
        temp1 = dot((m_Q*x_Q_dot4  - 2*f_dot1*cross(pqr,b3) - f*cross(pqr,cross(pqr, b3)) -(T_dot2*p + 2*T_dot1*p_dot1 + T*p_dot2)),b1)/f;
        temp2 = dot((m_Q*x_Q_dot4  - 2*f_dot1*cross(pqr,b3) - f*cross(pqr,cross(pqr, b3)) -(T_dot2*p + 2*T_dot1*p_dot1 + T*p_dot2)),b2)/f;
        theta_dot2 = (temp1 - dot(psi_dot2*e3 + cross(psi_dot1*e3, theta_dot1*c2),b2))/dot(c2, b2);
        phi_dot2 = -temp2 - psi_dot2*dot(e3,b1)-dot(cross(psi_dot1*e3, theta_dot1*c2),b1) - theta_dot2*dot(c2,b1);
        alpha = Dynamics.pqr2RpyDot([phi_dot2 theta_dot2 psi_dot2].', euler);
        moment = cross(pqr, I*pqr) + I*alpha;
        
        x_trajec(1:3,i) = x_L;
        x_trajec(4:6,i) = euler;
        x_trajec(7:9,i) = p;
        x_trajec(10:12,i) = x_L_dot1;
        x_trajec(13:15,i) = pqr;
        %     x_trajec(13:15,i) = [phi_dot1 theta_dot1 psi_dot1].';
        x_trajec(16:18,i) = omega;
        
        u_trajec(1,i) = f;
        u_trajec(2:4,i) = moment;
    else
        x_L = x_trajec(1:3,i);
        x_L_dot1 = x_trajec(4:6,i);
        x_L_dot2 = x_trajec(7:9,i);
        x_L_dot3 = x_trajec(10:12,i);
        x_L_dot4 = x_trajec(13:15,i);
        x_L_dot5 = x_trajec(16:18,i);
        
        T = m_L*norm(x_L_dot2 + g*e3); %Tension
        p = (-m_L*x_L_dot2 - m_L*g*e3)/T; %cable direction
        p = p/norm(p);
        omega = cross(m_L*x_L_dot3/T, p); %cable direction rate
        
        p_dot1 = cross(omega, p);
        omega_dot1 = cross(m_L*x_L_dot4/T, p) + cross(m_L*x_L_dot3/T, p_dot1);
        p_dot2 = cross(omega_dot1, p) + cross(omega, p_dot1);
        omega_dot2 = m_L/T*(cross(x_L_dot5, p) + 2*cross(x_L_dot4, p_dot1) + cross(x_L_dot3, p_dot2));
        p_dot3 = cross(omega_dot2,p) + 2*cross(omega_dot1,p_dot1) + cross(omega,p_dot2);
        
        x_Q_dot2 = x_L_dot2 - l*p_dot2;
        x_Q_dot3 = x_L_dot3 - l*p_dot3;
        
        f = norm(m_Q*x_Q_dot2 + m_Q*g*e3 - T*p);
        b3 = m_Q*x_Q_dot2 + m_Q*g*e3 - T*p;
        b3 = b3/norm(b3);
        
        c1 = [cos(psi) sin(psi) 0].';
        c2 = [-sin(psi) cos(psi) 0].';
        b1 = cross(c2,b3);
        b1 = b1/norm(b1);
        b2 = cross(b3,b1);
        R = [b1 b2 b3];
        euler = rotm2eul(R,'ZYX').';
        euler = euler(3:-1:1);
        
        T_dot1 = -m_L*dot(x_L_dot3, p);
        f_dot1 = m_Q*dot(x_Q_dot3, b3) - dot(T_dot1*p + T*p_dot1,b3);
        theta_dot1 = (dot((m_Q/f*x_Q_dot3 - f_dot1/f*b3 - (T_dot1*p + T*p_dot1)/f),b1)-psi_dot1*dot(e3,b2))/dot(c2,b2);
        phi_dot1 = -dot((m_Q/f*x_Q_dot3 - f_dot1/f*b3 - (T_dot1*p + T*p_dot1)/f),b2)-dot(psi_dot1*e3+theta_dot1*c2,b1);
        pqr = Dynamics.rpyDot2Pqr([phi_dot1 theta_dot1 psi_dot1].', euler);
        
%         T_dot2 = -m_L*(dot(x_L_dot4,p) + dot(x_L_dot3,p_dot1));
%         f_dot2 = m_Q*dot(x_Q_dot4,b3) - f*dot(cross(pqr,cross(pqr,b3)),b3) - dot(T_dot2*p + 2*T_dot1*p_dot1 + T*p_dot2,b3);
        
        x_trajec(1:3,i) = x_L;
        x_trajec(4:6,i) = euler;
        x_trajec(7:9,i) = p;
        x_trajec(10:12,i) = x_L_dot1;
        x_trajec(13:15,i) = pqr;
        %     x_trajec(13:15,i) = [phi_dot1 theta_dot1 psi_dot1].';
        x_trajec(16:18,i) = omega;
    end
end

end