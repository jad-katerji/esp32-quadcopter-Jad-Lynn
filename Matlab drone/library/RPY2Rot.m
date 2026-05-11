function bRi = RPY2Rot(angles)
    phi = angles(1);
    thetta = angles(2);
    psi = angles(3);

    R_3 = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
    R_2 = [cos(thetta) 0 -sin(thetta); 0 1 0; sin(thetta) 0 cos(thetta)];
    R_1 = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];

    bRi = R_3*R_2*R_1;

end