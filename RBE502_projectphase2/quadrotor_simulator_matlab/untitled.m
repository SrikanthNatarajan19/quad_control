    

    A = zeros(12, 12);
    B = zeros(12, 4);
    
    A(1,4) = 1;
    A(2,5) = 1;
    A(3,6) = 1;
    A(8,11) = 1;
    
    A(4,7) = g * sin(psi); 
    A(4,8) = g * cos(psi);
    A(4,9) = (-1 * g * theta * sin(psi)) + (g * phi * sin(psi));
    A(5,7) = -1 * g * cos(psi);
    A(5,8) = g * sin(psi);
    A(5,9) = (g * theta * cos(psi)) + (g * phi * sin(psi));
    
    
    A(7,8) = -1 * p * sin(theta) + r * cos(theta);
    A(7,10) = cos(theta);
    A(7,12) = sin(theta);

    A(8,7) =(p * sin(theta) * sec(phi)^2) - (r * cos(theta) * sec(phi)^2);
    A(8,8) = (p * cos(theta) * tan(phi)) + (r * sin(theta) * tan(phi));
    A(8,10)= sin(theta) * tan(phi);
    A(8,12) = -1 * cos(theta) * tan(phi);

    A(9,7) = (-1 * sin(theta) * tan(phi) * sec(phi) * p) + (cos(theta) * tan(phi) * sec(phi) * r);
    A(9,8) = (-1 * cos(theta) * p / cos(phi)) + (-1 * sin(theta) * r / cos(phi));
    A(9,10) = -1 * sin(theta) / cos(phi);
    A(9,12) = cos(theta) / cos(phi);

   
    B(6,1) = 1 / m;          
    B(10,2) = 1 / I(1,1);    
    B(11,3) = 1 / I(2,2);    
    B(12,4) = 1 / I(3,3);   

   
    A_d = eye(12) + A * dt;
    B_d = B * dt;

    disp('A')
    disp(A_d)
    disp('B')
    disp(B_d)