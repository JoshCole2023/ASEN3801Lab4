function motor_forces = ComputeMotorForces(Fc, Gc, d, km)
% Used for 2.4

Zc = Fc(3);

Lc = Gc(1);
Mc = Gc(2);
Nc = Gc(3);

A = [
1 1 1 1
-d/sqrt(2) -d/sqrt(2) d/sqrt(2) d/sqrt(2)
 d/sqrt(2) -d/sqrt(2) -d/sqrt(2) d/sqrt(2)
 km -km km -km
];

b = [Zc; Lc; Mc; Nc];

motor_forces = A \ b;

end