syms b1 b2 b3 b4 lMtilde min_lMtilde c1 c2 c3 lTtilde kT kPE e0 vMtilde d1 d2 d3 d4

fprintf('Derivative of tendon force length curve \n')
fprintf('======================================= \n')

f_t = c1*exp(kT*(lTtilde - c2)) - c3;
simplify(diff(f_t, lTtilde))

fprintf('Integral of tendon force length curve \n')
fprintf('===================================== \n')

f_t = c1*exp(kT*(lTtilde - c2)) - c3;
simplify(int(f_t, lTtilde))

fprintf('Derivative of active fiber force length curve \n')
fprintf('============================================= \n')

f_act = b1 * exp((-0.5*(lMtilde-b2)^2) / ((b3 + b4*lMtilde)^2));
simplify(diff(f_act, lMtilde))

fprintf('Derivative of active fiber force velocity curve \n')
fprintf('=============================================== \n')
tempV = d2 * vMtilde + d3;
tempLogArg = tempV + sqrt(tempV^2 + 1.0);
f_v = d1 * log(tempLogArg) + d4;
simplify(diff(f_v, vMtilde))

fprintf('Derivative of passive fiber force length curve \n')
fprintf('============================================== \n')

offset = exp(kPE * (min_lMtilde - 1.0) / e0);
denom = exp(kPE) - offset;
f_pas = (exp(kPE * (lMtilde - 1.0) / e0) - offset) / denom;
simplify(diff(f_pas, lMtilde))

fprintf('Integral of passive fiber force length curve \n')
fprintf('============================================ \n')

f_pas_int = simplify(int(f_pas, lMtilde));
pretty(f_pas_int)

temp1 = exp(kPE * lMtilde / e0);
temp2 = exp(kPE * min_lMtilde / e0);
temp3 = exp(kPE * (e0 + 1.0) / e0);
numer = - e0 * temp1 + kPE * lMtilde * temp2;
denom = kPE * (temp2 - temp3);
f_pas_int_decomp = numer / denom;

fprintf('Expressions equal: ')
isequal(f_pas_int, f_pas_int_decomp)