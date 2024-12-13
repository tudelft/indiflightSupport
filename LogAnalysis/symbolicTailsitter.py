import sympy as sp

w1, w2, s1, s2, s1dd, s2dd = sp.symbols('w1 w2 s1 s2 s1dd s2dd')
dw1, dw2, ds1, ds2, ds1dd, ds2dd = sp.symbols('dw1 dw2 ds1 ds2 ds1dd ds2dd')
p1, p2, p3, p4, p5, p6 = sp.symbols('p1 p2 p3 p4 p5 p6')

u = sp.Matrix([w1, w2, s1, s2, s1dd, s2dd])
du = sp.Matrix([dw1, dw2, ds1, ds2, ds1dd, ds2dd])
p = sp.Matrix([p1, p2, p3, p4, p5, p6])

pitch = sp.Matrix([p1 * (w1*w1) * s1
                    + p2 * (w2*w2) * s2
                    + p3 * (w1*w1)
                    + p4 * (w2*w2)
                    + p5 * s1dd
                    + p6 * s2dd])

incremental_model = pitch.jacobian(u) @ du
Apitch = incremental_model.jacobian(p)


