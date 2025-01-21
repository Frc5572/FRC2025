using SymPy

@syms x, c_x, c_y, p_x, p_y, k, r
# Create equation passing through point with unknown slope `k`
y = k * (x - p_x) + p_y
# This equation also intersects the circle
eq = (x - c_x)^2 + (y - c_y)^2 - r^2
# Get coefficients of `eq`
collected_expr = collect(factor(eq), x)
a = collected_expr.coeff(x, 2)
b = collected_expr.coeff(x, 1)
c = collected_expr.coeff(x, 0)
# Determinant determines how many intersections on the circle. We want only 1 per line (makes the line tangent)
det = b^2 - 4*a*c
# Solve for `k`
res = collect(solveset(Eq(det, 0), k))
@assert length(res) == 2

y1 = subs(y, k=>res[1])
y2 = subs(y, k=>res[2])

println("y1:")
println(y1)
println("y2:")
println(y2)


