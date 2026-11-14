# Lander-C

A 2D powered-landing simulator written in plain C.

# Status
Free fall validated against the analytical solution (RK4 max error ~1e-12 m)

Thrust and mass depletion validated against the rocket equation

Vertical PD controller lands from 100 m at under 1 m/s

# Build
make
make test