### A Pluto.jl notebook ###
# v0.19.40

using Markdown
using InteractiveUtils

# ╔═╡ ec63da8e-00aa-11ef-34c4-f9d8729e2c5c
# ╠═╡ show_logs = false
begin
	using Pkg
	Pkg.activate(joinpath(@__DIR__, "..", ".."))
end

# ╔═╡ 21ebbf0a-597b-4c99-83a6-a6268e069228
# ╠═╡ show_logs = false
using ControlSystems,RobustAndOptimalControl, ForwardDiff, LinearAlgebra, Plots

# ╔═╡ b0b28c23-ed5a-45a1-9fa5-a56893b46db7
begin
	# helper function to simulate ramp input
	rampsim(sys) = lsim(sys[:, 1], (x,t)->[min(t, 0.5)], 0:0.01:3.5, method=:zoh)
	# System model
	function cartpole(x, u)
	    mc, mp, l, g = 1.0, 0.2, 0.5, 9.81
	
	    q  = x[1:2]
	    qd = x[3:4]
	
	    s = sin(q[2])
	    c = cos(q[2])
	
	    H = [mc+mp mp*l*c; mp*l*c mp*l^2]
	    C = [0.1 -mp*qd[2]*l*s; 0 0]
	    G = [0, mp * g * l * s]
	    B = [1, 0]
	
	    qdd = -H \ (C * qd + G - B * u[1])
	    return [qd; qdd]
	end
	
	# Linearization
	x0 = [0, π, 0, 0]
	u0 = [0]
	
	Ac = ForwardDiff.jacobian(x->cartpole(x, u0), x0)
	Bc = ForwardDiff.jacobian(u->cartpole(x0, u), u0)
	Cc = [1 0 0 0; 0 1 0 0]
	Λ = Diagonal([0.4, deg2rad(25)]) # Maximum output ranges
	Cc = Λ \ Cc
	
	# Create StateSpace object
	sys = ss(Ac, Bc, Cc, 0)

	desired_poles = [-4.85, -4.85, -5, -5]
	L = place(sys, desired_poles, :c)

	Bw = [0 0; 0 0; 1 0; 0 1]
	R1 = Bw*I*Bw'
	R2 = 0.0001I
	K = kalman(sys, R1, R2)

	controller = observer_controller(sys, L, K)
	sim_pp  = rampsim(feedback(sys*controller))
	# sim_pp  = impulse(feedback(sys, controller), 8, method=:zoh)
end

# ╔═╡ 7c4dc3e4-dd95-4112-97b0-b9913ea9c8dc

	@gif for i in 1:3:length(sim_pp.t)
	    p, a = sim_pp.y[:, i]
	    plot([p; p - 0.5sin(a)], [0; 0.5cos(a)], 
			lw=1, 
			markershape=:square, 
			markersize=[8, 1], 
			lab="PP",
			xlims=(-5, 5), 
			ylims=(-0.2, 0.6), 
			title="Inverted pendulum control", 
			dpi=200, 
			aspect_ratio=1,
			size(600, 600),
		)
	end

# ╔═╡ c80e167f-309d-42f0-a0bb-d141d9ea65cb
function model()
	M = 0.5
	m = 0.2
	b = 0.1
	I = 0.006
	g = 9.8
	l = 0.3
	
	q = (M + m) * (I + m * l^2) - (m * l)^2
	
	s = tf('s')
	
	P_pend = (m * l * s / q) / (s^3 + (b * (I + m * l^2)) * s^2 /q - ((M + m) * m * g * l) * s / q - b * m * g * l / q)
	P_cart = ((( I + m * l^2) / q) * s^2 - (m * g * l / q)) / (s^4 + (b * (I + m * l^2)) * s^3 / q - ((M + m) * m * g * l) * s^2 / q - b * m * g * l * s / q)

	Kp = 100
	Ki = 1
	Kd = 1
	
	C = pid(Kp, Ki, Kd)
	
	T = feedback(P_pend, C)
	T2 = feedback(1, P_pend * C) * P_cart
	t = 0:0.01:10
	t, impulse(T,t), impulse(T2, t)
end

# ╔═╡ 7d297f81-a255-4c0f-a170-b913a190d875
t, pendulum, cart  = model()

# ╔═╡ 7efae060-15f1-4280-b29c-b55ef5649ed8
plot(t, pendulum.y';
)

# ╔═╡ Cell order:
# ╠═ec63da8e-00aa-11ef-34c4-f9d8729e2c5c
# ╠═21ebbf0a-597b-4c99-83a6-a6268e069228
# ╠═b0b28c23-ed5a-45a1-9fa5-a56893b46db7
# ╠═7c4dc3e4-dd95-4112-97b0-b9913ea9c8dc
# ╠═c80e167f-309d-42f0-a0bb-d141d9ea65cb
# ╠═7d297f81-a255-4c0f-a170-b913a190d875
# ╠═7efae060-15f1-4280-b29c-b55ef5649ed8
