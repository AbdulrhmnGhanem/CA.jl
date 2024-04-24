### A Pluto.jl notebook ###
# v0.19.40

using Markdown
using InteractiveUtils

# ╔═╡ 92e82f39-9f72-492e-9984-6674b2302912
# ╠═╡ show_logs = false
begin
	using Pkg
	Pkg.activate(joinpath(@__DIR__, "..", ".."))
end

# ╔═╡ c9d61951-b8fc-4a8b-856a-b34e6a9e4b32
using ModelPredictiveControl, ControlSystemsBase, Plots

# ╔═╡ fb01feae-024e-11ef-0f2f-95d33d8bb5d7
md"# CSTR using MPC

[Original source](https://github.com/JuliaControl/ModelPredictiveControl.jl/blob/58e355e57a9bd6ceaeeab0018b246b071df72f06/docs/src/manual/linmpc.md)
"

# ╔═╡ 110688cb-30a4-427b-8388-d47b4b636e38
Ts = 2.0

# ╔═╡ 8a205a47-cce5-479d-8c2b-432ea06c2293
function build_model()
	G = [ tf(1.90, [18, 1]) tf(1.90, [18, 1]);
	  tf(-0.74,[8, 1])  tf(0.74, [8, 1]) ]
	model = setop!(LinModel(G, Ts), uop=[20, 20], yop=[50, 30])
end

# ╔═╡ f1eec7ea-3314-4920-bfc9-8755123a98b2
function controller(model)
	mpc = LinMPC(model, Hp=10, Hc=2, Mwt=[1, 1], Nwt=[0.1, 0.1])
	# constrains: The tank level should also never fall below 45
	mpc = setconstraint!(mpc, ymin=[45, -Inf])
	u, y = model.uop, model()
	initstate!(mpc, u, y)
	mpc
end

# ╔═╡ d4f2ecc7-cf74-496b-87fb-ffa15f59c0dc
function simulate(model, signals)
	mpc = controller(model)
	N = 200
    u_data, y_data, ry_data = zeros(model.nu, N), zeros(model.ny, N), zeros(model.ny, N)
	rys, uls = signals
    for i = 1:N
		ry, ul = rys[i], uls[i]
        y = model() # simulated measurements
        u = mpc(ry) # or equivalently : u = moveinput!(mpc, ry)
        u_data[:,i], y_data[:,i], ry_data[:,i] = u, y, ry
        updatestate!(mpc, u, y) # update mpc state estimate
        updatestate!(model, u + [0; ul]) # update simulator with the load disturbance
    end
	t_data = Ts * (0:(size(y_data, 2) - 1))
    return t_data, u_data, y_data, ry_data
end

# ╔═╡ c172547e-ec49-484d-ac61-5d1d6ad303da
function visualize(sim_result)
	t_data, u_data, y_data, ry_data = sim_result
	p1 = plot(t_data, y_data[1,:];
		label="meas.",
		ylabel="level",
	)
	plot!(t_data, ry_data[1,:];
		label="setpoint",
		linestyle=:dash,
		linetype=:steppost,
	)
	plot!(t_data, fill(45, size(t_data));
		label="min",
		linestyle=:dot,
		linewidth=1.5,
	)
	
	p2 = plot(t_data, y_data[2,:],
		label="meas.",
		legend=:topleft,
		ylabel="temp.",
	)
	plot!(t_data, ry_data[2,:],
		label="setpoint",
		linestyle=:dash,
		linetype=:steppost,
	)
	p3 = plot(t_data, u_data[1,:];
		label="cold",
		linetype=:steppost,
		ylabel="flow rate",
	)
	plot!(t_data, u_data[2,:];
		label="hot",
		linetype=:steppost,
		xlabel="time (s)",
	)
    
	return plot(p1, p2, p3, layout=(3,1))
end

# ╔═╡ d192ff0a-e3bf-4242-86cc-56bf2156544a
function signals()
	N = 200
	# setpoint signals
	ry = fill([50, 30], N)
	ry[51:100] = fill([50, 35], 50)
	ry[101:end] = fill([54, 30], 100)

	# the load disturbance
	ul = fill(0, N)
	ul[151:end] = fill(-20, 50)
	
	ry, ul
end

# ╔═╡ 93a11f4c-bd96-4b68-b29d-12710c6adb6e
begin
	model = build_model()
	s = signals()
	simulate(model, s) |> visualize
end

# ╔═╡ Cell order:
# ╠═92e82f39-9f72-492e-9984-6674b2302912
# ╟─fb01feae-024e-11ef-0f2f-95d33d8bb5d7
# ╠═c9d61951-b8fc-4a8b-856a-b34e6a9e4b32
# ╠═110688cb-30a4-427b-8388-d47b4b636e38
# ╠═8a205a47-cce5-479d-8c2b-432ea06c2293
# ╠═f1eec7ea-3314-4920-bfc9-8755123a98b2
# ╠═d4f2ecc7-cf74-496b-87fb-ffa15f59c0dc
# ╠═c172547e-ec49-484d-ac61-5d1d6ad303da
# ╠═d192ff0a-e3bf-4242-86cc-56bf2156544a
# ╠═93a11f4c-bd96-4b68-b29d-12710c6adb6e
