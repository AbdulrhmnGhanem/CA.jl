### A Pluto.jl notebook ###
# v0.19.27

using Markdown
using InteractiveUtils

# ╔═╡ 02a2e3c8-3fdb-11ee-3415-31c1bc35e4c2
# ╠═╡ show_logs = false
begin
	using Pkg
	Pkg.activate(joinpath(@__DIR__, "..", ".."))
end

# ╔═╡ fbef6dc8-a217-4bf6-91a5-34ccd1932a12
begin
	using ModelingToolkit, DifferentialEquations, CairoMakie, Plots, PlutoUI
	using ModelingToolkitStandardLibrary.Mechanical.MultiBody2D
	using ModelingToolkitStandardLibrary.Mechanical.Translational
end

# ╔═╡ cd833817-084b-4411-afb9-14607b3f3cbc
md"# Reversed Pendulum Model"

# ╔═╡ fa6232f2-70d7-4d91-997b-d3197c863264
begin
	@parameters t

	m_ball = 10.0
	m_link = 1.0
	g = 9.81
	l = 10  # Length of the pendulum
	I_pendulum = (1/3) * m_link * l^2  # Moment of inertia of a thin rod about its end
	
	@named link = Link(; m = 0.1, l = l, I = I_pendulum, g = -g)
	@named ball = Mass(; m = m_ball, s = 0)
	@named fixed = Translational.Fixed()
	
	eqs = [
	    connect(link.TX1, ball.flange),
	    connect(link.TY1, fixed.flange)
	]
	
	@named model = ODESystem(eqs, t, [], [], systems = [link, ball, fixed])
	
	sys = structural_simplify(model)
end;

# ╔═╡ 7f6b5676-32b8-4ef2-a292-f0df1fba3765
begin
	unset_vars = setdiff(states(sys), keys(ModelingToolkit.defaults(sys)))
	prob = ODEProblem(sys, unset_vars .=> 0.0, (0.0, 60), []; jac = true)
	sol = solve(prob, Rodas5P())
end;

# ╔═╡ 5597c8e6-9350-41e9-b3b3-ac9c344185e1
function plot_link(sol, sys, tmax)
    tm = Observable(0.0)
    idx = Dict(reverse.(enumerate(states(sys))))

    fig = Figure()
    a = Axis(fig[1,1], aspect=DataAspect())
    hidedecorations!(a)
    s = @lift(sol($tm, idxs=[link.x1, link.x2, link.y1, link.y2]))

    m1x1 = @lift($s[1])
    m1x2 = @lift($s[2])

    m1y1 = @lift($s[3])
    m1y2 = @lift($s[4])

    sz1 = 0.5

    CairoMakie.lines!(a, @lift([$m1x1, $m1x2]), @lift([$m1y1, $m1y2]), linewidth=5, color=:red)
    CairoMakie.scatter!(a, @lift([$m1x2]), @lift([$m1y2]), markersize=40, markercolor=:red)


    CairoMakie.ylims!(a, -20, 10)
    CairoMakie.xlims!(a, -20, 20)

    framerate = 30
    timestamps = range(0, tmax, step=1/framerate)

    record(fig, "pendulum.gif", timestamps;
            framerate = framerate) do t
        tm[] = t
    end

    nothing
end

# ╔═╡ a5532a80-3e25-4653-a8b7-84c63dcf1d4a
begin
	plot_link(sol, sys, 60)
	LocalResource(joinpath(@__DIR__, "pendulum.gif"))
end

# ╔═╡ fa5cc99e-9b0f-48a9-b080-7a5e0b543766
Plots.plot(sol, idxs=[link.A])

# ╔═╡ Cell order:
# ╟─cd833817-084b-4411-afb9-14607b3f3cbc
# ╠═02a2e3c8-3fdb-11ee-3415-31c1bc35e4c2
# ╠═fbef6dc8-a217-4bf6-91a5-34ccd1932a12
# ╠═fa6232f2-70d7-4d91-997b-d3197c863264
# ╠═7f6b5676-32b8-4ef2-a292-f0df1fba3765
# ╠═5597c8e6-9350-41e9-b3b3-ac9c344185e1
# ╠═a5532a80-3e25-4653-a8b7-84c63dcf1d4a
# ╠═fa5cc99e-9b0f-48a9-b080-7a5e0b543766
