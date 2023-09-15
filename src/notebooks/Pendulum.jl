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
	using ModelingToolkit, DifferentialEquations, CairoMakie, PlutoUI
	using ModelingToolkitStandardLibrary.Mechanical.MultiBody2D
	using ModelingToolkitStandardLibrary.Mechanical.TranslationalPosition
	using Makie.GeometryBasics
end

# ╔═╡ cd833817-084b-4411-afb9-14607b3f3cbc
md"# Inverted Pendulum Model"

# ╔═╡ 2f6c7627-7283-4f57-bbe4-e2cec75de3b5
md"## Normal pendulum"

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
	@named fixed = TranslationalPosition.Fixed()
	
	eqs = [
	    ModelingToolkit.connect(link.TX1, ball.flange),
	    ModelingToolkit.connect(link.TY1, fixed.flange)
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
function plot_link(sol, sys, tmax; filename)
    tm = Observable(0.0)
    idx = Dict(reverse.(enumerate(states(sys))))

    fig = Figure()
    a = Axis(fig[1,1], aspect=DataAspect())
    hidedecorations!(a)
    hidespines!(a)
    s = @lift(sol($tm, idxs=[link.x1, link.x2, link.y1, link.y2]))

    m1x1 = @lift($s[1])
    m1x2 = @lift($s[2])

    m1y1 = @lift($s[3])
    m1y2 = @lift($s[4])

    sz1 = 0.5

	colormap = :julia_colorscheme
	colorrange = (1, 10)

    CairoMakie.lines!(a, @lift([$m1x1, $m1x2]), @lift([$m1y1, $m1y2]);
		linewidth=5, color=1, colormap, colorrange
	)
    CairoMakie.scatter!(a, @lift([$m1x2]), @lift([$m1y2]);
		markersize=40, color=[7], colormap, colorrange
	)


    CairoMakie.ylims!(a, -20, 5)
    CairoMakie.xlims!(a, -20, 20)

    framerate = 30
    timestamps = range(0, tmax, step=1/framerate)

    record(fig, filename, timestamps;
            framerate = framerate) do t
        tm[] = t
    end

    nothing
end

# ╔═╡ a5532a80-3e25-4653-a8b7-84c63dcf1d4a
begin
	plot_link(sol, sys, 60; filename="pendulum.gif")
	LocalResource(joinpath(@__DIR__, "pendulum.gif"))
end

# ╔═╡ 8e7b2555-a969-455f-b25d-bee26065eef3
md"## Pendulum on a slider"

# ╔═╡ bf64c002-8443-4df8-9350-dceb08ce7751
function plot_slider_pendulum(sol, sys, tmax; filename)
    tm = Observable(0.0)
    idx = Dict(reverse.(enumerate(states(sys))))

    fig = Figure()
    a = Axis(fig[1,1], aspect=DataAspect())
	hidedecorations!(a)
    hidespines!(a)

    s = @lift(sol($tm, idxs=[link.x1, link.x2, link.y1, link.y2]))

    m1x1 = @lift($s[1])
    m1x2 = @lift($s[2])

    m1y1 = @lift($s[3])
    m1y2 = @lift($s[4])

    sz1 = 0.5

	colormap = :julia_colorscheme
	colorrange = (1, 10)

	# draw the slider housing
	Housing = Polygon(
		Point2f[(-10.5, -0.5), (10.5, -0.5), (10.5, 0.5), (-10.5, 0.5)],
		[Point2f[(-10.2, -0.2), (10.2, -0.2), (10.2, 0.2), (-10.2, 0.2)]]
		
	)
	CairoMakie.poly!(Housing; color=1, colormap, colorrange)

	# draw the screw
	CairoMakie.scatter!(a, @lift([$m1x2]), @lift([$m1y2]);
		markersize=15, color=[6], colormap, colorrange,
	)

    CairoMakie.lines!(a, @lift([$m1x1, $m1x2]), @lift([$m1y1, $m1y2]);
		linewidth=5, color=1, colormap, colorrange,
	)
    CairoMakie.scatter!(a, @lift([$m1x1]), @lift([$m1y1]);
		markersize=40, color=[7], colormap, colorrange,
	)
	
	
    CairoMakie.ylims!(a, -20, 5)
    CairoMakie.xlims!(a, -20, 20)

    framerate = 30
    timestamps = range(0, tmax, step=1/framerate)

    record(fig, filename, timestamps;
            framerate = framerate) do t
        tm[] = t
    end

    nothing
end

# ╔═╡ 9406380c-8552-4b8b-b342-2bbc2de8e4b9
begin
	slider_eqs = [
		    ModelingToolkit.connect(link.TX1, ball.flange),
			ModelingToolkit.connect(link.TY2, fixed.flange),
	]
		
	@named slider_penulum_model = ODESystem(slider_eqs, t, [], [], systems = [link, ball, fixed])
	
	slider_penulum = structural_simplify(slider_penulum_model)
	prob_2 = ODEProblem(slider_penulum, unset_vars .=> 0, (0.0, 60), []; jac = true)
	sol_2 = solve(prob_2)
end;

# ╔═╡ a0dda65e-730b-46c3-93b2-760778741c0e
begin
	plot_slider_pendulum(sol_2, slider_penulum, 60; filename="slider_pendulum.gif")
	LocalResource(joinpath(@__DIR__, "slider_pendulum.gif"))
end

# ╔═╡ Cell order:
# ╟─cd833817-084b-4411-afb9-14607b3f3cbc
# ╠═02a2e3c8-3fdb-11ee-3415-31c1bc35e4c2
# ╠═fbef6dc8-a217-4bf6-91a5-34ccd1932a12
# ╟─2f6c7627-7283-4f57-bbe4-e2cec75de3b5
# ╠═fa6232f2-70d7-4d91-997b-d3197c863264
# ╠═7f6b5676-32b8-4ef2-a292-f0df1fba3765
# ╟─5597c8e6-9350-41e9-b3b3-ac9c344185e1
# ╠═a5532a80-3e25-4653-a8b7-84c63dcf1d4a
# ╟─8e7b2555-a969-455f-b25d-bee26065eef3
# ╟─bf64c002-8443-4df8-9350-dceb08ce7751
# ╠═9406380c-8552-4b8b-b342-2bbc2de8e4b9
# ╠═a0dda65e-730b-46c3-93b2-760778741c0e
