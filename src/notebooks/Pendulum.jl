using MuJoCo, LinearAlgebra, MatrixEquations

model = load_model(joinpath(@__DIR__, "cartpole.xml"))

data = init_data(model)

println("Initial position: ", data.qpos)
println("Initial velocity: ", data.qvel)

# Number of states and controlled inputs
nx = 2*model.nv
nu = model.nu

# Finite-difference parameters
ϵ = 1e-6
centred = true

# Compute the Jacobians
A = mj_zeros(nx, nx)
B = mj_zeros(nx, nu)
mjd_transitionFD(model, data, ϵ, centred, A, B, nothing, nothing)
@show A, B

Q = diagm([1, 10, 1, 5]) # Weights for the state vector
R = diagm([1])           # Weights for the controls

S = zeros(nx, nu)
_, _, K, _ = ared(A,B,R,Q,S)
@show K

function lqr_balance!(m::Model, d::Data)
    state = vcat(d.qpos, d.qvel)
    d.ctrl .= -K * state
    nothing
end

init_visualiser()
visualise!(model, data; controller=lqr_balance!)
# reset!(model, data)