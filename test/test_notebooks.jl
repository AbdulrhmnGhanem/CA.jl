using Test

@testset "Notebooks run without errors" begin
    include("../src/notebooks/Pendulum.jl")
    @test isfile("pendulum.gif")
    @test isfile("slider_pendulum.gif")
end