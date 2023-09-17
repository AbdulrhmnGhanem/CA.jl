using CA
using Test

@testset "CA.jl" begin
    @testset "notebooks" begin
        include("test_notebooks.jl")
    end
end
