using OMJulia


omc = OMJulia.OMCSession()
model_path = replace(@__FILE__, r".jl$" => ".mo")
loaded_file_sucessfully = sendExpression(omc, "loadFile(\"$model_path\")")
@assert loaded_file_sucessfully

# this is only to handle simulation output artifacts easily, not necessary for the simulation itself
sendExpression(omc, "cd(\"$(@__DIR__)\")")

# run the simulation and save its output to a csv file
sendExpression(omc, "simulate(InvertedPendulum.InvertedPendulumPulse, outputFormat=\"csv\")")
