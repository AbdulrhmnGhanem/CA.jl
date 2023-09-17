using OMJulia
omc = OMJulia.OMCSession()

version = split(sendExpression(omc, "getVersion()"), " ")[2]
@assert version == "v1.21.0"

## Installing PlanarMechanics package
# https://openmodelica.org/doc/OpenModelicaUsersGuide/latest/packagemanager.html#using-the-package-manager-from-the-interactive-environment
sendExpression(omc, "updatePackageIndex()")
println(sendExpression(omc, "getErrorString()"))
sendExpression(omc, "installPackage(PlanarMechanics, \"1.6.0\")")
println(sendExpression(omc, "getErrorString()"))
