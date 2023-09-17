Nothing to see here 👀

# Installation

```
julia --project=. -e 'using Pkg; Pkg.instantiate()'
```

## Set up Open Modelica

1. Install Open Modelica from [here](https://openmodelica.org/download/download-linux)
2. Install the required packages
```
julia --project=. src/install_modelica_packages.jl
```
