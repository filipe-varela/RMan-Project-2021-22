FROM julia:latest

RUN julia -e 'using Pkg; Pkg.add("Symbolics");'

WORKDIR /rman-julia

COPY . .

CMD ["julia", "main.jl"]