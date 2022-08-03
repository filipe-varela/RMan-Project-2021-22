# Copyright 2022 Filipe Varela
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
# http://www.apache.org/licenses/LICENSE-2.0
#  
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
 
using Symbolics # to get symbolic variables in the workspace

include("./dhc/table.jl")
import .DHCModule

Robot, q = DHCModule.generating_robot_kinematic()

println(Robot)
println(q)

# @variables t
# @variables q1(t), q2(t), q3(t), q4(t), q5(t), q6(t), q7(t)

# q = [q1, q2, q3, q4, q5, q6, q7]

# println(q[1])
# println("Hello world!")
# println(substitute(a, (Dict(a => 0.0))))