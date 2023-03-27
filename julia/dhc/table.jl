module DHCModule
    export generating_robot_kinematic
    
    using Symbolics

    function generating_robot_kinematic()
        @variables t
        @variables q1(t), q2(t), q3(t), q4(t), q5(t), q6(t), q7(t)

        q = [q1, q2, q3, q4, q5, q6, q7];

        Robot = [ 0         q1      0       π/2     0;
                0         q2      .25     π/2     π/2;
                0         q3      .26     -π/2    0;
                0         q4      0       π/2     -π/2;
                .3385     q5      0       π/2     π/2;
                0         q6      0       -π/2    0;
                .1385     q7      0       0       0];
        return Robot, q
    end
end