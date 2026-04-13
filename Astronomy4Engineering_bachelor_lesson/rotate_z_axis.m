function matrix_R=rotate_z_axis(degree)

matrix_R=[cos(degree) sin(degree) 0;
    -sin(degree) cos(degree) 0;
    0 0 1;];

end