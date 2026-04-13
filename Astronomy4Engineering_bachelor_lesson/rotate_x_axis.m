function matrix_R=rotate_x_axis(degree)

matrix_R=[1 0 0;
    0 cos(degree) sin(degree);
    0 -sin(degree) cos(degree);];

end